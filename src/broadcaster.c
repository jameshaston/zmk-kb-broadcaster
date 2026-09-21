/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/hid.h>
#include <zmk/usb.h>
#include <zmk/ble.h>
#include <zmk/split/central.h>
#include <zmk/workqueue.h>
#include <raw_hid/events.h>

#include <kb_broadcaster/protocol.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Thread model (zmk v0.3 pin):
 * - Key/layer/keycode + USB-conn + BLE-profile + split-peripheral events are
 *   raised on the system workqueue; central battery events on ZMK's
 *   low-priority workqueue (battery.c k_timer -> lowprio work); BT
 *   connection callbacks (BT_CONN_CB) run on the BT RX thread.
 * - The raw-hid transport's raw_hid_sent listeners run INLINE on the
 *   raiser's thread (zmk-raw-hid hog.c/usb_hid.c), so emitting from any
 *   thread would block it (USB leg: k_sem_take K_MSEC(30)) and race the
 *   shared seq/statics across threads.
 *
 * Therefore producers NEVER touch emitter state: every emit site packs a
 * message and enqueues it (k_msgq_put, K_NO_WAIT). A single consumer work
 * item on the low-priority workqueue serializes all dedupe state, seq
 * updates, buffer fills, and raises. The low-prio queue is deliberate: the
 * broadcast stream is telemetry, so queue latency must never land on the
 * system workqueue where the keyboard's own event path (incl. hold-tap's
 * k_msleep(10)) already runs.
 */

/*
 * Shared sequence counter, owned exclusively by the consumer work handler
 * (low-prio workqueue only). Wraps at 255 (uint8_t); the host detects gaps
 * by checking for discontinuity, not monotonic increase, so wrap-around is
 * safe.
 */
static uint8_t seq_counter = 0;

/*
 * Emit messages. Producers fill a msg and enqueue it; the consumer dedupes,
 * stamps seq/timestamp, fills the packet buffer, and raises.
 */
enum emit_kind {
    EMIT_POSITION,
    EMIT_LAYER,
    EMIT_KEYCODE,
    EMIT_BATTERY,
    EMIT_CONN_EVENT,   /* BT conn connected/disconnected delta */
    EMIT_CONN_RESCAN,  /* full re-scan of live connections (rare) */
    EMIT_CONNECTIVITY, /* re-snapshot live connectivity state */
    EMIT_HELLO,
};

struct position_msg {
    uint32_t position;
    uint8_t source;
    uint8_t state;
    uint32_t timestamp; /* pre-truncated LE32 payload value */
};

struct layer_msg {
    uint8_t layer;
    uint8_t state;
    uint32_t timestamp;
};

struct keycode_msg {
    uint16_t usage_page;
    uint16_t keycode;
    uint8_t implicit_mods;
    uint8_t explicit_mods;
    uint8_t state;
    uint32_t timestamp;
};

struct battery_msg {
    uint8_t wire_source; /* 0xFF=central, else slot index */
    uint8_t index;       /* dedupe stash slot (KB_BATT_INDEX_*) */
    uint8_t soc;
};

struct conn_msg {
    uint8_t is_host;  /* role BT_CONN_ROLE_PERIPHERAL */
    uint8_t connected;
};

struct emit_msg {
    enum emit_kind kind;
    union {
        struct position_msg position;
        struct layer_msg layer;
        struct keycode_msg keycode;
        struct battery_msg battery;
        struct conn_msg conn;
    };
};

/* Consumer-owned dedupe/emitter state. Only the work handler touches these. */

/* Host modifiers byte of the last emitted 0x04 (diff baseline). */
static uint8_t last_sent_mods = 0;

/*
 * Dedupe table for 0x03. Keyed on the wire-truncated (usage_page, keycode)
 * pair. Mirrors the HID report limit of 6 simultaneous keys; a 7th concurrent
 * keycode is emitted without tracking (degenerate case, safe).
 */
struct active_key {
    uint16_t usage_page;
    uint16_t keycode;
};

static struct active_key active_keys[6];
static uint8_t active_key_count = 0;

/*
 * Last emitted SoC per wire source. 0xFF sentinel = never sent (not a valid
 * SoC), so the first reading always emits. Index 0 = central, 1 = slot0.
 * Sources beyond slot0 are emitted untracked. Also read by HELLO's
 * battery_central field (0xFF = not yet reported at boot).
 */
enum { KB_BATT_INDEX_CENTRAL = 0, KB_BATT_INDEX_SLOT0 = 1, KB_BATT_INDEX_COUNT = 2 };
static uint8_t last_sent_battery[KB_BATT_INDEX_COUNT] = {0xFF, 0xFF};

/*
 * Connectivity tracking. Host conns = role BT_CONN_ROLE_PERIPHERAL (a host
 * connected to us); split conns = role BT_CONN_ROLE_CENTRAL (we connected to
 * a split peripheral). Counts are adjusted only by the consumer from conn
 * messages; the baseline comes from the init-time snapshot (bt_conn_foreach,
 * run on the workqueue — never inside a BT_CONN_CB, which would take the
 * conn-list lock the BT callback path may already hold).
 */
struct kb_conn_tracking {
    uint8_t host_conns;
    uint8_t split_conns;
};
static struct kb_conn_tracking conn_track;

/*
 * Last emitted 0x07 wire snapshot, for byte-for-byte dedupe. Sentinels force
 * the first emit. Absorbs the double-fire from the BLE profile event + our
 * conn callbacks on the same connect/disconnect.
 */
struct kb_conn_wire {
    uint8_t usb_conn;
    uint8_t ble_profile;
    uint8_t ble_connected;
    uint8_t split_status;
};
static struct kb_conn_wire last_conn = {0xFF, 0xFF, 0xFF, 0xFF};

/* Previous USB state, to detect non-HID -> HID transitions for HELLO. */
static enum zmk_usb_conn_state prev_usb_state = ZMK_USB_CONN_NONE;

/* Conn pointer of the last post-encryption HELLO (only touched on the BT RX
 * thread by the security/disconnect callbacks; reset per connection). */
static struct bt_conn *last_hello_conn;

/* Emit pipeline: queue + consumer work (low-prio workqueue). */
K_MSGQ_DEFINE(kb_bcast_msgq, sizeof(struct emit_msg), 32, 4);

/*
 * 0x04 emitter (consumer side). The host modifiers byte is the source of
 * truth (SET_MODIFIERS folds masked + implicit mods, hid.c:45), so this
 * captures SELECT_MODS (&kp LC(A)), &mod_morph masks, and caps_word — which
 * diffing explicit-only mods would miss. Reads the report AFTER event
 * processing, keeping the value order-stable versus the caps_word listener.
 * body.modifiers is a single uint8, so the cross-thread read is atomic on
 * this target. delta is a hint (1=byte grew, 0=released); the host byte is
 * authoritative. Emits nothing when the byte is unchanged.
 */
static void mods_emit(void) {
    const uint8_t mods = (uint8_t)zmk_hid_get_keyboard_report()->body.modifiers;

    if (mods == last_sent_mods) {
        return;
    }

    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_MODIFIERS;
    buf[KB_MODS_OFFSET_HOST_MODS] = mods;
    buf[KB_MODS_OFFSET_EXPLICIT_MODS] = (uint8_t)zmk_hid_get_explicit_mods();
    buf[KB_MODS_OFFSET_DELTA] = mods > last_sent_mods ? 1 : 0;
    buf[KB_MODS_OFFSET_SOURCE] = KB_SOURCE_LOCAL;
    sys_put_le32((uint32_t)(k_uptime_get() & 0xFFFFFFFF), &buf[KB_MODS_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    last_sent_mods = mods;

    LOG_DBG("MODIFIERS mods=0x%02X explicit=0x%02X delta=%u seq=%u",
            mods, buf[KB_MODS_OFFSET_EXPLICIT_MODS], buf[KB_MODS_OFFSET_DELTA],
            buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });
}

/*
 * 0x01 emitter (consumer side). No dedupe — position events are the raw
 * half-stream (central re-raises peripheral positions with .source = slot).
 */
static void position_event_emit(const struct position_msg *msg) {
    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_KEY_POSITION;
    sys_put_le16((uint16_t)msg->position, &buf[KB_POS_OFFSET_POSITION]);
    buf[KB_POS_OFFSET_STATE] = msg->state;
    buf[KB_POS_OFFSET_SOURCE] = msg->source;
    sys_put_le32(msg->timestamp, &buf[KB_POS_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("KEY_POSITION pos=%u state=%u source=%u seq=%u",
            (uint16_t)msg->position, msg->state, msg->source, buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });
}

/* 0x02 emitter (consumer side). locked stays hardcoded 0 at the v0.3 pin. */
static void layer_event_emit(const struct layer_msg *msg) {
    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_LAYER_CHANGE;
    buf[KB_LAYER_OFFSET_LAYER] = msg->layer;
    buf[KB_LAYER_OFFSET_STATE] = msg->state;
    buf[KB_LAYER_OFFSET_LOCKED] = 0;
    buf[KB_LAYER_OFFSET_RESERVED] = 0;
    sys_put_le32(msg->timestamp, &buf[KB_LAYER_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("LAYER_CHANGE layer=%u state=%u seq=%u", msg->layer, msg->state,
            buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });
}

/*
 * 0x03 emitter + dedupe (consumer side). Suppress a press when that
 * (page, keycode) is already reported down, and a release when it was never
 * reported down. This rejects the synthetic duplicates from &key_repeat
 * (behavior_key_repeat.c:46/63, incl. its re-raise carrying mutated implicit
 * mods), hold-tap captured event replays (behavior_hold_tap.c:231), and
 * sticky-key re-raises (behavior_sticky_key.c:350). A release removes the
 * slot so a later physical press of the same key resumes emission
 * (double-tap works).
 */
static void keycode_event_process(const struct keycode_msg *msg) {
    const uint16_t keycode = msg->keycode;

    bool tracked = false;
    for (uint8_t i = 0; i < active_key_count; i++) {
        if (active_keys[i].usage_page == msg->usage_page &&
            active_keys[i].keycode == keycode) {
            tracked = true;
            if (msg->state) {
                return; /* key already reported down */
            }
            active_keys[i] = active_keys[--active_key_count];
            break;
        }
    }

    if (!tracked && !msg->state) {
        return; /* release for a key never reported down */
    }

    if (!tracked && msg->state && active_key_count < ARRAY_SIZE(active_keys)) {
        active_keys[active_key_count].usage_page = msg->usage_page;
        active_keys[active_key_count].keycode = keycode;
        active_key_count++;
    }

    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_KEYCODE;
    sys_put_le16(msg->usage_page, &buf[KB_KEYCODE_OFFSET_USAGE_PAGE]);
    sys_put_le16(keycode, &buf[KB_KEYCODE_OFFSET_KEYCODE]);
    buf[KB_KEYCODE_OFFSET_STATE] = msg->state;
    buf[KB_KEYCODE_OFFSET_IMPLICIT_MODS] = msg->implicit_mods;
    buf[KB_KEYCODE_OFFSET_EXPLICIT_MODS] = msg->explicit_mods;
    buf[KB_KEYCODE_OFFSET_SOURCE] = KB_SOURCE_LOCAL;
    sys_put_le32(msg->timestamp, &buf[KB_KEYCODE_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("KEYCODE page=%u key=%u state=%u implicit=0x%02X explicit=0x%02X seq=%u",
            msg->usage_page, keycode, msg->state, msg->implicit_mods,
            msg->explicit_mods, buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });

    mods_emit();
}

/*
 * 0x06 emitter (consumer side). wire_source is what goes on the wire (0xFF
 * for the central half, else the split slot index); index selects the dedupe
 * stash slot. Periodic battery events are deduped so a 0x06 fires only on an
 * actual SoC change.
 */
static void battery_event_emit(uint8_t wire_source, uint8_t index, uint8_t soc) {
    if (index < KB_BATT_INDEX_COUNT && last_sent_battery[index] == soc) {
        return; /* dedupe: unchanged since last emit */
    }

    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_BATTERY;
    buf[KB_BATTERY_OFFSET_SOURCE] = wire_source;
    buf[KB_BATTERY_OFFSET_STATE_OF_CHARGE] = soc;
    buf[KB_BATTERY_OFFSET_RESERVED] = 0;
    sys_put_le32((uint32_t)(k_uptime_get() & 0xFFFFFFFF), &buf[KB_BATTERY_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    if (index < KB_BATT_INDEX_COUNT) {
        last_sent_battery[index] = soc;
    }

    LOG_DBG("BATTERY source=%u soc=%u seq=%u", wire_source, soc, buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });
}

/*
 * 0x07 emitter (consumer side). Reads live state: zmk_usb_get_conn_state()
 * enum values 0/1/2 map directly onto KB_USB_CONN_*; the active BLE profile
 * index is always valid at this pin (ble.c:254 — a static uint8_t, so there
 * is no error path; revisit if the zmk pin moves past v0.3).
 */
static void connectivity_emit(void) {
    const uint8_t usb = (uint8_t)zmk_usb_get_conn_state();
    const int active = zmk_ble_active_profile_index();
    const uint8_t profile = (uint8_t)active;

    const struct kb_conn_wire wire = {
        .usb_conn = usb,
        .ble_profile = profile,
        .ble_connected = conn_track.host_conns > 0 ? 1 : 0,
        .split_status = conn_track.split_conns > 0 ? KB_SPLIT_STATUS_SOME
                                                   : KB_SPLIT_STATUS_DISCONNECTED,
    };

    if (memcmp(&wire, &last_conn, sizeof(wire)) == 0) {
        return; /* dedupe: state unchanged */
    }
    last_conn = wire;

    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_CONNECTIVITY;
    buf[KB_CONN_OFFSET_USB_CONN] = wire.usb_conn;
    buf[KB_CONN_OFFSET_BLE_PROFILE] = wire.ble_profile;
    buf[KB_CONN_OFFSET_BLE_CONNECTED] = wire.ble_connected;
    buf[KB_CONN_OFFSET_SPLIT_STATUS] = wire.split_status;
    buf[KB_CONN_OFFSET_RESERVED] = 0;
    sys_put_le32((uint32_t)(k_uptime_get() & 0xFFFFFFFF), &buf[KB_CONN_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("CONNECTIVITY usb=%u profile=%u ble=%u split=%u seq=%u",
            wire.usb_conn, wire.ble_profile, wire.ble_connected, wire.split_status,
            buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });
}

/*
 * 0x05 announcement (consumer side; best-effort, D4). The transport offers
 * no readiness signal, so the ~1 s boot emit can still be dropped until USB
 * enumerates / the active BLE profile connects / encryption completes.
 * Counters: re-emitted on (a) first host connection, (b) every non-HID ->
 * HID USB transition, (c) a host connection reaching BT_SECURITY_L2
 * (post-encryption; the first notify on an unencrypted link is otherwise
 * eaten by the pairing handshake). 0x07 replays connectivity and 0x06/0x04
 * are change-driven, so a missed HELLO is not fatal. battery_central comes
 * from the 0x06 stash (0xFF = not yet reported); slot0 uses the
 * config-gated read and reads 0xFF when the C6 config is off or the slot is
 * unknown (D3). No dedupe — it is an announcement.
 */
static void hello_emit(void) {
    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    const int active = zmk_ble_active_profile_index();
    uint8_t periph_battery = KB_SOURCE_LOCAL; /* 0xFF = unavailable/unknown */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    uint8_t level = 0;
    if (zmk_split_central_get_peripheral_battery_level(0, &level) == 0) {
        periph_battery = level;
    }
#endif

    buf[0] = KB_EVENT_HELLO;
    buf[KB_HELLO_OFFSET_PROTOCOL_VERSION] = KB_BROADCAST_PROTOCOL_VERSION;
    buf[KB_HELLO_OFFSET_BLE_PROFILE] = (uint8_t)active;
    buf[KB_HELLO_OFFSET_USB_CONNECTED] = (uint8_t)zmk_usb_get_conn_state();
    buf[KB_HELLO_OFFSET_SPLIT_CONNECTED] = conn_track.split_conns > 0 ? 1 : 0;
    buf[KB_HELLO_OFFSET_BATTERY_CENTRAL] = last_sent_battery[KB_BATT_INDEX_CENTRAL];
    buf[KB_HELLO_OFFSET_BATTERY_PERIPH] = periph_battery;
    buf[KB_HELLO_OFFSET_RESERVED] = 0;
    sys_put_le32((uint32_t)(k_uptime_get() & 0xFFFFFFFF), &buf[KB_HELLO_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("HELLO version=%u profile=%u usb=%u split=%u central=%u periph=%u seq=%u",
            buf[KB_HELLO_OFFSET_PROTOCOL_VERSION], buf[KB_HELLO_OFFSET_BLE_PROFILE],
            buf[KB_HELLO_OFFSET_USB_CONNECTED], buf[KB_HELLO_OFFSET_SPLIT_CONNECTED],
            buf[KB_HELLO_OFFSET_BATTERY_CENTRAL], buf[KB_HELLO_OFFSET_BATTERY_PERIPH],
            buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });
}

/* Forward declaration: conn_scan_cb is used by the consumer's conn-rescan
 * path below and defined in the connectivity section further down. */
static void conn_scan_cb(struct bt_conn *conn, void *data);

/* ---- Consumer: drains the msgq on the low-prio workqueue ---- */

static void kb_bcast_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    struct emit_msg msg;
    while (k_msgq_get(&kb_bcast_msgq, &msg, K_NO_WAIT) == 0) {
        switch (msg.kind) {
        case EMIT_POSITION:
            position_event_emit(&msg.position);
            break;
        case EMIT_LAYER:
            layer_event_emit(&msg.layer);
            /* Catch hold-tap holds that change mods without a keycode event. */
            mods_emit();
            break;
        case EMIT_KEYCODE:
            keycode_event_process(&msg.keycode);
            break;
        case EMIT_BATTERY:
            battery_event_emit(msg.battery.wire_source, msg.battery.index, msg.battery.soc);
            break;
        case EMIT_CONN_RESCAN:
            conn_track.host_conns = 0;
            conn_track.split_conns = 0;
            bt_conn_foreach(BT_CONN_TYPE_LE, conn_scan_cb, NULL);
            connectivity_emit();
            break;
        case EMIT_CONN_EVENT:
            if (msg.conn.connected) {
                if (msg.conn.is_host) {
                    const bool first_host = conn_track.host_conns == 0;
                    conn_track.host_conns++;
                    /* 0x07 first (new state), then the D4 snapshot. */
                    connectivity_emit();
                    if (first_host) {
                        hello_emit();
                    }
                } else {
                    conn_track.split_conns++;
                    connectivity_emit();
                }
            } else {
                if (msg.conn.is_host) {
                    if (conn_track.host_conns > 0) {
                        conn_track.host_conns--;
                    }
                } else {
                    if (conn_track.split_conns > 0) {
                        conn_track.split_conns--;
                    }
                }
                connectivity_emit();
            }
            break;
        case EMIT_CONNECTIVITY: {
            /* HELLO on every non-HID -> HID USB transition (enumeration,
             * re-plug, resume). HELLO is an announcement (no dedupe). The
             * consumer evaluates the transition so it sees USB events in
             * order. */
            const enum zmk_usb_conn_state usb_now = zmk_usb_get_conn_state();
            if (usb_now == ZMK_USB_CONN_HID && prev_usb_state != ZMK_USB_CONN_HID) {
                connectivity_emit();
                hello_emit();
            } else {
                connectivity_emit();
            }
            prev_usb_state = usb_now;
            break;
        }
        case EMIT_HELLO:
            hello_emit();
            break;
        }
    }
}

/*
 * Consumer work item, compile-time initialized (K_WORK_DEFINE) so that an
 * early producer (e.g. battery's APPLICATION-init timer racing ours at the
 * same init priority) can never submit a zeroed work item with a NULL
 * handler. The handler runs exclusively on the low-priority workqueue.
 */
K_WORK_DEFINE(kb_bcast_work, kb_bcast_work_handler);

/* ---- Producers: pack a message, enqueue, request the consumer work ---- */

static void kb_enqueue(const struct emit_msg *msg) {
    if (k_msgq_put(&kb_bcast_msgq, msg, K_NO_WAIT) != 0) {
        LOG_WRN("Broadcast queue full; dropping event kind %d", (int)msg->kind);
        return;
    }
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &kb_bcast_work);
}

static int position_state_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Producer only: no shared emitter state is touched on this thread. */
    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_POSITION,
        .position = {.position = ev->position,
                     .source = ev->source,
                     .state = ev->state ? 1 : 0,
                     .timestamp = (uint32_t)(ev->timestamp & 0xFFFFFFFF)},
    });

    return ZMK_EV_EVENT_BUBBLE;
}

static int layer_state_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_LAYER,
        .layer = {.layer = ev->layer,
                  .state = ev->state ? 1 : 0,
                  .timestamp = (uint32_t)(ev->timestamp & 0xFFFFFFFF)},
    });

    return ZMK_EV_EVENT_BUBBLE;
}

static int keycode_state_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_KEYCODE,
        .keycode = {.usage_page = ev->usage_page,
                    .keycode = (uint16_t)ev->keycode,
                    .implicit_mods = ev->implicit_modifiers,
                    .explicit_mods = ev->explicit_modifiers,
                    .state = ev->state ? 1 : 0,
                    .timestamp = (uint32_t)(ev->timestamp & 0xFFFFFFFF)},
    });

    return ZMK_EV_EVENT_BUBBLE;
}

static int battery_state_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_BATTERY,
        .battery = {.wire_source = KB_SOURCE_LOCAL,
                    .index = KB_BATT_INDEX_CENTRAL,
                    .soc = ev->state_of_charge},
    });

    return ZMK_EV_EVENT_BUBBLE;
}

static int peripheral_battery_state_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_BATTERY,
        .battery = {.wire_source = ev->source,
                    .index = KB_BATT_INDEX_SLOT0 + ev->source,
                    .soc = ev->state_of_charge},
    });

    return ZMK_EV_EVENT_BUBBLE;
}

static int usb_conn_state_listener(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* The consumer reads live state and detects the non-HID -> HID
     * transition (HELLO trigger) in order. */
    kb_enqueue(&(struct emit_msg){.kind = EMIT_CONNECTIVITY});

    return ZMK_EV_EVENT_BUBBLE;
}

static int ble_active_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    kb_enqueue(&(struct emit_msg){.kind = EMIT_CONNECTIVITY});

    return ZMK_EV_EVENT_BUBBLE;
}

/* ---- Connectivity: BT conn callbacks (BT RX thread) are producers too ---- */

static void conn_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        return;
    }
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }

    /* Classify by role only (BT RX thread); the consumer applies the delta. */
    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_CONN_EVENT,
        .conn = {.is_host = info.role == BT_CONN_ROLE_PERIPHERAL ? 1 : 0,
                 .connected = 1},
    });
}

static void conn_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);

    last_hello_conn = NULL; /* re-arm the post-encryption HELLO (BT RX thread) */

    /*
     * bt_conn_get_info() on a disconnected conn can still resolve the role
     * (ZMK's own ble.c:545 does exactly this), but a stale/unresolvable
     * conn must not corrupt counts — request a full re-scan instead, which
     * the consumer runs on the workqueue (safe context for
     * bt_conn_foreach).
     */
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0) {
        kb_enqueue(&(struct emit_msg){.kind = EMIT_CONN_RESCAN});
        return;
    }

    kb_enqueue(&(struct emit_msg){
        .kind = EMIT_CONN_EVENT,
        .conn = {.is_host = info.role == BT_CONN_ROLE_PERIPHERAL ? 1 : 0,
                 .connected = 0},
    });
}

static void conn_security_changed(struct bt_conn *conn, bt_security_t level,
                                  enum bt_security_err err) {
    if (err != 0 || level < BT_SECURITY_L2) {
        return;
    }
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }
    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    /* Re-emit HELLO once per encrypted host connection (D4): the
     * ACL-connect HELLO can be eaten by the -EPERM/handshake drop on an
     * unencrypted link (hog.c -EPERM -> set L2). Pointer-compare, no
     * per-conn allocation. */
    if (last_hello_conn != conn) {
        last_hello_conn = conn;
        kb_enqueue(&(struct emit_msg){.kind = EMIT_HELLO});
    }
}

BT_CONN_CB_DEFINE(kb_broadcaster_conn_cb) = {
    .connected = conn_connected,
    .disconnected = conn_disconnected,
    .security_changed = conn_security_changed,
};

/*
 * Init-time connectivity snapshot + boot HELLO. Runs ~1 s after boot (BT is
 * up by then), on the low-prio workqueue — never inside a BLE callback — so
 * bt_conn_foreach is safe here (it takes the conn-list lock; the BT callback
 * path may already hold it). Establishes the connectivity baseline covering
 * connections that existed before BT_CONN_CB registration, then fires the
 * boot HELLO (best-effort; re-emitted by the triggers above).
 */
static struct k_work_delayable conn_snapshot_work;

static void conn_scan_cb(struct bt_conn *conn, void *data) {
    ARG_UNUSED(data);

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }
    if (info.role == BT_CONN_ROLE_PERIPHERAL) {
        conn_track.host_conns++;
    } else if (info.role == BT_CONN_ROLE_CENTRAL) {
        conn_track.split_conns++;
    }
}

static void conn_snapshot_work_handler(struct k_work *item) {
    ARG_UNUSED(item);

    conn_track.host_conns = 0;
    conn_track.split_conns = 0;
    bt_conn_foreach(BT_CONN_TYPE_LE, conn_scan_cb, NULL);

    /* Sentinel-initialized last_conn forces the first real 0x07 emit. */
    connectivity_emit();
    hello_emit();
    prev_usb_state = zmk_usb_get_conn_state();
}

static int kb_broadcaster_init(const struct device *dev) {
    ARG_UNUSED(dev);

    k_work_init_delayable(&conn_snapshot_work, conn_snapshot_work_handler);
    k_work_schedule_for_queue(zmk_workqueue_lowprio_work_q(), &conn_snapshot_work,
                              K_MSEC(1000));
    return 0;
}

SYS_INIT(kb_broadcaster_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* ---- Registrations ---- */

ZMK_LISTENER(kb_broadcaster_position, position_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_position, zmk_position_state_changed);

ZMK_LISTENER(kb_broadcaster_layer, layer_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_layer, zmk_layer_state_changed);

ZMK_LISTENER(kb_broadcaster_keycode, keycode_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_keycode, zmk_keycode_state_changed);

ZMK_LISTENER(kb_broadcaster_battery, battery_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_battery, zmk_battery_state_changed);

ZMK_LISTENER(kb_broadcaster_peripheral_battery, peripheral_battery_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_peripheral_battery, zmk_peripheral_battery_state_changed);

ZMK_LISTENER(kb_broadcaster_usb_conn, usb_conn_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_usb_conn, zmk_usb_conn_state_changed);

ZMK_LISTENER(kb_broadcaster_ble_profile, ble_active_profile_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_ble_profile, zmk_ble_active_profile_changed);
