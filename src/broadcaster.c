/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <raw_hid/events.h>

#include <kb_broadcaster/protocol.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Shared sequence counter for all broadcast packets. Wraps at 255 (uint8_t);
 * the host detects gaps by checking for discontinuity, not monotonic increase,
 * so wrap-around is safe.
 */
static uint8_t seq_counter = 0;

static int position_state_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /*
     * Static buffer: the transport (zmk-raw-hid usb_hid.c/hog.c) memcpy's
     * the payload synchronously during raise_raw_hid_sent_event dispatch,
     * so the buffer only needs to live until that call returns. Static is
     * the conservative choice matching zmk-raw-hid's own send_report.
     * Assumes no re-entrant position events (ZMK does not nest the same
     * event type).
     */
    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_KEY_POSITION;
    sys_put_le16((uint16_t)ev->position, &buf[KB_POS_OFFSET_POSITION]);
    buf[KB_POS_OFFSET_STATE] = ev->state ? 1 : 0;
    buf[KB_POS_OFFSET_SOURCE] = ev->source;
    sys_put_le32((uint32_t)(ev->timestamp & 0xFFFFFFFF), &buf[KB_POS_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("KEY_POSITION pos=%u state=%u source=%u seq=%u",
            (uint16_t)ev->position, ev->state ? 1 : 0, ev->source, buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });

    return ZMK_EV_EVENT_BUBBLE;
}

static int layer_state_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    static uint8_t buf[KB_PACKET_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0] = KB_EVENT_LAYER_CHANGE;
    buf[KB_LAYER_OFFSET_LAYER] = ev->layer;
    buf[KB_LAYER_OFFSET_STATE] = ev->state ? 1 : 0;
    buf[KB_LAYER_OFFSET_LOCKED] = ev->locked ? 1 : 0;
    buf[KB_LAYER_OFFSET_RESERVED] = 0;
    sys_put_le32((uint32_t)(ev->timestamp & 0xFFFFFFFF), &buf[KB_LAYER_OFFSET_TIMESTAMP]);
    buf[KB_SEQ_OFFSET] = seq_counter++;

    LOG_DBG("LAYER_CHANGE layer=%u state=%u locked=%u seq=%u",
            ev->layer, ev->state ? 1 : 0, ev->locked ? 1 : 0, buf[KB_SEQ_OFFSET]);

    raise_raw_hid_sent_event((struct raw_hid_sent_event){
        .data = buf,
        .length = KB_PACKET_SIZE,
    });

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(kb_broadcaster_position, position_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_position, zmk_position_state_changed);

ZMK_LISTENER(kb_broadcaster_layer, layer_state_listener);
ZMK_SUBSCRIPTION(kb_broadcaster_layer, zmk_layer_state_changed);