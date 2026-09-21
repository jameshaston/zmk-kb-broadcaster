# zmk-kb-broadcaster

Broadcast ZMK keyboard state over the raw-HID pipe provided by
[zmk-raw-hid](https://github.com/zzeneg/zmk-raw-hid) in a compact,
versioned protocol. Consumed by desktop/companion apps to render keys, layers,
modifiers, battery, and connectivity live.

## Requirements

- ZMK core pinned to the **v0.3 tag**. All event structs/APIs this module
  touches were line-verified at that pin (notably: `zmk_layer_state_changed`
  has no `locked` field there — see caveats).
- `zmk-raw-hid` module (transport) and the `raw_hid_adapter` shield (which
  sets `CONFIG_USB_HID_DEVICE_COUNT=2`).
- Module enable: `CONFIG_ZMK_KB_BROADCASTER=y` (this module; implies RAW_HID).
- **Peripheral battery** (right half): add
  `CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING=y` to the central
  half's `.conf` (central-only menuconfig; the right half silently drops it).
  The module builds with or without it.

## Packet framing

- One **32-byte raw-HID report**. The length passed to the transport is
  always `KB_PACKET_SIZE` (32); unused bytes are zeroed.
- **Byte 0** = event type. **Byte 31** = shared sequence counter (uint8,
  wraps at 255).
- Payloads are little-endian. Timestamps are `uint32`-truncated uptime in
  milliseconds (`k_uptime_get()`).
- **Source byte** semantics: `0xFF` = local (this half). Split-originated
  events carry the peripheral slot index (0-based).

### Sequence semantics

The transport has **no queue, no buffering, and no ACK**. A single shared
counter increments on every emitted packet; the app must detect dropped
packets by observing a **discontinuity** in byte 31 (per source+type), not a
monotonic relation (the counter wraps). The app should persist the last
observed seq across reconnects.

### Threading model (why every packet arrives serialized)

All emissions run on one consumer work item on ZMK's low-priority workqueue:
event listeners and BT-connection callbacks only enqueue messages, and a
single worker performs dedupe, seq updates, buffer fills, and the transport
raise. This keeps the shared seq/statics race-free across the system
workqueue (events), low-prio queue (battery), and BT RX thread (conn
callbacks), and keeps the transport's inline (USB-leg) blocking off the BT
RX and system-workqueue threads. Queue depth is bounded (32); on overflow the
newest packet is dropped and logged (the app's seq-gap detection flags it).

## Protocol v2 — event reference

`KB_BROADCAST_PROTOCOL_VERSION` = **2**. Unknown types must be ignored
(additive: `v2` firmware works with a `v1` app).

### 0x01 KEY_POSITION — bytes 1-8 (len 8)

| Offset | Field | Type |
|--------|-------|------|
| 1-2    | position | uint16 LE |
| 3      | state (1=pressed, 0=released) | u8 |
| 4      | source (0xFF local, else slot) | u8 |
| 5-8    | timestamp | uint32 LE |

No dedupe: the raw half-stream, incl. peripheral positions re-raised by the
central with `source` = slot.

### 0x02 LAYER_CHANGE — bytes 1-8 (len 8)

| Offset | Field | Type |
|--------|-------|------|
| 1      | layer index | u8 |
| 2      | state (1=active, 0=inactive) | u8 |
| 3      | locked — **always 0 at the v0.3 pin** | u8 |
| 4      | reserved (0) | u8 |
| 5-8    | timestamp | uint32 LE |

`zmk_layer_state_changed` has no `locked` field before
`5138c6fb` (#2717); wiring `ev->locked` requires bumping the zmk pin past
that commit.

### 0x03 KEYCODE — bytes 1-12 (len 12)

| Offset | Field | Type |
|--------|-------|------|
| 1-2    | usage page | uint16 LE |
| 3-4    | keycode (usage id) | uint16 LE — truncated from u32, ids fit |
| 5      | state (1=down, 0=up) | u8 |
| 6      | implicit mods (HID bits) | u8 |
| 7      | explicit mods (HID bits, separate — not OR'd) | u8 |
| 8      | source — fixed 0xFF (keycodes are central-resolved) | u8 |
| 9-12   | timestamp | uint32 LE |

Duplicate synthetic events (`&key_repeat` replays, hold-tap captured-event
replays, sticky-key re-raises) are suppressed by the firmware, so the app can
expect one down/up pair per actual key. A 7th concurrent keycode is emitted
untracked (degenerate case).

### 0x04 MODIFIERS — bytes 1-8 (len 8)

| Offset | Field | Type |
|--------|-------|------|
| 1      | host modifiers byte — `zmk_hid_get_keyboard_report()->body.modifiers`, i.e. what the host sees (`(explicit & ~masked) \| implicit`) | u8 |
| 2      | explicit mods (`zmk_hid_get_explicit_mods()`) | u8 |
| 3      | delta (1 = byte grew, 0 = released) | u8 |
| 4      | source — fixed 0xFF (mods are central-global) | u8 |
| 5-8    | timestamp | uint32 LE |

Diff-based: emitted only when the host byte changes (checked after every
keycode and layer event). The host byte is authoritative; `delta` is a hint
(a mod swap like LC→RC reports 1).

### 0x05 HELLO — bytes 1-11 (len 11)

| Offset | Field | Type |
|--------|-------|------|
| 1      | protocol version (2) | u8 |
| 2      | active BLE profile index (0xFF none) | u8 |
| 3      | usb_connected (see 0x07) | u8 |
| 4      | split_connected (0/1) | u8 |
| 5      | central battery SoC (0xFF = not yet reported) | u8 |
| 6      | peripheral slot0 battery (0xFF = unavailable / C6 off) | u8 |
| 7      | reserved (0) | u8 |
| 8-11   | timestamp | uint32 LE |

**Best-effort announcement, not deduped.** Emitted on: (a) ~1 s after boot,
(b) the first host BLE connection, (c) every non-HID → HID USB transition
(enumeration / re-plug / resume), (d) a host connection reaching
`BT_SECURITY_L2` (post-encryption — the first notify on an unencrypted link
is otherwise eaten by the pairing handshake). The app must not hard-depend on
it; connectivity (0x07), battery (0x06) and mods (0x04) are change-driven and
recover state regardless.

### 0x06 BATTERY — bytes 1-7 (len 7)

| Offset | Field | Type |
|--------|-------|------|
| 1      | source (0xFF central, else slot index) | u8 |
| 2      | state of charge (0-100) | u8 |
| 3      | reserved (0) | u8 |
| 4-7    | timestamp | uint32 LE |

Deduped: one emit per actual SoC change per source. Peripheral emissions
(and the HELLO slot-0 byte) require the C6 config above; the module builds
either way.

### 0x07 CONNECTIVITY — bytes 1-9 (len 9)

| Offset | Field | Type |
|--------|-------|------|
| 1      | usb_conn: 0=NONE, 1=POWERED, 2=HID | u8 |
| 2      | active BLE profile index (0xFF none) | u8 |
| 3      | ble_connected (≥1 host connected) | u8 |
| 4      | split_status: 0=disconnected, 1=some, 2=all | u8 |
| 5      | reserved (0) | u8 |
| 6-9    | timestamp | uint32 LE |

Deduped snapshot: emitted when the 4-byte state changes (USB plug/unplug,
host connect/disconnect, split connect/disconnect, profile switch).
Connection counts are classified by role (`BT_CONN_ROLE_PERIPHERAL` = host,
`BT_CONN_ROLE_CENTRAL` = split) and an init-time scan covers connections
established before firmware callbacks registered. `split_status` of 2
("all") is reserved for multi-peripheral keyboards (corne: single peripheral
→ 0 or 1).

### Reserved

`0x08` (WPM) is reserved with no stream in v2 (`KB_EVENT_WPM`). Do not emit
without defining a wire format first.

## Caveats

- **zmk v0.3 pin / `locked`:** fixed at 0; only bump the pin (≥ `5138c6fb`)
  if layer-lock reporting is needed.
- **BLE profile index byte:** at this pin `zmk_ble_active_profile_index()`
  always returns a valid profile (it is a `static uint8_t` internally), so
  the "0xFF none" mapping is currently unreachable; it exists to survive a
  future pin bump.
- **Central battery reporting:** `ZMK_BATTERY_REPORTING` has no Kconfig
  default; it is implied on by `ZMK_BLE` (non-posix) and additionally
  requires a battery sensor node in the board DTS (nice_nano_v2 has one).
  Without it, the central emits no 0x06 and HELLO reports 0xFF for the
  central battery.
- **C6 dependency:** without
  `CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING=y`, no peripheral
  0x06 is emitted and HELLO slot-0 stays `0xFF`. The module builds either
  way.
- **HELLO best-effort:** see 0x05 above.
- **ATT MTU:** the BLE/HOG leg requires a host ATT MTU ≥ 35 to deliver the
  32-byte report; conventional clients negotiate 185+ (host-dependent).
- **Loss detection:** seq-gap detection is the app's responsibility; the
  transport never queues or re-sends.
- **Thread model:** all emissions are serialized on one worker (see
  "Thread model" above); producers never touch shared emitter state.

## Transport revision note

The transport review (synchronous raise, USB static-buffer memcpy, 30 ms
USB semaphore timeout, BLE ATT PDU memcpy) was performed against
`zmk-raw-hid` commit `6a37765dfab6197292e7a9f47305dcf87386d56a` (main).
Ship-time pinning to a reviewed tag is part of the release checklist.

## Version

`KB_BROADCAST_PROTOCOL_VERSION` = 2 (additive over v1; `0x03`-`0x07` are
new, byte 31 shared seq is new).