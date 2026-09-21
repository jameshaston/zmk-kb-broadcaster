/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#define KB_BROADCAST_PROTOCOL_VERSION 2

#define KB_EVENT_KEY_POSITION   0x01
#define KB_EVENT_LAYER_CHANGE   0x02
#define KB_EVENT_KEYCODE        0x03
#define KB_EVENT_MODIFIERS      0x04
#define KB_EVENT_HELLO          0x05
#define KB_EVENT_BATTERY        0x06
#define KB_EVENT_CONNECTIVITY   0x07
/*
 * 0x08 is reserved for a future WPM stream (comment-only reservation).
 * No firmware emits it in protocol v2; do not add without a wire format.
 */
#define KB_EVENT_WPM            0x08

#define KB_PACKET_SIZE          32
#define KB_PAYLOAD_OFFSET       1
#define KB_SEQ_OFFSET           31

/*
 * Payload-len convention: number of payload bytes (byte 0 = type, byte 31 = seq).
 * Offsets are byte indices within the 32-byte packet.
 */

/*
 * KEY_POSITION payload layout (bytes 1-8):
 *   bytes 1-2: position (uint16 LE)
 *   byte 3: state (1=pressed, 0=released)
 *   byte 4: source (0xFF=local, else peripheral slot)
 *   bytes 5-8: timestamp (uint32 LE)
 */
#define KB_POS_OFFSET_POSITION  1
#define KB_POS_OFFSET_STATE     3
#define KB_POS_OFFSET_SOURCE    4
#define KB_POS_OFFSET_TIMESTAMP 5
#define KB_POS_PAYLOAD_LEN      8

/*
 * LAYER_CHANGE payload layout (bytes 1-8):
 *   byte 1: layer index
 *   byte 2: state (1=activated, 0=deactivated)
 *   byte 3: locked — HARDCODED 0. The locked field only exists in
 *           zmk_layer_state_changed after zmk pin >= 5138c6fb (#2717);
 *           at the v0.3 tag (corne build pin) wiring ev->locked does not compile.
 *   byte 4: reserved (0)
 *   bytes 5-8: timestamp (uint32 LE)
 */
#define KB_LAYER_OFFSET_LAYER      1
#define KB_LAYER_OFFSET_STATE      2
#define KB_LAYER_OFFSET_LOCKED     3
#define KB_LAYER_OFFSET_RESERVED   4
#define KB_LAYER_OFFSET_TIMESTAMP  5
#define KB_LAYER_PAYLOAD_LEN       8

/*
 * KEYCODE payload layout (bytes 1-12):
 *   bytes 1-2: usage page (uint16 LE)
 *   bytes 3-4: keycode usage id (uint16 LE; event field is uint32 but ids fit u16)
 *   byte 5: state (1=down, 0=up)
 *   byte 6: implicit mods (HID bits; caps_word may mutate in-flight — listener-order dependent)
 *   byte 7: explicit mods (HID bits — separate from implicit, never OR'd)
 *   byte 8: source (fixed 0xFF/local; keycode events carry no per-half source — reserved)
 *   bytes 9-12: timestamp (uint32 LE)
 */
#define KB_KEYCODE_OFFSET_USAGE_PAGE       1
#define KB_KEYCODE_OFFSET_KEYCODE          3
#define KB_KEYCODE_OFFSET_STATE            5
#define KB_KEYCODE_OFFSET_IMPLICIT_MODS    6
#define KB_KEYCODE_OFFSET_EXPLICIT_MODS    7
#define KB_KEYCODE_OFFSET_SOURCE           8
#define KB_KEYCODE_OFFSET_TIMESTAMP        9
#define KB_KEYCODE_PAYLOAD_LEN             12

/*
 * MODIFIERS payload layout (bytes 1-8):
 *   byte 1: host modifiers byte — zmk_hid_get_keyboard_report()->body.modifiers
 *           (what the host sees; folds in explicit & ~masked | implicit)
 *   byte 2: explicit mods (zmk_hid_get_explicit_mods())
 *   byte 3: delta (1=byte grew / mod pressed, 0=released)
 *   byte 4: source (fixed 0xFF/local — mods are central-global)
 *   bytes 5-8: timestamp (uint32 LE)
 */
#define KB_MODS_OFFSET_HOST_MODS     1
#define KB_MODS_OFFSET_EXPLICIT_MODS 2
#define KB_MODS_OFFSET_DELTA         3
#define KB_MODS_OFFSET_SOURCE        4
#define KB_MODS_OFFSET_TIMESTAMP     5
#define KB_MODS_PAYLOAD_LEN          8

/*
 * HELLO payload layout (bytes 1-11):
 *   byte 1: protocol version (KB_BROADCAST_PROTOCOL_VERSION)
 *   byte 2: active BLE profile index
 *   byte 3: usb connection (KB_USB_CONN_*)
 *   byte 4: split connected (0/1)
 *   byte 5: central battery state of charge
 *   byte 6: peripheral slot0 battery (0xFF if unavailable / config-gated)
 *   byte 7: reserved (0)
 *   bytes 8-11: timestamp (uint32 LE)
 */
#define KB_HELLO_OFFSET_PROTOCOL_VERSION 1
#define KB_HELLO_OFFSET_BLE_PROFILE      2
#define KB_HELLO_OFFSET_USB_CONNECTED    3
#define KB_HELLO_OFFSET_SPLIT_CONNECTED  4
#define KB_HELLO_OFFSET_BATTERY_CENTRAL  5
#define KB_HELLO_OFFSET_BATTERY_PERIPH   6
#define KB_HELLO_OFFSET_RESERVED         7
#define KB_HELLO_OFFSET_TIMESTAMP        8
#define KB_HELLO_PAYLOAD_LEN             11

/*
 * BATTERY payload layout (bytes 1-7):
 *   byte 1: source (0xFF=central, else peripheral slot index)
 *   byte 2: state of charge (0-100)
 *   byte 3: reserved (0)
 *   bytes 4-7: timestamp (uint32 LE)
 */
#define KB_BATTERY_OFFSET_SOURCE          1
#define KB_BATTERY_OFFSET_STATE_OF_CHARGE 2
#define KB_BATTERY_OFFSET_RESERVED        3
#define KB_BATTERY_OFFSET_TIMESTAMP       4
#define KB_BATTERY_PAYLOAD_LEN            7

/*
 * CONNECTIVITY payload layout (bytes 1-9):
 *   byte 1: usb connection (KB_USB_CONN_*)
 *   byte 2: active BLE profile index
 *   byte 3: ble connected (0/1)
 *   byte 4: split status (KB_SPLIT_STATUS_*)
 *   byte 5: reserved (0)
 *   bytes 6-9: timestamp (uint32 LE)
 */
#define KB_CONN_OFFSET_USB_CONN      1
#define KB_CONN_OFFSET_BLE_PROFILE   2
#define KB_CONN_OFFSET_BLE_CONNECTED 3
#define KB_CONN_OFFSET_SPLIT_STATUS  4
#define KB_CONN_OFFSET_RESERVED      5
#define KB_CONN_OFFSET_TIMESTAMP     6
#define KB_CONN_PAYLOAD_LEN          9

#define KB_USB_CONN_NONE     0
#define KB_USB_CONN_POWERED  1
#define KB_USB_CONN_HID      2

#define KB_SPLIT_STATUS_DISCONNECTED 0
#define KB_SPLIT_STATUS_SOME         1
#define KB_SPLIT_STATUS_ALL          2

/*
 * HID keyboard modifier bits (bit order matches the USB HID report;
 * self-contained aliases so the app header does not depend on
 * <dt-bindings/zmk/modifiers.h>).
 */
#define KB_MOD_LCTL 0x01
#define KB_MOD_LSFT 0x02
#define KB_MOD_LALT 0x04
#define KB_MOD_LGUI 0x08
#define KB_MOD_RCTL 0x10
#define KB_MOD_RSFT 0x20
#define KB_MOD_RALT 0x40
#define KB_MOD_RGUI 0x80

#define KB_SOURCE_LOCAL 0xFF