/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#define KB_BROADCAST_PROTOCOL_VERSION 1

#define KB_EVENT_KEY_POSITION   0x01
#define KB_EVENT_LAYER_CHANGE   0x02
#define KB_EVENT_KEYCODE        0x03
#define KB_EVENT_MODIFIERS      0x04
#define KB_EVENT_HELLO          0x05
#define KB_EVENT_BATTERY        0x06

#define KB_PACKET_SIZE          32
#define KB_PAYLOAD_OFFSET       1
#define KB_SEQ_OFFSET           31

#define KB_POS_OFFSET_POSITION  1
#define KB_POS_OFFSET_STATE     3
#define KB_POS_OFFSET_SOURCE    4
#define KB_POS_OFFSET_TIMESTAMP 5
#define KB_POS_PAYLOAD_LEN      8

/*
 * LAYER_CHANGE payload layout (bytes 1-8):
 *   byte 1: layer index
 *   byte 2: state (1=activated, 0=deactivated)
 *   byte 3: locked (1=toggled, 0=momentary) — always 0 at ZMK v0.3
 *            (the locked field was added to zmk_layer_state_changed after v0.3)
 *   byte 4: reserved (0)
 *   bytes 5-8: timestamp (uint32 LE)
 */
#define KB_LAYER_OFFSET_LAYER      1
#define KB_LAYER_OFFSET_STATE      2
#define KB_LAYER_OFFSET_LOCKED     3
#define KB_LAYER_OFFSET_RESERVED   4
#define KB_LAYER_OFFSET_TIMESTAMP  5
#define KB_LAYER_PAYLOAD_LEN       8

#define KB_SOURCE_LOCAL 0xFF