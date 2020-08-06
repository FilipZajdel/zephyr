/*
 * Copyright (c) 2016-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

struct set_channel {
	uint8_t page;
	uint8_t channel;
} __packed;

struct set_short_addr {
	uint16_t short_addr;
} __packed;

struct set_pan_id {
	uint16_t pan_id;
} __packed;

struct set_ieee_addr {
	uint64_t ieee_addr;
} __packed;
