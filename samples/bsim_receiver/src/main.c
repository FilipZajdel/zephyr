/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "hw_counter.h"
#include "bs_radio.h"

#define TEST_TX_BUF_SIZE (50)

static uint8_t tx_buf[TEST_TX_BUF_SIZE];
uint8_t rx_ctr;


void radio_event_cb(struct bs_radio_event_data *event_data)
{
	switch (event_data->type) {
	case BS_RADIO_EVENT_TX_DONE:
		printk("APP >>> App data has been sent\n");
		break;
	case BS_RADIO_EVENT_RX_DONE:
		printk("APP >>> Received %d bytes.\n",
		       event_data->rx_done.len);
                for (int i=0; i<event_data->rx_done.len; i++) {
                        printk("%d\t", event_data->rx_done.data[i]);
                }
		rx_ctr++;
		break;
	default:
		printk("%u Radio event is not supported\n", event_data->type);
	}
}

static void fill_buffer(uint8_t *buf, uint16_t len);
static void sleep_ms_rand(uint32_t min, uint32_t max);

void main(void)
{
	srand(time(NULL));
	fill_buffer(tx_buf, TEST_TX_BUF_SIZE);
	bs_radio_start(radio_event_cb);

	while(rx_ctr < 5) k_msleep(1500);

	bs_radio_stop();
	bs_radio_deinit();
}

void sleep_ms_rand(uint32_t min, uint32_t max)
{
	uint64_t timeout_ms = min + (rand() % (max - min));
	k_msleep(timeout_ms);
}

void fill_buffer(uint8_t *buf, uint16_t len)
{
	for (; len > 0; len--) {
		buf[len - 1] = (rand() % 112 + 89) * 1.5;
	}
}
