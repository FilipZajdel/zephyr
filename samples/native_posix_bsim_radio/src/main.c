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

static uint8_t tx_buf [TEST_TX_BUF_SIZE];


void radio_event_cb(enum bs_radio_event_types event_type, void *event_data)
{
	union bs_radio_event_data *data =
		(union bs_radio_event_data *)event_data;

	switch (event_type) {
	case BS_RADIO_EVENT_TX_DONE:
		printk("My data has been sent YAYY\n");
		break;
	case BS_RADIO_EVENT_RX_DONE:
		printk("APP >>> I Have received %d bytes: \n", data->rx_done.len);
		break;
	default:
		printk("%u Event is not supported\n", event_type);
	}
}

void fill_buffer(uint8_t *buf, uint16_t len);

void main(void)
{
        fill_buffer(tx_buf, TEST_TX_BUF_SIZE);
        bs_radio_start(radio_event_cb);
        bs_radio_tx(tx_buf, TEST_TX_BUF_SIZE, false);

        printf("Tx\n");

        while (1) {
                bs_radio_tx(tx_buf, TEST_TX_BUF_SIZE, false);
                k_msleep(200);
        }

        k_sleep(K_FOREVER);
}

void fill_buffer(uint8_t *buf, uint16_t len)
{
        srand(time(NULL));

        for (; len > 0; len--) {
                buf[len-1] = (rand() % 112 + 89) * 1.5;
        }
}
