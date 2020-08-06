/*
 * Copyright (c) 2016-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(wpancmd);

#include <stdio.h>
#include <zephyr.h>
#include <net/buf.h>
#include <stdlib.h>
#include <net/ieee802154_radio.h>
#include <ieee802154/ieee802154_frame.h>
#include <net_private.h>

#include "wpancmd-utils.h"
#include "wpancmd.h"

#define CMD_TIMEOUT_S (2)

static struct ieee802154_radio_api *radio_api;
static struct device *ieee802154_dev;

/* IEEE802.15.4 frame + 1 byte len + 1 byte LQI */
uint8_t tx_buf[IEEE802154_MTU + 1 + 1];

/**
 * Stack for the tx thread.
 */
static K_THREAD_STACK_DEFINE(radio_loop_stack, 1024);
static struct k_thread radio_loop;

static int set_channel(void *data, int len)
{
	int channel = atoi(data);

	printk("Setting the channel to %d\n", channel);
	return radio_api->set_channel(ieee802154_dev, channel);
}

static int set_ieee_addr(void *data, int len)
{
	struct set_ieee_addr *req = data;

	printk("len %u", len);

	if (IEEE802154_HW_FILTER &
	    radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.ieee_addr = (uint8_t *)&req->ieee_addr;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_IEEE_ADDR,
					 &filter);
	}

	return 0;
}

static int set_short_addr(void *data, int len)
{
	struct set_short_addr *req = data;

	printk("len %u", len);

	if (IEEE802154_HW_FILTER &
	    radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.short_addr = req->short_addr;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_SHORT_ADDR,
					 &filter);
	}

	return 0;
}

static int set_pan_id(void *data, int len)
{
	struct set_pan_id *req = data;

	printk("len %u", len);

	if (IEEE802154_HW_FILTER &
	    radio_api->get_capabilities(ieee802154_dev)) {
		struct ieee802154_filter filter;

		filter.pan_id = req->pan_id;

		return radio_api->filter(ieee802154_dev, true,
					 IEEE802154_FILTER_TYPE_PAN_ID,
					 &filter);
	}

	return 0;
}

static int start(void)
{
	printk("Start IEEE 802.15.4 device");

	return radio_api->start(ieee802154_dev);
}

static int stop(void)
{
	printk("Stop IEEE 802.15.4 device");

	return radio_api->stop(ieee802154_dev);
}

static int tx(struct net_pkt *pkt)
{
	struct net_buf *buf = net_buf_frag_last(pkt->buffer);
	uint8_t seq = net_buf_pull_u8(buf);
	int retries = 3;
	int ret;

	do {
		ret = radio_api->tx(ieee802154_dev, IEEE802154_TX_MODE_DIRECT,
				    pkt, buf);
	} while (ret && retries--);

	if (ret) {
		printk("Error sending data, seq %u", seq);
		/* Send seq = 0 for unsuccessful send */
		seq = 0U;
	}

	return ret;
}

static int cca()
{
	return radio_api->cca(ieee802154_dev);
}

static int net_pkt_fill(struct net_pkt **pkt, uint8_t *bytes, uint8_t len)
{
	/* Maximum 2 bytes are added to the len */
	*pkt = net_pkt_alloc_with_buffer(NULL, len + 2, AF_UNSPEC, 0,
					 K_NO_WAIT);
	if (!(*pkt)) {
		return -ENOMEM;
	}

	net_pkt_write(*pkt, bytes, len);
	printk("pkt %p len %u\n", *pkt, len);
	return 0;
}

static void tx_thread(void)
{
	printk("Radio loop started\nInsert command >> \n");

	while (1) {
		char *cmd_buf = NULL;
		ssize_t buf_size = 0;
		uint8_t args_idx;
		uint8_t args_len;
		uint8_t cmd;

		k_msleep(250);

                /* Wait for user input 
                 * Note that time spent waiting in this function is detached
                 * from simulation time - simulation is stuck at this moment.
                 */
		if (getline_timeout(&cmd_buf, &buf_size, CMD_TIMEOUT_S) <= 0) {
			continue;
		}

		cmd = cmd_decode(cmd_buf, &args_idx, &args_len);

		switch (cmd) {
		case HELP:
			printk("Available commands:\n");
			print_commands();
			break;
		case RESET:
			printk("Reset device");
			break;
		case TX: {
			struct net_pkt *pkt;
			struct net_buf *buf;
			uint8_t data[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 };

			net_pkt_fill(&pkt, data, ARRAY_SIZE(data));
			buf = net_buf_frag_last(pkt->buffer);
			net_pkt_hexdump(pkt, ">");

			if (!cca()) {
				tx(pkt);
			} else {
				LOG_WRN("CCA Failed - Could not sent packet");
			}
			net_pkt_unref(pkt);
			break;
		}
		case START:
			start();
			break;
		case STOP:
			stop();
			break;
		case SET_CHANNEL:
			set_channel(cmd_buf + args_idx, args_len);
			break;
		case SET_IEEE_ADDR:
			set_ieee_addr(cmd_buf + args_idx, args_len);
			break;
		case SET_SHORT_ADDR:
			set_short_addr(cmd_buf + args_idx, args_len);
			break;
		case SET_PAN_ID:
			set_pan_id(cmd_buf + args_idx, args_len);
			break;
		case SHUT_DOWN:
			stop();
			posix_exit(0);
		default:
			printk("%s %x: Not handled", cmd_buf, cmd);
			break;
		}

		k_yield();
		free(cmd_buf);
		printk("Insert command:\n");
	}
}

int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	size_t len = net_pkt_get_len(pkt);
	uint8_t *p = tx_buf;
	int ret;

	LOG_INF("Got data, pkt %p, len %d", pkt, len);

	net_pkt_hexdump(pkt, "<");

	if (len > (sizeof(tx_buf) - 2)) {
		LOG_ERR("Too large packet");
		ret = -ENOMEM;
		goto out;
	}

	/**
	 * Add length 1 byte
	 */
	*p++ = (uint8_t)len;

	/* This is needed to work with pkt */
	net_pkt_cursor_init(pkt);

	ret = net_pkt_read(pkt, p, len);
	if (ret < 0) {
		LOG_ERR("Cannot read pkt");
		goto out;
	}

	p += len;

	/**
	 * Add LQI at the end of the packet
	 */
	*p = net_pkt_ieee802154_lqi(pkt);

out:
	net_pkt_unref(pkt);

	return ret;
}

void main(void)
{
	net_pkt_init();
	ieee802154_dev =
		device_get_binding(CONFIG_IEEE802154_NATIVE_POSIX_DRV_NAME);

	if (ieee802154_dev) {
		radio_api = (struct ieee802154_radio_api *)
				    ieee802154_dev->driver_api;

		k_thread_create(&radio_loop, radio_loop_stack,
				K_THREAD_STACK_SIZEOF(radio_loop_stack),
				(k_thread_entry_t)tx_thread, NULL, NULL, NULL,
				K_PRIO_COOP(8), 0, K_NO_WAIT);
	} else {
		printk("Couldn't bind %s\n",
		       CONFIG_IEEE802154_NATIVE_POSIX_DRV_NAME);
	}
}
