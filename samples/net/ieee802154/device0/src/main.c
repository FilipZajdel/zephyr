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
#include "ieee802154_types.h"

static struct ieee802154_radio_api *radio_api;
static struct device *ieee802154_dev;

static int tx(struct net_pkt *pkt)
{
	struct net_buf *buf = net_buf_frag_last(pkt->buffer);
	int retries = 3;
	int ret;

	do {
		ret = radio_api->tx(ieee802154_dev, IEEE802154_TX_MODE_DIRECT,
				    pkt, buf);
	} while (ret && retries--);

	if (ret) {
		printk("Error sending data\n");
	}

	return ret;
}

/** Fill the packet with the data
 * 
 * Arguments:
 * pkt		-	The pointer that pkt will be allocated on
 * psdu		-	The data that pkt will be filled with. psdu[0] must 
 * 				be the data length
 * 
 * Returns:
 * 0		-	Success
 * Negative - 	Failure
 */
static int net_pkt_fill(struct net_pkt **pkt, uint8_t *psdu)
{
	uint8_t packet_len = psdu[0];
	
	*pkt = net_pkt_alloc_with_buffer(NULL, packet_len, AF_UNSPEC, 0,
					 K_FOREVER);
	if (!(*pkt)) {
		return -ENOMEM;
	}

	net_pkt_cursor_init(*pkt);
	net_pkt_write(*pkt, psdu+1, packet_len);
	printk("pkt %p len %u\n", *pkt, packet_len);
	return 0;
}

int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	size_t len = net_pkt_get_len(pkt);
	uint8_t rx_buf[127];
	uint8_t *p = rx_buf;
	int ret;

	LOG_INF("Got data, pkt %p, len %d", pkt, len);

	net_pkt_hexdump(pkt, "<");

	if (len > (127 - 2)) {
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

static void example_init(void)
{
	enum ieee802154_config_type config_type =
		IEEE802154_FPB_ADDR_MATCH_ZIGBEE;
	const struct ieee802154_config config = {
		/** TODO: FILL The configuration */
	};

	uint8_t channel = 20;
	int8_t power = -12;

	net_pkt_init();
	ieee802154_dev =
		device_get_binding(CONFIG_IEEE802154_NATIVE_POSIX_DRV_NAME);

	if (ieee802154_dev) {
		radio_api = (struct ieee802154_radio_api *)
				    ieee802154_dev->driver_api;
	} else {
		printk("Couldn't bind %s\n",
		       CONFIG_IEEE802154_NATIVE_POSIX_DRV_NAME);
		posix_exit(-EFAULT);
	}

	printf("%10s\n", "Radio configuration:");
	printf("%-10s %u\n", "Channel:", channel);
	printf("%-10s %d dBm\n", "Power:", power);
	printf("Configuration can be changed either via "
	       "menuconfig or prj.conf\n\n");

	radio_api->set_channel(ieee802154_dev, channel);
	radio_api->set_txpower(ieee802154_dev, power);
	radio_api->configure(ieee802154_dev, config_type, &config);
	int status = radio_api->start(ieee802154_dev);

	if (status) {
		printf("Radio couldn't be started - "
		       "make sure that program was started in babble sim mode (-bsim)");
		posix_exit(-EFAULT);
	}
}

static void generate_rand_arr(uint8_t *arr, uint16_t size)
{
	srand(time(NULL));
	for (int i = 0; i < size; i++) {
		arr[i] = rand();
	}
}

static void generate_data_frames(frame_any_t *frames[], uint16_t nframes)
{
	srand(time(NULL));

	while (nframes-- > 0) {
		uint8_t payload_len = rand() % 64;
		data_frame_t *data_frame =
			frame_create(FRAME_TYPE_DATA, true, payload_len, 0);
		generate_rand_arr(data_frame->payload, payload_len);
		FRAME_FCF_SET(*(uint16_t *)data_frame->frame_control,
			      FRAME_FCF_AR, 1);

		frames[nframes] = data_frame;
	}
}

static void destroy_data_frames(frame_any_t *frames[], uint16_t nframes)
{
	for (int i = 0; i < nframes; i++) {
		frame_destroy(&frames[i]);
	}
}

static void print_frames(frame_any_t *frames[], int nframes)
{
	for (int f_ctr = 0; f_ctr < nframes; f_ctr++) {
		uint8_t buf[127];
		int frame_len = frame_to_bytes(frames[f_ctr], buf);

		for (int i = 0; i < frame_len; i++) {
			int shift = 7;
			printf("[%d]\t", i);
			do {
				printf("%d\t", (buf[i] >> shift) & 1);
			} while (shift--);
			printf("|\t{0x%x}\t{%u}\n", buf[i], buf[i]);
		}
	}
}

static void send_frames(frame_any_t *frames[], int nframes)
{
	for (int i = 0; i < nframes; i++) {
		int tx_status;
		uint8_t tx_data[128];
		struct net_pkt *netpkt;

		tx_data[0] = frame_to_bytes(frames[i], &tx_data[1]);

		net_pkt_fill(&netpkt, tx_data);
		net_pkt_hexdump(netpkt, ">");
		tx_status = tx(netpkt);
		net_pkt_unref(netpkt);

		printf("Frame transmission %s\n",
		       !tx_status ? "succeded" : "failed");
		k_msleep(1000);
	}
}

void main(void)
{
	frame_any_t *frames[1];

	/* Do the device binding, start the radio and initialize net_pkt */
	example_init();

	/* Generate `DATA` frames and fill the payload with random data */
	generate_data_frames(frames, ARRAY_SIZE(frames));

	/* Give some time receivers to start receiving */
	k_msleep(100);

	/* Attempt to send all frames */
	send_frames(frames, ARRAY_SIZE(frames));

	/* Clean allocated frames */
	destroy_data_frames(frames, ARRAY_SIZE(frames));

	/* Terminate */
	posix_exit(0);
}
