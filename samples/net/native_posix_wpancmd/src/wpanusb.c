/*
 * Copyright (c) 2016-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_LEVEL 0
#include <logging/log.h>
LOG_MODULE_REGISTER(wpanusb);

#include <stdio.h>
#include <zephyr.h>
#include <net/buf.h>
#include <stdlib.h>
#include <net/ieee802154_radio.h>
#include <ieee802154/ieee802154_frame.h>
#include <net_private.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

#include "wpanusb.h"

#define COMMAND_ARG_DELIIMITER ' '

static struct ieee802154_radio_api *radio_api;
static struct device *ieee802154_dev;

static struct k_fifo tx_queue;

/* IEEE802.15.4 frame + 1 byte len + 1 byte LQI */
uint8_t tx_buf[IEEE802154_MTU + 1 + 1];

/**
 * Stack for the tx thread.
 */
static K_THREAD_STACK_DEFINE(tx_stack, 1024);
static struct k_thread tx_thread_data;

struct request_command commands[] = { { RESET, "reset" },
				      { TX, "tx" },
				      { XMIT_ASYNC, "xmitasync" },
				      { ED, "ed" },
				      { SET_CHANNEL, "setchannel" },
				      { START, "start" },
				      { STOP, "stop" },
				      { SET_SHORT_ADDR, "setshortaddr" },
				      { SET_PAN_ID, "setpanid" },
				      { SET_IEEE_ADDR, "setieeeaddr" },
				      { SET_TXPOWER, "settxpower" },
				      { SET_CCA_MODE, "setccamode" },
				      { SET_CCA_ED_LEVEL, "setccaedlevel" },
				      { SET_CSMA_PARAMS, "setcsmaparams" },
				      { SET_PROMISCUOUS_MODE,
					"setpromiscuousmode" },
				      { SHUT_DOWN, "quit" },
				      { SHUT_DOWN, "exit" },
				      { SHUT_DOWN, "q" },
				      { SHUT_DOWN, "out" } };

#define COMMANDS_NUM ARRAY_SIZE(commands);

/**
 * Attempts to get a line from stdin until @timeout interval passes.
 * 
 * Arguments:
 * dest [out]	-	The data taken from stdin will be stored here. 
 * 					If dest is NULL, then the data will be allocated and must
 * 					be deallocated with a free(*dest) invocation.
 * size			-	The size of provided buffer.
 * timeout_s	-	The time to wait until function returns.
 * 
 * Returns:
 * negative		-	The errno code - ETIME on timeout, ENOMEM when either
 * 					memory allocation fails or provided buffer is too small.
 * 0 or positive-	The number of bytes read.
 */
int getline_timeout(char **dest, unsigned int *size, unsigned long timeout_s)
{
	struct timeval tv = { .tv_sec = timeout_s, .tv_usec = 0 };
	fd_set set;
	int ret;

	FD_ZERO(&set);
	FD_SET(STDIN_FILENO, &set);

	ret = select(1, &set, NULL, NULL, &tv);

	if (ret > 0) {
		char buf[128] = "";

		int nread = read(STDIN_FILENO, buf, 127);
		buf[nread] = 0;
		if (!(*dest)) {
			*dest = malloc(nread * sizeof(**dest));

			if (!*dest) {
				return -ENOMEM;
			}
		} else if (*size < nread) {
			return -ENOMEM;
		}

		strncpy(*dest, buf, nread);
		ret = nread;
	} else {
		ret = -ETIME;
	}

	return ret;
}

/**
 * Checks if two strings are identical until delimiter occurence in str1.
 * 
 * For example strcmp_until("string1 aaa", "string1 bbb", ' ') will return 0 (identical),
 * because strings are identical until ' '
 */
int strcmp_until(const char *str1, const char *str2, const char delimiter)
{
	int i = 0;
	int delim_at = 0;

	do {
		delim_at++;
	} while ((str1[delim_at] != delimiter) && delim_at < strlen(str1));

	for (i = 0; i < delim_at; i++) {
		if (str1[i] != str2[i]) {
			return -1;
		}
	}

	return 0;
}

/**
 * Search the string for the specified character
 * 
 * Return index in string or -1 if couldn't find the character
 */
int find_first_in_str(const char *str, char ch_to_match)
{
	for (int i = 0; i < strlen(str); i++) {
		if (str[i] == ch_to_match) {
			return i;
		}
	}

	return -1;
}

int str_replace(char *str_to_replace, char old_char, char new_char)
{
	int repl_ctr;
	char *p_str;

	for (repl_ctr = 0, p_str = str_to_replace; strlen(p_str); p_str++) {
		if (*p_str == old_char) {
			*p_str = new_char;
			repl_ctr++;
		}
	}

	return repl_ctr;
}

/**
 * Cuts out char from str replacing it with empty (concatenating)
 */
void str_cut(char *str_to_cut, char char_to_cut)
{
	for (char *p_str = str_to_cut; strlen(p_str); p_str++) {
		if (*p_str == char_to_cut) {
			char buf[strlen(p_str)];
			strcpy(buf, p_str + 1);
			strcpy(p_str, buf);
		}
	}
}

enum wpanusb_requests cmd_decode(const char *cmd, uint8_t *arg_idx,
				 uint8_t *args_len)
{
	enum wpanusb_requests request = -1;

	for (int i = 0; i < sizeof(commands) / sizeof(*commands); i++) {
		if (strcmp_until(cmd, commands[i].command,
				 COMMAND_ARG_DELIIMITER) == 0) {
			request = commands[i].request;

			*arg_idx =
				find_first_in_str(cmd, COMMAND_ARG_DELIIMITER) +
				1;
			*args_len = strlen(cmd + *arg_idx);
			break;
		}
	}

	return request;
}

/* Decode wpanusb commands */

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

	printk("len %d seq %u\n", buf->len, seq);
	for (int i = 0; i < buf->len; i++) {
		printk("%u\n", buf->data[i]);
	}
	printk("The end of buf\n");

	do {
		ret = radio_api->tx(ieee802154_dev, IEEE802154_TX_MODE_DIRECT,
				    pkt, buf);
		printk("%s\n", ret == 0 ? "Data sent" : "Retry to send");
	} while (ret && retries--);

	if (ret) {
		printk("Error sending data, seq %u", seq);
		/* Send seq = 0 for unsuccessful send */
		seq = 0U;
	}

	return ret;
}

int cca()
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

	/* Add seq to TX */
	// if (setup->bRequest == TX) {
	// 	net_pkt_write_u8(pkt, setup->wIndex);
	// }

	// net_pkt_write(pkt, *data, *len);

	// LOG_DBG("pkt %p len %u seq %u", pkt, *len, setup->wIndex);
	printk("pkt %p len %u\n", *pkt, len);
	return 0;
}

static void tx_thread(void)
{
	printk("Radio loop started at channel 20\n");
	start();
	set_channel("20", 3);
	srand(time(NULL));

	printk("Insert command:\n>\t");

	while (1) {
		uint8_t cmd;
		struct net_pkt *pkt;
		struct net_buf *buf;
		char *cmd_buf = NULL;
		ssize_t n_read = 0;
		ssize_t buf_size = 0;
		uint8_t args_idx;
		uint8_t args_len;
		uint8_t data[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 };

		n_read = getline_timeout(&cmd_buf, &buf_size, 2);
		if (n_read >= 0) {
			str_cut(cmd_buf, '\n');

			if (0 == strcmp("help", cmd_buf) ||
			    0 == strcmp("-h", cmd_buf)) {
				printk("Available commands:\n");
				for (int i = 0; i < ARRAY_SIZE(commands); i++) {
					printk("%s\n", commands[i].command);
				}

				free(cmd_buf);
				printk("Insert command> \n");
				continue;
			}
		} else {
			k_msleep(250); // To let the scheduler work
			continue;
		}

		k_msleep(200);
		cmd = cmd_decode(cmd_buf, &args_idx, &args_len);

		// pkt = k_fifo_get(&tx_queue, K_FOREVER);
		net_pkt_fill(&pkt, data, ARRAY_SIZE(data));
		buf = net_buf_frag_last(pkt->buffer);
		net_pkt_hexdump(pkt, ">");

		switch (cmd) {
		case RESET:
			printk("Reset device");
			break;
		case TX:
			if (!cca()) {
				tx(pkt);
			} else {
				printk("CCA Failed\n");
			}
			break;
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
			posix_exit(0);
		default:
			printk("%s %x: Not handled", cmd_buf, cmd);
			break;
		}

		net_pkt_unref(pkt);

		k_yield();
		free(cmd_buf);
		k_msleep(rand() % 250); // To let the scheduler time to work
		printk("Insert command:\n>\t");
	}
}

static void init_tx_queue(void)
{
	/* Transmit queue init */
	k_fifo_init(&tx_queue);

	k_thread_create(&tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)tx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(8), 0, K_NO_WAIT);
}

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	size_t len = net_pkt_get_len(pkt);
	uint8_t *p = tx_buf;
	int ret;
	// uint8_t ep;

	printk("Got data, pkt %p, len %d", pkt, (int)len);

	net_pkt_hexdump(pkt, "<");

	if (len > (sizeof(tx_buf) - 2)) {
		printk("Too large packet");
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
		printk("Cannot read pkt");
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
	printk("Starting wpanusb\n");

	/* Initialize net_pkt */
	net_pkt_init();

	ieee802154_dev =
		device_get_binding(CONFIG_IEEE802154_NATIVE_POSIX_DRV_NAME);

	if (ieee802154_dev) {
		radio_api = (struct ieee802154_radio_api *)
				    ieee802154_dev->driver_api;
		printk("radio_api %p of device %s initialized\n", radio_api,
		       ieee802154_dev->name);
		/* Initialize transmit queue */
		init_tx_queue();
	} else {
		printk("Couldn't bind %s\n",
		       CONFIG_IEEE802154_NATIVE_POSIX_DRV_NAME);
	}
}
