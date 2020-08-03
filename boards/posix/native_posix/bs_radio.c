/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "hw_models_top.h"
#include "bs_radio.h"
#include "bs_types.h"
#include "bs_tracing.h"
#include "bs_utils.h"
#include "bs_pc_2G4.h"
#include "bs_radio_argparse.h"

#define BUF_SIZE (150)
#define RX_TX_BPS (1000000)
#define TX_POWER_LVL (20)

#define RADIO_SAMPLING_INTERVAL (2500)
#define RADIO_TX_INTERVAL (1)
#define IEEE802154_PHYADDRESS (0xDEAD)

/** For debug */
#define BOOL_TO_STR(b) (b ? "true" : "false")
#define INT_TO_BOOL(num) ((num == 0) ? false : true)

typedef struct __attribute__((packed)) {
	uint8_t frame_control[2];
	uint8_t seq_no;
	struct __attribute__((packed)) {
		uint8_t dest_pan_ID[2];
		uint8_t dest_addr[8];
		uint8_t src_pan_ID[2];
		uint8_t src_addr[8];
	} address;

	uint8_t payload[102];
	uint8_t fcs[2];
} ieee802154_mac_frame_t;

typedef struct __attribute__((packed)) {
	uint8_t preamble[4];
	uint8_t sfd;
	uint8_t lof;
	ieee802154_mac_frame_t payload;
} ieee802154_phy_frame_t;

enum bs_radio_states {
	RADIO_STATE_RX_IDLE,
	RADIO_STATE_RX,
	RADIO_STATE_TX_PREPARE,
	RADIO_STATE_TX,
};

uint64_t bs_radio_timer;

static enum bs_radio_states radio_state;
static uint8_t rx_buf[BUF_SIZE];
static uint8_t tx_buf[BUF_SIZE];
static uint16_t rx_len;
static uint16_t tx_len;

static bs_radio_event_cb_t radio_event_cb = NULL;

/** Probably not needed */
/** Just for convenience now */
static p2G4_rx_t rx_s = {
    .phy_address = IEEE802154_PHYADDRESS,
    .radio_params = {
        .modulation = P2G4_MOD_BLE,
        .center_freq = 20,
    },
    .antenna_gain = 1,
    .sync_threshold = 100,
    .header_threshold = 100,
    .pream_and_addr_duration = 0,
    .scan_duration = 1500,
    .header_duration = 0,
    .bps = RX_TX_BPS,
    .abort = {NEVER, NEVER},
};

static p2G4_tx_t tx_s = {
    .start_time = NEVER,
    .end_time = NEVER,
    .phy_address = IEEE802154_PHYADDRESS,
    .radio_params = {
        .modulation = P2G4_MOD_BLE,
        .center_freq = 20,
    },
    .power_level = TX_POWER_LVL,
    .packet_size = sizeof(ieee802154_phy_frame_t),
    .abort = {NEVER, NEVER}
};

static int try_receive(uint32_t scan_duration, uint64_t *rx_end);
static uint64_t packet_bitlen(uint32_t packet_len, uint64_t bps);
static char *rx_status_to_str(uint8_t status, char *buf);
static void send_data(uint64_t tx_start_time, uint64_t *end_time);

void bs_radio_init(void)
{
	struct bs_radio_args *args;

	bs_radio_timer = NEVER;
	radio_state = RADIO_STATE_RX_IDLE;
	rx_len = 0;
	tx_len = 0;
	memset(rx_buf, 0, BUF_SIZE);
	memset(tx_buf, 0, BUF_SIZE);

	args = bs_radio_argparse_get();
	if (args->is_bsim) {
		int initcom_err = p2G4_dev_initcom_c(
			args->device_nbr, args->s_id, args->p_id, NULL);

		if (initcom_err) {
			bs_trace_warning(
				"Failed to initialize communication with %s\n",
				args->p_id);
		}
	}
}

void bs_radio_start(bs_radio_event_cb_t event_cb)
{
	radio_event_cb = event_cb;
	bs_radio_timer = hwm_get_time() + RADIO_SAMPLING_INTERVAL;

	hwm_find_next_timer();
}

/**
 * Performs fsm states transitions.
 * 
 * By default (and most often) BS Radio remains
 * in BS_RADIO_RX_IDLE state.
 * 
 * Transition to other state can be invoked by
 * either calling bs_radio_tx() function or by
 * start of data reception.
 * 
 * Note that when BS Radio is in BS_RADIO_TX state
 * it cannot perform transition to BS_RADIO_RX unless
 * transmittion is finished. 
 * 
 * Also, when BS Radio remains in BS_RADIO_RX it cannot
 * start transmitting unless reception is finished.
 */
void bs_radio_triggered(void)
{
	static uint64_t last_rx_attempt_end;
	static uint64_t tx_time_end;
	uint64_t current_time;

	current_time = hwm_get_time();

	switch (radio_state) {
	case RADIO_STATE_RX_IDLE: {
		int rec_status = try_receive(
			packet_bitlen(sizeof(ieee802154_phy_frame_t), RX_TX_BPS),
			&last_rx_attempt_end);

		if (0 == rec_status) {
			bs_trace_debug(0, "Im going from `RX_IDLE` to `RX`\n");
			radio_state = RADIO_STATE_RX;
			bs_radio_timer = last_rx_attempt_end;
			hwm_find_next_timer();
		} else {
			radio_state = RADIO_STATE_RX_IDLE;
			bs_radio_timer =
				last_rx_attempt_end + RADIO_SAMPLING_INTERVAL;
			hwm_find_next_timer();
		}
		break;
	}
	case RADIO_STATE_RX:
		if (current_time >= last_rx_attempt_end) {
			/* Now we can say that the data is received */
			union bs_radio_event_data
				rx_event_data = { .rx_done = {
							  .len = rx_len,
							  .data = rx_buf,
						  } };
			printf("Received %d bytes at %lu\n", rx_len,
			       current_time);
			radio_event_cb(BS_RADIO_EVENT_RX_DONE, &rx_event_data);
		}
		bs_trace_debug(0, "Im going from `RX` to `RX_IDLE`\n");
		radio_state = RADIO_STATE_RX_IDLE;

		bs_radio_timer = hwm_get_time() + RADIO_SAMPLING_INTERVAL - 1;
		hwm_find_next_timer();
		break;
	case RADIO_STATE_TX_PREPARE:
		bs_trace_debug(0, "Im going from `TX_PREAPARE` to `TX`\n");
		send_data(hwm_get_time() + 1, &tx_time_end);
		radio_state = RADIO_STATE_TX;
		bs_radio_timer = tx_time_end;
		hwm_find_next_timer();
		break;
	case RADIO_STATE_TX:
		if (current_time >= tx_time_end) { // FIX this is -> state machine will not work if current_time < tx_time_end
                        /* Now we can say that the data was sent */
			bs_trace_debug(0, "Data sent at: %lu\n", current_time);
			radio_event_cb(BS_RADIO_EVENT_TX_DONE, NULL);
		}
		bs_trace_debug(0, "Im going from `TX` to `RX_IDLE`\n");
		radio_state = RADIO_STATE_RX_IDLE;
		bs_radio_timer = hwm_get_time() + RADIO_TX_INTERVAL;
		hwm_find_next_timer();
	default:
		break;
	}
}

int bs_radio_tx(uint8_t *data, uint16_t data_len, bool cca)
{
	/* (void *)cca; */ /* Unused */

	if ((data == NULL) || (data_len == 0) ||
	    radio_state == RADIO_STATE_RX) {
		bs_trace_debug(0, "Radio is now receiving\n");
		return -1;
	}

	if ((radio_state == RADIO_STATE_TX) ||
	    (radio_state == RADIO_STATE_TX_PREPARE)) {
		bs_trace_debug(0, "Radio is now transmitting\n");
		return -1;
	}

	bs_trace_debug(0, "Im going from `RX_IDLE` to `TX_PREPARE`\n");
	radio_state = RADIO_STATE_TX_PREPARE;

	tx_len = data_len;
	memcpy(tx_buf, data, data_len);

	bs_radio_timer = hwm_get_time() + RADIO_TX_INTERVAL;
	hwm_find_next_timer();

	return 0;
}

/*
        Internal functions
*/

/**
 * Attempt data reception. Blocks until data is received.
 * Function will succeed when device received pream + address match
 * from the phy.
 * 
 * Return 0 if reception succeded, -1 otherwise.
 * 
 * arg scan_duration - The time of scanning in us. Note that function will
 *                     block for scan_duration or until receives something
 * arg rx_end        - If function succeded this is the time when reception 
 *                     finished.
 */
static int try_receive(uint32_t scan_duration, uint64_t *rx_end)
{
	int ret;
	p2G4_rx_done_t rx_done_s;
	char rx_status_str[30];
	uint8_t *frame = NULL;

	rx_s.pream_and_addr_duration = packet_bitlen(2, RX_TX_BPS);
	rx_s.header_duration = 0;

	rx_s.start_time = hwm_get_time();
	rx_s.scan_duration = scan_duration;

	ret = p2G4_dev_req_rx_c_b(&rx_s, &rx_done_s, &frame, 0, NULL);

	// printf("Received %d bytes of data\n", rx_done_s.packet_size);
	// bs_trace_debug(
	// 	0,
	// 	"%s -- Data received with status %s at %lu end_time %lu with ret %d\n",
	// 	__func__, rx_status_to_str(rx_done_s.status, rx_status_str),
	// 	rx_done_s.end_time, rx_done_s.end_time, ret);

	*rx_end = rx_done_s.end_time;
	if ((ret >= 0) && (rx_done_s.packet_size > 0)) {
		memcpy(rx_buf, frame, rx_done_s.packet_size);
		rx_len = rx_done_s.packet_size;
		free(frame);
		return 0;
	}

	return -1;
}

void send_data(uint64_t tx_start_time, uint64_t *end_time)
{
	p2G4_tx_done_t tx_done_s;
	int result;

	tx_s.packet_size = tx_len;
	tx_s.start_time = tx_start_time;
	tx_s.end_time = tx_s.start_time + packet_bitlen(tx_len, RX_TX_BPS);
	bs_trace_debug(0, "Data tx end: %llu\n", tx_s.end_time);
	bs_trace_debug(0, "Len of data to sent: %lu\n", tx_len);

	result = p2G4_dev_req_tx_c_b(&tx_s, tx_buf, &tx_done_s);

	printf("%s -- (result)-> %s (start time)-> %lu (end time)-> %lu\n",
	       __func__, BOOL_TO_STR(!INT_TO_BOOL(result)), tx_s.start_time,
	       tx_done_s.end_time);
	*end_time = tx_s.end_time;
}

/**
 * Calculate the time in air of a number of bytes
 */
uint64_t packet_bitlen(uint32_t packet_len, uint64_t bps)
{
	uint64_t bits = packet_len * 8;
	uint64_t bits_per_us = bps / 1000000;

	return bits / bits_per_us;
}

char *rx_status_to_str(uint8_t status, char *buf)
{
	switch (status) {
	case P2G4_RXSTATUS_OK:
		strcpy(buf, "P2G4_RXSTATUS_OK");
		break;
	case P2G4_RXSTATUS_CRC_ERROR:
		strcpy(buf, "P2G4_RXSTATUS_OK");
		break;
	case P2G4_RXSTATUS_HEADER_ERROR:
		strcpy(buf, "P2G4_RXSTATUS_OK");
		break;
	case P2G4_RXSTATUS_NOSYNC:
		strcpy(buf, "P2G4_RXSTATUS_OK");
		break;
	case P2G4_RXSTATUS_INPROGRESS:
		strcpy(buf, "P2G4_RXSTATUS_OK");
		break;
	default:
		strcpy(buf, "UNKNOWN");
		break;
	}

	return buf;
}
