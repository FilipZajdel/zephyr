/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
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

#define RX_TX_BUF_SIZE (150)
#define RX_TX_BPS (1000000)
#define TX_POWER_LVL (20)

#define RADIO_SAMPLING_INTERVAL (1000)
#define RADIO_TX_INTERVAL (1)
#define IEEE802154_PHYADDRESS (0xDEAD)

struct radio_config {
	p2G4_freq_t channel;
	p2G4_power_t tx_power;
};

enum bs_radio_states {
	RADIO_STATE_RX_IDLE,
	RADIO_STATE_RX,
	RADIO_STATE_TX_PREPARE,
	RADIO_STATE_TX,
};

uint64_t bs_radio_timer;

static uint64_t last_phy_sync_time;
static enum bs_radio_states radio_state;
static struct radio_config radio_config;
static uint8_t rx_buf[RX_TX_BUF_SIZE];
static uint8_t tx_buf[RX_TX_BUF_SIZE];
static uint16_t rx_len;
static uint16_t tx_len;
static bool is_running;
static uint8_t device_eui64[8];

static bs_radio_event_cb_t radio_event_cb = NULL;

static p2G4_rx_t ongoing_rx = {
    .phy_address = IEEE802154_PHYADDRESS,
    .radio_params = {
        .modulation = P2G4_MOD_BLE,
        .center_freq = 20,
    },
    .antenna_gain = 0,
    .sync_threshold = 100,
    .header_threshold = 100,
    .pream_and_addr_duration = 0,
    .scan_duration = 1500,
    .header_duration = 0,
    .bps = RX_TX_BPS,
    .abort = {NEVER, NEVER},
};

static p2G4_tx_t ongoing_tx = {
    .start_time = NEVER,
    .end_time = NEVER,
    .phy_address = IEEE802154_PHYADDRESS,
    .radio_params = {
        .modulation = P2G4_MOD_BLE,
        .center_freq = 20,
    },
    .power_level = TX_POWER_LVL,
    .packet_size = 0,
    .abort = {NEVER, NEVER}
};

static int try_receive(uint32_t scan_duration, uint64_t *end_time);
static uint64_t packet_bitlen(uint32_t packet_len, uint64_t bps);
static char *rx_status_to_str(uint8_t status, char *buf);
static void send_data(uint64_t tx_start_time, uint64_t *end_time);
static void fill_eui64(void);

/**
 * Initialize the communication with BabbleSim.
 */
void bs_radio_init(void)
{
	struct bs_radio_args *args;

	bs_radio_timer = NEVER;
	radio_state = RADIO_STATE_RX_IDLE;
	radio_config.channel = 0;
	radio_config.tx_power = 0;
	rx_len = 0;
	tx_len = 0;
	is_running = false;
	memset(rx_buf, 0, RX_TX_BUF_SIZE);
	memset(tx_buf, 0, RX_TX_BUF_SIZE);

	args = bs_radio_argparse_get();
	if (args->is_bsim) {
		int initcom_err = p2G4_dev_initcom_c(
			args->device_nbr, args->s_id, args->p_id, NULL);

		if (initcom_err) {
			bs_trace_warning(
				"Failed to initialize communication with %s\n",
				args->p_id);
		}

		fill_eui64();
	}
}

/**
 * Closes the connection with BabbleSim.
 */
void bs_radio_deinit(void)
{
	p2G4_dev_disconnect_c();
}

/**
 * Starts waiting for an incoming reception.
 */
void bs_radio_start(bs_radio_event_cb_t event_cb)
{
	if (!bs_radio_argparse_get()->is_bsim) {
		return;
	}

	is_running = true;
	radio_event_cb = event_cb;

	bs_radio_timer = hwm_get_time() + RADIO_SAMPLING_INTERVAL;
	hwm_find_next_timer();
}

/**
 * Stops all ongoing operations.
 */
void bs_radio_stop()
{
	if (!bs_radio_argparse_get()->is_bsim) {
		return;
	}

	is_running = false;
	radio_state = RADIO_STATE_RX_IDLE;
	bs_radio_timer = NEVER;

	hwm_find_next_timer();
}

/**
 * Set the frequency of the radio.
 * 
 * Arguments:
 * freq         - a frequency that radio will operate on
 * 
 * Returns:
 * 0            -  Success
 * negative     -  Frequency couldn't be set (radio is busy)
 */
int bs_radio_frequency_set(uint16_t freq)
{
	if (radio_state != RADIO_STATE_RX_IDLE) {
		bs_trace_warning("Channel can't be set during ongoing "
				 "operation\n");
		return -1;
	}

	radio_config.channel = freq;
	return 0;
}

uint16_t bs_radio_frequency_get(void)
{
	return radio_config.channel;
}

/**
 * Set the transmissing power.
 * 
 * Arguments:
 * power_dBm    -    The value of tx power in dBm
 * 
 * Returns:
 * 0            -    Success
 * negative     -    The power couldn't be set (radio is busy)
 */
int bs_radio_tx_power_set(uint16_t power_dBm)
{
	if (radio_state != RADIO_STATE_RX_IDLE) {
		bs_trace_warning("TX Power can't be set during "
				 "ongoing operation\n");
		return -1;
	}

	radio_config.tx_power = power_dBm;
	return 0;
}

/** Returns current setting of tx power */
uint16_t bs_radio_tx_power_get(void)
{
	return radio_config.tx_power;
}

/**
 * Perform RSSI sensing.
 * 
 * The result is returned by a callback bs_radio_event_cb_t either as 
 * BS_RADIO_EVENT_RSSI_DONE or BS_RADIO_EVENT_RSSI_FAILED.
 * The function will fail when it's called in the middle of ongoing 
 * transmission.
 * 
 * Returns:
 * 0            - rssi can be sensed
 * negative     - rssi cannot be sensed
 */
int bs_radio_rssi(uint64_t duration_us)
{
	bs_trace_warning("%s not supported\n", __func__);
	return -ENOTSUP;
}

/**
 * Performs fsm transitions.
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
 * ongoing transmittion is finished. 
 * 
 * Similarly, when BS Radio remains in BS_RADIO_RX state
 * it cannot start transmitting unless reception is finished.
 */
void bs_radio_triggered(void)
{
	static uint64_t last_rx_try_end;
	static uint64_t last_tx_end;
	uint64_t current_time;

	if (!is_running) {
		return;
	}

	current_time = hwm_get_time();

	switch (radio_state) {
	case RADIO_STATE_RX_IDLE: {
		int ret = try_receive(packet_bitlen(RX_TX_BUF_SIZE, RX_TX_BPS),
				      &last_rx_try_end);

		if (0 == ret) {
			bs_trace_debug(0, "Im going from `RX_IDLE` to `RX`\n");
			radio_state = RADIO_STATE_RX;
			bs_radio_timer = last_rx_try_end;
			// hwm_find_next_timer();
		} else {
			radio_state = RADIO_STATE_RX_IDLE;
			bs_radio_timer =
				last_rx_try_end + 1;// + RADIO_SAMPLING_INTERVAL;
			// hwm_find_next_timer();
		}
		break;
	}
	case RADIO_STATE_RX:
		if (current_time >= last_rx_try_end) {
			/* Now we can say that the data is received */
			struct bs_radio_event_data
				rx_event_data = { .type = BS_RADIO_EVENT_RX_DONE,
						  .rx_done = {
							  .len = rx_len,
							  .data = rx_buf,
						  } };
			radio_event_cb(&rx_event_data);

			bs_trace_debug(0, "Im going from `RX` to `RX_IDLE`\n");
			radio_state = RADIO_STATE_RX_IDLE;
			bs_radio_timer = current_time + RADIO_SAMPLING_INTERVAL;
			// hwm_find_next_timer();
			last_rx_try_end = NEVER;
		} else {
			bs_trace_warning(
				"Bad state, it shouldn't have happened\n");
			radio_state = RADIO_STATE_RX_IDLE;
			bs_radio_timer = NEVER;
			// hwm_find_next_timer();
		}
		break;
	case RADIO_STATE_TX_PREPARE:
		if (last_phy_sync_time <= current_time) {
			bs_trace_debug(0,
				       "Im going from `TX_PREPARE` to `TX`\n");
			send_data(current_time + 1, &last_tx_end);
			bs_trace_debug(0, "After data send\n");
			radio_state = RADIO_STATE_TX;
			bs_radio_timer = last_tx_end;
			// hwm_find_next_timer();
		} else {
			bs_radio_timer = last_phy_sync_time;
			// hwm_find_next_timer();
		}
		break;
	case RADIO_STATE_TX:
		if (current_time >= last_tx_end) {
			/* Now we can say that the data was sent */
			bs_trace_debug(0, "Data sent at: %lu\n", current_time);

			struct bs_radio_event_data tx_event_data = {
				.type = BS_RADIO_EVENT_TX_DONE,
			};
			radio_event_cb(&tx_event_data);

			bs_trace_debug(0, "Im going from `TX` to `RX_IDLE`\n");
			radio_state = RADIO_STATE_RX_IDLE;
			last_tx_end = NEVER;
			bs_radio_timer = current_time + RADIO_TX_INTERVAL;
			// hwm_find_next_timer();
		} else {
			bs_trace_warning(
				"Bad state, it shouldn't have happened\n");
			radio_state = RADIO_STATE_RX_IDLE;
			bs_radio_timer = NEVER;
			// hwm_find_next_timer();
		}
	default:
		break;
	}
}

/**
 * Start the transmission.
 * 
 * If the device is not currently receiving or transmitting,
 * it will send data. Otherwise it will return with error.
 * 
 * Arguments:
 * data         - the data to send
 * data_len     - number of bytes in the buffer
 * cca          - (unused) set whether to perform cca or not
 *
 * Returns:
 * 0            - data successfully sent
 * negative     - data couldn't be sent
 */
int bs_radio_tx(uint8_t *data, uint16_t data_len, bool cca)
{
	if (!bs_radio_argparse_get()->is_bsim) {
		return -EFAULT;
	}

	/* (void *)cca; */ /* Unused */
	if (!is_running) {
		bs_trace_debug(0, "Radio was not started\n");
		return -1;
	}

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

/**
 * Perform cca.
 * 
 * CCA result is returned by a bs_radio_event_cb_t call with the status of
 * either BS_RADIO_EVENT_CCA_DONE or BS_RADIO_EVENT_CCA_FAILED.
 * 
 * Returns:
 * 0            -       Success (cca will be performed)
 * negative     -       CCA cannot be performed (radio is busy)
 */
int bs_radio_cca(void)
{
	int cca_result;

	if (radio_state == RADIO_STATE_RX) {
		cca_result = -EBUSY;
	} else if (radio_state == RADIO_STATE_TX ||
		   radio_state == RADIO_STATE_TX_PREPARE) {
		cca_result = -EIO;
	} else {
		cca_result = 0;
	}

	return cca_result;
}

/**
 * Returns the eui64.
 * 
 * The address is generated based on BabbleSim device id in given simulation.
 * Meaning command line parameter "-d". Every device in a given simulation
 * has different mac, but can have the same across different simulations
 * if run with the same id.
 * 
 * Arguments:
 * mac [out]   -   Pointer to copy the eui64 - can't be NULL
 */
void bs_radio_get_mac(uint8_t *mac)
{
	if (mac == NULL) {
		bs_trace_error("%s - mac cannot be NULL");
		return;
	}

	memcpy(mac, device_eui64, 8);
}

/**
 * Private functions
 */

/**
 * Attempt data reception. Blocks until data is received.
 * Function will succeed when device received pream + address match
 * from the phy.
 * 
 * Return 0 if reception succeded, -1 otherwise.
 * 
 * Arguments:
 * scan_duration - The time of scanning in us. Note that function will
 *                 block for scan_duration or until receives something
 * end_time[out] - If function succeded this is the time when reception 
 *                 finishes.
 * 
 * Returns:
 * 0             - Data received successfully
 * negative      - No data to receive
 */
static int try_receive(uint32_t scan_duration, uint64_t *end_time)
{
	int ret;
	p2G4_rx_done_t rx_done_s;
	uint8_t *frame = NULL;
	char status_buf[20];

	ongoing_rx.radio_params.center_freq = radio_config.channel;

	ongoing_rx.pream_and_addr_duration = packet_bitlen(2, RX_TX_BPS);
	ongoing_rx.header_duration = 0;

	ongoing_rx.start_time = hwm_get_time();
	ongoing_rx.scan_duration = scan_duration;

	ret = p2G4_dev_req_rx_c_b(&ongoing_rx, &rx_done_s, &frame, 0, NULL);
	*end_time = rx_done_s.end_time;
	last_phy_sync_time = rx_done_s.end_time;

	if ((ret >= 0) && (rx_done_s.packet_size > 0)) {
		memcpy(rx_buf, frame, rx_done_s.packet_size);
		rx_len = rx_done_s.packet_size;
		free(frame);
		bs_trace_debug(0, "RX status: %s\n",
		       rx_status_to_str(rx_done_s.status, status_buf));
		return 0;
	}

	return -1;
}

/**
 * Send data from tx_buf of tx_len bytes
 * 
 * Arguments:
 * start_time     - When the transmission will start
 * end_time [out] - When the transmission finished
 */
static void send_data(uint64_t tx_start_time, uint64_t *end_time)
{
	p2G4_tx_done_t tx_done_s;
	int result;

	ongoing_tx.radio_params.center_freq = radio_config.channel;
	ongoing_tx.power_level = radio_config.tx_power;
	ongoing_tx.packet_size = tx_len;
	ongoing_tx.start_time = tx_start_time;
	ongoing_tx.end_time =
		ongoing_tx.start_time + packet_bitlen(tx_len, RX_TX_BPS);
	bs_trace_debug(0, "Data tx end time: %llu\n", ongoing_tx.end_time);

	result = p2G4_dev_req_tx_c_b(&ongoing_tx, tx_buf, &tx_done_s);

	*end_time = ongoing_tx.end_time;
	last_phy_sync_time = ongoing_tx.end_time;
}

/* Calculate the time in air of a number of bytes */
static uint64_t packet_bitlen(uint32_t packet_len, uint64_t bps)
{
	uint64_t bits_per_us = bps / 1000000;
	return (packet_len * 8) / bits_per_us;
}

/* Generate MAC address using BabbleSim device's id */
static void fill_eui64(void)
{
	struct bs_radio_args *args = bs_radio_argparse_get();

	for (int i = 2; i < 8; i++) {
		device_eui64[i] = 0xFF;
	}
	device_eui64[1] = (uint8_t)((args->device_nbr >> 8) && 0xFF);
	device_eui64[0] = (uint8_t)(args->device_nbr && 0xFF);
}

static char *rx_status_to_str(uint8_t status, char *buf)
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
