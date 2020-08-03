/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BS_RADIO_H
#define _BS_RADIO_H

#include <stdint.h>
#include <stdbool.h>
#include "bs_radio_argparse.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct bs_radio_args bs_radio_args;

enum bs_radio_event_types {
	/** On reception success */
	BS_RADIO_EVENT_RX_DONE,

	/** On reception failure */
	BS_RADIO_EVENT_RX_FAILED,

	/** On transmission success */
	BS_RADIO_EVENT_TX_DONE,

	/** On transmittion failure */
	BS_RADIO_EVENT_TX_FAILED,

	/** On CCA success */
	BS_RADIO_EVENT_CCA_DONE,

	/** On CCA failure */
	BS_RADIO_EVENT_CCA_FAILED,

	/** On energy measurement success */
	BS_RADIO_EVENT_ENERGY_DONE,

	/** On energy measurement failure */
	BS_RADIO_EVENT_ENERGY_FAILED
};

struct bs_radio_event_data {
	enum bs_radio_event_types type;
	union {
		struct {
			uint16_t len;
			uint8_t *data;
		} rx_done;

		struct {
			uint16_t rssi;
		} energy_done;
	};
};

typedef void (*bs_radio_event_cb_t)(struct bs_radio_event_data *);

/* HW Models API */
void bs_radio_init(void);
void bs_radio_triggered(void);
void bs_radio_deinit(void);


/* User API */
void bs_radio_start(bs_radio_event_cb_t event_cb);
void bs_radio_stop();

int bs_radio_tx(uint8_t *data, uint16_t data_len, bool cca);
int bs_radio_rssi(void);

int bs_radio_channel_set(uint16_t channel);
uint16_t bs_radio_channel_get(void);
int bs_radio_tx_power_set(uint16_t power);

#ifdef __cplusplus
}
#endif

#endif /* _BS_RADIO_H */
