/* ieee802154_native_posix.h - Native Posix 802.15.4 driver */

/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_NATIVE_POSIX_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_NATIVE_POSIX_H_

#define NATIVE_POSIX_FCS_LENGTH   (2)
#define NATIVE_POSIX_PSDU_LENGTH  (125)
#define NATIVE_POSIX_PHR_LENGTH   (1)
#define NATIVE_POSIX_802154_RX_BUFFERS (20)
#define NATIVE_POSIX_IEEE802154_RX_STACK_SIZE	(4096)

#include <stdint.h>
#include <zephyr.h>

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


struct native_posix_802154_rx_frame {
	void *fifo_reserved; /* 1st word reserved for use by fifo. */
	uint8_t *psdu; /* Pointer to a received frame. */
	uint32_t time; /* RX timestamp. */
	uint8_t lqi; /* Last received frame LQI value. */
	int8_t rssi; /* Last received frame RSSI value. */
	bool ack_fpb; /* FPB value in ACK sent for the received frame. */
};

struct native_posix_802154_data {
	/* Pointer to the network interface. */
	struct net_if *iface;

	/* 802.15.4 HW address. */
	uint8_t mac[8];

	/* RX thread stack. */
	K_THREAD_STACK_MEMBER(rx_stack, NATIVE_POSIX_IEEE802154_RX_STACK_SIZE);

	/* RX thread control block. */
	struct k_thread rx_thread;

	/* RX fifo queue. */
	struct k_fifo rx_fifo;

	/* Buffers for passing received frame pointers and data to the
	 * RX thread via rx_fifo object.
	 */
	struct native_posix_802154_rx_frame rx_frames[NATIVE_POSIX_802154_RX_BUFFERS];

	/* Frame pending bit value in ACK sent for the last received frame. */
	bool last_frame_ack_fpb;

	/* CCA complete sempahore. Unlocked when CCA is complete. */
	struct k_sem cca_wait;

	/* CCA result. Holds information whether channel is free or not. */
	bool channel_free;

	/* TX synchronization semaphore. Unlocked when frame has been
	 * sent or send procedure failed.
	 */
	struct k_sem tx_wait;

	/* TX ack synchronization semaphore. Unlocked when ack was received */
	struct k_sem tx_ack_wait;

	/* TX buffer. First byte is PHR (length), remaining bytes are
	 * MPDU data.
	 */
	uint8_t tx_psdu[NATIVE_POSIX_PHR_LENGTH + NATIVE_POSIX_PSDU_LENGTH + NATIVE_POSIX_FCS_LENGTH];

	/* TX result, updated in radio transmit callbacks. */
	uint8_t tx_result;

	/* A buffer for the received ACK frame. psdu pointer be NULL if no
	 * ACK was requested/received.
	 */
	struct native_posix_802154_rx_frame ack_frame;

	/* Callback handler of the currently ongoing energy scan.
	 * It shall be NULL if energy scan is not in progress.
	 */
	energy_scan_done_cb_t energy_scan_done;

	/* Callback handler to notify of any important radio events.
	 * Can be NULL if event notification is not needed.
	 */
	ieee802154_event_cb_t event_handler;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_NATIVE_POSIX_H_ */
