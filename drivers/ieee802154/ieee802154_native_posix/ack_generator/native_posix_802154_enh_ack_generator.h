/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Module that contains an enhanced acknowledgement (Enh-Ack) generator
 * for the 802.15.4 radio driver.
 *
 */

#ifndef NATIVE_POSIX_802154_ENH_ACK_GENERATOR_H
#define NATIVE_POSIX_802154_ENH_ACK_GENERATOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initializes the Enhanced ACK generator module. */
void native_posix_802154_enh_ack_generator_init(void);

/** Creates an Enhanced ACK in response to the provided frame.
 *
 * This function creates an Enhanced ACK frame and inserts it into a radio
 * buffer.
 *
 * @param [in]  p_frame  Pointer to the buffer that contains PHR and PSDU of
 *                       the frame to respond to.
 *
 * @returns  Pointer to a constant buffer that contains PHR and PSDU
 *           of the created Enhanced ACK frame.
 */
const uint8_t *native_posix_802154_enh_ack_generator_create(const uint8_t *p_frame);

#ifdef __cplusplus
}
#endif

#endif /* NATIVE_POSIX_802154_ENH_ACK_GENERATOR_H */
