/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NATIVE_POSIX_802154_CONFIG_H__
#define NATIVE_POSIX_802154_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup native_posix_802154_config 802.15.4 driver configuration
 * @{
 * @ingroup nrf_802154
 * @brief Configuration of the 802.15.4 radio driver for nRF SoCs.
 */

/**
 * @defgroup native_posix_802154_config_radio Radio driver configuration
 * @{
 */

/**
 * @def NATIVE_POSIX_802154_PENDING_SHORT_ADDRESSES
 *
 * The number of slots containing short addresses of nodes for which the
 * pending data is stored.
 *
 */
#ifndef NATIVE_POSIX_802154_PENDING_SHORT_ADDRESSES
#define NATIVE_POSIX_802154_PENDING_SHORT_ADDRESSES 10
#endif

/**
 * @def NATIVE_POSIX_802154_PENDING_EXTENDED_ADDRESSES
 *
 * The number of slots containing extended addresses of nodes for which the
 * pending data is stored.
 *
 */
#ifndef NATIVE_POSIX_802154_PENDING_EXTENDED_ADDRESSES
#define NATIVE_POSIX_802154_PENDING_EXTENDED_ADDRESSES 10
#endif

/**
 * @def NATIVE_POSIX_802154_RX_BUFFERS
 *
 * The number of buffers in the receive queue.
 *
 */
#ifndef NATIVE_POSIX_802154_RX_BUFFERS
#define NATIVE_POSIX_802154_RX_BUFFERS 16
#endif

/**
 * @def NATIVE_POSIX_802154_DELAYED_TRX_ENABLED
 *
 * If the delayed transmission and the receive window features are available.
 *
 */
#ifndef NATIVE_POSIX_802154_DELAYED_TRX_ENABLED
#define NATIVE_POSIX_802154_DELAYED_TRX_ENABLED 1
#endif

/**
 * @}
 * @defgroup native_posix_802154_config_csma CSMA/CA procedure configuration
 * @{
 */

/**
 * @def NATIVE_POSIX_802154_CSMA_CA_ENABLED
 *
 * If CSMA-CA is to be enabled by the driver. Disabling CSMA-CA improves
 * the driver performance.
 *
 */
#ifndef NATIVE_POSIX_802154_CSMA_CA_ENABLED
#define NATIVE_POSIX_802154_CSMA_CA_ENABLED 1
#endif

/**
 * @def NATIVE_POSIX_802154_CSMA_CA_MAX_BE
 *
 * The maximum value of the backoff exponent, BE, in the CSMA-CA algorithm
 * (see IEEE 802.15.4-2015: 6.2.5.1).
 *
 */
#ifndef NATIVE_POSIX_802154_CSMA_CA_MAX_BE
#define NATIVE_POSIX_802154_CSMA_CA_MAX_BE 5
#endif

/**
 * @}
 * @defgroup native_posix_802154_config_timeout ACK timeout feature configuration
 * @{
 */

/**
 * @def NATIVE_POSIX_802154_ACK_TIMEOUT_ENABLED
 *
 * Indicates whether the ACK timeout feature is to be enabled in the driver.
 *
 */
#ifndef NATIVE_POSIX_802154_ACK_TIMEOUT_ENABLED
#define NATIVE_POSIX_802154_ACK_TIMEOUT_ENABLED 1
#endif

/**
 * @def NATIVE_POSIX_802154_ACK_TIMEOUT_DEFAULT_TIMEOUT
 *
 * The default timeout in microseconds (us) for the ACK timeout feature.
 *
 */
#ifndef NATIVE_POSIX_802154_ACK_TIMEOUT_DEFAULT_TIMEOUT
#define NATIVE_POSIX_802154_ACK_TIMEOUT_DEFAULT_TIMEOUT 7000
#endif

/**
 * @def NATIVE_POSIX_802154_ACK_TIMEOUT_DEFAULT_TIMEOUT
 *
 * The default timeout in microseconds (us) for the precise ACK timeout feature.
 *
 */
#ifndef NATIVE_POSIX_802154_PRECISE_ACK_TIMEOUT_DEFAULT_TIMEOUT
#define NATIVE_POSIX_802154_PRECISE_ACK_TIMEOUT_DEFAULT_TIMEOUT 210
#endif

/**
 * @def NATIVE_POSIX_802154_MAX_ACK_IE_SIZE
 *
 * The maximum supported size of the 802.15.4-2015 IE header and content fields
 * in an Enh-Ack.
 *
 */
#ifndef NATIVE_POSIX_802154_MAX_ACK_IE_SIZE
#define NATIVE_POSIX_802154_MAX_ACK_IE_SIZE 8
#endif

/**
 * @}
 * @defgroup native_posix_802154_config_transmission Transmission start notification
 * feature configuration
 * @{
 */

/**
 * @def NATIVE_POSIX_802154_TX_STARTED_NOTIFY_ENABLED
 *
 * Indicates whether the notifications of the started transmissions are to be
 * enabled in the driver.
 *
 * @note This feature is enabled by default if the ACK timeout feature or
 *       CSMA-CA is enabled.
 *       These features depend on the notifications of the transmission start.
 */
#ifndef NATIVE_POSIX_802154_TX_STARTED_NOTIFY_ENABLED
#if NATIVE_POSIX_802154_ACK_TIMEOUT_ENABLED || NATIVE_POSIX_802154_CSMA_CA_ENABLED
#define NATIVE_POSIX_802154_TX_STARTED_NOTIFY_ENABLED 1
#else
#define NATIVE_POSIX_802154_TX_STARTED_NOTIFY_ENABLED 0
#endif
#endif /* NATIVE_POSIX_802154_TX_STARTED_NOTIFY_ENABLED */

/**
 *@}
 **/

#ifdef __cplusplus
}
#endif

#endif /* NATIVE_POSIX_802154_CONFIG_H__ */

/**
 *@}
 **/
