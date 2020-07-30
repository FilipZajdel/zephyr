/* Copyright (c) 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef NRF_802154_UTILS_H__
#define NRF_802154_UTILS_H__

#include <assert.h>
#include <stdint.h>

/**
 * @defgroup nrf_802154_utils Utils definitions used in the 802.15.4 driver
 * @{
 * @ingroup nrf_802154
 * @brief Definitions of utils used in the 802.15.4 driver.
 */

/**@brief RTC clock frequency. */
#define NRF_802154_RTC_FREQUENCY               32768UL

/**@brief Defines the number of microseconds in one second. */
#define NRF_802154_US_PER_S                    1000000ULL

/**@brief Number of bits to shift RTC_FREQUENCY and US_PER_S to achieve the division by greatest common divisor. */
#define NRF_802154_FREQUENCY_US_PER_S_GCD_BITS 6

/**@brief Ceil division helper. */
#define NRF_802154_DIVIDE_AND_CEIL(A, B)       (((A) + (B)-1) / (B))

/**@brief Macro to get the number of elements in an array.
 *
 * @param[in] X   Array.
 */
#define NUMELTS(X)             (sizeof((X)) / sizeof(X[0]))

/**
 *@}
 **/

#endif // NRF_802154_UTILS_H__
