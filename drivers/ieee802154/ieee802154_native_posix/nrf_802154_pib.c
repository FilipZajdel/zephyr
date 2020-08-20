/* Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
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

/**
 * @file
 *   This file implements storage of PIB attributes in nRF 802.15.4 radio driver.
 *
 */

#include "nrf_802154_pib.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154_config.h"
#include "nrf_802154_const.h"
#include "nrf_802154_utils.h"
#include "bs_radio.h"

typedef struct
{
    int8_t               tx_power;                             ///< Transmit power.
    uint8_t              pan_id[PAN_ID_SIZE];                  ///< Pan Id of this node.
    uint8_t              short_addr[SHORT_ADDRESS_SIZE];       ///< Short Address of this node.
    uint8_t              extended_addr[EXTENDED_ADDRESS_SIZE]; ///< Extended Address of this node.
    bool                 promiscuous : 1;                      ///< Indicating if radio is in promiscuous mode.
    bool                 auto_ack    : 1;                      ///< Indicating if auto ACK procedure is enabled.
    bool                 pan_coord   : 1;                      ///< Indicating if radio is configured as the PAN coordinator.
    uint8_t              channel     : 5;                      ///< Channel on which the node receives messages.
} nrf_802154_pib_data_t;

// Static variables.
static nrf_802154_pib_data_t m_data; ///< Buffer containing PIB data.

void nrf_802154_pib_init(void)
{
    m_data.promiscuous = false;
    m_data.auto_ack    = true;
    m_data.pan_coord   = false;
    m_data.channel     = 11;

    memset(m_data.pan_id, 0xff, sizeof(m_data.pan_id));
    m_data.short_addr[0] = 0xfe;
    m_data.short_addr[1] = 0xff;
    memset(m_data.extended_addr, 0, sizeof(m_data.extended_addr));
}

bool nrf_802154_pib_promiscuous_get(void)
{
    return m_data.promiscuous;
}

void nrf_802154_pib_promiscuous_set(bool enabled)
{
    m_data.promiscuous = enabled;
}

bool nrf_802154_pib_auto_ack_get(void)
{
    return m_data.auto_ack;
}

void nrf_802154_pib_auto_ack_set(bool enabled)
{
    m_data.auto_ack = enabled;
}

bool nrf_802154_pib_pan_coord_get(void)
{
    return m_data.pan_coord;
}

void nrf_802154_pib_pan_coord_set(bool enabled)
{
    m_data.pan_coord = enabled;
}

uint8_t nrf_802154_pib_channel_get(void)
{
    return m_data.channel;
}

void nrf_802154_pib_channel_set(uint8_t channel)
{
    m_data.channel = channel;
}

void nrf_802154_pib_tx_power_set(int8_t dbm)
{
    m_data.tx_power = dbm;
}

const uint8_t * nrf_802154_pib_pan_id_get(void)
{
    return m_data.pan_id;
}

void nrf_802154_pib_pan_id_set(const uint8_t * p_pan_id)
{
    memcpy(m_data.pan_id, p_pan_id, PAN_ID_SIZE);
}

const uint8_t * nrf_802154_pib_extended_address_get(void)
{
    return m_data.extended_addr;
}

void nrf_802154_pib_extended_address_set(const uint8_t * p_extended_address)
{
    memcpy(m_data.extended_addr, p_extended_address, EXTENDED_ADDRESS_SIZE);
}

const uint8_t * nrf_802154_pib_short_address_get(void)
{
    return m_data.short_addr;
}

void nrf_802154_pib_short_address_set(const uint8_t * p_short_address)
{
    memcpy(m_data.short_addr, p_short_address, SHORT_ADDRESS_SIZE);
}
