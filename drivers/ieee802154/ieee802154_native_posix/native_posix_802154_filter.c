/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *   This file implements incoming frame filtering according to 3 and 4 levels
 *   of filtering.
 *
 * Filtering details are specified in 802.15.4-2015: 6.7.2.
 * 1st and 2nd filtering level is performed by FSM module depending on
 * promiscuous mode, when FCS is received.
 */

#include "native_posix_802154_filter.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "native_posix_802154_const.h"
#include "native_posix_802154_frame_parser.h"
#include "native_posix_802154_pib.h"

#define FCF_CHECK_OFFSET (PHR_SIZE + FCF_SIZE)
#define PANID_CHECK_OFFSET (DEST_ADDR_OFFSET)
#define SHORT_ADDR_CHECK_OFFSET (DEST_ADDR_OFFSET + SHORT_ADDRESS_SIZE)
#define EXTENDED_ADDR_CHECK_OFFSET (DEST_ADDR_OFFSET + EXTENDED_ADDRESS_SIZE)

/**
 * @brief Check if given frame version is allowed for given frame type.
 *
 * @param[in] frame_type     Type of incoming frame.
 * @param[in] frame_version  Version of incoming frame.
 *
 * @retval true   Given frame version is allowed for given frame type.
 * @retval false  Given frame version is not allowed for given frame type.
 */
static bool frame_type_and_version_filter(uint8_t frame_type,
					  uint8_t frame_version)
{
	bool result;

	switch (frame_type) {
	case FRAME_TYPE_BEACON:
	case FRAME_TYPE_DATA:
	case FRAME_TYPE_ACK:
	case FRAME_TYPE_COMMAND:
		result = (frame_version != FRAME_VERSION_3);
		break;

	case FRAME_TYPE_MULTIPURPOSE:
		result = (frame_version == FRAME_VERSION_0);
		break;

	case FRAME_TYPE_FRAGMENT:
	case FRAME_TYPE_EXTENDED:
		result = true;
		break;

	default:
		result = false;
	}

	return result;
}

/**
 * @brief Check if given frame type may include destination address fields.
 *
 * @note Actual presence of destination address fields in the frame is
 * 		 indicated by FCF.
 *
 * @param[in] frame_type  Type of incoming frame.
 *
 * @retval true   Given frame type may include addressing fields.
 * @retval false  Given frame type may not include addressing fields.
 */
static bool dst_addressing_may_be_present(uint8_t frame_type)
{
	bool result;

	switch (frame_type) {
	case FRAME_TYPE_BEACON:
	case FRAME_TYPE_DATA:
	case FRAME_TYPE_ACK:
	case FRAME_TYPE_COMMAND:
	case FRAME_TYPE_MULTIPURPOSE:
		result = true;
		break;

	case FRAME_TYPE_FRAGMENT:
	case FRAME_TYPE_EXTENDED:
		result = false;
		break;

	default:
		result = false;
	}

	return result;
}

/**
 * @brief Get offset of end of addressing fields for given frame assuming its
 * 		  version is 2006.
 *
 * If given frame contains errors that prevent getting offset, this function
 * returns false. If there are no destination address fields in given frame,
 * this function returns true and does not modify @p p_num_bytes. If there is
 * destination address in given frame, this function returns true and inserts
 * offset of addressing fields end to @p p_num_bytes.
 *
 * @param[in]  p_data       Pointer to a buffer containing PHR and PSDU of the
 * 							incoming frame.
 * @param[out] p_num_bytes  Offset of addressing fields end.
 * @param[in]  frame_type   Type of incoming frame.
 *
 * @retval NATIVE_POSIX_802154_RX_ERROR_NONE               No errors in given frame were
 * 												  detected - it may be
 *                                                further processed.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR  The frame is valid but
 * 												  addressed to another node.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME      Detected an error in given
 * 												  frame - it should be
 *                                                discarded.
 */
static native_posix_802154_rx_error_t
dst_addressing_end_offset_get_2006(const uint8_t *p_data, uint8_t *p_num_bytes,
				   uint8_t frame_type)
{
	native_posix_802154_rx_error_t result;

	switch (p_data[DEST_ADDR_TYPE_OFFSET] & DEST_ADDR_TYPE_MASK) {
	case DEST_ADDR_TYPE_SHORT:
		*p_num_bytes = SHORT_ADDR_CHECK_OFFSET;
		result = NATIVE_POSIX_802154_RX_ERROR_NONE;
		break;

	case DEST_ADDR_TYPE_EXTENDED:
		*p_num_bytes = EXTENDED_ADDR_CHECK_OFFSET;
		result = NATIVE_POSIX_802154_RX_ERROR_NONE;
		break;

	case DEST_ADDR_TYPE_NONE:
		if (native_posix_802154_pib_pan_coord_get() ||
		    (frame_type == FRAME_TYPE_BEACON)) {
			switch (p_data[SRC_ADDR_TYPE_OFFSET] &
				SRC_ADDR_TYPE_MASK) {
			case SRC_ADDR_TYPE_SHORT:
			case SRC_ADDR_TYPE_EXTENDED:
				*p_num_bytes = PANID_CHECK_OFFSET;
				result = NATIVE_POSIX_802154_RX_ERROR_NONE;
				break;

			default:
				result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
			}
		} else {
			result = NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR;
		}

		break;

	default:
		result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
	}

	return result;
}

/**
 * @brief Get offset of end of addressing fields for given frame assuming its
 * 		  version is 2015.
 *
 * If given frame contains errors that prevent getting offset, this function
 * returns false. If there are no destination address fields in given frame,
 * this function returns true and does not modify @p p_num_bytes. If there is
 * destination address in given frame, this function returns true and inserts
 * offset of addressing fields end to @p p_num_bytes.
 *
 * @param[in]  p_data       Pointer to a buffer containing PHR and PSDU of the
 * 							incoming frame.
 * @param[out] p_num_bytes  Offset of addressing fields end.
 * @param[in]  frame_type   Type of incoming frame.
 *
 * @retval NATIVE_POSIX_802154_RX_ERROR_NONE               No errors in given frame were
 * 												  detected - it may be
 *                                                further processed.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR  The frame is valid but
 * 												  addressed to another node.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME      Detected an error in given
 * 												  frame - it should be
 *                                                discarded.
 */
static native_posix_802154_rx_error_t
dst_addressing_end_offset_get_2015(const uint8_t *p_data, uint8_t *p_num_bytes,
				   uint8_t frame_type)
{
	native_posix_802154_rx_error_t result;

	switch (frame_type) {
	case FRAME_TYPE_BEACON:
	case FRAME_TYPE_DATA:
	case FRAME_TYPE_ACK:
	case FRAME_TYPE_COMMAND: {
		uint8_t end_offset =
			native_posix_802154_frame_parser_dst_addr_end_offset_get(p_data);

		if (end_offset == NATIVE_POSIX_802154_FRAME_PARSER_INVALID_OFFSET) {
			result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
		} else {
			*p_num_bytes = end_offset;
			result = NATIVE_POSIX_802154_RX_ERROR_NONE;
		}
	} break;

	case FRAME_TYPE_MULTIPURPOSE:
		/** TODO: Implement dst addressing filtering according to 2015 spec */
		result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
		break;

	case FRAME_TYPE_FRAGMENT:
	case FRAME_TYPE_EXTENDED:
		/* No addressing data */
		result = NATIVE_POSIX_802154_RX_ERROR_NONE;
		break;

	default:
		result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
	}

	return result;
}

/**
 * @brief Get offset of end of addressing fields for given frame.
 *
 * If given frame contains errors that prevent getting offset, this function
 * returns false. If there are no destination address fields in given frame,
 * this function returns true and does not modify @p p_num_bytes. If there is
 * destination address in given frame, this function returns true and
 * inserts offset of addressing fields end to @p p_num_bytes.
 *
 * @param[in]  p_data       Pointer to a buffer containing PHR and PSDU of the
 * 							incoming frame.
 * @param[out] p_num_bytes  Offset of addressing fields end.
 * @param[in]  frame_type   Type of incoming frame.
 *
 * @retval NATIVE_POSIX_802154_RX_ERROR_NONE               No errors in given frame were
 * 												  detected - it may be
 *                                                further processed.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR  The frame is valid but
 * 												  addressed to another node.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME      Detected an error in given
 * 												  frame - it should be
 *                                                discarded.
 */
static native_posix_802154_rx_error_t
dst_addressing_end_offset_get(const uint8_t *p_data, uint8_t *p_num_bytes,
			      uint8_t frame_type, uint8_t frame_version)
{
	native_posix_802154_rx_error_t result;

	switch (frame_version) {
	case FRAME_VERSION_0:
	case FRAME_VERSION_1:
		result = dst_addressing_end_offset_get_2006(p_data, p_num_bytes,
							    frame_type);
		break;

	case FRAME_VERSION_2:
		result = dst_addressing_end_offset_get_2015(p_data, p_num_bytes,
							    frame_type);
		break;

	default:
		result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
	}

	return result;
}

/**
 * Verify if destination PAN Id of incoming frame allows processing by this
 * node.
 *
 * @param[in] p_panid     Pointer of PAN ID of incoming frame.
 * @param[in] frame_type  Type of the frame being filtered.
 *
 * @retval true   PAN Id of incoming frame allows further processing of the
 * 				  frame.
 * @retval false  PAN Id of incoming frame does not allow further processing.
 */
static bool dst_pan_id_check(const uint8_t *p_panid, uint8_t frame_type)
{
	bool result;

	if ((0 == memcmp(p_panid, native_posix_802154_pib_pan_id_get(), PAN_ID_SIZE)) ||
	    (0 == memcmp(p_panid, BROADCAST_ADDRESS, PAN_ID_SIZE))) {
		result = true;
	} else if ((FRAME_TYPE_BEACON == frame_type) &&
		   (0 == memcmp(native_posix_802154_pib_pan_id_get(), BROADCAST_ADDRESS,
				PAN_ID_SIZE))) {
		result = true;
	} else {
		result = false;
	}

	return result;
}

/**
 * Verify if destination short address of incoming frame allows processing by
 * this node.
 *
 * @param[in] p_dst_addr  Pointer of destination address of incoming frame.
 * @param[in] frame_type  Type of the frame being filtered.
 *
 * @retval true   Destination address of incoming frame allows further
 * 				  processing of the frame.
 * @retval false  Destination address of incoming frame does not allow further
 * 				  processing.
 */
static bool dst_short_addr_check(const uint8_t *p_dst_addr, uint8_t frame_type)
{
	bool result;

	if ((0 == memcmp(p_dst_addr, native_posix_802154_pib_short_address_get(),
			 SHORT_ADDRESS_SIZE)) ||
	    (0 == memcmp(p_dst_addr, BROADCAST_ADDRESS, SHORT_ADDRESS_SIZE))) {
		result = true;
	} else {
		printf("%s failed (dst)%x%x is not (my addr)%x%x\n", __func__, 
				p_dst_addr[1], p_dst_addr[0], 
				native_posix_802154_pib_short_address_get()[1], native_posix_802154_pib_short_address_get()[0]);
		result = false;
	}

	return result;
}

/**
 * Verify if destination extended address of incoming frame allows processing
 * by this node.
 *
 * @param[in] p_dst_addr  Pointer of destination address of incoming frame.
 * @param[in] frame_type  Type of the frame being filtered.
 *
 * @retval true   Destination address of incoming frame allows further
 * 				  processing of the frame.
 * @retval false  Destination address of incoming frame does not allow further
 * 				  processing.
 */
static bool dst_extended_addr_check(const uint8_t *p_dst_addr,
				    uint8_t frame_type)
{
	bool result;

	if (0 == memcmp(p_dst_addr, native_posix_802154_pib_extended_address_get(),
			EXTENDED_ADDRESS_SIZE)) {
		result = true;
	} else {
		result = false;
	}

	return result;
}

/**
 * Verify if destination addressing of incoming frame allows processing by this
 * node. This function checks addressing according to IEEE 802.15.4-2015.
 *
 * @param[in] p_data  Pointer to a buffer containing PHR and PSDU of the
 * 					  incoming frame.
 *
 * @retval NATIVE_POSIX_802154_RX_ERROR_NONE               Destination address of
 * 												  incoming frame allows further
 * 												  processing of the frame.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME      Received frame is invalid.
 * @retval NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR  Destination address of
 * 												  incoming frame does not allow
 * 												  further processing.
 */
static native_posix_802154_rx_error_t dst_addr_check(const uint8_t *p_data,
					    uint8_t frame_type)
{
	bool result;
	native_posix_802154_frame_parser_mhr_data_t mhr_data;

	result = native_posix_802154_frame_parser_mhr_parse(p_data, &mhr_data);

	if (!result) {
		return NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
	}

	if (mhr_data.p_dst_panid != NULL) {
		if (!dst_pan_id_check(mhr_data.p_dst_panid, frame_type)) {
			printf("dst_pan_id_check failed\n");
			return NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR;
		}
	}

	switch (mhr_data.dst_addr_size) {
	case SHORT_ADDRESS_SIZE:
		printf("dst short addr check\n");
		return dst_short_addr_check(mhr_data.p_dst_addr, frame_type) ?
			       NATIVE_POSIX_802154_RX_ERROR_NONE :
			       NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR;

	case EXTENDED_ADDRESS_SIZE:
		printf("dst extended addr check\n");
		return dst_extended_addr_check(mhr_data.p_dst_addr,
					       frame_type) ?
			       NATIVE_POSIX_802154_RX_ERROR_NONE :
			       NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR;

	case 0:
		/** Allow frames destined to the Pan Coordinator without destination
		 * address or beacon frames without destination address
		 */
		return (native_posix_802154_pib_pan_coord_get() ||
			(frame_type == FRAME_TYPE_BEACON)) ?
			       NATIVE_POSIX_802154_RX_ERROR_NONE :
			       NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR;

	default:
		assert(false);
	}

	return NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
}

native_posix_802154_rx_error_t native_posix_802154_filter_frame_part(const uint8_t *p_data,
						   uint8_t *p_num_bytes)
{
	native_posix_802154_rx_error_t result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
	uint8_t frame_type = p_data[FRAME_TYPE_OFFSET] & FRAME_TYPE_MASK;
	uint8_t frame_version =
		p_data[FRAME_VERSION_OFFSET] & FRAME_VERSION_MASK;

	switch (*p_num_bytes) {
	case FCF_CHECK_OFFSET:
		if (p_data[0] < IMM_ACK_LENGTH || p_data[0] > MAX_PACKET_SIZE) {
			result = NATIVE_POSIX_802154_RX_ERROR_INVALID_LENGTH;
			break;
		}

		if (!frame_type_and_version_filter(frame_type, frame_version)) {
			result = NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME;
			break;
		}

		if (!dst_addressing_may_be_present(frame_type)) {
			result = NATIVE_POSIX_802154_RX_ERROR_NONE;
			break;
		}

		result = dst_addressing_end_offset_get(
			p_data, p_num_bytes, frame_type, frame_version);
		break;

	default:
		result = dst_addr_check(p_data, frame_type);
		break;
	}
switch(result) {
case NATIVE_POSIX_802154_RX_ERROR_NONE: printf("NATIVE_POSIX_802154_RX_ERROR_NONE\n"); break;
/** Received a malformed frame. */
case NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME: printf("NATIVE_POSIX_802154_RX_ERROR_INVALID_FRAME\n"); break;
/** Received a frame with an invalid checksum. */
case NATIVE_POSIX_802154_RX_ERROR_INVALID_FCS: printf("NATIVE_POSIX_802154_RX_ERROR_INVALID_FCS\n"); break;
/** Received a frame with a mismatched destination address. */
case NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR: printf("NATIVE_POSIX_802154_RX_ERROR_INVALID_DEST_ADDR\n"); break;
/** Runtime error occurred (for example, CPU was held for too long). */
case NATIVE_POSIX_802154_RX_ERROR_RUNTIME: printf("NATIVE_POSIX_802154_RX_ERROR_RUNTIME\n"); break;
/** Radio timeslot ended during the frame reception. */
case NATIVE_POSIX_802154_RX_ERROR_TIMESLOT_ENDED: printf("NATIVE_POSIX_802154_RX_ERROR_TIMESLOT_ENDED\n"); break;
/** Procedure was aborted by another operation. */
case NATIVE_POSIX_802154_RX_ERROR_ABORTED: printf("NATIVE_POSIX_802154_RX_ERROR_ABORTED\n"); break;
/** Delayed reception request was rejected due to a denied timeslot request. */
case NATIVE_POSIX_802154_RX_ERROR_DELAYED_TIMESLOT_DENIED: printf("NATIVE_POSIX_802154_RX_ERROR_DELAYED_TIMESLOT_DENIED\n"); break;
/** Delayed reception timeslot ended. */
case NATIVE_POSIX_802154_RX_ERROR_DELAYED_TIMEOUT: printf("NATIVE_POSIX_802154_RX_ERROR_DELAYED_TIMEOUT\n"); break;
/** Received a frame with invalid length. */
case NATIVE_POSIX_802154_RX_ERROR_INVALID_LENGTH: printf("NATIVE_POSIX_802154_RX_ERROR_INVALID_LENGTH\n"); break;
/** Delayed operation in the ongoing state was aborted by other request. */
case NATIVE_POSIX_802154_RX_ERROR_DELAYED_ABORTED: printf("NATIVE_POSIX_802154_RX_ERROR_DELAYED_ABORTED\n"); break;
}
	return result;
}
