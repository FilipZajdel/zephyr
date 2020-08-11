#include "ieee802154_types.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>


typedef struct {
	to_bytes_t to_bytes;
} addr_any_t;

static int data_frame_to_bytes(void *data_frame, uint8_t *bytes)
{
	data_frame_t *frame = data_frame;
	int offset = 0;

	memcpy(bytes, frame->frame_control, 3);
	offset += 3;
	offset += ((addr_any_t *)frame->address)
			  ->to_bytes(frame->address, bytes + offset);

	/* omitting security - it's not supported now */
	memcpy(bytes + offset, frame->payload, frame->_payload_len);
	offset += frame->_payload_len;
	memcpy(bytes + offset, frame->fcs, 2);
	offset += 2;

	return offset;
}

static void data_frame_dtor(void **data_frame)
{
	data_frame_t *frame = *data_frame;

	if (frame->address) {
		free(frame->address);
	}
	if (frame->security) {
		free(frame->security);
	}
	if (frame->payload) {
		free(frame->security);
	}

	free(*data_frame);
}

static int ext_addr_to_bytes(void *paddr, uint8_t *buf)
{
	address_fields_ext_t *addr = paddr;

	memcpy(buf, addr->dest_pan_ID, 2);
	memcpy(buf + 2, addr->dest_addr, 8);
	memcpy(buf + 10, addr->src_pan_ID, 2);
	memcpy(buf + 12, addr->src_addr, 8);

	return 20; /* buf offset */
}

static int short_addr_to_bytes(void *paddr, uint8_t *buf)
{
	address_fields_short_t *addr = paddr;

	memcpy(buf, addr->dest_pan_ID, 2);
	memcpy(buf + 2, addr->dest_addr, 2);
	memcpy(buf + 4, addr->src_pan_ID, 2);
	memcpy(buf + 6, addr->src_addr, 2);

	return 8; /* buf offset */
}

void *frame_create(unsigned long frame_type, ...)
{
	va_list args;
	void *frame = NULL;

	switch (frame_type) {
	case FRAME_TYPE_DATA:
		va_start(args, frame_type);
		bool use_ext_addr = va_arg(args, unsigned long);
		uint8_t payload_len = va_arg(args, unsigned long);

		data_frame_t *data_frame = malloc(sizeof(data_frame_t));
		if (!data_frame) {
			break;
		}

		if (use_ext_addr) {
			address_fields_ext_t *addr =
				malloc(sizeof(address_fields_ext_t));
			if (!addr) {
				free(data_frame);
				break;
			}
			addr->to_bytes = ext_addr_to_bytes;
			data_frame->address = addr;
		} else {
			address_fields_short_t *addr =
				malloc(sizeof(address_fields_short_t));
			if (!addr) {
				free(data_frame);
				break;
			}
			addr->to_bytes = short_addr_to_bytes;
			data_frame->address = addr;
		}

		data_frame->payload = malloc(payload_len);
		if (!data_frame->payload) {
			free(data_frame->address);
			free(data_frame);
		}

		data_frame->to_bytes = data_frame_to_bytes;
		data_frame->_payload_len = payload_len;
		data_frame->dtor = data_frame_dtor;
		data_frame->security = NULL;
		FRAME_FCF_SET(*((uint16_t *)data_frame->frame_control),
			      FRAME_FCF_TYPE, FRAME_TYPE_DATA);
		frame = data_frame;
		break;
	default:
		printf("%ld frame type is not supported\n", frame_type);
	}

	va_end(args);
	return frame;
}

void frame_destroy(void **pframe)
{
	frame_any_t *frame = *pframe;
	frame->dtor(pframe);
}

int frame_to_bytes(void *frame, uint8_t *buf)
{
	frame_any_t *fframe = frame;
	if (!frame) {
		return -EINVAL;
	}

	return fframe->to_bytes(frame, buf);
}