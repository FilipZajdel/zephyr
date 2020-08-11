#ifndef _IEEE802154_TYPES_H
#define _IEEE802154_TYPES_H

#include <stdint.h>

#define FRAME_TYPE_DATA (0x2)

#define FRAME_FCF_TYPE (0)
#define FRAME_FCF_SEC_EN (3)
#define FRAME_FCF_FPB (4)
#define FRAME_FCF_AR (5)
#define FRAME_FCF_PAN_ID_COMP (6)
#define FRAME_FCF_DEST_ADDR_MODE (10)
#define FRAME_FCF_VER (12)
#define FRAME_FCF_SRC_ADDR_MODE (14)

#define FRAME_FCF_TYPE_MASK ((1UL << 0) | (1UL << 1) | (1UL << 2))
#define FRAME_FCF_SEC_EN_MASK (1UL << 3)
#define FRAME_FCF_FPB_MASK (1UL << 4)
#define FRAME_FCF_AR_MASK (1UL << 5)
#define FRAME_FCF_PAN_ID_COMP_MASK (1UL << 6)
#define FRAME_FCF_DEST_ADDR_MODE_MASK ((1UL << 10) | (1UL << 11))
#define FRAME_FCF_VER_MASK ((1UL << 12) | (1UL << 13))
#define FRAME_FCF_SRC_ADDR_MODE_MASK ((1UL << 14) | (1UL << 15))


#define FRAME_FCF_SET(fcf, BIT_NAME, value)                                    \
	do {                                                                   \
		uint16_t backup = fcf | (BIT_NAME##_MASK);                    \
		fcf &= ~(BIT_NAME##_MASK);                                    \
		fcf |= (value << BIT_NAME);                                   \
		fcf &= backup;                                                \
	} while (0);

#define FRAME_FCF_GET(fcf, BIT_NAME) (fcf & BIT_NAME##_MASK) >> BIT_NAME

typedef int (*to_bytes_t)(void *self, uint8_t *);
typedef void (*dtor_t)(void **self);

typedef struct __attribute__((packed)) {
	to_bytes_t to_bytes;
	uint8_t dest_pan_ID[2];
	uint8_t dest_addr[8];
	uint8_t src_pan_ID[2];
	uint8_t src_addr[8];
} address_fields_ext_t;

typedef struct __attribute__((packed)) {
	to_bytes_t to_bytes;
	uint8_t dest_pan_ID[2];
	uint8_t dest_addr[2];
	uint8_t src_pan_ID[2];
	uint8_t src_addr[2];
} address_fields_short_t;

typedef struct __attribute__((packed)) {
	dtor_t dtor;
	to_bytes_t to_bytes;

	uint8_t frame_control[2];
	uint8_t seq_number;
	void *address;
	void *security;
	uint8_t *payload;
	uint8_t _payload_len; /* Don't touch it */
	uint8_t fcs[2];
} data_frame_t;

typedef struct __attribute__((packed)) {
	dtor_t dtor;
	to_bytes_t to_bytes;
} frame_any_t;

/**
 * Creates the frame of frame_type
 * 
 * Arguments for frame types:
 * FRAME_TYPE_DATA:
 * 					-	(arg1) use_extended_addr (bool)
 * 					-	(arg2) payload_len		 (uint8_t)
 * 					-	(arg3) security_type	 (uint8_t) [not supported]
 */
void *frame_create(unsigned long frame_type, ...);
void frame_destroy(void **pframe);
int frame_to_bytes(void *frame, uint8_t *buf);

#endif /* _IEEE802154_TYPES_H */
