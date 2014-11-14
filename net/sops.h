/*
 * sops.h
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

#ifndef SOPS_H_
#define SOPS_H_

#include <stdint.h>

typedef struct {
	uint16_t	proto_ident;
	uint8_t		dest;
	uint8_t		src;
	uint8_t		msg_id;
	uint8_t		opcode;
	uint8_t		payload_len;
	uint8_t		cksum;
} SOPS_header_t;

#define SOPS_HEADER_LEN sizeof(SOPS_header_t)
#define SOPS_PROTO_IDENT 0xDCDC

int SOPS_make_packet ( uint8_t dest, uint8_t opcode, uint8_t payload_len, uint8_t* buffer, size_t len );

// SOPS opcodes
#define SOPS_OUTLET_ON	0xAA
#define SOPS_OUTLET_OFF	0x55

#endif /* SOPS_H_ */
