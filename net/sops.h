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
#define SOPS_ACK_LEN (SOPS_HEADER_LEN + 1)
#define SOPS_POWER_LEN (SOPS_HEADER_LEN + 3)

#define SOPS_PROTO_IDENT 0xDCDC
#define SOPS_PROTO_IDENT0 0xDC
#define SOPS_PROTO_IDENT1 0xDC

// Header offsets
#define SOPS_HEADER_ID0				0
#define SOPS_HEADER_ID1				1
#define SOPS_HEADER_DEST			2
#define SOPS_HEADER_SRC				3
#define SOPS_HEADER_MSGID			4
#define SOPS_HEADER_OPCODE			5
#define SOPS_HEADER_PAYLOAD_LEN		6
#define SOPS_HEADER_CKSUM			7

// SOPS opcodes
#define SOPS_OUTLET_ON	0xAA
#define SOPS_OUTLET_OFF	0x55
#define SOPS_OUTLET_ACK	0x00
#define SOPS_OUTLET_REQ_POWER 0x11
#define SOPS_OUTLET_RES_POWER 0x22


// SOPS packet types
typedef enum {
	ACK,
	POWER,
	INVALID
} PacketType;


int SOPS_make_packet ( uint8_t dest, uint8_t opcode, uint8_t payload_len, uint8_t* buffer, size_t len );
PacketType SOPS_decode ( uint8_t *packet, size_t len );

#endif /* SOPS_H_ */
