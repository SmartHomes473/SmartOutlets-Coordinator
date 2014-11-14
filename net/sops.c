/*
 * sops.c
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

// C headers
#include <stddef.h>
#include <stdint.h>

// kernel headers
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

// project headers
#include "smartoutlets_cfg.h"
#include "net/sops.h"


extern Semaphore_Handle sopsMessageId_sem;
static uint8_t __next_message_id ( void );


int SOPS_make_packet ( uint8_t dest, uint8_t opcode, uint8_t payload_len, uint8_t* buffer, size_t len )
{
	SOPS_header_t *header;
	uint8_t cksum;

	if (len < SOPS_HEADER_LEN) {
		return -1;
	}

	header = (SOPS_header_t*)buffer;

	// populate header
	header->proto_ident = SOPS_PROTO_IDENT;
	header->dest = dest;
	header->src = CONFIG_NET_ID;
	header->msg_id = __next_message_id();
	header->opcode = opcode;
	header->payload_len = payload_len;


	// calculate checksum
	cksum = SOPS_PROTO_IDENT + dest + CONFIG_NET_ID + opcode + payload_len + header->msg_id;
	cksum = (uint8_t)0xFF - cksum;
	header->cksum = cksum;

	return 0;
}

static uint8_t __next_message_id ( void )
{
	static uint8_t next_id = 0;
	uint8_t ret_id;

	// synchronization for thread safety
	Semaphore_pend(sopsMessageId_sem, BIOS_WAIT_FOREVER);

	ret_id = next_id++;

	Semaphore_post(sopsMessageId_sem);

	return ret_id;
}
