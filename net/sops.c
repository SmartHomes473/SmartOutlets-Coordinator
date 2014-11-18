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
	uint8_t msg_id, cksum;

	if (len < SOPS_HEADER_LEN) {
		return -1;
	}

	// generate message id
	msg_id = __next_message_id();

	// calculate checksum
	cksum = SOPS_PROTO_IDENT + dest + CONFIG_NET_ID + opcode + payload_len + msg_id;
	cksum = (uint8_t)0xFF - cksum;

	// fill buffer
	// TODO: try to use a SOPS header struct to map onto buffer
	buffer[0] = (uint8_t)(SOPS_PROTO_IDENT>>8);
	buffer[1] = (uint8_t)(SOPS_PROTO_IDENT&0XFF);
	buffer[2] = dest;
	buffer[3] = CONFIG_NET_ID;
	buffer[4] = __next_message_id();
	buffer[5] = opcode;
	buffer[6] = payload_len;

	cksum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];
	cksum = (uint8_t)0xFF - cksum;

	buffer[7] = cksum;

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
