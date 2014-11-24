/*
 * outlet_tasks.c
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

#include "outlets/outlet_tasks.h"
#include "drivers/rfm12b.h"
#include "net/sops.h"

void OUTLET_on ( OutletTask *task )
{
	uint8_t packet[SOPS_HEADER_LEN];
	uint8_t ack[SOPS_ACK_LEN];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_ON, 0, packet, sizeof(packet));

	do {
		// turn on outlet
		RFM12B_tx(packet, sizeof(packet));

		// re-transmit if we don't receive an ACK
		RFM12B_rx(ack, sizeof(ack));
		if (SOPS_decode(ack, sizeof(ack)) == ACK) {
			break;
		}
	} while (1);
}

void OUTLET_off ( OutletTask *task )
{
	uint8_t packet[SOPS_HEADER_LEN];
	uint8_t ack[SOPS_ACK_LEN];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_OFF, 0, packet, sizeof(packet));

	do {
		// turn off outlet
		RFM12B_tx ( packet, sizeof(packet) );

		// re-transmit if we don't receive an ACK
		RFM12B_rx(ack, sizeof(ack));
		if (SOPS_decode(ack, sizeof(ack)) == ACK) {
			break;
		}
	} while (1);
}

