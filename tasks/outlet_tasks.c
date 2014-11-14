/*
 * outlet_tasks.c
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

#include "tasks/outlet_tasks.h"
#include "drivers/rfm12b.h"
#include "net/sops.h"

void OUTLET_on ( OutletTask *task )
{
	uint8_t packet[SOPS_HEADER_LEN];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_ON, 0, packet, sizeof(packet));

	do {
		RFM12B_tx ( packet, sizeof(packet) );

		// TODO: listen for ACK and re-send

		break;
	} while (1);
}

void OUTLET_off ( OutletTask *task )
{
	uint8_t packet[SOPS_HEADER_LEN];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_OFF, 0, packet, sizeof(packet));

	do {
		RFM12B_tx ( packet, sizeof(packet) );

		// TODO: listen for ACK and re-send

		break;
	} while (1);
}

