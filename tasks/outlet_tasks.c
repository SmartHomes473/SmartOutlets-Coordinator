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
	uint8_t message[SOPS_HEADER_LEN+1];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_ON, 0, message, sizeof(message));

	do {
		RFM12B_tx ( message, sizeof(message) );

		// TODO: listen for ACK and re-send

		break;
	} while (1);
}


