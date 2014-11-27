/*
 * outlet_tasks.c
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

#include "outlets/outlet_tasks.h"
#include "drivers/rfm12b.h"
#include "net/sops.h"
#include "net/smrtctl.h"

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
	uint8_t ack[SOPS_ACK_LEN+5];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_OFF, 0, packet, sizeof(packet));

	do {
		// turn off outlet
		RFM12B_tx(packet, sizeof(packet));

		// re-transmit if we don't receive an ACK
		RFM12B_rx(ack, sizeof(ack));
		if (SOPS_decode(ack, sizeof(ack)) == ACK) {
			break;
		}
	} while (1);
}

void OUTLET_get_power ( OutletTask *task )
{
	uint32_t power;
	uint8_t packet[SOPS_HEADER_LEN];
	uint8_t resp[SOPS_POWER_LEN];

	// make packet
	SOPS_make_packet(task->target, SOPS_OUTLET_REQ_POWER, 0, packet, sizeof(packet));

	do {
		// request power state
		RFM12B_tx(packet, sizeof(packet));

		// read response
		RFM12B_rx(resp, sizeof(resp));

		// re-transmit if we don't receive a valid response
		if (SOPS_decode(resp, sizeof(resp)) == POWER) {
			break;
		}
	} while (1);

	// XXX: not sure about the endianess of the response, probably little endian
	power = resp[SOPS_HEADER_LEN] + ((uint32_t)resp[SOPS_HEADER_LEN+1]<<8) + ((uint32_t)resp[SOPS_HEADER_LEN+2]<<16);

	// send power to SMRTControl
	SMRTCTL_tx_power(power);
}
