/*
 * smrtctl.c
 *
 *  Created on: Nov 19, 2014
 *      Author: nick
 */

// C headers
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

// Kernel headers
#include <xdc/cfg/global.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

// driver headers
#include "drivers/xbee.h"
#include "outlets/outlet_tasks.h"

typedef enum {
	RX_HEADER,
	RX_PAYLOAD,
	GET_POWER,
	POWER_ON,
	POWER_OFF,
	IDLE
} SMRTCTL_State;

// FIXME: placeholder value
#define SMRTCTL_SOP				0x0F
#define SMRTCTL_HEADER_LENGTH	0x04
#define SMRTCTL_DEVICE_ID		0x02

void SMRTCTL_task  ( UArg arg0, UArg arg1 )
{
	SMRTCTL_State 	 state;
	OutletTask		*task;
	size_t			 buf_len, read;
	uint16_t		 data_len;
	uint8_t			 buf[32];
	uint8_t			*buf_tail;

	// initialization
	buf_tail = buf;
	buf_len = 0;
	state = IDLE;

	while (1) {

		switch (state) {
		case IDLE:
			buf_tail = buf;
			buf_len = XBEE_recv(buf_tail, 1);

			// if we read the SOP character, begin processing the header
			if (buf_len && (*buf_tail == SMRTCTL_SOP)) {
				state = RX_HEADER;
			}

			// yield the CPU if there was no data available
			if (!buf_len) {
				Task_yield();
			}

			break;

		case RX_HEADER:

			// prepare to receive header
			buf_len = 0;
			buf_tail = buf;

			//
			while (1) {
				read = XBEE_recv(buf_tail, 1);

				// if there isn't data, yield the CPU
				if (!read) {
					Task_yield();
					continue;
				}

				// if we encounter the SOP character, restart checking the header
				if (*buf_tail == SMRTCTL_SOP) {
					break;
				}

				// move along the tail pointer
				++buf_tail;
				++buf_len;

				// header fully received, ready for processing
				if (buf_len == SMRTCTL_HEADER_LENGTH) {
					/*
					 * TODO: Verify the header and that the packet belongs to us.
					 */

					// start receiving the payload
					state = RX_PAYLOAD;
					break;
				}
			}

			break;

		case RX_PAYLOAD:
			// ignore packets that aren't ours
			if (buf[0] != SMRTCTL_DEVICE_ID) {
				state = IDLE;
				break;
			}

			data_len = (uint16_t)buf[3] + ((uint16_t)buf[2]<<8);
			buf_len = 0;
			buf_tail = buf;

			// read characters into our buffer
			while (data_len) {
				read = XBEE_recv(buf_tail, 1);

				// if there isn't data, yield the CPU
				if (!read) {
					Task_yield();
					continue;
				}

				// move along the tail pointer
				++buf_tail;
				++buf_len;
				--data_len;

				// we've reached the end of our buffer, something went wrong
				if (buf_tail == buf + sizeof(buf)) {
					state = IDLE;
					break;
				}
			}

			// all payloads must be at least 1 character long
			if (buf_len < 1) {
				state = IDLE;
				break;
			}

			// the first byte is the opcode
			switch (buf[0]) {
			// power on
			case 0x11:
				state = POWER_ON;
				break;

			// power off
			case 0x22:
				state = POWER_OFF;
				break;

			// power on
			case 0x33:
				state = GET_POWER;
				break;
			}

			break;

		case POWER_ON:
			task = (OutletTask*)malloc(sizeof(OutletTask));
			task->target = buf[1];
			task->action = OUTLET_on;

			// new task
			Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
			Queue_enqueue(outletTask_queue, (Queue_Elem*)task);
			Semaphore_post(taskQueue_mutex);

			// signal there is a new task
			Semaphore_post(outletTaskQueue_sem);

			state = IDLE;

			break;

		case POWER_OFF:
			task = (OutletTask*)malloc(sizeof(OutletTask));
			task->target = buf[1];
			task->action = OUTLET_off;

			// new task
			Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
			Queue_enqueue(outletTask_queue, (Queue_Elem*)task);
			Semaphore_post(taskQueue_mutex);

			// signal there is a new task
			Semaphore_post(outletTaskQueue_sem);

			state = IDLE;

			break;

		case GET_POWER:
			task = (OutletTask*)malloc(sizeof(OutletTask));
			task->target = buf[1];
			task->action = OUTLET_get_power;

			// new task
			Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
			Queue_enqueue(outletTask_queue, (Queue_Elem*)task);
			Semaphore_post(taskQueue_mutex);

			// signal there is a new task
			Semaphore_post(outletTaskQueue_sem);

			state = IDLE;

			break;
		}
	}
}

void SMRTCTL_tx_power ( uint32_t power ) {
	uint8_t packet[9] = { 0x0F, 0x02, 0xA1, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04 };

	// fill packet
	packet[5] = (uint8_t)power;
	packet[6] = (uint8_t)(power>>8);
	packet[7] = (uint8_t)(power>>16);

	// write packet to XBee transmit buffer
	Semaphore_pend(xbeeTxBuffer_mutex, BIOS_WAIT_FOREVER);
	Buffer32_write(XBEE_tx_buffer, packet, sizeof(packet));
	Semaphore_post(xbeeTxBuffer_mutex);

	// notify the handler thread
	Semaphore_post(xbeeTxTask_sem);
}
