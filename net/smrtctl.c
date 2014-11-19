/*
 * smrtctl.c
 *
 *  Created on: Nov 19, 2014
 *      Author: nick
 */

// C headers
#include <stdint.h>
#include <stddef.h>

// Kernel headers
#include <xdc/cfg/global.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

// driver headers
#include "drivers/xbee.h"

typedef enum {
	RX_HEADER,
	RX_PAYLOAD,
	IDLE
} SMRTCTL_State;

// FIXME: placeholder value
#define SMRTCTL_SOP				0x11
#define SMRTCTL_HEADER_LENGTH	0x11

void SMRTCTL_task  ( UArg arg0, UArg arg1 )
{
	SMRTCTL_State 	 state;
	size_t			 buf_len, read;
	uint8_t			 buf[32];
	uint8_t			*buf_tail;

	// initialization
	buf_tail = buf;
	buf_len = 0;
	state = IDLE;

	while (1) {

		switch (state) {
		case IDLE:

			buf_len = XBEE_recv(buf_tail, 1);

			// if we read the SOP character, begin processing the header
			if (buf_len && *buf_tail == SMRTCTL_SOP) {
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
				}
			}

			break;

		case RX_PAYLOAD:

			/*
			 * TODO: Receive the payload
			 */

			// FIXME: remove this yield before implementing this state
			Task_yield();

			break;
		}
	}
}
