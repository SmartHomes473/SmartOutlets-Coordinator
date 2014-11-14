/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty_min.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>


/* BIOS Header files */
#include <ti/sysbios/BIOS.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>


/* Board Header file */
#include "Board.h"

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1) {
	while (1) {
		Task_sleep((UInt) arg0);
		GPIO_toggle(Board_LED0);
	}
}

Void uartTaskFxn(UArg arg0, UArg arg1) {
//	const unsigned char hello[] = "Hello World!\n";
//	UART_Handle handle;
//	UART_Params params;
//
//	// Configure UART
//	UART_Params_init(&params);
//	params.baudRate = 9600;
//	params.writeDataMode = UART_DATA_BINARY;
//	params.writeMode = UART_MODE_BLOCKING;
//	params.readMode = UART_MODE_BLOCKING;
//	params.readDataMode = UART_DATA_BINARY;
//	params.readReturnMode = UART_RETURN_FULL;
//	params.readEcho = UART_ECHO_OFF;
//
//	// Open UART connection
//	handle = UART_open(Board_UART0, &params);
//
//	// Write message than sleep
//	while (1) {
//		UART_writePolling(handle, hello, sizeof(hello)-1);
//		Task_sleep(1000);
//	}
}

/*
 *  ======== main ========
 */
int main(void) {
	// Initialize clock to 16MHz
	CLOCK_init();

	/* Call board init functions. */
	Board_initGeneral();
	Board_initGPIO();

	/* Turn on user LED  */
	GPIO_write(Board_LED0, Board_LED_ON);

	/* Start BIOS */
	BIOS_start();

	return (0);
}
