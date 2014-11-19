/*
 * xbee.c
 *
 *  Created on: Nov 19, 2014
 *      Author: nick
 */


// C headers
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// Kernel headers
#include <xdc/cfg/global.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

// MSP430 header
#include "msp430fr5969.h"

// project headers
#include "sys/buffer.h"


// XBEE pin definitions
#define XBEE_SEL0	P2SEL0
#define XBEE_SEL1	P2SEL1
#define XBEE_RX 		BIT6
#define XBEE_TX 		BIT5

// XBEE serial register mappings onto USCI_A1
#define XBEECTLW0	UCA1CTLW0
#define XBEEBR0		UCA1BR0
#define XBEEBR1		UCA1BR1
#define XBEEMCTLW	UCA1MCTLW
#define XBEEIE		UCA1IE
#define XBEEIFG		UCA1IFG
#define XBEETXBUF	UCA1TXBUF
#define XBEERXBUF	UCA1RXBUF

// XBEE buffers
static Buffer32_t __xbee_tx_buffer;
static Buffer32_t __xbee_rx_buffer;

// Buffer pointers for external access
Buffer32_t *XBEE_tx_buffer = &__xbee_tx_buffer;
Buffer32_t *XBEE_rx_buffer = &__xbee_rx_buffer;

void XBEE_init ( )
{
	// initialize UART on USCI_A1
	XBEE_SEL0 |= XBEE_RX | XBEE_TX;
	XBEE_SEL1 &= ~(XBEE_RX | XBEE_TX);

	// reset USCI_A1
	XBEECTLW0 = UCSWRST;

	// set clock to 16MHz system clock
	XBEECTLW0 |= UCSSEL__SMCLK;

	// set modulation for 9600 baud
	XBEEBR0 = 104;
	XBEEBR1 = 0;
	XBEEMCTLW |= UCOS16 | UCBRF_2 | 0xD600;

	// enable USCI_A1
	XBEECTLW0 &= ~UCSWRST;

	// lock and initialize RX buffer
	Semaphore_pend(xbeeRxBuffer_mutex, BIOS_WAIT_FOREVER);
	Buffer32_init(XBEE_rx_buffer);
	Semaphore_post(xbeeRxBuffer_mutex);

	// lock and initialize TX buffer
	Semaphore_pend(xbeeTxBuffer_mutex, BIOS_WAIT_FOREVER);
	Buffer32_init(XBEE_tx_buffer);
	Semaphore_post(xbeeTxBuffer_mutex);

	// TODO: configure XBEE for 115000 baud
}

void XBEE_send ( uint8_t *data, size_t len )
{
	// write data to buffer
	Semaphore_pend(xbeeTxBuffer_mutex, BIOS_WAIT_FOREVER);
	Buffer32_write(XBEE_tx_buffer, data, len);
	Semaphore_post(xbeeTxBuffer_mutex);

	// signal TX task to begin sending
	Semaphore_post(xbeeTxTask_sem);
}

size_t XBEE_recv ( uint8_t *buffer, size_t len )
{
	size_t read;

	// read data from buffer
	Semaphore_pend(xbeeRxBuffer_mutex, BIOS_WAIT_FOREVER);
	read = Buffer32_read(XBEE_rx_buffer, buffer, len);
	Semaphore_post(xbeeRxBuffer_mutex);

	return read;
}

void XBEE_TX_RX_IRQ ( )
{
	if (XBEEIFG&UCTXIFG) {
		// signal to TX thread to send a byte
		Semaphore_post(xbeeTxReady_sem);

		// clear pending interrupt request
		XBEEIFG &= ~UCTXIFG;
	}

	// signal XBEE RX thread to read a byte
	if (XBEEIFG&UCRXIFG) {
		Semaphore_post(xbeeRxReady_sem);

		// clear pending interrupt request
		XBEEIFG &= ~UCRXIFG;
	}
}

void XBEE_TX_task ( UArg arg0, UArg arg1 )
{
	uint8_t		*tx_head;
	size_t		 tx_len;
	uint8_t		 tx_buffer[32];

	// enable TX interrupts
	XBEEIE |= UCTXIE;

	while (1) {
		// wait for data to send
		Semaphore_pend(xbeeTxTask_sem, BIOS_WAIT_FOREVER);

		// copy data out of the buffer
		Semaphore_pend(xbeeTxBuffer_mutex, BIOS_WAIT_FOREVER);
		tx_len = Buffer32_read(XBEE_tx_buffer, tx_buffer, sizeof(tx_buffer));
		Semaphore_post(xbeeTxBuffer_mutex);

		tx_head = tx_buffer;
		while (tx_head < tx_buffer + tx_len) {
			// wait for hardware serial buffer to empty
			Semaphore_pend(xbeeTxReady_sem, BIOS_WAIT_FOREVER);

			XBEETXBUF = *tx_head++;
		}
	}
}

void XBEE_RX_task ( UArg arg0, UArg arg1 )
{
	// enable RX interrupts
	XBEEIE |= UCRXIE;

	while (1) {
		// wait for incoming data
		Semaphore_pend(xbeeRxReady_sem, BIOS_WAIT_FOREVER);

		// write data into buffer
		Semaphore_pend(xbeeRxBuffer_mutex, BIOS_WAIT_FOREVER);
		Buffer32_write_byte(XBEE_rx_buffer, XBEERXBUF);
		Semaphore_post(xbeeRxBuffer_mutex);
	}
}
