/*
 * rfm12b.c
 *
 *  Created on: Nov 13, 2014
 *      Author: nick
 */

#include "msp430fr5969.h"
#include "drivers/rfm12b.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// transmit buffer
static uint8_t rfm12b_tx_buffer[RFM12B_MAX_PACKET_LEN];
static uint8_t *rfm12b_tx_payload = rfm12b_tx_buffer + RFM12B_PREAMBLE_LEN;
static uint8_t *rfm12b_tx_head = rfm12b_tx_buffer;
static uint8_t *rfm12b_tx_tail = rfm12b_tx_buffer;

// receive buffer
static uint8_t rfm12b_rx_buffer[RFM12B_MAX_PACKET_LEN];
static uint8_t *rfm12b_rx_head = rfm12b_rx_buffer;
static uint8_t *rfm12b_rx_tail = rfm12b_rx_buffer;

extern Semaphore_Handle rfm12b_sem;

// SPI pin definitions
#define SPI_MISO	BIT7
#define SPI_MOSI	BIT6
#define SPI_SCLK	BIT2
#define SPI_CS		BIT5

// nIRQ pin definition
// XXX: this will likely change
#define NIRQ		BIT4

// RFM12B configuration definitions
#define RF_SYNC_BYTE1	0x2D
#define RF_SYNC_BYTE0	0xD4
#define RF_CONFIG_IDLE	0x80F8
#define RF_CONFIG_RX	0x80F8
#define RF_CONFIG_TX	0x80B8
#define RF_POWER_IDLE	0x8201
#define RF_POWER_RX		0x8281
#define RF_POWER_TX		0x8221
#define RF_CENTER_FREQ	0xA680
#define RF_DATA_RATE	0xC647
#define RF_RECV_CTL		0x94C5
#define RF_DATA_FILTER	0xC2EC
#define RF_FIFO_SYNC	0xCA83
#define RF_FIFO_RESET	0xCA81
#define RF_SYNC_MODE	0xCED4
#define RF_AFC_CMD		0xC483
#define RF_TX_CTL		0x9820
#define RF_PLL_CFG		0xCC77
#define RF_WAKEUP		0xE000
#define RF_DUTY_CYCLE	0xC800
#define RF_LOW_BATTERY	0xC000

// RFM12B commands
#define RF_FIFO_READ	0xB000
#define RF_STATUS_READ	0x1000


#define __spi_send_byte(BYTE) {\
	while (!(UCB0IFG&UCTXIFG));\
	UCB0TXBUF = BYTE;\
}


#define __spi_flush() {\
	while (UCB0STAT&UCBUSY);\
}


#define __spi_start() {\
	P1OUT &= ~SPI_CS;\
}


#define __spi_end() {\
	__spi_flush();\
	P1OUT |= SPI_CS;\
}


#define RFM12B_cmd(CMD) {\
	__spi_start();\
	__spi_send_byte((uint8_t)(CMD>>8));\
	__spi_send_byte((uint8_t)(CMD&0xFF));\
	__spi_end();\
	__delay_cycles(150);\
}

typedef enum {
	RX,
	TX,
	IDLE
} RFM12B_mode;

static RFM12B_mode rfm12b_mode = IDLE;

void RFM12B_init ( )
{
	// set USCI_B0 pins to SPI mode
	P1SEL1 |= SPI_MISO | SPI_MOSI;
	P2SEL1 |= SPI_SCLK;

	// setup chip select
	P1SEL1 &= ~SPI_CS;
	P1OUT |= SPI_CS;
	P1DIR |= SPI_CS;

	// setup nIRQ to receive interrupts
	P1SEL1 &= ~NIRQ;
	P1DIR &= ~NIRQ;

	// reset USCI_B0
	UCB0CTLW0 = UCSWRST;

	// configure as 3-wire 8-bit SPI master
	UCB0CTLW0 |= UCMST | UCSYNC | UCCKPH | UCMSB;

	// use subsystem clock
	UCB0CTLW0 |= UCSSEL__SMCLK;

	// set SPI clock speed to 2MHz (16MHz/8)
	UCB0BR0 = 0x08;
	UCB0BR1 = 0x00;

	// start SPI
	UCB0CTLW0 &= ~UCSWRST;

	// initialize the RFM12B in RX mode
	RFM12B_cmd(RF_CONFIG_RX);
	RFM12B_cmd(RF_POWER_RX);
	RFM12B_cmd(RF_CENTER_FREQ);
	RFM12B_cmd(RF_DATA_RATE);
	RFM12B_cmd(RF_RECV_CTL);
	RFM12B_cmd(RF_DATA_FILTER);
	RFM12B_cmd(RF_FIFO_SYNC);
	RFM12B_cmd(RF_SYNC_MODE);
	RFM12B_cmd(RF_AFC_CMD);
	RFM12B_cmd(RF_TX_CTL);
	RFM12B_cmd(RF_PLL_CFG);
	RFM12B_cmd(RF_WAKEUP);
	RFM12B_cmd(RF_DUTY_CYCLE);
	RFM12B_cmd(RF_LOW_BATTERY);
	RFM12B_cmd(RF_STATUS_READ);

	// initialize buffer
	rfm12b_tx_buffer[0] = 0xAA;
	rfm12b_tx_buffer[1] = 0xAA;
	rfm12b_tx_buffer[2] = RF_SYNC_BYTE1;
	rfm12b_tx_buffer[3] = RF_SYNC_BYTE0;
}


#define __rfm12b_irq_enable() {\
	P1IE |= NIRQ;\
}

#define __rfm12b_irq_clear() {\
	P1IFG &= ~NIRQ;\
}

#define __rfm12b_irq_init() {\
	P1DIR &= ~NIRQ;\
	P1IES |= NIRQ;\
	__rfm12b_irq_clear();\
}

#define __rfm12b_irq_disable() {\
	P1IE &= ~NIRQ;\
}


void RFM12B_tx ( const uint8_t *data, size_t len )
{
	// prepare TX buffer, copying at most RFM12B_MAX_PACKET_SIZE bytes
	len = (len > RFM12B_MAX_PAYLOAD) ? RFM12B_MAX_PAYLOAD : len;
	memcpy(rfm12b_tx_payload, data, len);

	// write the postamble
	memset(rfm12b_tx_payload+len, 0xAA, RFM12B_POSTAMBLE_LEN);

	// set head and tail
	rfm12b_tx_head = rfm12b_tx_buffer;
	rfm12b_tx_tail = rfm12b_tx_payload + len + RFM12B_POSTAMBLE_LEN;

	// initialize and clear pending interrupts on
	__rfm12b_irq_init();

	// enable TX mode and power on transmitter
	RFM12B_cmd(RF_STATUS_READ);
	RFM12B_cmd(RF_CONFIG_TX);
	RFM12B_cmd(RF_POWER_TX);
	rfm12b_mode = TX;

	// start SPI transaction
	__spi_start();

	// begin transmission
//	__spi_send_byte(0xB8);

	__rfm12b_irq_enable();

	// block until the transmission is finished
	Semaphore_pend(rfm12b_sem, BIOS_WAIT_FOREVER);

	// finish SPI transaction
	__spi_end();

	// switch to idle mode
	RFM12B_cmd(RF_CONFIG_RX);
	RFM12B_cmd(RF_POWER_RX);
	rfm12b_mode = IDLE;
}


#define __rfm12b_rx_irq_enable() {\
	P1IES &= ~NIRQ;\
	P1IFG &= ~NIRQ;\
	P1IE |= NIRQ;\
}

#define __rfm12b_rx_irq_disable() {\
	P1IE &= ~NIRQ;\
}

#define __rfm12b_rx_irq_clear() {\
	P1IFG &= ~NIRQ;\
}


size_t RFM12B_rx ( uint8_t *buffer, size_t len )
{
	size_t rx_len;

	// prepare the buffer
	rfm12b_rx_head = rfm12b_rx_buffer;
	rfm12b_rx_tail = rfm12b_rx_buffer + len;

	// enable RX mode and power on receiver
	RFM12B_cmd(RF_CONFIG_RX);
	RFM12B_cmd(RF_POWER_RX);

	// enable receive IRQ on nIRQ
	__rfm12b_rx_irq_enable();

	// timeout after 10ms
	// FIXME: this timeout value is bogus, we need to do
	//        some actual testing to get a reasonable value.
	Semaphore_pend(rfm12b_sem, 10);

	// copy from internal buffer to user supplied buffer
	rx_len = rfm12b_rx_tail - rfm12b_rx_buffer;
	rx_len = rx_len > len ? len : rx_len;
	memcpy(buffer, rfm12b_rx_buffer, rx_len);

	// return to idle state
	RFM12B_cmd(RF_CONFIG_IDLE);
	RFM12B_cmd(RF_CONFIG_IDLE);

	return rx_len;
}


void RFM12B_ISR ( void )
{
	if (!(P1IFG&NIRQ)) {
		P1IFG = 0;
		return;
	}

	// TODO: consolidate rfm12b_tx_buffer and rfm12b_rx_buffer????

	// TX IRQ
	switch (rfm12b_mode) {
	case TX:

		if (rfm12b_tx_head == rfm12b_tx_buffer) {
			__spi_send_byte(0xB8);
			__spi_flush();
			__delay_cycles(150);
		}

		// send current byte and increment head
		__spi_send_byte(*rfm12b_tx_head);
		++rfm12b_tx_head;

		// if we're at the end of the buffer
		if (rfm12b_tx_head == rfm12b_tx_tail) {
			// disable IRQ
			__rfm12b_irq_disable();

			// resume thread
			Semaphore_post(rfm12b_sem);
		}

		// clear pending interrupt
		__rfm12b_irq_clear();

		break;

	case RX:

		// read bytes
		RFM12B_cmd(RF_FIFO_READ);
		*rfm12b_rx_tail = UCB0RXBUF;

		++rfm12b_rx_tail;

		if (rfm12b_rx_head == rfm12b_rx_tail) {
			// disable IRQ
			__rfm12b_rx_irq_disable();

			// resume thread
			Semaphore_post(rfm12b_sem);
		}

		// clear the interrupt if NIRQ is satisfied
		if (P1IN&NIRQ) {
			__rfm12b_rx_irq_clear();
		}

		break;

	case IDLE:
	default:
		__rfm12b_irq_clear();
	}

	return;
}
