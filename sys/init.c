/*
 * init.c
 *
 *  Created on: Nov 13, 2014
 *      Author: nick
 */

#include "msp430fr5969.h"

void CLOCK_init ( )
{
	// FRAM wait state for >8MHz clock
	FRCTL0 = FRCTLPW | NWAITS_1;

	// unlock clock control register
	CSCTL0_H = CSKEY >> 8;

	// set DCO to 16MHz
	CSCTL1 = DCORSEL | DCOFSEL_4;

	// set SMCLK and MCLK to DCO
	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;

	// set all clock dividers
	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;

	// lock clock control register
	CSCTL0_H = 0;
}
