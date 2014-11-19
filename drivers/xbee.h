/*
 * xbee.h
 *
 *  Created on: Nov 19, 2014
 *      Author: nick
 */

#ifndef XBEE_H_
#define XBEE_H_

#include "sys/buffer.h"

extern Buffer32_t *XBEE_tx_buffer;
extern Buffer32_t *XBEE_rx_buffer;

void XBEE_init ( );
void XBEE_send ( uint8_t *buffer, size_t len );
size_t XBEE_recv ( uint8_t *buffer, size_t len );

#endif /* XBEE_H_ */
