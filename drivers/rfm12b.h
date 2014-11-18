/*
 * rfm12b.h
 *
 *  Created on: Nov 13, 2014
 *      Author: nick
 */

#ifndef RFM12B_H_
#define RFM12B_H_

#include <stddef.h>
#include <stdint.h>

#define RFM12B_MAX_PAYLOAD		30
#define RFM12B_PREAMBLE_LEN		4
#define RFM12B_POSTAMBLE_LEN	2
#define RFM12B_MAX_PACKET_LEN	36

void RFM12B_init ( );
void RFM12B_tx ( const uint8_t *data, size_t len );

#endif /* RFM12B_H_ */
