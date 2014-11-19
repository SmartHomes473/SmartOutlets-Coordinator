/*
 * buffer.h
 *
 *  Created on: Nov 19, 2014
 *      Author: nick
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint8_t *head;
	uint8_t *tail;
	bool	 data_ready;
	uint8_t  buffer[32];
} Buffer32_t;

void Buffer32_init ( Buffer32_t *buffer );
size_t Buffer32_read ( Buffer32_t *buffer, uint8_t *dest, size_t len );
void Buffer32_write ( Buffer32_t *buffer, uint8_t *data, size_t len );
void Buffer32_write_byte ( Buffer32_t *buffer, uint8_t data );

#endif /* BUFFER_H_ */
