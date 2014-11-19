/*
 * buffer.c
 *
 *  Created on: Nov 19, 2014
 *      Author: nick
 */

// C headers
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// project headers
#include "sys/buffer.h"

void Buffer32_init ( Buffer32_t *buffer )
{
	buffer->head = buffer->buffer;
	buffer->tail = buffer->tail;
	buffer->data_ready = false;
}

size_t Buffer32_read ( Buffer32_t *buffer, uint8_t *dest, size_t len )
{
	size_t read = 0;

	while ((buffer->head != buffer->tail) && read < len) {
		// read from head into buffer
		dest[read++] = *(buffer->head);

		// move head, wrapping if past end of buffer
		buffer->head = (++buffer->head == buffer->buffer + sizeof(buffer->buffer)) ?
				buffer->buffer : buffer->head;
	}

	// mark buffer as empty if there is no data left
	if (buffer->head == buffer->tail) {
		buffer->data_ready = false;
	}

	return read;
}

void Buffer32_write_byte ( Buffer32_t *buffer, uint8_t data )
{
	// write byte to buffer
	*buffer->tail = data;

	// move tail, wrapping if past end of buffer
	buffer->tail = (++buffer->tail == buffer->buffer + sizeof(buffer->buffer)) ?
			buffer->buffer : buffer->tail;

	// mark buffer has data
	buffer->data_ready = true;
}

void Buffer32_write ( Buffer32_t *buffer, uint8_t *data, size_t len )
{
	size_t wrote = 0;

	while (wrote < len) {
		// read from head into buffer
		*(buffer->tail) = data[wrote++];

		// move head, wrapping if past end of buffer
		buffer->tail = (++buffer->tail == buffer->buffer + sizeof(buffer->buffer)) ?
				buffer->buffer : buffer->tail;
	}

	// mark buffer has data
	buffer->data_ready = true;
}
