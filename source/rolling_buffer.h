/*
 * rolling_buffer.h
 *
 *  Created on: Apr 30, 2025
 *      Author: ezrar
 */

#ifndef ROLLING_BUFFER_H_
#define ROLLING_BUFFER_H_

#include <stdint.h>

#define BUFFER_SIZE 256
#define BUFFER_SIZE_BITS 8 

typedef struct {
	uint8_t index; //intentionally 8 bits so overflows after 256 back to 0
	uint32_t mic_vals[BUFFER_SIZE];
	uint64_t buffer_power;
} mic_buffer;

extern mic_buffer mic_data;

void init_mic_buffer(mic_buffer *buf);

void push_to_buffer(mic_buffer *buf, uint16_t new_val);

uint64_t get_average_power(mic_buffer *buf);

#endif 

