#include "rolling_buffer.h"
#include <stdlib.h>


void init_mic_buffer(mic_buffer *buf) {
    buf->index = 0;
    buf->buffer_power = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buf->mic_vals[i] = 0;
    }
}

void push_to_buffer(mic_buffer *buf, uint16_t new_val) {
    static int32_t dc_offset = 0;
    const uint16_t ALPHA_NUM = 995;
    const uint16_t ALPHA_DEN = 1000;
    
    int32_t filtered_val = (int32_t)new_val - dc_offset;
    dc_offset = (dc_offset * ALPHA_NUM + new_val * (ALPHA_DEN - ALPHA_NUM)) / ALPHA_DEN;
    
    uint32_t new_power = (uint32_t)filtered_val * filtered_val;
    
    buf->buffer_power -= buf->mic_vals[buf->index];
    buf->mic_vals[buf->index] = new_power;
    buf->buffer_power += new_power;
    
    buf->index = (buf->index + 1);
}

uint64_t get_average_power(mic_buffer *buf) {
    return buf->buffer_power >> BUFFER_SIZE_BITS;
}


