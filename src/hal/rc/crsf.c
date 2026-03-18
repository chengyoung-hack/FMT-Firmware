/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "hal/rc/crsf.h"

#define CRSF_SYNC_BYTE      0xC8
#define CRSF_FRAME_LENGTH   0x18  // Total frame length including type and CRC

/* Frame timeout threshold in RT-Thread ticks (50ms) */
#define CRSF_FRAME_TIMEOUT  (RT_TICK_PER_SECOND / 20)

/*
 * CRSF protocol uses:
 * - 420000 baud rate
 * - 8 data bits
 * - No parity
 * - 1 stop bit (8N1)
 */

// Channel values range: 172 to 1811 (corresponds to 1000-2000us)
#define CRSF_CHANNEL_MIN  172
#define CRSF_CHANNEL_MAX  1811
#define CRSF_CHANNEL_CENTER ((CRSF_CHANNEL_MIN + CRSF_CHANNEL_MAX) / 2)

// Target range for output: 1000-2000us
#define CRSF_TARGET_MIN  1000.0f
#define CRSF_TARGET_MAX  2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define CRSF_SCALE_FACTOR ((CRSF_TARGET_MAX - CRSF_TARGET_MIN) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN))
#define CRSF_SCALE_OFFSET (int)(CRSF_TARGET_MIN - (CRSF_SCALE_FACTOR * CRSF_CHANNEL_MIN + 0.5f))

/**
 * @brief Calculate CRC for CRSF frame
 * 
 * @param crc CRC accumulator
 * @param byte new byte to add
 * @return updated CRC value
 */
static uint8_t crsf_crc8(const uint8_t* ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];                 // 先 XOR
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5; // 多项式 0xD5 (x^8 + x^7 + x^6 + x^4 + x^2 + 1)
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Decode CRSF channel data
 * 
 * CRSF packs 16 channels into 22 bytes of data.
 * Each channel is 11 bits (0-2047 range).
 * 
 * @param decoder CRSF decoder structure
 * @return true if decode successful
 */
static bool crsf_decode_channels(crsf_decoder_t* decoder)
{
    uint8_t* frame = &decoder->crsf_frame[2]; // Skip sync and length bytes
    
    // Decode 16 channels from 22 bytes
    // Each channel is 11 bits
    decoder->crsf_val[0]  = (frame[0] | (frame[1] << 8)) & 0x07FF;
    decoder->crsf_val[1]  = (frame[1] >> 3) | (frame[2] << 5) & 0x07FF;
    decoder->crsf_val[2]  = (frame[2] >> 6) | (frame[3] << 2) | ((frame[4] & 0x03) << 10) & 0x07FF;
    decoder->crsf_val[3]  = ((frame[4] & 0x0F) >> 2) | (frame[5] << 4) & 0x07FF;
    decoder->crsf_val[4]  = ((frame[5] & 0x0F) >> 4) | (frame[6] << 7) & 0x07FF;
    decoder->crsf_val[5]  = (frame[6] >> 1) | (frame[7] << 9) & 0x07FF;
    decoder->crsf_val[6]  = (frame[7] >> 4) | (frame[8] << 6) & 0x07FF;
    decoder->crsf_val[7]  = (frame[8] >> 7) | (frame[9] << 1) | ((frame[10] & 0x01) << 9) & 0x07FF;
    decoder->crsf_val[8]  = ((frame[10] & 0x03) >> 1) | (frame[11] << 3) & 0x07FF;
    decoder->crsf_val[9]  = ((frame[11] & 0x07) >> 5) | (frame[12] << 8) & 0x07FF;
    decoder->crsf_val[10] = (frame[12] >> 2) | (frame[13] << 10) & 0x07FF;
    decoder->crsf_val[11] = (frame[13] >> 5) | (frame[14] << 5) & 0x07FF;
    decoder->crsf_val[12] = (frame[14] >> 8) | (frame[15] << 7) & 0x07FF;
    decoder->crsf_val[13] = (frame[15] >> 4) | (frame[16] << 6) & 0x07FF;
    decoder->crsf_val[14] = (frame[16] >> 7) | (frame[17] << 2) | ((frame[18] & 0x03) << 10) & 0x07FF;
    decoder->crsf_val[15] = ((frame[18] & 0x0F) >> 2) | (frame[19] << 4) & 0x07FF;
    
    // Scale channel values from CRSF range (172-1811) to target range (1000-2000)
    for (int i = 0; i < MAX_CRSF_CHANNEL; i++) {
        // Constrain input values
        if (decoder->crsf_val[i] < CRSF_CHANNEL_MIN) {
            decoder->crsf_val[i] = CRSF_CHANNEL_MIN;
        } else if (decoder->crsf_val[i] > CRSF_CHANNEL_MAX) {
            decoder->crsf_val[i] = CRSF_CHANNEL_MAX;
        }
        
        // Scale to target range
        decoder->crsf_val[i] = (uint16_t)(decoder->crsf_val[i] * CRSF_SCALE_FACTOR + CRSF_SCALE_OFFSET);
    }
    
    decoder->rc_count = MAX_CRSF_CHANNEL;
    return true;
}

/**
 * @brief Decode CRSF frame
 * 
 * @param decoder CRSF decoder structure
 * @param frame_time timestamp of frame reception
 * @return true if frame decoded successfully
 */
static bool crsf_decode(crsf_decoder_t* decoder, uint32_t frame_time)
{
    uint8_t frame_length = decoder->frame_length;
    
    // Validate frame length
    if (frame_length < 4 || frame_length > CRSF_FRAME_SIZE_MAX) {
        decoder->crsf_frame_drops++;
        decoder->crsf_decode_state = CRSF_SEARCHING;
        return false;
    }
    
    // Verify CRC (last byte is CRC)
    // 帧结构：[SYNC=0x64][LEN][TYPE][DATA...][CRC]
    // LEN字节表示后续字节数（包括TYPE、DATA和CRC）
    // CRC计算范围：从Type字节开始，到DATA结束（不包括CRC字节）
    // 所以要计算的长度是 frame_length - 1（排除最后的CRC字节）
    uint8_t crc = crsf_crc8(&decoder->crsf_frame[2], frame_length - 1);
    uint8_t expected_crc = decoder->crsf_frame[frame_length + 1];
    
    if (crc != expected_crc) {
        decoder->crsf_frame_drops++;
        decoder->crsf_decode_state = CRSF_SEARCHING;
        return false;
    }
    
    // Check frame type - we're interested in RC data frames
    // Type 0x16 is CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    if (decoder->type == 0x16) {
        if (!crsf_decode_channels(decoder)) {
            decoder->crsf_frame_drops++;
            return false;
        }
        
        decoder->last_frame_time = frame_time;
        decoder->crsf_data_ready = 1;
        decoder->crsf_decode_state = CRSF_SEARCHING; // Reset to searching for next frame
        return true;
    }
    
    // Other frame types - just mark as synced but no RC data
    decoder->crsf_decode_state = CRSF_SEARCHING; // Reset to searching for next frame
    return false;
}

rt_err_t crsf_decoder_init(crsf_decoder_t* decoder)
{
    RT_ASSERT(decoder != NULL);
    
    // Initialize decoder structure
    decoder->rc_count = 0;
    decoder->max_channels = MAX_CRSF_CHANNEL;
    decoder->crsf_failsafe = false;
    decoder->crsf_frame_drop = false;
    decoder->crsf_frame_drops = 0;
    decoder->partial_frame_count = 0;
    decoder->last_rx_time = 0;
    decoder->last_frame_time = 0;
    decoder->crsf_data_ready = 0;
    decoder->crsf_lock = 0;
    decoder->crsf_decode_state = CRSF_SEARCHING;
    decoder->frame_length = 0;
    decoder->type = 0;
    
    // Initialize ring buffer
    decoder->crsf_rb = ringbuffer_create(256);
    if (decoder->crsf_rb == NULL) {
        return -RT_ERROR;
    }
    
    // Clear frame buffer
    rt_memset(decoder->crsf_frame, 0, sizeof(decoder->crsf_frame));
    rt_memset(decoder->crsf_val, 0, sizeof(decoder->crsf_val));
    
    return RT_EOK;
}

uint32_t crsf_input(crsf_decoder_t* decoder, const uint8_t* values, uint32_t size)
{
    uint32_t processed = 0;
    uint32_t current_time = rt_tick_get();
    
    for (uint32_t i = 0; i < size; i++) {
        uint8_t byte = values[i];
        
        switch (decoder->crsf_decode_state) {
        case CRSF_SEARCHING:
            // Look for sync byte (0x64)
            if (byte == CRSF_SYNC_BYTE) {
                decoder->crsf_frame[0] = byte;
                decoder->partial_frame_count = 1;
                decoder->last_rx_time = current_time;
                decoder->crsf_decode_state = CRSF_SYNCED;
            }
            break;
            
        case CRSF_SYNCED:
            // Collect frame bytes
            if (decoder->partial_frame_count == 1) {
                // Second byte is frame length (excluding sync and length bytes)
                decoder->frame_length = byte;
                decoder->crsf_frame[1] = byte;
                decoder->partial_frame_count = 2;
                decoder->last_rx_time = current_time;
                
                // Validate length
                if (decoder->frame_length == 0 || decoder->frame_length > (CRSF_FRAME_SIZE_MAX - 2)) {
                    decoder->crsf_decode_state = CRSF_SEARCHING;
                }
            } else if (decoder->partial_frame_count == 2) {
                // Third byte is frame type
                decoder->type = byte;
                decoder->crsf_frame[2] = byte;
                decoder->partial_frame_count = 3;
                decoder->last_rx_time = current_time;
            } else if (decoder->partial_frame_count < (decoder->frame_length + 2)) {
                // Collect remaining frame data + CRC
                decoder->crsf_frame[decoder->partial_frame_count] = byte;
                decoder->partial_frame_count++;
                decoder->last_rx_time = current_time;
                
                // Check if frame is complete
                if (decoder->partial_frame_count == (decoder->frame_length + 2)) {
                    // Frame complete, decode it
                    crsf_decode(decoder, current_time);
                }
            } else {
                // Should not happen, reset
                decoder->crsf_decode_state = CRSF_SEARCHING;
            }
            break;
        }
        
        processed++;
    }
    
    return processed;
}

bool crsf_update(crsf_decoder_t* decoder)
{
    uint32_t current_time = rt_tick_get();
    
    // Check for frame timeout when in SYNCED state
    if (decoder->crsf_decode_state == CRSF_SYNCED) {
        // If no data received for more than CRSF_FRAME_TIMEOUT ticks, reset to searching
        if ((current_time - decoder->last_rx_time) > CRSF_FRAME_TIMEOUT) {
            decoder->crsf_frame_drops++;
            decoder->crsf_decode_state = CRSF_SEARCHING;
            decoder->partial_frame_count = 0;
        }
    }
    
    // Return the data ready status
    return decoder->crsf_data_ready;
}