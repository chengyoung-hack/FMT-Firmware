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

// CRSF frame sync byte: 0xC8 is the standard sync byte for CRSF protocol
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

// CRSF原始值范围（协议标准）
#define CRSF_CHANNEL_MIN  172
#define CRSF_CHANNEL_MAX  1811

// 目标PWM范围（可自定义，此处按原意保持1000~2000）
#define CRSF_TARGET_MIN   1000
#define CRSF_TARGET_MAX   2000

// 改用整数运算宏（避免浮点）
#define CRSF_SCALE_NUMERATOR   (CRSF_TARGET_MAX - CRSF_TARGET_MIN)   // 1000
#define CRSF_SCALE_DENOMINATOR (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) // 1639



// Added: Low-pass filter configuration for smoothing
#define CRSF_FILTER_ALPHA  0.3f  // Filter coefficient (0.0-1.0, smaller = smoother)

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
    // 确保帧长度足够（至少 sync + len + type + 22 payload + crc）
    // 这里假设调用前已验证过，或者 decoder->crsf_frame 已指向完整帧
    
    // 指向有效载荷起始（跳过 sync[0], len[1], type[2]）
    uint8_t* payload = &decoder->crsf_frame[3];
    
    // 解码16个11位通道
    decoder->crsf_val[0]  = (payload[0] | (payload[1] << 8)) & 0x07FF;
    decoder->crsf_val[1]  = ((payload[1] >> 3) | (payload[2] << 5)) & 0x07FF;
    decoder->crsf_val[2]  = ((payload[2] >> 6) | (payload[3] << 2) | ((payload[4] & 0x03) << 10)) & 0x07FF;
    decoder->crsf_val[3]  = ((payload[4] >> 2) | (payload[5] << 6)) & 0x07FF;                     // 修正
    decoder->crsf_val[4]  = ((payload[5] >> 4) | (payload[6] << 4)) & 0x07FF;                     // 修正
    decoder->crsf_val[5]  = ((payload[6] >> 6) | (payload[7] << 2) | ((payload[8] & 0x01) << 10)) & 0x07FF; // 修正
    decoder->crsf_val[6]  = ((payload[8] >> 1) | (payload[9] << 7)) & 0x07FF;                     // 修正
    decoder->crsf_val[7]  = ((payload[9] >> 4) | (payload[10] << 4)) & 0x07FF;                    // 修正
    decoder->crsf_val[8]  = ((payload[10] >> 7) | (payload[11] << 1) | ((payload[12] & 0x03) << 9)) & 0x07FF; // 修正
    decoder->crsf_val[9]  = ((payload[12] >> 2) | (payload[13] << 6)) & 0x07FF;                   // 修正
    decoder->crsf_val[10] = ((payload[13] >> 5) | (payload[14] << 3)) & 0x07FF;                   // 修正
    decoder->crsf_val[11] = ((payload[14] >> 8) | (payload[15] << 5)) & 0x07FF;                   // 修正（>>8 实际为0，保留写法）
    decoder->crsf_val[12] = ((payload[15] >> 3) | (payload[16] << 5)) & 0x07FF;                   // 修正
    decoder->crsf_val[13] = ((payload[16] >> 6) | (payload[17] << 2) | ((payload[18] & 0x07) << 10)) & 0x07FF; // 修正
    decoder->crsf_val[14] = ((payload[18] >> 1) | (payload[19] << 7)) & 0x07FF;                   // 修正
    decoder->crsf_val[15] = ((payload[19] >> 4) | (payload[20] << 4)) & 0x07FF;                   // 修正
    
    // 缩放通道值（CRSF原始范围 172~1811 -> 目标范围 1000~2000）
    for (int i = 0; i < MAX_CRSF_CHANNEL; i++) {
        // 约束输入值到有效范围
        if (decoder->crsf_val[i] < CRSF_CHANNEL_MIN) {
            decoder->crsf_val[i] = CRSF_CHANNEL_MIN;
        } else if (decoder->crsf_val[i] > CRSF_CHANNEL_MAX) {
            decoder->crsf_val[i] = CRSF_CHANNEL_MAX;
        }
        
        // 缩放（注意你的 CRSF_SCALE_FACTOR 和 OFFSET 需与映射公式一致）
        // decoder->crsf_val[i] = (uint16_t)(decoder->crsf_val[i] * CRSF_SCALE_FACTOR + CRSF_SCALE_OFFSET);
        // 使用时：
        uint32_t temp = ((uint32_t)(decoder->crsf_val[i] - CRSF_CHANNEL_MIN) * CRSF_SCALE_NUMERATOR);
        decoder->crsf_val[i] = (uint16_t)((temp + CRSF_SCALE_DENOMINATOR/2) / CRSF_SCALE_DENOMINATOR + CRSF_TARGET_MIN);
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
            // Look for sync byte (0xC8)
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