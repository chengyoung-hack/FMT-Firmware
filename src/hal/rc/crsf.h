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

#ifndef CRSF_H__
#define CRSF_H__

#include <firmament.h>
#include <stdint.h>

#include "module/utils/ringbuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CRSF_CHANNEL     16
#define CRSF_FRAME_SIZE      26
#define CRSF_FRAME_SIZE_MAX  64
#define CRSF_ADDRESS_BROADCAST 0x00
#define CRSF_ADDRESS_USB       0x10
#define CRSF_ADDRESS_BLUETOOTH 0x12
#define CRSF_ADDRESS_WIFI      0x13
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RADIO_RECEIVER    0xEB

typedef enum {
    CRSF_SYNCED = 0,
    CRSF_SEARCHING = 1
} crsf_decode_state_t;

typedef struct {
    uint16_t rc_count;
    uint16_t max_channels;
    bool crsf_failsafe;
    bool crsf_frame_drop;
    uint32_t crsf_frame_drops;
    uint32_t partial_frame_count;
    uint32_t last_rx_time;
    uint32_t last_frame_time;
    bool crsf_data_ready;
    uint8_t crsf_lock;
    crsf_decode_state_t crsf_decode_state;
    ringbuffer* crsf_rb;
    uint8_t crsf_frame[CRSF_FRAME_SIZE_MAX];
    uint16_t crsf_val[MAX_CRSF_CHANNEL];
    uint8_t frame_length;
    uint8_t type;
} crsf_decoder_t;

rt_inline void crsf_lock(crsf_decoder_t* decoder)
{
    decoder->crsf_lock = 1;
}

rt_inline void crsf_unlock(crsf_decoder_t* decoder)
{
    decoder->crsf_lock = 0;
}

rt_inline uint8_t crsf_islock(crsf_decoder_t* decoder)
{
    return decoder->crsf_lock;
}

rt_inline uint8_t crsf_data_ready(crsf_decoder_t* decoder)
{
    return decoder->crsf_data_ready;
}

rt_inline void crsf_data_clear(crsf_decoder_t* decoder)
{
    decoder->crsf_data_ready = 0;
}

rt_err_t crsf_decoder_init(crsf_decoder_t* decoder);
uint32_t crsf_input(crsf_decoder_t* decoder, const uint8_t* values, uint32_t size);
bool crsf_update(crsf_decoder_t* decoder);

#ifdef __cplusplus
}
#endif

#endif