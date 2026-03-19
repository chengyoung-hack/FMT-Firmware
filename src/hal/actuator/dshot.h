/*
 * Copyright (c) 2024 The Firmament Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DSHOT_H__
#define DSHOT_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dshot速度定义 */
typedef enum {
    DSHOT150 = 150,
    DSHOT300 = 300,
    DSHOT600 = 600,
    DSHOT1200 = 1200
} dshot_speed_t;

/* Dshot命令定义 */
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,                    // 6
    DSHOT_CMD_SPIN_DIRECTION_1,            // 7
    DSHOT_CMD_SPIN_DIRECTION_2,            // 8
    DSHOT_CMD_3D_MODE_OFF,                 // 9
    DSHOT_CMD_3D_MODE_ON,                  // 10
    DSHOT_CMD_SETTINGS_REQUEST,            // 11
    DSHOT_CMD_SAVE_SETTINGS,               // 12
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,  // 20
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,// 21
    DSHOT_CMD_LED0_ON,                     // 22
    DSHOT_CMD_LED1_ON,                     // 23
    DSHOT_CMD_LED2_ON,                     // 24
    DSHOT_CMD_LED3_ON,                     // 25
    DSHOT_CMD_LED0_OFF,                    // 26
    DSHOT_CMD_LED1_OFF,                    // 27
    DSHOT_CMD_LED2_OFF,                    // 28
    DSHOT_CMD_LED3_OFF,                    // 29
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
    DSHOT_CMD_MAX = 47
} dshot_command_t;

/* Dshot配置结构体 */
struct dshot_configure {
    dshot_speed_t speed;    // Dshot速度
    rt_bool_t telem_req;    // 是否请求遥测
};

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_H__ */