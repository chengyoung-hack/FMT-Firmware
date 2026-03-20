/******************************************************************************
 * Copyright 2020-2026 The Firmament Authors. All Rights Reserved.
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
#ifndef _DRV_ACT_H_
#define _DRV_ACT_H_

#include <firmament.h>

rt_err_t drv_act_init(void);

#endif
/******************************************************************************
 * Copyright 2020-2026 The Firmament Authors. All Rights Reserved.
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

#ifndef __DRV_ACT_H__
#define __DRV_ACT_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 执行器通道数量 */
#define ACTUATOR_CHANNEL_NUM      8

/* 执行器输出范围 */
#define ACTUATOR_OUTPUT_MIN       1000
#define ACTUATOR_OUTPUT_MAX       2000

/* 执行器控制结构体 */
struct actuator_ctrl {
    rt_uint16_t channel[ACTUATOR_CHANNEL_NUM];  /* 通道值 */
};

/* 执行器驱动操作接口 */
rt_err_t drv_act_init(void);
rt_err_t drv_act_set_output(const struct actuator_ctrl* ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ACT_H__ */