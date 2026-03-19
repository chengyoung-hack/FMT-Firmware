/******************************************************************************
 * Copyright 2020-2021 The Firmament Authors. All Rights Reserved.
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
#include <firmament.h>

#include "hal/actuator/actuator.h"
#include "hal/actuator/dshot.h"

// #define DRV_DBG(...) console_printf(__VA_ARGS__)
#define DRV_DBG(...)

#ifdef RT_USING_DSHOT

#define DSHOT_TIM_FREQ            120000000
#define DSHOT150_BIT_0            214
#define DSHOT150_BIT_1            428
#define DSHOT300_BIT_0            107
#define DSHOT300_BIT_1            214
#define DSHOT600_BIT_0            54
#define DSHOT600_BIT_1            107
#define DSHOT_DMA_BUF_SIZE        18  /* 16位数据 + 2个停止位 */
#define DSHOT_MAX_CHANNEL         10
#define DSHOT_CMD_MOTOR_STOP      0

/* DMA中断处理函数 */
static void dshot_dma_tc_handler(uint8_t chan)
{
    dshot_pwm[chan].dma_trans_complete = 1;
}

/* DMA0中断处理函数 */
void DMA0_Channel1_IRQHandler(void)
{
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(DMA0, DMA_CH1, DMA_INT_FLAG_FTF)) {
        dshot_dma_tc_handler(0);
        dshot_dma_tc_handler(1);
        dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
    }
    rt_interrupt_leave();
}

void DMA0_Channel2_IRQHandler(void)
{
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(DMA0, DMA_CH2, DMA_INT_FLAG_FTF)) {
        dshot_dma_tc_handler(2);
        dshot_dma_tc_handler(3);
        dma_interrupt_flag_clear(DMA0, DMA_CH2, DMA_INT_FLAG_FTF);
    }
    rt_interrupt_leave();
}

void DMA0_Channel4_IRQHandler(void)
{
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(DMA0, DMA_CH4, DMA_INT_FLAG_FTF)) {
        dshot_dma_tc_handler(4);
        dshot_dma_tc_handler(5);
        dma_interrupt_flag_clear(DMA0, DMA_CH4, DMA_INT_FLAG_FTF);
    }
    rt_interrupt_leave();
}

void DMA0_Channel5_IRQHandler(void)
{
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(DMA0, DMA_CH5, DMA_INT_FLAG_FTF)) {
        dshot_dma_tc_handler(8);
        dma_interrupt_flag_clear(DMA0, DMA_CH5, DMA_INT_FLAG_FTF);
    }
    rt_interrupt_leave();
}

void DMA0_Channel6_IRQHandler(void)
{
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(DMA0, DMA_CH6, DMA_INT_FLAG_FTF)) {
        dshot_dma_tc_handler(6);
        dshot_dma_tc_handler(7);
        dshot_dma_tc_handler(9);
        dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_FTF);
    }
    rt_interrupt_leave();
}

typedef struct {
    uint32_t tim_periph;
    uint32_t tim_channel;
    uint32_t gpio_periph;
    uint32_t gpio_pin;
    uint32_t gpio_af;
    uint32_t dma_periph;
    dma_channel_enum dma_channel;
    uint32_t dma_subperi;
} dshot_channel_t;

typedef struct {
    uint16_t dshot_buff[DSHOT_DMA_BUF_SIZE];
    uint16_t* raw_buff;
    volatile uint8_t dma_trans_complete;
} dshot_item_t;

static dshot_item_t dshot_pwm[DSHOT_MAX_CHANNEL];

/* Dshot引脚配置：TIMER0 CH0-CH3, TIMER1 CH0/CH1/CH3, TIMER3 CH1-CH3 */
static const dshot_channel_t dshot_config[DSHOT_MAX_CHANNEL] = {
    /* Channel 0: PE9 - TIMER0 CH0 */
    { TIMER0, TIMER_CH_0, GPIOE, GPIO_PIN_9, GPIO_AF_1, DMA0, DMA_CH1, DMA_SUBPERI6 },
    /* Channel 1: PE11 - TIMER0 CH1 */
    { TIMER0, TIMER_CH_1, GPIOE, GPIO_PIN_11, GPIO_AF_1, DMA0, DMA_CH1, DMA_SUBPERI6 },
    /* Channel 2: PE13 - TIMER0 CH2 */
    { TIMER0, TIMER_CH_2, GPIOE, GPIO_PIN_13, GPIO_AF_1, DMA0, DMA_CH2, DMA_SUBPERI6 },
    /* Channel 3: PE14 - TIMER0 CH3 */
    { TIMER0, TIMER_CH_3, GPIOE, GPIO_PIN_14, GPIO_AF_1, DMA0, DMA_CH2, DMA_SUBPERI6 },
    /* Channel 4: PD13 - TIMER3 CH1 */
    { TIMER3, TIMER_CH_1, GPIOD, GPIO_PIN_13, GPIO_AF_2, DMA0, DMA_CH4, DMA_SUBPERI6 },
    /* Channel 5: PD14 - TIMER3 CH2 */
    { TIMER3, TIMER_CH_2, GPIOD, GPIO_PIN_14, GPIO_AF_2, DMA0, DMA_CH4, DMA_SUBPERI6 },
    /* Channel 6: PA3 - TIMER1 CH3 */
    { TIMER1, TIMER_CH_3, GPIOA, GPIO_PIN_3, GPIO_AF_1, DMA0, DMA_CH6, DMA_SUBPERI6 },
    /* Channel 7: PA15 - TIMER1 CH0 */
    { TIMER1, TIMER_CH_0, GPIOA, GPIO_PIN_15, GPIO_AF_1, DMA0, DMA_CH6, DMA_SUBPERI6 },
    /* Channel 8: PD15 - TIMER3 CH3 */
    { TIMER3, TIMER_CH_3, GPIOD, GPIO_PIN_15, GPIO_AF_2, DMA0, DMA_CH5, DMA_SUBPERI6 },
    /* Channel 9: PB3 - TIMER1 CH1 */
    { TIMER1, TIMER_CH_1, GPIOB, GPIO_PIN_3, GPIO_AF_1, DMA0, DMA_CH6, DMA_SUBPERI6 }
};

/* Dshot值位表 */
static const uint16_t dshot_bit0[DSHOT_MAX_CHANNEL] = {
    DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0,
    DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0,
    DSHOT300_BIT_0, DSHOT300_BIT_0
};

static const uint16_t dshot_bit1[DSHOT_MAX_CHANNEL] = {
    DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1,
    DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1,
    DSHOT300_BIT_1, DSHOT300_BIT_1
};

static void dshot_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);

    /* Configure PE9 PE11 PE13 PE14 (TIMER0 CH0 CH1 CH2 CH3) as alternate function */
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14);

    /* Configure PA15 PB3 PA3 (TIMER1 CH0 CH1 CH3) as alternate function */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_15);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3 | GPIO_PIN_15);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_3);

    /* Configure PD13 PD14 PD15 (TIMER3 CH1 CH2 CH3) as alternate function */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}

static void dshot_timer_init(void)
{
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocintpara;
    
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER3);
    
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    
    timer_deinit(TIMER0);
    timer_deinit(TIMER1);
    timer_deinit(TIMER3);
    
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 0xFFFF;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    
    timer_init(TIMER0, &timer_initpara);
    timer_init(TIMER1, &timer_initpara);
    timer_init(TIMER3, &timer_initpara);
    
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    
    /* Configure TIMER0 channels */
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);
    
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 0);
    
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
    /* Configure TIMER1 channels */
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocintpara);
    
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, 0);
    
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
    /* Configure TIMER3 channels */
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER3, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER3, TIMER_CH_3, &timer_ocintpara);
    
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 0);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, 0);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_3, 0);
    
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
    /* Enable timers */
    timer_primary_output_config(TIMER0, ENABLE);
    timer_primary_output_config(TIMER1, ENABLE);
    timer_primary_output_config(TIMER3, ENABLE);
}

static void dshot_dma_init(void)
{
    rcu_periph_clock_enable(RCU_DMA0);
    
    for (uint8_t i = 0; i < DSHOT_MAX_CHANNEL; i++) {
        dshot_pwm[i].raw_buff = dshot_pwm[i].dshot_buff;
        dshot_pwm[i].dma_trans_complete = 1;
        
        const dshot_channel_t* cfg = &dshot_config[i];
        
        dma_channel_disable(cfg->dma_periph, cfg->dma_channel);
        dma_flag_clear(cfg->dma_periph, cfg->dma_channel, DMA_FLAG_FTF);
        dma_flag_clear(cfg->dma_periph, cfg->dma_channel, DMA_FLAG_HTF);
        
        dma_single_data_parameter_struct dma_init_struct;
        dma_init_struct.periph_addr = (uint32_t)&TIMER_CH0CV(cfg->tim_periph) + (cfg->tim_channel * 4);
        dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        dma_init_struct.memory0_addr = (uint32_t)dshot_pwm[i].raw_buff;
        dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
        dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
        dma_init_struct.number = DSHOT_DMA_BUF_SIZE;
        dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
        dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
        
        dma_single_data_mode_init(cfg->dma_periph, cfg->dma_channel, &dma_init_struct);
        dma_channel_subperipheral_select(cfg->dma_periph, cfg->dma_channel, cfg->dma_subperi);
        
        dma_interrupt_enable(cfg->dma_periph, cfg->dma_channel, DMA_CHXCTL_FTFIE);
    }
}

static void dshot_prepare_dmabuffer(uint8_t chan, uint16_t value, bool telem_req)
{
    uint16_t* p_buff = dshot_pwm[chan].raw_buff;
    uint16_t packet = 0;
    
    /* 构建Dshot包: 11位油门值 + 1位遥测请求 + 4位CRC */
    packet = (value << 1) | (telem_req ? 1 : 0);
    
    /* 计算CRC: 高12位异或 */
    uint16_t csum = packet;
    csum ^= csum >> 8;
    csum ^= csum >> 4;
    csum &= 0xF;
    
    packet = (packet << 4) | csum;
    
    /* 填充DMA缓冲区 */
    for (uint8_t i = 0; i < 16; i++) {
        if (packet & (1 << (15 - i))) {
            p_buff[i] = dshot_bit1[chan];
        } else {
            p_buff[i] = dshot_bit0[chan];
        }
    }
    
    /* 添加停止位 */
    p_buff[16] = 0;
    p_buff[17] = 0;
}

static void dshot_dma_start(uint8_t chan)
{
    const dshot_channel_t* cfg = &dshot_config[chan];
    
    dshot_pwm[chan].dma_trans_complete = 0;
    
    /* 配置DMA传输 */
    dma_transfer_number_config(cfg->dma_periph, cfg->dma_channel, DSHOT_DMA_BUF_SIZE);
    dma_memory_address_config(cfg->dma_periph, cfg->dma_channel, DMA_MEMORY_0, (uint32_t)dshot_pwm[chan].raw_buff);
    dma_channel_enable(cfg->dma_periph, cfg->dma_channel);
    
    /* 配置定时器DMA触发源 */
    switch (cfg->tim_channel) {
        case TIMER_CH_0:
            timer_dma_enable(cfg->tim_periph, TIMER_DMA_CH0D);
            break;
        case TIMER_CH_1:
            timer_dma_enable(cfg->tim_periph, TIMER_DMA_CH1D);
            break;
        case TIMER_CH_2:
            timer_dma_enable(cfg->tim_periph, TIMER_DMA_CH2D);
            break;
        case TIMER_CH_3:
            timer_dma_enable(cfg->tim_periph, TIMER_DMA_CH3D);
            break;
    }
    timer_channel_output_mode_config(cfg->tim_periph, cfg->tim_channel, TIMER_OC_MODE_PWM0);
    timer_enable(cfg->tim_periph);
}

static rt_err_t dshot_config(actuator_dev_t dev, const struct actuator_configure* cfg)
{
    DRV_DBG("dshot configured: speed:%d\n", cfg->dshot_config.speed);
    return RT_EOK;
}

static rt_err_t dshot_control(actuator_dev_t dev, int cmd, void* arg)
{
    switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
        dshot_gpio_init();
        dshot_timer_init();
        dshot_dma_init();
        break;
    case ACT_CMD_CHANNEL_DISABLE:
        timer_disable(TIMER0);
        timer_disable(TIMER1);
        timer_disable(TIMER3);
        break;
    default:
        return RT_EINVAL;
    }
    return RT_EOK;
}

static rt_size_t dshot_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size)
{
    const rt_uint16_t* val_ptr = chan_val;
    bool telem_req = dev->config.dshot_config.telem_req;
    
    for (uint8_t i = 0; i < DSHOT_MAX_CHANNEL; i++) {
        if (chan_sel & (1 << i)) {
            uint16_t val = *val_ptr;
            
            /* 处理Dshot命令（值<48）和油门值（48-2047）*/
            if (val > 2047) {
                val_ptr++;
                continue;
            }
            
            /* 如果是停止命令，使用0值 */
            if (val == DSHOT_CMD_MOTOR_STOP) {
                val = 0;
            }
            
            dshot_prepare_dmabuffer(i, val, telem_req);
            dshot_dma_start(i);
            val_ptr++;
        }
    }
    
    /* 等待所有DMA传输完成 */
    for (uint8_t i = 0; i < DSHOT_MAX_CHANNEL; i++) {
        if (chan_sel & (1 << i)) {
            while (!dshot_pwm[i].dma_trans_complete) {
                /* 忙等待，实际应用中可能需要添加超时处理 */
            }
        }
    }
    
    return size;
}

const static struct actuator_ops __dshot_ops = {
    .act_config = dshot_config,
    .act_control = dshot_control,
    .act_read = NULL,
    .act_write = dshot_write
};

static struct actuator_device dshot_dev = {
    .chan_mask = 0x3FF,
    .range = { 48, 2047 },
    .config = {
        .protocol = ACT_PROTOCOL_DSHOT,
        .chan_num = DSHOT_MAX_CHANNEL,
        .pwm_config = { 0 },
        .dshot_config = { .speed = DSHOT300 } },
    .ops = &__dshot_ops
};

rt_err_t drv_dshot_init(void)
{
    return hal_actuator_register(&dshot_dev, "dshot_out", RT_DEVICE_FLAG_RDWR, NULL);
}

#endif /* RT_USING_DSHOT */