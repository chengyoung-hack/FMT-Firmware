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
#include <firmament.h>

#include "hal/actuator/actuator.h"

// #define DRV_DBG(...) console_printf(__VA_ARGS__)
#define DRV_DBG(...)

#define PWM_FREQ_50HZ  (50)
#define PWM_FREQ_125HZ (125)
#define PWM_FREQ_250HZ (250)
#define PWM_FREQ_400HZ (400)

#define MAX_PWM_OUT_CHAN      10            // Main Out has 10 pwm channel
#define TIMER_FREQUENCY       2500000       // Timer frequency: 2.5M
#define PWM_DEFAULT_FREQUENCY PWM_FREQ_50HZ // pwm default frequency

#define VAL_TO_DC(_val, _freq) ((float)((_val) * (_freq)) / 1000000.0f)
#define DC_TO_VAL(_dc, _freq)  (1000000.0f / (_freq) * (_dc))

#define PWM_ARR(freq) (TIMER_FREQUENCY / freq) // CCR reload value

#ifdef RT_USING_DSHOT
/* DShot configuration */
#define DSHOT_TIM_FREQ            120000000
#define DSHOT150_BIT_0            214
#define DSHOT150_BIT_1            428
#define DSHOT300_BIT_0            107
#define DSHOT300_BIT_1            214
#define DSHOT600_BIT_0            54
#define DSHOT600_BIT_1            107
#define DSHOT_DMA_BUF_SIZE        18  /* 16位数据 + 2个停止位 */
#define DSHOT_CMD_MOTOR_STOP      0

/* DShot channel configuration structure */
typedef struct {
    uint32_t tim_periph;      /* Timer peripheral (TIMER0, TIMER1, TIMER3) */
    uint16_t tim_channel;     /* Timer channel (TIMER_CH_0, TIMER_CH_1, etc.) */
    uint32_t gpio_port;       /* GPIO port (GPIOA, GPIOB, GPIOD, GPIOE) */
    uint32_t gpio_pin;        /* GPIO pin */
    uint32_t gpio_af;         /* GPIO alternate function */
    uint32_t dma_periph;      /* DMA peripheral (DMA0) */
    dma_channel_enum dma_channel; /* DMA channel */
} dshot_channel_t;

/* DShot data structures - must be global for interrupt handlers */
/* Note: DMA_CH1, CH3, CH5, CH6 are used by USART, so we only use CH2, CH4, CH7 for DShot */
static const dshot_channel_t dshot_config[MAX_PWM_OUT_CHAN] = {
    /* Channel 0: PE9 - TIMER0 CH0 - DMA0_CH2 */
    { TIMER0, TIMER_CH_0, GPIOE, GPIO_PIN_9, GPIO_AF_1, DMA0, DMA_CH2 },
    /* Channel 1: PE11 - TIMER0 CH1 - DMA0_CH2 */
    { TIMER0, TIMER_CH_1, GPIOE, GPIO_PIN_11, GPIO_AF_1, DMA0, DMA_CH2 },
    /* Channel 2: PE13 - TIMER0 CH2 - DMA0_CH4 */
    { TIMER0, TIMER_CH_2, GPIOE, GPIO_PIN_13, GPIO_AF_1, DMA0, DMA_CH4 },
    /* Channel 3: PE14 - TIMER0 CH3 - DMA0_CH4 */
    { TIMER0, TIMER_CH_3, GPIOE, GPIO_PIN_14, GPIO_AF_1, DMA0, DMA_CH4 },
    /* Channel 4: PD13 - TIMER3 CH1 - DMA0_CH7 */
    { TIMER3, TIMER_CH_1, GPIOD, GPIO_PIN_13, GPIO_AF_2, DMA0, DMA_CH7 },
    /* Channel 5: PD14 - TIMER3 CH2 - DMA0_CH7 */
    { TIMER3, TIMER_CH_2, GPIOD, GPIO_PIN_14, GPIO_AF_2, DMA0, DMA_CH7 },
    /* Channel 6: PA3 - TIMER1 CH3 - DMA0_CH7 */
    { TIMER1, TIMER_CH_3, GPIOA, GPIO_PIN_3, GPIO_AF_1, DMA0, DMA_CH7 },
    /* Channel 7: PA15 - TIMER1 CH0 - DMA0_CH7 */
    { TIMER1, TIMER_CH_0, GPIOA, GPIO_PIN_15, GPIO_AF_1, DMA0, DMA_CH7 },
    /* Channel 8: PD15 - TIMER3 CH3 - DMA0_CH4 */
    { TIMER3, TIMER_CH_3, GPIOD, GPIO_PIN_15, GPIO_AF_2, DMA0, DMA_CH4 },
    /* Channel 9: PB3 - TIMER1 CH1 - DMA0_CH2 */
    { TIMER1, TIMER_CH_1, GPIOB, GPIO_PIN_3, GPIO_AF_1, DMA0, DMA_CH2 }
};

/* Dshot bit tables */
static uint16_t dshot_bit0[MAX_PWM_OUT_CHAN] = {
    DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0,
    DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0, DSHOT300_BIT_0,
    DSHOT300_BIT_0, DSHOT300_BIT_0
};

static uint16_t dshot_bit1[MAX_PWM_OUT_CHAN] = {
    DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1,
    DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1, DSHOT300_BIT_1,
    DSHOT300_BIT_1, DSHOT300_BIT_1
};

/* Dshot PWM data structure */
struct dshot_pwm_data {
    uint16_t raw_buff[DSHOT_DMA_BUF_SIZE];
    volatile uint8_t dma_trans_complete;
};
static struct dshot_pwm_data dshot_pwm[MAX_PWM_OUT_CHAN];

/* Forward declarations for DShot functions */
static void dshot_gpio_init(void);
static void dshot_timer_init(uint16_t speed);
static void dshot_dma_init(void);
static void dshot_prepare_dmabuffer(uint8_t chan, uint16_t value, bool telem_req);
static void dshot_dma_start(uint8_t chan);

/* DMA interrupt handler for DShot */
/* Note: DMA0_Channel1, Channel3, Channel5, Channel6 are used by USART, so we only handle Channel2, Channel4, Channel7 */
void DMA0_Channel2_IRQHandler(void)
{
    rt_interrupt_enter();
    if(dma_interrupt_flag_get(DMA0, DMA_CH2, DMA_INT_FLAG_FTF)) {
        /* Clear flag */
        dma_interrupt_flag_clear(DMA0, DMA_CH2, DMA_INT_FLAG_FTF);
        
        /* Mark all channels using DMA_CH2 as complete */
        for(uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
            if(dshot_config[i].dma_channel == DMA_CH2) {
                dshot_pwm[i].dma_trans_complete = 1;
            }
        }
    }
    rt_interrupt_leave();
}

void DMA0_Channel4_IRQHandler(void)
{
    rt_interrupt_enter();
    if(dma_interrupt_flag_get(DMA0, DMA_CH4, DMA_INT_FLAG_FTF)) {
        /* Clear flag */
        dma_interrupt_flag_clear(DMA0, DMA_CH4, DMA_INT_FLAG_FTF);
        
        /* Mark all channels using DMA_CH4 as complete */
        for(uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
            if(dshot_config[i].dma_channel == DMA_CH4) {
                dshot_pwm[i].dma_trans_complete = 1;
            }
        }
    }
    rt_interrupt_leave();
}

void DMA0_Channel7_IRQHandler(void)
{
    rt_interrupt_enter();
    if(dma_interrupt_flag_get(DMA0, DMA_CH7, DMA_INT_FLAG_FTF)) {
        /* Clear flag */
        dma_interrupt_flag_clear(DMA0, DMA_CH7, DMA_INT_FLAG_FTF);
        
        /* Mark all channels using DMA_CH7 as complete */
        for(uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
            if(dshot_config[i].dma_channel == DMA_CH7) {
                dshot_pwm[i].dma_trans_complete = 1;
            }
        }
    }
    rt_interrupt_leave();
}
#endif /* RT_USING_DSHOT */

static uint32_t __pwm_freq = PWM_DEFAULT_FREQUENCY;
static float __pwm_dc[MAX_PWM_OUT_CHAN];
static uint16_t __act_val[MAX_PWM_OUT_CHAN];
static uint8_t __current_protocol = ACT_PROTOCOL_PWM;

/* Forward declarations */
static rt_err_t act_config(actuator_dev_t dev, const struct actuator_configure* cfg);
static rt_err_t act_control(actuator_dev_t dev, int cmd, void* arg);
static rt_size_t act_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size);
static rt_size_t act_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size);

const static struct actuator_ops __act_ops = {
    .act_config = act_config,
    .act_control = act_control,
    .act_read = act_read,
    .act_write = act_write
};

static struct actuator_device act_dev = {
    .chan_mask = 0x3FF,
    .range = { 1000, 2000 },
    .config = {
        .protocol = ACT_PROTOCOL_PWM,
        .chan_num = MAX_PWM_OUT_CHAN,
        .pwm_config = { .pwm_freq = 50 },
        .dshot_config = { .speed = 300, .telem_req = false } },
    .ops = &__act_ops
};

/* PWM functions */
static void pwm_gpio_init(void)
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

static void __write_pwm(uint8_t chan_id, float dc)
{
    if (chan_id >= MAX_PWM_OUT_CHAN) {
        return;
    }

    __pwm_dc[chan_id] = dc;

    switch (chan_id) {
    case 0:
        /* TIMER0 CH0 */
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, (uint32_t)(TIMER_CAR(TIMER0) * dc));
        break;
    case 1:
        /* TIMER0 CH1 */
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, (uint32_t)(TIMER_CAR(TIMER0) * dc));
        break;
    case 2:
        /* TIMER0 CH2 */
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, (uint32_t)(TIMER_CAR(TIMER0) * dc));
        break;
    case 3:
        /* TIMER0 CH3 */
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, (uint32_t)(TIMER_CAR(TIMER0) * dc));
        break;
    case 4:
        /* TIMER1 CH0 */
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, (uint32_t)(TIMER_CAR(TIMER1) * dc));
        break;
    case 5:
        /* TIMER1 CH1 */
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, (uint32_t)(TIMER_CAR(TIMER1) * dc));
        break;
    case 6:
        /* TIMER1 CH3 */
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, (uint32_t)(TIMER_CAR(TIMER1) * dc));
        break;
    case 7:
        /* TIMER3 CH1 */
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, (uint32_t)(TIMER_CAR(TIMER3) * dc));
        break;
    case 8:
        /* TIMER3 CH2 */
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_2, (uint32_t)(TIMER_CAR(TIMER3) * dc));
        break;
    case 9:
        /* TIMER3 CH3 */
        timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_3, (uint32_t)(TIMER_CAR(TIMER3) * dc));
        break;
    default:
        break;
    }
}

static void __read_pwm(uint8_t chan_id, float* dc)
{
    if (chan_id >= MAX_PWM_OUT_CHAN) {
        return;
    }

    *dc = __pwm_dc[chan_id];
}

static rt_err_t __set_pwm_frequency(uint16_t freq)
{
    uint16_t arr;

    switch (freq) {
    case PWM_FREQ_50HZ:
    case PWM_FREQ_125HZ:
    case PWM_FREQ_250HZ:
    case PWM_FREQ_400HZ:
        break;
    default:
        /* invalid frequency */
        return RT_EINVAL;
    }

    __pwm_freq = freq;
    arr = PWM_ARR(__pwm_freq);

    /* configure timer reload value */
    timer_autoreload_value_config(TIMER0, arr - 1);
    timer_autoreload_value_config(TIMER1, arr - 1);
    timer_autoreload_value_config(TIMER3, arr - 1);

    /* the timer compare value should be re-configured */
    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        float dc = __pwm_dc[i];
        __write_pwm(i, dc);
    }

    return RT_EOK;
}

#ifdef RT_USING_DSHOT
/* DShot implementation functions */
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

static void dshot_timer_init(uint16_t speed)
{
    timer_oc_parameter_struct timer_ocintpara;
    uint16_t timer_psc;
    timer_parameter_struct timer_initpara;
    uint32_t tim_freq = DSHOT_TIM_FREQ;
    uint16_t bit0_val, bit1_val;

    /* Determine timing values based on speed */
    switch(speed) {
        case 150:
            bit0_val = DSHOT150_BIT_0;
            bit1_val = DSHOT150_BIT_1;
            break;
        case 300:
            bit0_val = DSHOT300_BIT_0;
            bit1_val = DSHOT300_BIT_1;
            break;
        case 600:
            bit0_val = DSHOT600_BIT_0;
            bit1_val = DSHOT600_BIT_1;
            break;
        default:
            bit0_val = DSHOT300_BIT_0;
            bit1_val = DSHOT300_BIT_1;
            break;
    }

    /* Update bit tables */
    for(int i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        dshot_bit0[i] = bit0_val;
        dshot_bit1[i] = bit1_val;
    }

    /* TIMER clock configuration */
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER3);

    /* Calculate prescaler for DShot timing */
    /* For DShot300: 120MHz / (1/(1.67us)) = 120MHz * 1.67us ≈ 200, so psc = 199 */
    /* But we use fixed values from original implementation */
    
    /* TIMER0 configuration (APB2 = 120MHz, TIMER0 clock = 240MHz) */
    timer_deinit(TIMER0);
    timer_struct_para_init(&timer_initpara);
    timer_psc = 199; /* For DShot300 timing */
    timer_initpara.prescaler         = timer_psc;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000; /* Large enough period */
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    /* Configure all channels in PWM mode 1 */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* CH0 configuration */
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration */
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration */
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration */
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(TIMER0);
    /* Keep timer disabled until DMA start */

    /* TIMER1 configuration */
    timer_deinit(TIMER1);
    timer_initpara.prescaler = timer_psc;
    timer_initpara.period = 1000;
    timer_init(TIMER1, &timer_initpara);

    /* CH0 configuration */
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration */
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration */
    timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(TIMER1);

    /* TIMER3 configuration (APB1 = 60MHz, TIMER3 clock = 120MHz) */
    timer_deinit(TIMER3);
    timer_psc = 99; /* Different prescaler for APB1 timers */
    timer_initpara.prescaler = timer_psc;
    timer_initpara.period = 1000;
    timer_init(TIMER3, &timer_initpara);

    /* CH1 configuration */
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration */
    timer_channel_output_config(TIMER3, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration */
    timer_channel_output_config(TIMER3, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(TIMER3);
}

static void dshot_dma_init(void)
{
    rcu_periph_clock_enable(RCU_DMA0);

    /* Configure DMA channels for DShot */
    for(uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        const dshot_channel_t* cfg = &dshot_config[i];
        uint32_t timer_ccr_addr;
        
        /* Get timer capture/compare register address based on channel */
        switch(cfg->tim_channel) {
            case TIMER_CH_0:
                timer_ccr_addr = (uint32_t)&TIMER_CH0CV(cfg->tim_periph);
                break;
            case TIMER_CH_1:
                timer_ccr_addr = (uint32_t)&TIMER_CH1CV(cfg->tim_periph);
                break;
            case TIMER_CH_2:
                timer_ccr_addr = (uint32_t)&TIMER_CH2CV(cfg->tim_periph);
                break;
            case TIMER_CH_3:
                timer_ccr_addr = (uint32_t)&TIMER_CH3CV(cfg->tim_periph);
                break;
            default:
                continue;
        }
        
        /* Deinitialize DMA channel */
        dma_deinit(cfg->dma_periph, cfg->dma_channel);
        
        /* Configure DMA channel */
        dma_periph_address_config(cfg->dma_periph, cfg->dma_channel, timer_ccr_addr);
        dma_memory_address_config(cfg->dma_periph, cfg->dma_channel, DMA_MEMORY_0, (uint32_t)dshot_pwm[i].raw_buff);
        dma_transfer_number_config(cfg->dma_periph, cfg->dma_channel, DSHOT_DMA_BUF_SIZE);
        
        /* Configure DMA mode */
        dma_memory_address_generation_config(cfg->dma_periph, cfg->dma_channel, DMA_MEMORY_INCREASE_ENABLE);
        dma_peripheral_address_generation_config(cfg->dma_periph, cfg->dma_channel, DMA_PERIPH_INCREASE_DISABLE);
        dma_memory_width_config(cfg->dma_periph, cfg->dma_channel, DMA_MEMORY_WIDTH_16BIT);
        dma_periph_width_config(cfg->dma_periph, cfg->dma_channel, DMA_PERIPH_WIDTH_16BIT);
        dma_priority_config(cfg->dma_periph, cfg->dma_channel, DMA_PRIORITY_ULTRA_HIGH);
        
        /* Configure DMA direction: memory to peripheral */
        dma_transfer_direction_config(cfg->dma_periph, cfg->dma_channel, DMA_MEMORY_TO_PERIPH);
        dma_circulation_disable(cfg->dma_periph, cfg->dma_channel);
        
        /* Select sub-peripheral */
        dma_channel_subperipheral_select(cfg->dma_periph, cfg->dma_channel, (dma_subperipheral_enum)cfg->dma_channel);
        
        /* Enable DMA channel interrupt */
        dma_interrupt_enable(cfg->dma_periph, cfg->dma_channel, DMA_INT_FLAG_FTF);
    }

    /* Enable DMA interrupts - only use channels not used by USART */
    /* DMA0_Channel1, Channel3, Channel5, Channel6 are used by USART */
    nvic_irq_enable(DMA0_Channel2_IRQn, 1, 0);
    nvic_irq_enable(DMA0_Channel4_IRQn, 1, 0);
    nvic_irq_enable(DMA0_Channel7_IRQn, 1, 0);
}

static void dshot_prepare_dmabuffer(uint8_t chan, uint16_t value, bool telem_req)
{
    uint16_t* p_buff = dshot_pwm[chan].raw_buff;
    uint16_t packet = 0;

    /* Handle Dshot commands (value < 48) and throttle values (48-2047) */
    if (value > 2047) {
        value = 2047;
    }
    if (value < 48 && value != DSHOT_CMD_MOTOR_STOP) {
        /* Invalid command, set to stop */
        value = DSHOT_CMD_MOTOR_STOP;
    }

    /* Build Dshot packet: 11-bit throttle + 1-bit telemetry request + 4-bit CRC */
    if (value == DSHOT_CMD_MOTOR_STOP) {
        packet = 0;
    } else {
        packet = (value << 1) | (telem_req ? 1 : 0);
        
        /* Calculate CRC: XOR high 12 bits */
        uint16_t csum = packet;
        csum ^= csum >> 8;
        csum ^= csum >> 4;
        csum &= 0xF;
        
        packet = (packet << 4) | csum;
    }

    /* Fill DMA buffer */
    for (uint8_t i = 0; i < 16; i++) {
        if (packet & (1 << (15 - i))) {
            p_buff[i] = dshot_bit1[chan];
        } else {
            p_buff[i] = dshot_bit0[chan];
        }
    }

    /* Add stop bits */
    p_buff[16] = 0;
    p_buff[17] = 0;
}

static void dshot_dma_start(uint8_t chan)
{
    const dshot_channel_t* cfg = &dshot_config[chan];

    dshot_pwm[chan].dma_trans_complete = 0;

    /* Configure DMA transfer */
    dma_transfer_number_config(cfg->dma_periph, cfg->dma_channel, DSHOT_DMA_BUF_SIZE);
    dma_memory_address_config(cfg->dma_periph, cfg->dma_channel, DMA_MEMORY_0, (uint32_t)dshot_pwm[chan].raw_buff);
    dma_channel_enable(cfg->dma_periph, cfg->dma_channel);

    /* Configure timer DMA trigger source */
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
#endif /* RT_USING_DSHOT */

static rt_err_t act_config(actuator_dev_t dev, const struct actuator_configure* cfg)
{
    if (cfg == NULL) {
        return RT_EINVAL;
    }

    __current_protocol = cfg->protocol;

    if (cfg->protocol == ACT_PROTOCOL_PWM) {
        DRV_DBG("actuator configured: protocol=pwm, freq=%d\n", cfg->pwm_config.pwm_freq);
        
        /* Configure PWM */
        pwm_gpio_init();
        
        /* Set PWM frequency */
        rt_err_t ret = __set_pwm_frequency(cfg->pwm_config.pwm_freq);
        if (ret != RT_EOK) {
            return ret;
        }
        
        /* Update device range for PWM */
        dev->range[0] = 1000;
        dev->range[1] = 2000;
        
    } else if (cfg->protocol == ACT_PROTOCOL_DSHOT) {
        DRV_DBG("actuator configured: protocol=dshot, speed=%d\n", cfg->dshot_config.speed);
        
#ifdef RT_USING_DSHOT
        /* Configure DShot */
        dshot_gpio_init();
        dshot_timer_init(cfg->dshot_config.speed);
        dshot_dma_init();
        
        /* Update device range for DShot */
        dev->range[0] = 48;
        dev->range[1] = 2047;
#else
        return RT_ENOSYS;
#endif
    } else {
        return RT_EINVAL;
    }

    /* Save configuration */
    dev->config.protocol = cfg->protocol;
    dev->config.chan_num = cfg->chan_num;
    dev->config.pwm_config = cfg->pwm_config;
    dev->config.dshot_config = cfg->dshot_config;

    return RT_EOK;
}

static rt_err_t act_control(actuator_dev_t dev, int cmd, void* arg)
{
    switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
        if (__current_protocol == ACT_PROTOCOL_PWM) {
            /* Enable all timers for PWM */
            timer_enable(TIMER0);
            timer_enable(TIMER1);
            timer_enable(TIMER3);
        }
        break;
    case ACT_CMD_CHANNEL_DISABLE:
        /* Disable all timers */
        timer_disable(TIMER0);
        timer_disable(TIMER1);
        timer_disable(TIMER3);
        break;
    case ACT_CMD_SET_PROTOCOL:
        {
            uint8_t* protocol = (uint8_t*)arg;
            if (*protocol == ACT_PROTOCOL_PWM || *protocol == ACT_PROTOCOL_DSHOT) {
                __current_protocol = *protocol;
                return RT_EOK;
            } else {
                return RT_EINVAL;
            }
        }
    default:
        return RT_EINVAL;
    }
    return RT_EOK;
}

static rt_size_t act_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size)
{
    if (size > MAX_PWM_OUT_CHAN) {
        size = MAX_PWM_OUT_CHAN;
    }

    for (rt_size_t i = 0; i < size; i++) {
        if (chan_sel & (1 << i)) {
            if (__current_protocol == ACT_PROTOCOL_PWM) {
                float dc;
                __read_pwm(i, &dc);
                chan_val[i] = (rt_uint16_t)(DC_TO_VAL(dc, __pwm_freq));
            } else {
                /* DShot doesn't support reading back values */
                chan_val[i] = 0;
            }
        }
    }

    return size;
}

static rt_size_t act_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size)
{
    if (size > MAX_PWM_OUT_CHAN) {
        size = MAX_PWM_OUT_CHAN;
    }

    const rt_uint16_t* val_ptr = chan_val;

    if (__current_protocol == ACT_PROTOCOL_PWM) {
        /* PWM output */
        for (rt_size_t i = 0; i < size; i++) {
            if (chan_sel & (1 << i)) {
                uint16_t val = *val_ptr;
                
                /* Clamp to valid range */
                if (val < 1000) val = 1000;
                if (val > 2000) val = 2000;
                
                float dc = VAL_TO_DC(val, __pwm_freq);
                __write_pwm(i, dc);
                val_ptr++;
            }
        }
    } else if (__current_protocol == ACT_PROTOCOL_DSHOT) {
#ifdef RT_USING_DSHOT
        /* DShot output */
        bool telem_req = dev->config.dshot_config.telem_req;
        
        for (rt_size_t i = 0; i < size; i++) {
            if (chan_sel & (1 << i)) {
                uint16_t val = *val_ptr;
                
                /* Handle DShot commands and throttle values */
                if (val > 2047) {
                    val = 2047;
                }
                
                dshot_prepare_dmabuffer(i, val, telem_req);
                dshot_dma_start(i);
                val_ptr++;
            }
        }
        
        /* Wait for all DMA transfers to complete */
        for (rt_size_t i = 0; i < size; i++) {
            if (chan_sel & (1 << i)) {
                while (!dshot_pwm[i].dma_trans_complete) {
                    /* Busy wait - in real applications, consider adding timeout */
                }
            }
        }
#else
        return 0;
#endif
    }

    return size;
}

rt_err_t drv_act_init(void)
{
    return hal_actuator_register(&act_dev, "main_out", RT_DEVICE_FLAG_RDWR, NULL);
}