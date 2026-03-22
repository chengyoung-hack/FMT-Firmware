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
#define PWM_FREQ_100HZ (100)
#define PWM_FREQ_125HZ (125)
#define PWM_FREQ_250HZ (250)
#define PWM_FREQ_400HZ (400)

#define MAX_PWM_OUT_CHAN      4            // Main Out has 10 pwm channel
#define TIMER_FREQUENCY       2500000       // Timer frequency: 2.5M
#define PWM_DEFAULT_FREQUENCY PWM_FREQ_50HZ // pwm default frequency

#define VAL_TO_DC(_val, _freq) ((float)((_val) * (_freq)) / 1000000.0f)
#define DC_TO_VAL(_dc, _freq)  (1000000.0f / (_freq) * (_dc))

#define PWM_ARR(freq) (TIMER_FREQUENCY / freq) // CCR reload value

#ifdef RT_USING_DSHOT
/* DShot configuration */
#define DSHOT_TIM_FREQ            120000000
#define DSHOT_DMA_BUF_SIZE        18  /* 16 data bits + 2 stop bits */
#define DSHOT_CMD_MOTOR_STOP      0

/* DShot channel → DMA mapping (all on DMA1, SUBPERI6 = TIMER0 channels)
 * DMA request source: DMAS=1 → CHxDEN triggered by update event, not compare match.
 * This avoids the DMA1_CH5 conflict with UART0 RX that would occur with TIMER0_UP. */
typedef struct {
    dma_channel_enum dma_channel;
    uint32_t         timer_ccr_addr;
} dshot_dma_map_t;

static dshot_dma_map_t dshot_dma_map[MAX_PWM_OUT_CHAN] = {
    { DMA_CH1, 0 },  /* CH0: DMA1_CH1 SUBPERI6 (TIMER0_CH0) → TIMER_CH0CV */
    { DMA_CH2, 0 },  /* CH1: DMA1_CH2 SUBPERI6 (TIMER0_CH1) → TIMER_CH1CV */
    { DMA_CH6, 0 },  /* CH2: DMA1_CH6 SUBPERI6 (TIMER0_CH2) → TIMER_CH2CV */
    { DMA_CH4, 0 },  /* CH3: DMA1_CH4 SUBPERI6 (TIMER0_CH3) → TIMER_CH3CV */
};

/* DShot bit timing (populated by dshot_timer_init) */
static uint16_t dshot_bit0;   /* 37.5 % duty */
static uint16_t dshot_bit1;   /* 75.0 % duty */

/* Per-channel DMA buffer: 16 bits + 2 stop words = 18 half-words */
static uint16_t dshot_raw[MAX_PWM_OUT_CHAN][DSHOT_DMA_BUF_SIZE];

/* Track DMA completion for optional synchronisation */
static volatile uint8_t dshot_dma_done_mask;

/* Forward declarations */
static void dshot_gpio_init(void);
static void dshot_timer_init(uint16_t speed);
static void dshot_dma_init(void);
static void dshot_prepare_dmabuffer(uint8_t chan, uint16_t value, bool telem_req);
static void dshot_fire(void);

/* DMA completion ISR – shared by all 4 DShot DMA channels.
 * On transfer complete: disable the DMA channel and the corresponding
 * timer channel-DMA enable bit so that subsequent update events do not
 * trigger stray DMA requests. */
static void dshot_dma_irq_handler(dma_channel_enum dma_ch, uint16_t timer_dma_bit, uint8_t chan_mask_bit)
{
    if (dma_interrupt_flag_get(DMA1, dma_ch, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA1, dma_ch, DMA_INT_FLAG_FTF);
        dma_channel_disable(DMA1, dma_ch);
        timer_dma_disable(TIMER0, timer_dma_bit);
        dshot_dma_done_mask |= chan_mask_bit;
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    rt_interrupt_enter();
    dshot_dma_irq_handler(DMA_CH1, TIMER_DMA_CH0D, 0x01);
    rt_interrupt_leave();
}

void DMA1_Channel2_IRQHandler(void)
{
    rt_interrupt_enter();
    dshot_dma_irq_handler(DMA_CH2, TIMER_DMA_CH1D, 0x02);
    rt_interrupt_leave();
}

void DMA1_Channel4_IRQHandler(void)
{
    rt_interrupt_enter();
    dshot_dma_irq_handler(DMA_CH4, TIMER_DMA_CH3D, 0x08);
    rt_interrupt_leave();
}

void DMA1_Channel6_IRQHandler(void)
{
    rt_interrupt_enter();
    dshot_dma_irq_handler(DMA_CH6, TIMER_DMA_CH2D, 0x04);
    rt_interrupt_leave();
}
#endif /* RT_USING_DSHOT */

static uint32_t __pwm_freq = PWM_DEFAULT_FREQUENCY;
static float __pwm_dc[MAX_PWM_OUT_CHAN];
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

static void pwm_timer_init(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    uint16_t timer_psc;
    timer_parameter_struct timer_initpara;

    /* TIMER clock configuration */
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER3);

    /* When TIMERSEL is set, the TIMER clock is equal to CK_AHB */
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);

    /* Timer_PSC = CK_AHB / TARGET_TIMER_CK */
    timer_psc = rcu_clock_freq_get(CK_AHB) / TIMER_FREQUENCY - 1;

    /* Timer init parameter */
    timer_initpara.prescaler = timer_psc;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = PWM_ARR(__pwm_freq) - 1;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;

    /* Timer deinit */
    timer_deinit(TIMER0);
    timer_deinit(TIMER1);
    timer_deinit(TIMER3);

    /* Timer0 must enable primary output to enable pwm output */
    timer_primary_output_config(TIMER0, ENABLE);

    /* Timer init */
    timer_init(TIMER0, &timer_initpara);
    timer_init(TIMER1, &timer_initpara);
    timer_init(TIMER3, &timer_initpara);

    /* Timer channel configuration in PWM mode */
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* Timer0 channel configure */
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* Timer1 channel configure */
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER1, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* Timer3 channel configure */
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER3, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER3, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    timer_auto_reload_shadow_enable(TIMER1);
    timer_auto_reload_shadow_enable(TIMER3);
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
    case PWM_FREQ_100HZ:
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
    timer_parameter_struct timer_initpara;

    switch (speed) {
    case 150: case 300: case 600: break;
    default: speed = 300; break;
    }

    rcu_periph_clock_enable(RCU_TIMER0);
    /* Timer clock = CK_AHB = 240 MHz (MUL4 mode) */
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);

    uint16_t timer_psc = rcu_clock_freq_get(CK_AHB) / DSHOT_TIM_FREQ - 1;  /* =1, 120 MHz */
    uint16_t dshot_period = DSHOT_TIM_FREQ / (speed * 1000) - 1;            /* DShot300: 399 */
    uint16_t period_ticks = dshot_period + 1;

    dshot_bit0 = period_ticks * 3 / 8;   /* 37.5 % */
    dshot_bit1 = period_ticks * 3 / 4;   /* 75.0 % */

    timer_deinit(TIMER0);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = timer_psc;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = dshot_period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    /* OC parameters common to all channels */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* CH0 – CH3: PWM mode 0, OC shadow DISABLED.
     * DMA is triggered by update event (DMAS=1), so CCR is written
     * at the start of each period.  No preload needed. */
    for (uint16_t ch = TIMER_CH_0; ch <= TIMER_CH_3; ch++) {
        timer_channel_output_config(TIMER0, ch, &timer_ocintpara);
        timer_channel_output_mode_config(TIMER0, ch, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(TIMER0, ch, TIMER_OC_SHADOW_DISABLE);
    }

    timer_auto_reload_shadow_enable(TIMER0);
    timer_primary_output_config(TIMER0, ENABLE);

    /* DMAS = 1: CHxDEN DMA requests are sourced from the update event
     * instead of the channel compare-match.  This way four independent
     * DMA channels (one per CCR) are all triggered simultaneously by
     * each timer overflow, and we do NOT need the TIMER0_UP DMA stream
     * (DMA1_CH5 SUBPERI6) which conflicts with UART0 RX. */
    timer_channel_dma_request_source_select(TIMER0, TIMER_DMAREQUEST_UPDATEEVENT);

    /* URS = 1: only counter overflow generates the update event flag.
     * Software UPG will still reset CNT and reload PSC/ARR but will
     * NOT generate UIF or a DMA request – this prevents an accidental
     * DMA transfer during the start-up sequence. */
    timer_update_source_config(TIMER0, TIMER_UPDATE_SRC_REGULAR);
}

static void dshot_dma_init(void)
{
    rcu_periph_clock_enable(RCU_DMA1);

    /* Cache CCR register addresses */
    dshot_dma_map[0].timer_ccr_addr = (uint32_t)&TIMER_CH0CV(TIMER0);
    dshot_dma_map[1].timer_ccr_addr = (uint32_t)&TIMER_CH1CV(TIMER0);
    dshot_dma_map[2].timer_ccr_addr = (uint32_t)&TIMER_CH2CV(TIMER0);
    dshot_dma_map[3].timer_ccr_addr = (uint32_t)&TIMER_CH3CV(TIMER0);

    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        dma_channel_enum dch = dshot_dma_map[i].dma_channel;

        dma_deinit(DMA1, dch);

        dma_periph_address_config(DMA1, dch, dshot_dma_map[i].timer_ccr_addr);
        dma_memory_address_config(DMA1, dch, DMA_MEMORY_0, (uint32_t)dshot_raw[i]);
        dma_transfer_number_config(DMA1, dch, DSHOT_DMA_BUF_SIZE);

        dma_memory_address_generation_config(DMA1, dch, DMA_MEMORY_INCREASE_ENABLE);
        dma_peripheral_address_generation_config(DMA1, dch, DMA_PERIPH_INCREASE_DISABLE);
        dma_memory_width_config(DMA1, dch, DMA_MEMORY_WIDTH_16BIT);
        dma_periph_width_config(DMA1, dch, DMA_PERIPH_WIDTH_16BIT);
        dma_priority_config(DMA1, dch, DMA_PRIORITY_ULTRA_HIGH);
        dma_transfer_direction_config(DMA1, dch, DMA_MEMORY_TO_PERIPH);
        dma_circulation_disable(DMA1, dch);

        /* SUBPERI6 = TIMER0 channel requests */
        dma_channel_subperipheral_select(DMA1, dch, DMA_SUBPERI6);

        /* Enable transfer-complete interrupt */
        dma_interrupt_enable(DMA1, dch, DMA_CHXCTL_FTFIE);

        /* Do NOT enable CHxDEN here – it is armed in dshot_fire(). */
    }

    nvic_irq_enable(DMA1_Channel1_IRQn, 1, 0);
    nvic_irq_enable(DMA1_Channel2_IRQn, 1, 0);
    nvic_irq_enable(DMA1_Channel4_IRQn, 1, 0);
    nvic_irq_enable(DMA1_Channel6_IRQn, 1, 0);
}

static void dshot_prepare_dmabuffer(uint8_t chan, uint16_t value, bool telem_req)
{
    uint16_t* p = dshot_raw[chan];
    uint16_t packet;

    if (value > 2047) value = 2047;

    if (value == DSHOT_CMD_MOTOR_STOP) {
        packet = 0;
    } else {
        packet = (value << 1) | (telem_req ? 1 : 0);
        uint16_t csum = packet;
        csum ^= csum >> 8;
        csum ^= csum >> 4;
        csum &= 0xF;
        packet = (packet << 4) | csum;
    }

    for (uint8_t i = 0; i < 16; i++) {
        p[i] = (packet & (1 << (15 - i))) ? dshot_bit1 : dshot_bit0;
    }
    p[16] = 0;
    p[17] = 0;
}

/* Arm DMA channels, fire TIMER0, and let update events clock the frame out.
 *
 * Sequence:
 *   1. Stop timer, force all outputs LOW (CCR=0).
 *   2. Software UPG to reset CNT and latch CCR=0 into the active registers.
 *      Because URS=REGULAR, this does NOT generate UIF or DMA requests.
 *   3. Disable stale CHxDEN bits (in case a previous frame is still in-flight).
 *   4. Reconfigure and enable the four DMA streams.
 *   5. Enable CHxDEN so the next update event will fire the four DMA channels.
 *   6. Set CNT = ARR.  When CEN=1 the very next clock overflows → update event
 *      → DMA writes raw[0] into every CCR → first DShot bit begins.
 *   7. Enable the timer.
 */
static void dshot_fire(void)
{
    /* 1 – stop timer */
    timer_disable(TIMER0);

    /* Force CCR = 0 → outputs LOW while we set things up */
    TIMER_CH0CV(TIMER0) = 0;
    TIMER_CH1CV(TIMER0) = 0;
    TIMER_CH2CV(TIMER0) = 0;
    TIMER_CH3CV(TIMER0) = 0;

    /* 2 – UPG resets CNT to 0 and latches CCR=0, but no UIF/DMA (URS=1) */
    timer_event_software_generate(TIMER0, TIMER_EVENT_SRC_UPG);

    /* 3 – make sure no channel-DMA enable bits are set */
    timer_dma_disable(TIMER0, TIMER_DMA_CH0D);
    timer_dma_disable(TIMER0, TIMER_DMA_CH1D);
    timer_dma_disable(TIMER0, TIMER_DMA_CH2D);
    timer_dma_disable(TIMER0, TIMER_DMA_CH3D);

    /* 4 – (re)configure the four DMA streams */
    dshot_dma_done_mask = 0;
    for (uint8_t i = 0; i < MAX_PWM_OUT_CHAN; i++) {
        dma_channel_enum dch = dshot_dma_map[i].dma_channel;

        dma_channel_disable(DMA1, dch);
        dma_interrupt_flag_clear(DMA1, dch, DMA_INT_FLAG_FTF);
        dma_interrupt_flag_clear(DMA1, dch, DMA_INT_FLAG_HTF);
        dma_interrupt_flag_clear(DMA1, dch, DMA_INT_FLAG_TAE);

        dma_periph_address_config(DMA1, dch, dshot_dma_map[i].timer_ccr_addr);
        dma_memory_address_config(DMA1, dch, DMA_MEMORY_0, (uint32_t)dshot_raw[i]);
        dma_transfer_number_config(DMA1, dch, DSHOT_DMA_BUF_SIZE);
        dma_interrupt_enable(DMA1, dch, DMA_CHXCTL_FTFIE);
        dma_channel_enable(DMA1, dch);
    }

    /* DSB: ensure all DMA register writes have been committed to the bus
     * before we touch TIMER0 registers.  Without this barrier the CPU
     * write-buffer may let the TIMER0 enables below take effect before the
     * DMA channels are fully configured, causing some channels to miss the
     * first update-event DMA request (release-build timing issue). */
    __DSB();

    /* Clear any pending update flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    /* 5 – arm the four channel-DMA enable bits */
    timer_dma_enable(TIMER0, TIMER_DMA_CH0D);
    timer_dma_enable(TIMER0, TIMER_DMA_CH1D);
    timer_dma_enable(TIMER0, TIMER_DMA_CH2D);
    timer_dma_enable(TIMER0, TIMER_DMA_CH3D);

    /* 6 – pre-position counter just before overflow */
    timer_counter_value_config(TIMER0, TIMER_CAR(TIMER0));

    /* 7 – go!  Next clock → overflow → update → DMA writes raw[0] → first bit */
    timer_enable(TIMER0);
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
        pwm_timer_init();
        
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
            // timer_enable(TIMER7);
        }
        break;
    case ACT_CMD_CHANNEL_DISABLE:
        /* Disable all timers */
        timer_disable(TIMER0);
        timer_disable(TIMER1);
        timer_disable(TIMER3);
        // timer_disable(TIMER7);
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
#ifdef RT_USING_DSHOT
    case ACT_CMD_DSHOT_SEND:
        {
            struct dshot_command* c = (struct dshot_command*)arg;
            return act_write(dev, c->chan_mask, c->value, c->size) == c->size ? RT_EOK : RT_ERROR;
        }
#endif
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
        bool telem_req = dev->config.dshot_config.telem_req;

        /* Fill DMA buffers for every requested channel */
        for (rt_size_t i = 0; i < size; i++) {
            if (chan_sel & (1 << i)) {
                uint16_t val = *val_ptr;
                if (val > 2047) val = 2047;
                dshot_prepare_dmabuffer(i, val, telem_req);
                val_ptr++;
            }
        }

        /* Arm DMA and fire – non-blocking, ISR handles completion */
        dshot_fire();
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

