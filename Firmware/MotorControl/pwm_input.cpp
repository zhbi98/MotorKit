
#include "pwm_input.hpp"
#include "odrive_main.h"

/*这些表达式按 1MHz 进行整数除法，因此对于非整数 MHz 的时钟频率将是不正确的。*/
/*TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz*/
#define TIM_2_5_CLOCK_HZ TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT false

void PwmInput::init() {
    uint32_t channels[] = {TIM_CHANNEL_3, TIM_CHANNEL_4};

    if (htim_ == nullptr) return;

    for (size_t i = 0; i < 2; i++) 
        HAL_TIM_IC_Start_IT(htim_, 
            channels[i]);
}

/**
 * @param gpio_idx: A gpio_idx number in [0, 3]
 */
void handle_pulse(int gpio_idx, uint32_t high_time) {
    if (high_time < PWM_MIN_LEGAL_HIGH_TIME || 
        high_time > PWM_MAX_LEGAL_HIGH_TIME) return;

    if (high_time < PWM_MIN_HIGH_TIME) high_time = PWM_MIN_HIGH_TIME;
    if (high_time > PWM_MAX_HIGH_TIME) high_time = PWM_MAX_HIGH_TIME;

    float fraction = (float)(high_time - PWM_MIN_HIGH_TIME) / (float)(PWM_MAX_HIGH_TIME - PWM_MIN_HIGH_TIME);
    float value = odrv.config_.pwm_mappings[gpio_idx].min +
                  (fraction * (odrv.config_.pwm_mappings[gpio_idx].max - 
                    odrv.config_.pwm_mappings[gpio_idx].min));

    /*解析通过 pulse 通信端口下发的配置指令，然后调用执行配置操作函数*/
    /*fibre::set_endpoint_from_float(odrv.config_.pwm_mappings[gpio_idx].endpoint, value);*/
}

/**@param gpio_idx: A gpio_idx number in [0, 3]*/
void PwmInput::on_capture(int gpio_idx, uint32_t timestamp) {
    static uint32_t last_timestamp[2] = {0};
    static bool last_pin_state[2] = {false};
    static bool last_sample_valid[2] = {false};

    if (gpio_idx > 1) return;

    Stm32Gpio gpio = gpios_[gpio_idx];
    if (!gpio) return;

    /*读取当前 PWM 信号的电平状态（高/低），用于判断是否发生了 “上升” 或 “下降” 沿，
    结合前后两次的电平状态（last_state 和 current_state），系统可以检测到信号边沿宽度，
    从而计算出 PWM 信号的高电平持续时间（pulse width）。*/
    bool current_pin_state = gpio.read();

    /*捕获寄存器只记录时间戳时间长度，不直接告诉你是 “上升还是下降沿”，所以我们需要自己判断！*/
    if (last_sample_valid[gpio_idx]
        && (last_pin_state[gpio_idx] != PWM_INVERT_INPUT) /*PWM_INVERT_INPUT 判断有效脉冲电平*/
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(gpio_idx, timestamp - last_timestamp[gpio_idx]);
    }

    last_timestamp[gpio_idx] = timestamp;
    last_pin_state[gpio_idx] = current_pin_state;
    last_sample_valid[gpio_idx] = true;
}

void PwmInput::on_capture() {
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC3)) { /*TIM_CHANNEL_3 捕获事件*/
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC3);
        on_capture(0, htim_->Instance->CCR3);
    }
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC4)) { /*TIM_CHANNEL_4 捕获事件*/
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC4);
        on_capture(1, htim_->Instance->CCR4);
    }
}
