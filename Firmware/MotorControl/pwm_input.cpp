
#include "pwm_input.hpp"
#include "odrive_main.h"

void PwmInput::init() {
    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;

    uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

    for (size_t i = 0; i < 4; ++i) {
        if (!fibre::is_endpoint_ref_valid(odrv.config_.pwm_mappings[i].endpoint))
            continue;
        HAL_TIM_IC_ConfigChannel(htim_, &sConfigIC, channels[i]);
        HAL_TIM_IC_Start_IT(htim_, channels[i]);
    }
}

//TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz
#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT        false

/**
 * @param channel: A channel number in [0, 3]
 */
void handle_pulse(int channel, uint32_t high_time) {
    if (high_time < PWM_MIN_LEGAL_HIGH_TIME || high_time > PWM_MAX_LEGAL_HIGH_TIME)
        return;

    if (high_time < PWM_MIN_HIGH_TIME)
        high_time = PWM_MIN_HIGH_TIME;
    if (high_time > PWM_MAX_HIGH_TIME)
        high_time = PWM_MAX_HIGH_TIME;
    float fraction = (float)(high_time - PWM_MIN_HIGH_TIME) / (float)(PWM_MAX_HIGH_TIME - PWM_MIN_HIGH_TIME);
    float value = odrv.config_.pwm_mappings[channel].min +
                  (fraction * (odrv.config_.pwm_mappings[channel].max - odrv.config_.pwm_mappings[channel].min));

    /*解析通过 pulse 通信端口下发的配置指令，然后调用执行配置操作函数*/
    /*fibre::set_endpoint_from_float(odrv.config_.pwm_mappings[channel].endpoint, value);*/
}

/**
 * @param channel: A channel number in [0, 3]
 */
void PwmInput::on_capture(uint32_t timestamp) {
    static uint32_t last_timestamp = 0;
    static bool last_pin_state = false;
    static bool last_sample_valid = false ;

    Stm32Gpio gpio = PWM_Pin_;/*get_gpio(gpios_[channel])*/
    if (!gpio) return;

    bool current_pin_state = gpio.read();

    if (last_sample_valid
        && (last_pin_state != PWM_INVERT_INPUT)
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(0, timestamp - last_timestamp);
    }

    last_timestamp = timestamp;
    last_pin_state = current_pin_state;
    last_sample_valid = true;
}

void PwmInput::on_capture() {
    if(__HAL_TIM_GET_FLAG(htim_, TIM_FLAG_CC1)) {
        __HAL_TIM_CLEAR_IT(htim_, TIM_IT_CC1);
        on_capture(htim_->Instance->CCR1);
    }
}
