#ifndef __PWM_INPUT_HPP
#define __PWM_INPUT_HPP

#include <stm32_gpio.hpp>
#include <tim.h>
#include <array>

class PwmInput {
public:
    PwmInput(TIM_HandleTypeDef* htim, Stm32Gpio PWM_Pin)
            : htim_(htim), PWM_Pin_(PWM_Pin) {}

    void init();
    void on_capture();

private:
    void on_capture(uint32_t timestamp);

    TIM_HandleTypeDef* htim_;
    Stm32Gpio PWM_Pin_; /*std::array<uint16_t, 4> gpios_*/
};

#endif // __PWM_INPUT_HPP