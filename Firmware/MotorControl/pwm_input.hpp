#ifndef __PWM_INPUT_HPP
#define __PWM_INPUT_HPP

#include <stm32_gpio.hpp>
#include <tim.h>
#include <array>

class PwmInput {
public:
    PwmInput(TIM_HandleTypeDef* htim, std::array<Stm32Gpio, 2> gpios)
            : htim_(htim), gpios_(gpios) {}

    void init();
    void on_capture();

private:
    void on_capture(int gpio_idx, uint32_t timestamp);

    TIM_HandleTypeDef* htim_;
    std::array<Stm32Gpio, 2> gpios_;
};

#endif // __PWM_INPUT_HPP