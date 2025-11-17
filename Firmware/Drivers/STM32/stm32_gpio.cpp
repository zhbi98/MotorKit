
#include "stm32_gpio.hpp"

#define N_EXTI 16

struct subscription_t {
    GPIO_TypeDef* port = nullptr;
    void (*callback)(void*) = nullptr;
    void* ctx = nullptr;
} subscriptions[N_EXTI];

const Stm32Gpio Stm32Gpio::none{nullptr, 0};

#define GPIO_MODE             0x00000003U
#define GPIO_OUTPUT_TYPE      0x00000010U

bool Stm32Gpio::config(uint32_t mode, uint32_t pull, uint32_t speed) {
    if (port_ == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port_ == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (port_ == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (port_ == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (port_ == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    } else if (port_ == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    } else if (port_ == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    } else if (port_ == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    } else {
        return false;
    }

    size_t position = get_pin_number();

    /*=== ↓↓↓ The following code is mostly taken from HAL_GPIO_Init ↓↓↓ ======*/
    /*=== ↓↓↓ 下面这段代码从 STM32 GPIO 初始化函数 HAL_GPIO_Init 复制而来 ↓↓↓ ===*/

    /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
    uint32_t temp = port_->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
    temp |= ((mode & GPIO_MODE) << (position * 2U));
    port_->MODER = temp;

    /* In case of Output or Alternate function mode selection */
    if((mode == GPIO_MODE_OUTPUT_PP) || (mode == GPIO_MODE_AF_PP) ||
       (mode == GPIO_MODE_OUTPUT_OD) || (mode == GPIO_MODE_AF_OD))
    {
        /* Check the Speed parameter */
        assert_param(IS_GPIO_SPEED(speed));
        /* Configure the IO Speed */
        temp = port_->OSPEEDR; 
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
        temp |= (speed << (position * 2U));
        port_->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = port_->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
        port_->OTYPER = temp;
    }

    /* Activate the Pull-up or Pull down resistor for the current IO */
    temp = port_->PUPDR;
    temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
    temp |= ((pull) << (position * 2U));
    port_->PUPDR = temp;

    /*=== ↑↑↑ The following code is mostly taken from HAL_GPIO_Init ↑↑↑ ======*/
    /*=== ↑↑↑ 上面这段代码从 STM32 GPIO 初始化函数 HAL_GPIO_Init 复制而来 ↑↑↑ ===*/

    return true;
}

bool Stm32Gpio::subscribe(bool rising_edge, bool falling_edge, 
    void (*callback)(void*), void* ctx) {
    uint32_t pin_number = get_pin_number();
    if (pin_number >= N_EXTI) {
        return false; // invalid pin number
    }

    struct subscription_t& subscription = subscriptions[pin_number];

    GPIO_TypeDef* no_port = nullptr;

    /*__atomic_compare_exchange_n 是 C++ 用于执行原子比较和交换操作的内置函数。
    这个函数的作用是比较指针 ptr 指向的值和 expected 指向的值，如果它们相等，
    就将 desired 的值写入 ptr。如果不相等，ptr 的当前值会被写入 expected。
    这里防止多个线程/任务同时注册同一个 EXTI 引脚*/
    if (!__atomic_compare_exchange_n(&subscription.port, &no_port, port_, 
        false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
        return false; // already in use
    }

    /*=== ↓↓↓ The following code is mostly taken from HAL_GPIO_Init ↓↓↓ ======*/
    /*=== ↓↓↓ 下面这段代码从 STM32 GPIO 初始化函数 HAL_GPIO_Init 复制而来 ↓↓↓ ===*/
    
    /*Enable SYSCFG Clock */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    uint32_t temp = SYSCFG->EXTICR[pin_number >> 2U];
    temp &= ~(0x0FU << (4U * (pin_number & 0x03U)));
    temp |= ((uint32_t)(GPIO_GET_INDEX(port_)) << (4U * (pin_number & 0x03U)));
    SYSCFG->EXTICR[pin_number >> 2U] = temp;

    /* Clear Rising Falling edge configuration */
    if (rising_edge) {
        EXTI->RTSR |= (uint32_t)pin_mask_;
    } else {
        EXTI->RTSR &= ~((uint32_t)pin_mask_);
    }

    if (falling_edge) {
        EXTI->FTSR |= (uint32_t)pin_mask_;
    } else {
        EXTI->FTSR &= ~((uint32_t)pin_mask_);
    }

    /* Clear EXTI line configuration */
    EXTI->EMR &= ~((uint32_t)pin_mask_);
    EXTI->IMR |= (uint32_t)pin_mask_;

    /*=== ↑↑↑ The following code is mostly taken from HAL_GPIO_Init ↑↑↑ ======*/
    /*=== ↑↑↑ 上面这段代码从 STM32 GPIO 初始化函数 HAL_GPIO_Init 复制而来 ↑↑↑ ===*/

    // Clear any previous triggers
    __HAL_GPIO_EXTI_CLEAR_IT(pin_mask_);
    
    subscription.ctx = ctx;
    subscription.callback = callback;
    return true;
}

void Stm32Gpio::unsubscribe() {
    uint32_t pin_number = get_pin_number();
    if (pin_number >= N_EXTI) {
        return; // invalid pin number
    }

    struct subscription_t& subscription = subscriptions[pin_number];

    if (subscription.port != port_) {
        return; // the subscription was not for this GPIO
    }

    EXTI->IMR |= (uint32_t)pin_mask_;
    __HAL_GPIO_EXTI_CLEAR_IT(pin_mask_);

    // At this point no more interrupts will be triggered for this GPIO

    subscription.callback = nullptr;
    subscription.ctx = nullptr;
    subscription.port = nullptr; // after this line, the subscription can be reused (possibly by another thread)
}

extern "C" void maybe_handle(uint16_t exti_number) {
    if(0/*__HAL_GPIO_EXTI_GET_IT(1 << exti_number) == RESET*/) {
        return; // This interrupt source did not trigger the interrupt line
    }

    /*__HAL_GPIO_EXTI_CLEAR_IT(1 << exti_number);*/
    
    if (exti_number >= N_EXTI) {
        return;
    }

    subscription_t& subscription = subscriptions[exti_number];
    if (subscription.callback) {
        (*subscription.callback)(subscription.ctx);
    }
}
