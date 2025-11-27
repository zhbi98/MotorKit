/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include <stdbool.h>

// STM specific includes
#include <stm32f4xx_hal.h>
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include <can.h>
#include <i2c.h>
#include <usb_device.h>
#include <main.h>
#include "cmsis_os.h"

#include <arm_math.h>

#include <Drivers/STM32/stm32_system.h>

/*------ Add by zhbi98 -------*/
#define HW_VERSION_VOLTAGE 56U
#define HW_VERSION_MAJOR 3U
#define HW_VERSION_MINOR 6U
/*----------------------------*/

/*------ Modify by zhbi98 ----*/
/*相电流采样电阻设置，该定义要与实际电阻相同或接近，但相差不要超过 10 倍，例如实际为 5mΩ 可以设置为 2mΩ，
但不能设置为 0.5mΩ，否则可能会导致采集不到正确的电流从而产生 ERROR_PHASE_RESISTANCE_OUT_OF_RANGE 和 
ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE 错误。*/
#define SHUNT_RESISTANCE (200e-5f/*500e-6f*/)
/*----------------------------*/

#define AXIS_COUNT (2)
#define CAN_CLK_MHZ (42UL)
#define CAN_CLK_HZ (42000000UL)
#define CAN_FREQ (2000000UL) /*42M/(16tq+4tq+1)*/

#define DEFAULT_BRAKE_RESISTANCE (2.0f) // [ohm]
#define DEFAULT_MIN_DC_VOLTAGE 8.0f

#define TIM_TIME_BASE TIM14

// Run control loop at the same frequency as the current measurements.
#define CONTROL_TIMER_PERIOD_TICKS  (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1))

#define TIM1_INIT_COUNT (TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128) // TODO: explain why this offset

// The delta from the control loop timestamp to the current sense timestamp is
// exactly 0 for M0 and TIM1_INIT_COUNT for M1.
#define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA (TIM_1_8_PERIOD_CLOCKS / 2 + 1 * 128)

#ifdef __cplusplus
#include <Drivers/DRV8301/drv8301.hpp>
#include <Drivers/STM32/stm32_gpio.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <MotorControl/pwm_input.hpp>
#include <MotorControl/thermistor.hpp>

using TGateDriver = Drv8301;
using TOpAmp = Drv8301;

#include <MotorControl/motor.hpp>
#include <MotorControl/encoder.hpp>

extern std::array<Axis, AXIS_COUNT> axes;
extern Motor motors[AXIS_COUNT];
extern OnboardThermistorCurrentLimiter fet_thermistors[AXIS_COUNT];
extern Encoder encoders[AXIS_COUNT];

extern USBD_HandleTypeDef& usb_dev_handle;

extern PwmInput pwm0_input;
#endif

// Period in [s]
#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )
static const int current_meas_hz = CURRENT_MEAS_HZ;

#if HW_VERSION_VOLTAGE >= 48
#define VBUS_S_DIVIDER_RATIO 19.0f
#elif HW_VERSION_VOLTAGE == 24
#define VBUS_S_DIVIDER_RATIO 11.0f
#else
#error "unknown board voltage"
#endif

// Linear range of the DRV8301 opamp output: 0.3V...5.7V. We set the upper limit
// to 3.0V so that it's symmetric around the center point of 1.65V.
#define CURRENT_SENSE_MIN_VOLT  0.3f
#define CURRENT_SENSE_MAX_VOLT  3.0f

// This board has no board-specific user configurations
static inline bool board_read_config() { return true; }
static inline bool board_write_config() { return true; }
static inline void board_clear_config() { }
static inline bool board_apply_config() { return true; }

void system_init();
bool board_init();
void start_timers();

#endif // __BOARD_CONFIG_H
