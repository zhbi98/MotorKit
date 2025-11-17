#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

// Hardware configuration
#include <board.h>

#ifdef __cplusplus
#include <communication/interface_usb.h>
#include <communication/interface_i2c.h>
#include <communication/interface_uart.h>
#include <task_timer.hpp>
extern "C" {
#endif

// OS includes
#include <cmsis_os.h>

// extern const float elec_rad_per_enc;
extern uint32_t _reboot_cookie;

extern uint64_t serial_number;
extern char serial_number_str[13];

#ifdef __cplusplus
}

typedef struct {
    bool fully_booted;
    uint32_t uptime; // [ms]
    uint32_t min_heap_space; // FreeRTOS heap [Bytes]
    uint32_t max_stack_usage_axis; // minimum remaining space since startup [Bytes]
    uint32_t max_stack_usage_usb;
    uint32_t max_stack_usage_uart;
    uint32_t max_stack_usage_startup;
    uint32_t max_stack_usage_can;
    uint32_t max_stack_usage_analog;

    uint32_t stack_size_axis;
    uint32_t stack_size_usb;
    uint32_t stack_size_uart;
    uint32_t stack_size_startup;
    uint32_t stack_size_can;
    uint32_t stack_size_analog;

    int32_t prio_axis;
    int32_t prio_usb;
    int32_t prio_uart;
    int32_t prio_startup;
    int32_t prio_can;
    int32_t prio_analog;

    USBStats_t& usb = usb_stats_;
    I2CStats_t& i2c = i2c_stats_;
} SystemStats_t;

struct PWMMapping_t {
    endpoint_ref_t endpoint = {0, 0};
    float min = 0;
    float max = 0;
};

// @brief general user configurable board configuration
struct BoardConfig_t {
    bool enable_uart_a = true;
    bool enable_uart_b = false;
    bool enable_uart_c = false;
    uint32_t uart_a_baudrate = 115200;
    uint32_t uart_b_baudrate = 115200;
    uint32_t uart_c_baudrate = 115200;
    bool enable_can_a = true;
    bool enable_i2c_a = false;
    ODriveIntf::StreamProtocolType uart0_protocol = ODriveIntf::STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT;
    ODriveIntf::StreamProtocolType uart1_protocol = ODriveIntf::STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT;
    ODriveIntf::StreamProtocolType uart2_protocol = ODriveIntf::STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT;
    ODriveIntf::StreamProtocolType usb_cdc_protocol = ODriveIntf::STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT;
    float max_regen_current = 0.0f;
    float brake_resistance = DEFAULT_BRAKE_RESISTANCE; /*如果没有使能刹车电阻，刹车电阻设置值不生效，刹车电阻为任何值都可以*/
    bool enable_brake_resistor = false; /*如果没有使能刹车电阻，刹车电阻设置值不生效，刹车电阻为任何值都可以*/
    float dc_bus_undervoltage_trip_level = DEFAULT_MIN_DC_VOLTAGE;      //<! [V] minimum voltage below which the motor stops operating
    float dc_bus_overvoltage_trip_level = 1.07f * HW_VERSION_VOLTAGE;   //<! [V] maximum voltage above which the motor stops operating.
                                                                        //<! This protects against cases in which the power supply fails to dissipate
                                                                        //<! the brake power if the brake resistor is disabled.
                                                                        //<! The default is 26V for the 24V board version and 52V for the 48V board version.

    /**
     * If enabled, if the measured DC voltage exceeds `dc_bus_overvoltage_ramp_start`,
     * the ODrive will sink more power than usual into the the brake resistor
     * in an attempt to bring the voltage down again.
     * 
     * The brake duty cycle is increased by the following amount:
     *  vbus_voltage == dc_bus_overvoltage_ramp_start  =>  brake_duty_cycle += 0%
     *  vbus_voltage == dc_bus_overvoltage_ramp_end  =>  brake_duty_cycle += 100%
     * 
     * Remarks:
     *  - This feature is active even when all motors are disarmed.
     *  - This feature is disabled if `brake_resistance` is non-positive.
     */
    bool enable_dc_bus_overvoltage_ramp = false;
    float dc_bus_overvoltage_ramp_start = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                      //!< Do not set this lower than your usual vbus_voltage,
                                                                      //!< unless you like fried brake resistors.
    float dc_bus_overvoltage_ramp_end = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                    //!< Must be larger than `dc_bus_overvoltage_ramp_start`,
                                                                    //!< otherwise the ramp feature is disabled.

    float dc_max_positive_current = INFINITY; // Max current [A] the power supply can source
    float dc_max_negative_current = -0.01f; // 电源可以吸收的反电动势电流，Max current [A] the power supply can sink. 
                                            // You most likely want a non-positive value here. Set to -INFINITY to disable.
    PWMMapping_t pwm_mappings[4];
    PWMMapping_t analog_mappings[17];
};

struct TaskTimes {
    TaskTimer sampling;
    TaskTimer control_loop_misc;
    TaskTimer control_loop_checks;
    TaskTimer dc_calib_wait;
};


// Forward Declarations
class Axis;
class Motor;

#include "autogen/interfaces.hpp"

// ODrive specific includes
#include <utils.hpp>
#include <low_level.h>
#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <controller.hpp>
#include <current_limiter.hpp>
#include <thermistor.hpp>
#include <trapTraj.hpp>
#include <endstop.hpp>
#include <mechanical_brake.hpp>
#include <axis.hpp>
#include <oscilloscope.hpp>
#include <communication/communication.h>
#include <communication/can/odrive_can.hpp>

// Defined in autogen/version.c based on git-derived version numbers
extern "C" {
extern const unsigned char fw_version_major_;
extern const unsigned char fw_version_minor_;
extern const unsigned char fw_version_revision_;
extern const unsigned char fw_version_unreleased_;
}

// general system functions defined in main.cpp
class ODrive : public ODriveIntf {
public:
    bool save_configuration() override;
    void erase_configuration() override;
    void reboot() override { NVIC_SystemReset(); }
    void enter_dfu_mode() override;
    bool any_error();
    void clear_errors() override;

    int32_t test_function(int32_t delta) override {
        static int cnt = 0;
        return cnt += delta;
    }

    void do_fast_checks();
    void sampling_cb();
    void control_loop_cb(uint32_t timestamp);

    Axis& get_axis(int num) { return axes[num]; }

    uint32_t get_interrupt_status(int32_t irqn);
    uint32_t get_dma_status(uint8_t stream_num);
    uint64_t get_drv_fault();
    void disarm_with_error(Error error);

    Error error_ = ERROR_NONE;
    float& vbus_voltage_ = ::vbus_voltage; // TODO: make this the actual variable
    float& ibus_ = ::ibus_; // TODO: make this the actual variable
    float ibus_report_filter_k_ = 1.0f;

    const uint64_t& serial_number_ = ::serial_number;

    // Hardware version is compared with OTP on startup to ensure that we're
    // running on the right board version.
    const uint8_t hw_version_major_ = HW_VERSION_MAJOR;
    const uint8_t hw_version_minor_ = HW_VERSION_MINOR;
    const uint8_t hw_version_variant_ = HW_VERSION_VOLTAGE;

    // the corresponding macros are defined in the autogenerated version.h
    const uint8_t fw_version_major_ = ::fw_version_major_;
    const uint8_t fw_version_minor_ = ::fw_version_minor_;
    const uint8_t fw_version_revision_ = ::fw_version_revision_;
    const uint8_t fw_version_unreleased_ = ::fw_version_unreleased_; // 0 for official releases, 1 otherwise

    bool& brake_resistor_armed_ = ::brake_resistor_armed; // TODO: make this the actual variable
    bool& brake_resistor_saturated_ = ::brake_resistor_saturated; // TODO: make this the actual variable
    float& brake_resistor_current_ = ::brake_resistor_current;

    SystemStats_t system_stats_;

    // Edit these to suit your capture needs
    Oscilloscope oscilloscope_{
        nullptr, // trigger_src
        0.5f, // trigger_threshold
        nullptr // data_src TODO: change data type
    };

    ODriveCAN can_;

    BoardConfig_t config_;
    uint32_t user_config_loaded_ = 0;
    bool misconfigured_ = false;

    uint32_t test_property_ = 0;

    uint32_t last_update_timestamp_ = 0;
    uint32_t n_evt_sampling_ = 0;
    uint32_t n_evt_control_loop_ = 0;
    bool task_timers_armed_ = false;
    TaskTimes task_times_;
    const bool otp_valid_ = ((uint8_t*)FLASH_OTP_BASE)[0] != 0xff;
};

extern ODrive odrv; // defined in main.cpp

#endif // __cplusplus

#endif /* __ODRIVE_MAIN_H */
