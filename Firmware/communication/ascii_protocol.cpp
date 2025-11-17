/*
* The ASCII protocol is a simpler, human readable alternative to the main native
* protocol.
* In the future this protocol might be extended to support selected GCode commands.
* For a list of supported commands see doc/ascii-protocol.md
*/

/* Includes ------------------------------------------------------------------*/

#include "odrive_main.h"
#include "communication.h"
#include "ascii_protocol.hpp"
#include <utils.hpp>

#include "autogen/interfaces.hpp"
#include "log.h"


/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)

/* Private variables ---------------------------------------------------------*/

/* static Introspectable root_obj = ODrive3TypeInfo<ODrive>::make_introspectable(odrv);*/

/* Private function prototypes -----------------------------------------------*/

/* Function implementations --------------------------------------------------*/

// @brief Sends a line on the specified output.
template<typename ... TArgs>
void AsciiProtocol::respond(bool include_checksum, const char * fmt, TArgs&& ... args) {
    char tx_buf[64];

    size_t len = snprintf(tx_buf, sizeof(tx_buf), fmt, std::forward<TArgs>(args)...);

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= tx_buf[i];
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "*%u\r\n", checksum);
    } else {
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "\r\n");
    }

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    /*sink_.write({(const uint8_t*)tx_buf, len});*/
    /*sink_.maybe_start_async_write();*/
}

/**
 * ASCII 协议是人类可读的，并且是面向行的，每行具有以下格式：
 * 格式：command *42 ; comment [new line character]
 * 
 * *42 代表 GCode 兼容的校验和，可以省略。当且仅当提供校验和时，设备还会在响应中包含校验和 (如果有的话)。
 * 如果提供了校验和但无效，则会忽略该行。校验和以星号 (*) 前所有字符的按位 x 计算。
 * 有效校验和的示例：r vbus_voltage *93。
 * 
 * 支持 GCode 兼容性 new line character。
 * 一旦遇到换行符，命令就会被解释（执行）。
 * 如果命令产生了响应，但 ODrive 还没有完成发送之前的响应，则新的响应将被静默丢弃。
 * 支持的换行符包括回车符 (r)、换行符 (n)、CR/LF (rn) 和！
 * ODrive 始终使用 CR/LF (rn) 作为新行终止序列
 */
// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void AsciiProtocol::process_line(char * buffer) {
    static_assert(sizeof(char) == sizeof(uint8_t));
    
    // scan line to find beginning of checksum and prune comment
    uint8_t checksum = 0;
    size_t checksum_start = SIZE_MAX;
    /*for (size_t i = 0; i < buffer.size(); ++i) {
        if (buffer.begin()[i] == ';') { // ';' is the comment start char
            buffer = buffer.take(i);
            break;
        }
        if (checksum_start > i) {
            if (buffer[i] == '*') {
                checksum_start = i + 1;
            } else {
                checksum ^= buffer[i];
            }
        }
    }*/

    // copy everything into a local buffer so we can insert null-termination
    char cmd[MAX_LINE_LENGTH + 1];
    size_t len = std::min(strlen(buffer)/*buffer.size()*/, MAX_LINE_LENGTH);
    /*memcpy(cmd, buffer.begin(), len);*/
    cmd[len] = 0; // null-terminate

    // optional checksum validation
    bool use_checksum = (checksum_start < len);
    if (use_checksum) {
        unsigned int received_checksum;
        int numscan = sscanf(&cmd[checksum_start], "%u", &received_checksum);
        if ((numscan < 1) || (received_checksum != checksum))
            return;
        len = checksum_start - 1; // prune checksum and asterisk
        cmd[len] = 0; // null-terminate
    }

    /**
     * 电机轨迹命令参考：Format: t motor destination
     * [t]: trajectory 模式
     * [motor]: 电机 ID 0 或 1.
     * [destination]: 目标位置[turns]。
     * 
     * Example: t 0 -2
     * 此命令将控制模式设置为 CONTROL_MODE_POSITION_CONTROL, 
     * 将输入模式设置为 INPUT_MODE_TRAP_TRAJ。
     * 此命令更新电机的看门狗计时器。
     */
    // check incoming packet type
    switch(cmd[0]) {
        case 'p': cmd_set_position(cmd, use_checksum);                break;  // position control
        case 'q': cmd_set_position_wl(cmd, use_checksum);             break;  // position control with limits
        case 'v': cmd_set_velocity(cmd, use_checksum);                break;  // velocity control
        case 'c': cmd_set_torque(cmd, use_checksum);                  break;  // current control
        case 't': cmd_set_trapezoid_trajectory(cmd, use_checksum);    break;  // trapezoidal trajectory
        case 'f': cmd_get_feedback(cmd, use_checksum);                break;  // feedback
        case 'h': cmd_help(cmd, use_checksum);                        break;  // Help
        case 'i': cmd_info_dump(cmd, use_checksum);                   break;  // Dump device info
        case 's': cmd_system_ctrl(cmd, use_checksum);                 break;  // System
        case 'r': cmd_read_property(cmd,  use_checksum);              break;  // read property
        case 'w': cmd_write_property(cmd, use_checksum);              break;  // write property
        case 'u': cmd_update_axis_wdg(cmd, use_checksum);             break;  // Update axis watchdog. 
        case 'e': cmd_encoder(cmd, use_checksum);                     break;  // Encoder commands
        default : cmd_unknown(nullptr, use_checksum);                 break;
    }
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_vbus_voltage()
{
    float vbus = odrv.vbus_voltage_;
    logInfo("%5f", vbus);
    return 0;
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_config_dc_max_negative_current(char * _current)
{
    uint8_t len = strlen(_current);
    double current = 0.0f;

    if ((!len) || (len > 16)) return 0;

    current = atof(_current);
    odrv.config_.dc_max_negative_current = current;

    logInfo("%5lf", current);

    return 1;
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_position(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_feed_forward, torque_feed_forward;

    int numscan = sscanf(pStr, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &torque_feed_forward);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        /**设置电机位置运动模式，再设置目标位置*/
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = pos_setpoint;
        /**判断参数个数，确定是否包含速度*/
        if (numscan >= 3)
            axis.controller_.input_vel_ = vel_feed_forward;
        if (numscan >= 4)
            axis.controller_.input_torque_ = torque_feed_forward;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_axis_requested_state(int motor_id, int _state)
{
    if (motor_id < 0 || motor_id > 1) return 0;
    if ((_state > Axis::AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION) ||
        (_state < Axis::AXIS_STATE_UNDEFINED)) return 0;

    Axis& axis = axes[motor_id];
    axis.requested_state_ = static_cast<Axis::AxisState>(_state);

    logInfo("%d", _state);

    return 1;
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_axis_controller_config_control_mode(int motor_id, int _mode)
{
    if (motor_id < 0 || motor_id > 1) return 0;
    if ((_mode > Controller::CONTROL_MODE_POSITION_CONTROL) ||
        (_mode < Controller::CONTROL_MODE_VOLTAGE_CONTROL)) return 0;

    Axis& axis = axes[motor_id];
    axis.controller_.config_.control_mode = \
        static_cast<Controller::ControlMode>(_mode);

    logInfo("%d", _mode);

    return 1;
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_axis_controller_config_vel_limit(int motor_id, char * _vel_lim)
{
    uint8_t len = strlen(_vel_lim);
    double vel_lim = 0.0f;

    if (motor_id < 0 || motor_id > 1) return 0;
    if ((!len) || (len > 16)) return 0;

    vel_lim = atof(_vel_lim);
    Axis& axis = axes[motor_id];
    axis.controller_.config_.vel_limit = vel_lim;

    logInfo("%5lf", vel_lim);

    return 1;
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_axis_controller_input_pos(int motor_id, char * _pos_setpoint)
{
    uint8_t len = strlen(_pos_setpoint);
    double pos_setpoint = 0.0f;

    if (motor_id < 0 || motor_id > 1) return 0;
    if ((!len) || (len > 16)) return 0;

    pos_setpoint = atof(_pos_setpoint);
    Axis& axis = axes[motor_id];
    axis.controller_.input_pos_ = pos_setpoint;
    axis.controller_.input_pos_updated();

    logInfo("%5lf", pos_setpoint);
    
    return 1;
}

// @brief Executes the set position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_axis_controller_input_vel(int motor_id, char * _vel_setpoint)
{
    uint8_t len = strlen(_vel_setpoint);
    double vel_setpoint = 0.0f;

    if (motor_id < 0 || motor_id > 1) return 0;
    if ((!len) || (len > 16)) return 0;

    vel_setpoint = atof(_vel_setpoint);
    Axis& axis = axes[motor_id];
    axis.controller_.input_vel_ = vel_setpoint;

    logInfo("%5lf", vel_setpoint);

    return 1;
}

// @brief Executes the set torque control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_axis_controller_input_torque(int motor_id, char * _torque_setpoint)
{
    uint8_t len = strlen(_torque_setpoint);
    double torque_setpoint = 0.0f;

    if (motor_id < 0 || motor_id > 1) return 0;
    if ((!len) || (len > 16)) return 0;

    torque_setpoint = atof(_torque_setpoint);
    Axis& axis = axes[motor_id];
    axis.controller_.input_torque_ = torque_setpoint;

    logInfo("%5lf", torque_setpoint);

    return 1;
}

// @brief Executes the set position with current and velocity limit command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_position_wl(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float pos_setpoint, vel_limit, torque_lim;

    int numscan = sscanf(pStr, "q %u %f %f %f", &motor_number, &pos_setpoint, &vel_limit, &torque_lim);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        /**设置电机位置运动模式，再设置目标位置*/
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = pos_setpoint;
        /**判断参数个数，确定是否包含速度限制，力矩限制*/
        if (numscan >= 3)
            axis.controller_.config_.vel_limit = vel_limit;
        if (numscan >= 4)
            axis.motor_.config_.torque_lim = torque_lim;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the set velocity command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_velocity(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float vel_setpoint, torque_feed_forward;
    int numscan = sscanf(pStr, "v %u %f %f", &motor_number, &vel_setpoint, &torque_feed_forward);
    if (numscan < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        /**设置电机速度运动模式，再设置目标速度*/
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
        axis.controller_.input_vel_ = vel_setpoint;
        /**判断参数个数，确定是否包含 torque_feed_forward*/
        if (numscan >= 3)
            axis.controller_.input_torque_ = torque_feed_forward;
        axis.watchdog_feed();
    }
}

// @brief Executes the set torque control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_torque(char * pStr, bool use_checksum) {
    unsigned motor_number;
    float torque_setpoint;

    if (sscanf(pStr, "c %u %f", &motor_number, &torque_setpoint) < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        /**设置电机恒定力矩运动模式，再设置目标力矩*/
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();
    }
}

// @brief Sets the encoder linear count
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_encoder(char * pStr, bool use_checksum) {
    if (pStr[1] == 's') {
        pStr += 2; // Substring two characters to the right (ok because we have guaranteed null termination after all chars)

        unsigned motor_number;
        int encoder_count;

        if (sscanf(pStr, "l %u %i", &motor_number, &encoder_count) < 2) {
            respond(use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis& axis = axes[motor_number];
            axis.encoder_.set_linear_count(encoder_count);
            axis.watchdog_feed();
            respond(use_checksum, "encoder set to %u", encoder_count);
        }
    } else {
        respond(use_checksum, "invalid command format");
    }
}

// @brief Executes the set trapezoid trajectory command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_set_trapezoid_trajectory(char* pStr, bool use_checksum) {
    unsigned motor_number;
    float goal_point;

    if (sscanf(pStr, "t %u %f", &motor_number, &goal_point) < 2) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        axis.controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        axis.controller_.input_pos_ = goal_point;
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
}

// @brief Executes the get position and velocity feedback command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_get_feedback(char * pStr, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "f %u", &motor_number) < 1) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        Axis& axis = axes[motor_number];
        respond(use_checksum, "%f %f",
                (double)axis.encoder_.pos_estimate_.any().value_or(0.0f),
                (double)axis.encoder_.vel_estimate_.any().value_or(0.0f));
    }
}

// @brief Shows help text
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_help(char * pStr, bool use_checksum) {
    (void)pStr;
    respond(use_checksum, "Please see documentation for more details");
    respond(use_checksum, "");
    respond(use_checksum, "Available commands syntax reference:");
    respond(use_checksum, "Position: q axis pos vel-lim I-lim");
    respond(use_checksum, "Position: p axis pos vel-ff I-ff");
    respond(use_checksum, "Velocity: v axis vel I-ff");
    respond(use_checksum, "Torque: c axis T");
    respond(use_checksum, "");
    respond(use_checksum, "Properties start at odrive root, such as axis0.requested_state");
    respond(use_checksum, "Read: r property");
    respond(use_checksum, "Write: w property value");
    respond(use_checksum, "");
    respond(use_checksum, "Save config: ss");
    respond(use_checksum, "Erase config: se");
    respond(use_checksum, "Reboot: sr");
}

// @brief Gets the hardware, firmware and serial details
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_info_dump(char * pStr, bool use_checksum) {
    // respond(use_checksum, "Signature: %#x", STM_ID_GetSignature());
    // respond(use_checksum, "Revision: %#x", STM_ID_GetRevision());
    // respond(use_checksum, "Flash Size: %#x KiB", STM_ID_GetFlashSize());
    respond(use_checksum, "Hardware version: %d.%d-%dV", odrv.hw_version_major_, odrv.hw_version_minor_, odrv.hw_version_variant_);
    respond(use_checksum, "Firmware version: %d.%d.%d", odrv.fw_version_major_, odrv.fw_version_minor_, odrv.fw_version_revision_);
    respond(use_checksum, "Serial number: %s", serial_number_str);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_system_ctrl(char * pStr, bool use_checksum) {
    switch (pStr[1])
    {
        case 's':   odrv.save_configuration();  break;  // Save config
        case 'e':   odrv.erase_configuration(); break;  // Erase config
        case 'r':   odrv.reboot();              break;  // Reboot
        case 'c':   odrv.clear_errors();        break;  // clear all errors and rearm brake resistor if necessary
        default:    /* default */               break;
    }
}

const char * configs[] = {"save", "erase", "reboot", "clearErr"};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int cmd_dev_config(char * _config)
{
    int8_t _res = 1;
    uint8_t id = 0;

    for (; id < 4; id++) {
        _res = strncmp(_config, configs[id], 
            strlen(configs[id]));
        if (!_res) break;
    }

    switch (id) {
    /*Save config*/
    case 0: 
        odrv.save_configuration(); 
        break; 
    /*Erase config*/
    case 1: 
        odrv.erase_configuration(); 
        break;
    /*Reboot device*/
    case 2: 
        odrv.reboot(); 
        break;
    /*Clear all errors and rearm brake resistor if necessary*/
    case 3: 
        odrv.clear_errors(); 
        break;
    /*Default Invalid input*/
    default: 
        return 0; 
        break;
    }

    logInfo("%s", _config);
    return 1;
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
typedef struct {
    ODrive::Error code;
    const char* desc;
} ODrive_ErrorInfo_t;

static const ODrive_ErrorInfo_t odrive_error[] = {
    {ODrive::ERROR_CONTROL_ITERATION_MISSED, "CONTROL_ITERATION_MISSED"},
    {ODrive::ERROR_DC_BUS_UNDER_VOLTAGE, "DC_BUS_UNDER_VOLTAGE"},
    {ODrive::ERROR_DC_BUS_OVER_VOLTAGE, "DC_BUS_OVER_VOLTAGE"},
    {ODrive::ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC_BUS_OVER_REGEN_CURRENT"},
    {ODrive::ERROR_DC_BUS_OVER_CURRENT, "DC_BUS_OVER_CURRENT"},
    {ODrive::ERROR_BRAKE_DEADTIME_VIOLATION, "BRAKE_DEADTIME_VIOLATION"},
    {ODrive::ERROR_BRAKE_DUTY_CYCLE_NAN, "BRAKE_DUTY_CYCLE_NAN"},
    {ODrive::ERROR_INVALID_BRAKE_RESISTANCE, "INVALID_BRAKE_RESISTANCE"},
};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void system_dumperrors() 
{
    ODrive::Error error_ = ODrive::ERROR_NONE;
    bool first = true;

    error_ = odrv.error_;

    if (error_ == ODrive::ERROR_NONE) {
        logInfo("system: Error(s): NONE");
        return;
    }

    logInfo("system: Error(s):");
    for (const auto& err : odrive_error) {
        if (error_ & err.code) {
            logInfo("%s", err.desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first)
        logInfo("unknown error: 0x%08lX", 
            (uint32_t)error_);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
typedef struct {
    Axis::Error code;
    const char* desc;
} Axis_ErrorInfo_t;

static const Axis_ErrorInfo_t axis_error[] = {
    {Axis::ERROR_INVALID_STATE, "INVALID_STATE"},
    {Axis::ERROR_MOTOR_FAILED, "MOTOR_FAILED"},
    {Axis::ERROR_SENSORLESS_ESTIMATOR_FAILED,"SENSORLESS_ESTIMATOR_FAILED"},
    {Axis::ERROR_ENCODER_FAILED, "ENCODER_FAILED"},
    {Axis::ERROR_CONTROLLER_FAILED, "CONTROLLER_FAILED"},
    {Axis::ERROR_WATCHDOG_TIMER_EXPIRED, "WATCHDOG_TIMER_EXPIRED"},
    {Axis::ERROR_MIN_ENDSTOP_PRESSED, "MIN_ENDSTOP_PRESSED"},
    {Axis::ERROR_MAX_ENDSTOP_PRESSED, "MAX_ENDSTOP_PRESSED"},
    {Axis::ERROR_ESTOP_REQUESTED, "ESTOP_REQUESTED"},
    {Axis::ERROR_HOMING_WITHOUT_ENDSTOP, "HOMING_WITHOUT_ENDSTOP"},
    {Axis::ERROR_OVER_TEMP, "OVER_TEMP"},
    {Axis::ERROR_UNKNOWN_POSITION, "UNKNOWN_POSITION"},
};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void axis_dumperrors(int motor_id) 
{
    Axis::Error error_ = Axis::ERROR_NONE;
    bool first = true;

    if (motor_id < 0 || motor_id > 1) return;
    Axis& axis = axes[motor_id];

    error_ = axis.error_;

    if (error_ == Axis::ERROR_NONE) {
        logInfo("axis: Error(s): NONE");
        return;
    }

    logInfo("axis: Error(s):");
    for (const auto& err : axis_error) {
        if (error_ & err.code) {
            logInfo("%s", err.desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first)
        logInfo("unknown error: 0x%08lX", 
            (uint32_t)error_);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
typedef struct {
    Motor::Error code;
    const char* desc;
} Motor_ErrorInfo_t;

static const Motor_ErrorInfo_t motor_error[] = {
    {Motor::ERROR_PHASE_RESISTANCE_OUT_OF_RANGE, "PHASE_RESISTANCE_OUT_OF_RANGE"},
    {Motor::ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE, "PHASE_INDUCTANCE_OUT_OF_RANGE"},
    {Motor::ERROR_DRV_FAULT, "DRV_FAULT"},
    {Motor::ERROR_CONTROL_DEADLINE_MISSED, "CONTROL_DEADLINE_MISSED"},
    {Motor::ERROR_MODULATION_MAGNITUDE, "MODULATION_MAGNITUDE"},
    {Motor::ERROR_CURRENT_SENSE_SATURATION, "CURRENT_SENSE_SATURATION"},
    {Motor::ERROR_CURRENT_LIMIT_VIOLATION, "CURRENT_LIMIT_VIOLATION"},
    {Motor::ERROR_MODULATION_IS_NAN, "MODULATION_IS_NAN"},
    {Motor::ERROR_MOTOR_THERMISTOR_OVER_TEMP, "MOTOR_THERMISTOR_OVER_TEMP"},
    {Motor::ERROR_FET_THERMISTOR_OVER_TEMP, "FET_THERMISTOR_OVER_TEMP"},
    {Motor::ERROR_TIMER_UPDATE_MISSED, "TIMER_UPDATE_MISSED"},
    {Motor::ERROR_CURRENT_MEASUREMENT_UNAVAILABLE, "CURRENT_MEASUREMENT_UNAVAILABLE"},
    {Motor::ERROR_CONTROLLER_FAILED, "CONTROLLER_FAILED"},
    {Motor::ERROR_I_BUS_OUT_OF_RANGE, "I_BUS_OUT_OF_RANGE"},
    {Motor::ERROR_BRAKE_RESISTOR_DISARMED, "BRAKE_RESISTOR_DISARMED"},
    {Motor::ERROR_SYSTEM_LEVEL, "SYSTEM_LEVEL"},
    {Motor::ERROR_BAD_TIMING, "BAD_TIMING"},
    {Motor::ERROR_UNKNOWN_PHASE_ESTIMATE, "UNKNOWN_PHASE_ESTIMATE"},
    {Motor::ERROR_UNKNOWN_PHASE_VEL, "UNKNOWN_PHASE_VEL"},
    {Motor::ERROR_UNKNOWN_TORQUE, "UNKNOWN_TORQUE"},
    {Motor::ERROR_UNKNOWN_CURRENT_COMMAND, "UNKNOWN_CURRENT_COMMAND"},
    {Motor::ERROR_UNKNOWN_CURRENT_MEASUREMENT, "UNKNOWN_CURRENT_MEASUREMENT"},
    {Motor::ERROR_UNKNOWN_VBUS_VOLTAGE, "UNKNOWN_VBUS_VOLTAGE"},
    {Motor::ERROR_UNKNOWN_VOLTAGE_COMMAND, "UNKNOWN_VOLTAGE_COMMAND"},
    {Motor::ERROR_UNKNOWN_GAINS, "UNKNOWN_GAINS"},
    {Motor::ERROR_CONTROLLER_INITIALIZING, "CONTROLLER_INITIALIZING"},
    {Motor::ERROR_UNBALANCED_PHASES, "UNBALANCED_PHASES"},
};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void motor_dumperrors(int motor_id) 
{
    Motor::Error error_ = Motor::ERROR_NONE;
    bool first = true;

    if (motor_id < 0 || motor_id > 1) return;
    Axis& axis = axes[motor_id];

    error_ = axis.motor_.error_;

    if (error_ == Motor::ERROR_NONE) {
        logInfo("motor: Error(s): NONE");
        return;
    }

    logInfo("motor: Error(s):");
    for (const auto& err : motor_error) {
        if (error_ & err.code) {
            logInfo("%s", err.desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first)
        logInfo("unknown error: 0x%08lX", 
            (uint32_t)error_);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
typedef struct {
    Controller::Error code;
    const char* desc;
} Controller_ErrorInfo_t;

static const Controller_ErrorInfo_t controller_error[] = {
    {Controller::ERROR_OVERSPEED, "OVERSPEED"},
    {Controller::ERROR_INVALID_INPUT_MODE, "INVALID_INPUT_MODE"},
    {Controller::ERROR_UNSTABLE_GAIN, "UNSTABLE_GAIN"},
    {Controller::ERROR_INVALID_MIRROR_AXIS, "INVALID_MIRROR_AXIS"},
    {Controller::ERROR_INVALID_LOAD_ENCODER, "INVALID_LOAD_ENCODER"},
    {Controller::ERROR_INVALID_ESTIMATE, "INVALID_ESTIMATE"},
    {Controller::ERROR_INVALID_CIRCULAR_RANGE, "INVALID_CIRCULAR_RANGE"},
    {Controller::ERROR_SPINOUT_DETECTED, "SPINOUT_DETECTED"},
};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void controller_dumperrors(int motor_id) 
{
    Controller::Error error_ = Controller::ERROR_NONE;
    bool first = true;

    if (motor_id < 0 || motor_id > 1) return;
    Axis& axis = axes[motor_id];

    error_ = axis.controller_.error_;

    if (error_ == Controller::ERROR_NONE) {
        logInfo("controller: Error(s): NONE");
        return;
    }

    logInfo("controller: Error(s):");
    for (const auto& err : controller_error) {
        if (error_ & err.code) {
            logInfo("%s", err.desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first)
        logInfo("unknown error: 0x%08lX", 
            (uint32_t)error_);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
typedef struct {
    Encoder::Error code;
    const char* desc;
} Encoder_ErrorInfo_t;

static const Encoder_ErrorInfo_t encoder_error[] = {
    {Encoder::ERROR_UNSTABLE_GAIN, "UNSTABLE_GAIN"},
    {Encoder::ERROR_CPR_POLEPAIRS_MISMATCH, "CPR_POLEPAIRS_MISMATCH"},
    {Encoder::ERROR_NO_RESPONSE, "NO_RESPONSE"},
    {Encoder::ERROR_UNSUPPORTED_ENCODER_MODE, "UNSUPPORTED_ENCODER_MODE"},
    {Encoder::ERROR_ILLEGAL_HALL_STATE, "ILLEGAL_HALL_STATE"},
    {Encoder::ERROR_INDEX_NOT_FOUND_YET, "INDEX_NOT_FOUND_YET"},
    {Encoder::ERROR_ABS_SPI_TIMEOUT, "ABS_SPI_TIMEOUT"},
    {Encoder::ERROR_ABS_SPI_COM_FAIL, "ABS_SPI_COM_FAIL"},
    {Encoder::ERROR_ABS_SPI_NOT_READY, "ABS_SPI_NOT_READY"},
    {Encoder::ERROR_HALL_NOT_CALIBRATED_YET, "HALL_NOT_CALIBRATED_YET"},
};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void encoder_dumperrors(int motor_id) 
{
    Encoder::Error error_ = Encoder::ERROR_NONE;
    bool first = true;

    if (motor_id < 0 || motor_id > 1) return;
    Axis& axis = axes[motor_id];

    error_ = axis.encoder_.error_;

    if (error_ == Encoder::ERROR_NONE) {
        logInfo("encoder: Error(s): NONE");
        return;
    }

    logInfo("encoder: Error(s):");
    for (const auto& err : encoder_error) {
        if (error_ & err.code) {
            logInfo("%s", err.desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first)
        logInfo("unknown error: 0x%08lX", 
            (uint32_t)error_);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
typedef struct {
    SensorlessEstimator::Error code;
    const char* desc;
} SensorlessEstimator_ErrorInfo_t;

static const SensorlessEstimator_ErrorInfo_t sensorless_error[] = {
    {SensorlessEstimator::ERROR_UNSTABLE_GAIN, "UNSTABLE_GAIN"},
    {SensorlessEstimator::ERROR_UNKNOWN_CURRENT_MEASUREMENT, "UNKNOWN_CURRENT_MEASUREMENT"},
};

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void sensorless_estimator_dumperrors(int motor_id) 
{
    SensorlessEstimator::Error error_ = SensorlessEstimator::ERROR_NONE;
    bool first = true;

    if (motor_id < 0 || motor_id > 1) return;
    Axis& axis = axes[motor_id];

    error_ = axis.sensorless_estimator_.error_;

    if (error_ == SensorlessEstimator::ERROR_NONE) {
        logInfo("sensorless_estimator: Error(s): NONE");
        return;
    }

    logInfo("sensorless_estimator: Error(s):");
    for (const auto& err : sensorless_error) {
        if (error_ & err.code) {
            logInfo("%s", err.desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first)
        logInfo("unknown error: 0x%08lX", 
            (uint32_t)error_);
}

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" int dumperrors(int clear)
{
    system_dumperrors();
    logInfo("axis0");
        axis_dumperrors(0);
        motor_dumperrors(0);
        sensorless_estimator_dumperrors(0);
        encoder_dumperrors(0);
        controller_dumperrors(0); 
    logInfo("axis1");
        axis_dumperrors(1);
        motor_dumperrors(1);
        sensorless_estimator_dumperrors(1);
        encoder_dumperrors(1);
        controller_dumperrors(1); 
    if (clear) odrv.clear_errors();
    return 1;
}

typedef struct {
    int code;
    const char* desc;
} _ErrorInfo_t;

// @brief Executes the system control command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
extern "C" void dumperrors_any(const char * tttle, int motor_id, 
    int error, _ErrorInfo_t * errors, int num) 
{
    int error_ = 0;

    if (motor_id < 0 || motor_id > 1) return;
    Axis& axis = axes[motor_id];

    error_ = error;

    if (error_ == 0) {
        logInfo("%s Error(s): NONE", tttle);
        return;
    }

    bool first = true;
    logInfo("%s Error(s):", tttle);

    for (int i = 0; i < num; i++) {
        if (error_ & errors[i].code) {
            logInfo("%s", errors[i].desc);
            first = false;
        }
    }

    /**理论上不会走到这里（除非有未定义的错误位）*/
    if (first) {
        logInfo("Unknown error: 0x%08lX", 
            (uint32_t)error_);
    }
}

// @brief Executes the read parameter command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_read_property(char * pStr, bool use_checksum) {
    char name[MAX_LINE_LENGTH];

    if (sscanf(pStr, "r %255s", name) < 1) {
        respond(use_checksum, "invalid command format");
    } else {

    }
}

// @brief Executes the set write position command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_write_property(char * pStr, bool use_checksum) {
    char name[MAX_LINE_LENGTH];
    char value[MAX_LINE_LENGTH];

    if (sscanf(pStr, "w %255s %255s", name, value) < 1) {
        respond(use_checksum, "invalid command format");
    } else {

    }
}

// @brief Executes the motor watchdog update command
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_update_axis_wdg(char * pStr, bool use_checksum) {
    unsigned motor_number;

    if (sscanf(pStr, "u %u", &motor_number) < 1) {
        respond(use_checksum, "invalid command format");
    } else if (motor_number >= AXIS_COUNT) {
        respond(use_checksum, "invalid motor %u", motor_number);
    } else {
        axes[motor_number].watchdog_feed();
    }
}

// @brief Sends the unknown command response
// @param pStr buffer of ASCII encoded values
// @param response_channel reference to the stream to respond on
// @param use_checksum bool to indicate whether a checksum is required on response
void AsciiProtocol::cmd_unknown(char * pStr, bool use_checksum) {
    (void)pStr;
    respond(use_checksum, "unknown command");
}

void AsciiProtocol::on_read_finished(bool result) {

}

void AsciiProtocol::start() {

}
