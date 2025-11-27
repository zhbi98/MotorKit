#include "autogen/interfaces.hpp"
#include "odrive_can.hpp"
#include "odrive_main.h"
#include <can.h>
#include "log.h"

CAN_TxHeaderTypeDef txHeader = {
    .StdId = 0x00, .ExtId = 0x00,
    .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8,
    .TransmitGlobalTime = DISABLE,
};

/**
 * Converts a floating-point value x to an integer representation 
 * within a specified range [x_min, x_max] using a given number of bits.
 * This is commonly used in CAN communication to quantize physical signals 
 * (e.g., temperature, voltage) into fixed-bit integers for transmission.
 *
 * @param x      The floating-point value to convert
 * @param x_min  Minimum of the input range
 * @param x_max  Maximum of the input range
 * @param bits   Number of bits used for quantization (e.g., 8, 12, 16)
 * @return       Quantized integer value in the range [0, (1<<bits)-1]
 */
uint32_t ODriveCAN::fto_i(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min; /*浮点范围*/
    float ref = x_min; /*输入范围起点*/

    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;

    return (uint32_t)((x - ref) * ((float)((1 << bits) - 1)) / span);
}

/**
 * Converts an integer value u back to a floating-point value 
 * within a specified range [x_min, x_max], using the inverse 
 * of the quantization process performed by FloatToInt.
 *
 * @param u      The integer value to convert (quantized input)
 * @param x_min  Minimum of the output range
 * @param x_max  Maximum of the output range
 * @param bits   Number of bits used during original quantization
 * @return       Reconstructed floating-point value
 */
float ODriveCAN::ito_f(uint32_t u, float x_min, float x_max, uint8_t bits)
{
    if (bits <= 0 || bits > 32) return x_min;

    float span = x_max - x_min;
    float max_uint = (float)((1U << bits) - 1);
    return x_min + (span * (float)u) / max_uint;
}

/**
 * CAN is configured in identifier masking mode, 
 * all packets are received, and the corresponding ID packets are filtered 
 * out in the form of software masks, which is commonly 
 * referred to as software filtering
 */
uint16_t ODriveCAN::build_stdId(uint8_t _nodeId, uint8_t _cmd)
{
    uint16_t cmd = 0xFF;
    /*# 4Bits node ID & 7Bits Msg*/
    cmd = ((_nodeId & 0x0F) << 7) | (_cmd & 0x7F);
    return cmd;
}

/**
 * CAN is configured in identifier masking mode, 
 * all packets are received, and the corresponding ID packets are filtered 
 * out in the form of software masks, which is commonly 
 * referred to as software filtering
 */
bool ODriveCAN::apply_cmd(uint8_t _cmd, uint8_t * _data, uint32_t _len)
{
    uint8_t canNodeId = config_.node_id;
    uint8_t motor_id = 0;

    if (!_len || _len > 7 || _data == NULL) return 0;

    switch (_cmd) {
    /*0x00~0x0F No Memory CMDs*/
    case 0x01: /*Enable Motor*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
        axis.controller_.input_vel_ = 0.0f;

        uint32_t start = *((uint32_t *)(&_data[1]));
        axis.requested_state_ = (start == 1) ? Axis::AXIS_STATE_CLOSED_LOOP_CONTROL : Axis::AXIS_STATE_IDLE;
        break;
    }
    case 0x02: /*Do Calibration*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];
        axis.requested_state_ = Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
        break;
    }
    case 0x03: /*Set Current SetPoint*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        /**设置电机恒定力矩运动模式，再设置目标力矩*/
        if (axis.controller_.config_.control_mode != Controller::CONTROL_MODE_TORQUE_CONTROL)
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
        float torque_setpoint = *((float *)(&_data[1]));
        axis.controller_.input_torque_ = torque_setpoint;
        axis.watchdog_feed();
        break;
    }
    case 0x04: /*Set Velocity SetPoint*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        /**设置电机速度运动模式，再设置目标速度*/
        if (axis.controller_.config_.control_mode != Controller::CONTROL_MODE_VELOCITY_CONTROL)
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
        float vel_setpoint = *((float *)(&_data[1]));
        axis.controller_.input_vel_ = vel_setpoint;
        axis.watchdog_feed();
        break;
    }
    case 0x05: /*Set Position SetPoint*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        /**设置电机位置运动模式，再设置目标位置*/
        if (axis.controller_.config_.control_mode != Controller::CONTROL_MODE_POSITION_CONTROL)
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        float pos_setpoint = *((float *)(&_data[1]));
        axis.controller_.input_pos_ = pos_setpoint;

        bool finished_ack = _data[5];
        if (finished_ack) { /*Need Position & Finished ACK*/
            _data[0] = motor_id;
            float pos_estimate = (float)axis.encoder_.pos_estimate_.any().value_or(0.0f);
            uint8_t * bin = (uint8_t *)&pos_estimate;
            for (int8_t i = 0; i < 4; i++) _data[i + 1] = *(bin + i);

            _data[5] = (axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;
            txHeader.StdId = (canNodeId << 7) | 0x23;
            CAN_Send(&txHeader, _data);
        }

        axis.watchdog_feed();
    }
        break;
    case 0x06: /*Set Position with Time*/
    {

    }
        break;
    case 0x07: /*Set Position with Velocity-Limit*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        /**设置电机位置运动模式，再设置目标位置*/
        if (axis.controller_.config_.control_mode != Controller::CONTROL_MODE_POSITION_CONTROL)
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
        float pos_setpoint = *((float *)(&_data[1]));
        axis.controller_.input_pos_ = pos_setpoint;

        /**判断参数个数，确定是否包含速度限制*/
        int32_t _vel_limit = (int32_t)(*(int32_t *)(_data[5]));
        float vel_limit = ito_f(_vel_limit, -200.0f, +200.0f, 12);
        axis.controller_.config_.vel_limit = vel_limit;

        /**The new rated speed needs to be synchronized to 
        the position tracker, which needs the rated 
        speed to generate the process speed.*/
        axis.controller_.input_pos_updated();
        axis.watchdog_feed();
    }
        break;


    /*0x10~0x1F CMDs with Memory*/
    case 0x11: /*Set Node-ID and Store to EEPROM*/
    {
        config_.node_id = *(uint32_t*)(_data);
        bool stored = (bool)_data[4];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) {/*It need to be stored*/
            odrv.save_configuration();
        }
        break;
    }
    case 0x12: /*Set Current-Limit and Store to EEPROM*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        float current_lim = *((float *)(&_data[1]));
        axis.motor_.config_.current_lim = current_lim;
        bool stored = (bool)_data[5];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) { /*It need to be stored*/
            odrv.save_configuration();
        }
        break;
    }
    case 0x13: /*Set Velocity-Limit and Store to EEPROM*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        float vel_limit = *((float *)(&_data[1]));
        axis.controller_.config_.vel_limit = vel_limit;
        bool stored = (bool)_data[5];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) { /*It need to be stored*/
            odrv.save_configuration();
        }
        break;
    }
    case 0x14: /*Set Acceleration （and Store to EEPROM）*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        float _limit = *((float *)(&_data[1]));
        /**The new speed acc needs to be synchronized to 
        the speed tracker, which needs the speed 
        acc to generate the process acc.*/
        axis.trap_traj_.config_.accel_limit = _limit;
        axis.trap_traj_.config_.decel_limit = _limit;
        bool stored = (bool)_data[5];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) { /*It need to be stored*/
            odrv.save_configuration();
        }
    }
        break;
    case 0x16: /*Set Auto-Enable and Store to EEPROM*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        uint32_t pre_calibrated = *((uint32_t *)(&_data[1]));
        axis.encoder_.config_.pre_calibrated = pre_calibrated;
    }
        break;
    case 0x17: /*Set pos gain*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        float pos_gain = *((float *)(&_data[1]));
        axis.controller_.config_.pos_gain = pos_gain;
        bool stored = (bool)_data[5];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) { /*It need to be stored*/
            odrv.save_configuration();
        }
    }
        break;
    case 0x18: /*Set vel gain*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        float vel_gain = *((float *)(&_data[1]));
        axis.controller_.config_.vel_gain = vel_gain;
        bool stored = (bool)_data[5];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) { /*It need to be stored*/
            odrv.save_configuration();
        }
    }
        break;
    case 0x19: /*Set Integrator gain*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        float vel_gain = *((float *)(&_data[1]));
        axis.controller_.config_.vel_integrator_gain = vel_gain;
        bool stored = (bool)_data[5];

        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;

        /*存储参数只在失能模式下生效*/
        if (stored && idle) { /*It need to be stored*/
            odrv.save_configuration();
        }
    }


    /*0x20~0x2F Inquiry CMDs*/
    case 0x21: /*Get Current*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        _data[0] = motor_id;
        float Ibus = (float)axis.motor_.I_bus_;
        uint8_t * bin = (uint8_t *)&Ibus;
        for (int8_t i = 0; i < 4; i++) _data[i + 1] = *(bin + i);
        /*Finished ACK*/
        _data[5] = axis.current_state_ == Axis::AXIS_STATE_IDLE ? 1 : 0;
        txHeader.StdId = (canNodeId << 7) | 0x21;
        CAN_Send(&txHeader, _data);
    }
        break;
    case 0x22: /*Get Velocity*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        _data[0] = motor_id;
        float vel_estimate = (float)axis.encoder_.vel_estimate_.any().value_or(0.0f);
        uint8_t * bin = (uint8_t *)&vel_estimate;
        for (int8_t i = 0; i < 4; i++) _data[i + 1] = *(bin + i);
        /*Finished ACK*/
        _data[5] = axis.current_state_ == Axis::AXIS_STATE_IDLE ? 1 : 0;
        txHeader.StdId = (canNodeId << 7) | 0x22;
        CAN_Send(&txHeader, _data);
    }
        break;
    case 0x23: /*Get Position*/
    {
        motor_id = _data[0];
        if (motor_id < 0 || motor_id > 1) return 0;
        Axis& axis = axes[motor_id];

        _data[0] = motor_id;
        float pos_estimate = (float)axis.encoder_.pos_estimate_.any().value_or(0.0f);
        uint8_t * bin = (uint8_t *)&pos_estimate;
        for (int8_t i = 0; i < 4; i++) _data[i + 1] = *(bin + i);
        /*Finished ACK*/
        _data[5] = axis.current_state_ == Axis::AXIS_STATE_IDLE ? 1 : 0;
        txHeader.StdId = (canNodeId << 7) | 0x23;
        CAN_Send(&txHeader, _data);
    }
        break;
    case 0x24: /*Get Offset*/
    {

    }
        break;

    case 0x7c: /*Clear Errors*/
        /*Might want to clear axis errors only*/
        odrv.clear_errors();
        break;

    case 0x7d: /*Save Configs*/
    {
        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;
        /*存储参数只在失能模式下生效*/
        if (!idle) break;
        /*CONFIG_STORE;*/
        odrv.save_configuration();
        break;
    }
    case 0x7e: /*Erase Configs*/
    {
        uint8_t idle = 1;
        for (auto& axis: axes) idle &= (
            axis.current_state_ == Axis::AXIS_STATE_IDLE) ? 1 : 0;
        /*擦除参数只在失能模式下生效*/
        if (!idle) break;
        /*CONFIG_RESTORE;*/
        odrv.erase_configuration();
        break;
    }
    case 0x7f: /*Reboot*/
        HAL_NVIC_SystemReset();
        break;
    default: break;
    }

    return 1;
}

// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ODriveCAN *> ctxMap;

/**
 * Applies the current configuration to the ODrive CAN interface.
 * This function sets up the configuration by associating the parent object
 * (this instance) with the config structure and initializing the CAN baud rate.
 * @return true, indicating that the configuration was successfully applied
 */
bool ODriveCAN::apply_config()
{
    config_.parent = this;
    set_baud_rate(config_.baud_rate);
    return true;
}

/**
 * Reinitializes the CAN peripheral used by ODrive (CAN1).
 * This function resets and reconfigures the CAN hardware at the low level,
 * typically after a communication failure or during startup.
 * @return true, indicating that the reinitialization was attempted successfully
 */
bool ODriveCAN::reinit() 
{
    MX_CAN1_Init();
    return true;
}

// Set one of only a few common baud rates.  
// CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector 
// Informatik GmbH for high reliability systems).
// Conveniently, the CAN peripheral's 42MHz clock lets us 
// easily create 21TQs for all common baud rates
bool ODriveCAN::set_baud_rate(uint32_t baud_rate) 
{
    CAN_Set_BaudRate(baud_rate);
    return true;
}
