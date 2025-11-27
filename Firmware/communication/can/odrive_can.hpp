#ifndef __ODRIVE_CAN_HPP
#define __ODRIVE_CAN_HPP

#include <cmsis_os.h>
#include "stm32f4xx_hal.h"
#include <autogen/interfaces.hpp>
#include <stdint.h>

// Anonymous enum for defining the most common CAN baud rates
enum {
    CAN_BAUD_125K = 125000,
    CAN_BAUD_250K = 250000,
    CAN_BAUD_500K = 500000,
    CAN_BAUD_1000K = 1000000,
    CAN_BAUD_1M = 1000000
};

class ODriveCAN : public ODriveIntf::CanIntf {
public:
    struct Config_t {
        uint32_t baud_rate = CAN_BAUD_250K;
        Protocol protocol = PROTOCOL_SIMPLE;
        ODriveCAN* parent = nullptr; // set in apply_config()
        uint32_t node_id = 0;
        void set_baud_rate(uint32_t value) {
            parent->set_baud_rate(value);
        }
    };

    ODriveCAN() {}

    bool apply_config();
    bool apply_cmd(uint8_t _cmd, uint8_t * _data, uint32_t _len);

    Error error_ = ERROR_NONE;

    Config_t config_;

    const uint32_t stack_size_ = 1024;  // Bytes

private:

    bool reinit();
    bool set_baud_rate(uint32_t baud_rate);
    uint32_t fto_i(float x, float x_min, float x_max, uint8_t bits);
    float ito_f(uint32_t u, float x_min, float x_max, uint8_t bits);
    uint16_t build_stdId(uint8_t _nodeId, uint8_t _cmd);
};

#endif  // __ODRIVE_CAN_HPP
