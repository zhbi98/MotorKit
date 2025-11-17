#ifndef __ODRIVE_CAN_HPP
#define __ODRIVE_CAN_HPP

#include <cmsis_os.h>
#include "stm32f4xx_hal.h"
#include <autogen/interfaces.hpp>

#define CAN_CLK_HZ (42000000)
#define CAN_CLK_MHZ (42)

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
        void set_baud_rate(uint32_t value) { parent->set_baud_rate(value); }
    };

    ODriveCAN() {}

    bool apply_config();
    bool start_server(CAN_HandleTypeDef* handle);

    Error error_ = ERROR_NONE;

    Config_t config_;

    osThreadId thread_id_;
    const uint32_t stack_size_ = 1024;  // Bytes

private:
    static const uint8_t kCanFifoNone = 0xff;

    bool reinit();
    void can_server_thread();
    bool set_baud_rate(uint32_t baud_rate);
    void process_rx_fifo(uint32_t fifo);
    bool send_message();
    bool subscribe();
    bool unsubscribe();

    // Hardware supports at most 28 filters unless we do optimizations. For now
    // we don't need that many.
    CAN_HandleTypeDef *handle_ = nullptr;
};

#endif  // __ODRIVE_CAN_HPP
