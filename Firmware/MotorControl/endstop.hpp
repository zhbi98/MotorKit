#ifndef __ENDSTOP_HPP
#define __ENDSTOP_HPP

#include "timer.hpp"
class Endstop {
   public:
    struct Config_t {
        float offset = 0;
        uint32_t debounce_ms = 50;
        bool enabled = false;
        bool is_active_high = false;

        // custom setters
        Endstop* parent = nullptr;
        void set_enabled(uint32_t value) { enabled = value; parent->apply_config(); }
        void set_debounce_ms(uint32_t value) { debounce_ms = value; parent->apply_config(); }
    };

    Endstop(Stm32Gpio End_STOP_Pin);

    Endstop::Config_t config_;
    Axis* axis_ = nullptr;

    bool apply_config();

    void update();
    constexpr bool get_state(){
        return endstop_state_;
    }

    constexpr bool rose(){
        return (endstop_state_ != last_state_) && endstop_state_;
    }

    constexpr bool fell(){
        return (endstop_state_ != last_state_) && !endstop_state_;
    }

    bool endstop_state_ = false;

   private:
    Stm32Gpio _End_STOP_Pin;
    bool last_state_ = false;
    bool pin_state_ = false;
    float pos_when_pressed_ = 0.0f;
    Timer<float> debounceTimer_;
};
#endif