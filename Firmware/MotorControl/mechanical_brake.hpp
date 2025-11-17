#ifndef __MECHANICAL_BRAKE_HPP
#define __MECHANICAL_BRAKE_HPP

#include <autogen/interfaces.hpp>

class MechanicalBrake : public ODriveIntf::MechanicalBrakeIntf  {
   public:
    struct Config_t {
        bool is_active_low = true;

        // custom setters
        MechanicalBrake* parent = nullptr;
    };

    MechanicalBrake(Stm32Gpio Brake_Pin);

    MechanicalBrake::Config_t config_;
    Axis* axis_ = nullptr;
    Stm32Gpio Brake_Pin_;

    void release();
    void engage();
};
#endif // __MECHANICAL_BRAKE_HPP