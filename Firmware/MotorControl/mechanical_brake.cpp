#include <odrive_main.h>

MechanicalBrake::MechanicalBrake(Stm32Gpio Brake_Pin) : 
	Brake_Pin_(Brake_Pin)
{
}

void MechanicalBrake::engage() {
	if (0){ /*通过硬件 IO 将电路切换到动能回收充电电路*/
		Brake_Pin_.write(config_.is_active_low ? 0 : 1);
	}
}

void MechanicalBrake::release() {
	if (0){ /*通过硬件 IO 将电路切出动能回收充电电路*/
		Brake_Pin_.write(config_.is_active_low ? 1 : 0);
	}
}
