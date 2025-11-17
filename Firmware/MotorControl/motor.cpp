
#include "motor.hpp"
#include "axis.hpp"
#include "low_level.h"
#include "odrive_main.h"

#include <algorithm>

static constexpr auto CURRENT_ADC_LOWER_BOUND =        (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f);
static constexpr auto CURRENT_ADC_UPPER_BOUND =        (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f);

/**
 * @brief This control law adjusts the output voltage such that a predefined
 * current is tracked. A hardcoded integrator gain is used for this.
 * 
 * TODO: this might as well be implemented using the FieldOrientedController.
 */
struct ResistanceMeasurementControlLaw : AlphaBetaFrameController {
    void reset() final {
        test_voltage_ = 0.0f;
        test_mod_ = std::nullopt;
    }

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final {

        if (Ialpha_beta.has_value()) {
            actual_current_ = Ialpha_beta->first;
            test_voltage_ += (kI * current_meas_period) * (target_current_ - actual_current_);
            I_beta_ += (kIBetaFilt * current_meas_period) * (Ialpha_beta->second - I_beta_);
        } else {
            actual_current_ = 0.0f;
            test_voltage_ = 0.0f;
        }
    
        if (std::abs(test_voltage_) > max_voltage_) {
            test_voltage_ = NAN;
            return Motor::ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        } else if (!vbus_voltage.has_value()) {
            return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
        } else {
            float vfactor = 1.0f / ((2.0f / 3.0f) * *vbus_voltage);
            test_mod_ = test_voltage_ * vfactor;
            return Motor::ERROR_NONE;
        }
    }

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final {
        if (!test_mod_.has_value()) {
            return Motor::ERROR_CONTROLLER_INITIALIZING;
        } else {
            *mod_alpha_beta = {*test_mod_, 0.0f};
            *ibus = *test_mod_ * actual_current_;
            return Motor::ERROR_NONE;
        }
    }

    float get_resistance() {
        /*根据相电压和采样到的相电流，根据欧姆定律计算出电机相电阻*/
        return test_voltage_ / target_current_;
    }

    float get_Ibeta() {
        return I_beta_;
    }

    const float kI = 1.0f; // [(V/s)/A]
    const float kIBetaFilt = 80.0f;
    float max_voltage_ = 0.0f;
    float actual_current_ = 0.0f;
    float target_current_ = 0.0f;
    float test_voltage_ = 0.0f;
    float I_beta_ = 0.0f; // [A] low pass filtered Ibeta response
    std::optional<float> test_mod_ = NAN;
};

/**
 * @brief This control law toggles rapidly between positive and negative output
 * voltage. By measuring how large the current ripples are, the phase inductance
 * can be determined.
 * 
 * TODO: this method assumes a certain synchronization between current measurement and output application
 */
struct InductanceMeasurementControlLaw : AlphaBetaFrameController {
    void reset() final {
        attached_ = false;
    }

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final
    {
        if (!Ialpha_beta.has_value()) {
            return {Motor::ERROR_UNKNOWN_CURRENT_MEASUREMENT};
        }

        float Ialpha = Ialpha_beta->first;

        if (attached_) {
            float sign = test_voltage_ >= 0.0f ? 1.0f : -1.0f;
            deltaI_ += -sign * (Ialpha - last_Ialpha_);
        } else {
            start_timestamp_ = input_timestamp;
            attached_ = true;
        }

        last_Ialpha_ = Ialpha;
        last_input_timestamp_ = input_timestamp;

        return Motor::ERROR_NONE;
    }

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp, std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final
    {
        test_voltage_ *= -1.0f;
        float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
        *mod_alpha_beta = {test_voltage_ * vfactor, 0.0f};
        *ibus = 0.0f;
        return Motor::ERROR_NONE;
    }

    float get_inductance() {
        // Note: A more correct formula would also take into account that there is a finite timestep.
        // However, the discretisation in the current control loop inverts the same discrepancy
        /*dt 计算公式是连续时间近似，未考虑离散控制周期的影响（更正确的公式还应考虑到离散控制周期的时间步长），但由于控制环路也使用相同模型，误差相互抵消*/
        /*根据最后一次输入的时间戳（来自定时器）和测试开始的时间戳和定时器时钟频率计算出 dt*/
        float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ; // at 216MHz this overflows after 19 seconds
        /*依据法拉第电磁感应定律在电感中 V=L(dI/dt) 则 L=V/(dI/dt) 这样就可以计算出电机相绕组电感，dI/dt 为电流变化率（单位：A/s）*/
        return std::abs(test_voltage_) / (deltaI_ / dt);
    }

    // Config
    float test_voltage_ = 0.0f;

    // State
    bool attached_ = false;
    float sign_ = 0;

    // Outputs
    uint32_t start_timestamp_ = 0;
    float last_Ialpha_ = NAN;
    uint32_t last_input_timestamp_ = 0;
    float deltaI_ = 0.0f;
};


Motor::Motor(TIM_HandleTypeDef* timer,
             uint8_t current_sensor_mask,
             float shunt_conductance,
             TGateDriver& gate_driver,
             TOpAmp& opamp,
             OnboardThermistorCurrentLimiter& fet_thermistor,
             OffboardThermistorCurrentLimiter& motor_thermistor) :
        timer_(timer),
        current_sensor_mask_(current_sensor_mask),
        shunt_conductance_(shunt_conductance),
        gate_driver_(gate_driver),
        opamp_(opamp),
        fet_thermistor_(fet_thermistor),
        motor_thermistor_(motor_thermistor) {
    apply_config();
    fet_thermistor_.motor_ = this;
    motor_thermistor_.motor_ = this;
}

/**
 * @brief Arms the PWM outputs that belong to this motor.
 *
 * Note that this does not activate the PWM outputs immediately, it just sets
 * a flag so they will be enabled later.
 * 
 * The sequence goes like this:
 *  - Motor::arm() sets the is_armed_ flag.
 *  - On the next timer update event Motor::timer_update_cb() gets called in an
 *    interrupt context
 *  - Motor::timer_update_cb() runs specified control law to determine PWM values
 *  - Motor::timer_update_cb() calls Motor::apply_pwm_timings()
 *  - Motor::apply_pwm_timings() sets the output compare registers and the AOE
 *    (automatic output enable) bit.
 *  - On the next update event the timer latches the configured values into the
 *    active shadow register and enables the outputs at the same time.
 * 
 * The sequence can be aborted at any time by calling Motor::disarm().
 *
 * @param control_law: An control law that is called at the frequency of current
 *        measurements. The function must return as quickly as possible
 *        such that the resulting PWM timings are available before the next
 *        timer update event.
 * @returns: True on success, false otherwise
 */
bool Motor::arm(PhaseControlLaw<3>* control_law) {
    /*启动电机 PWM 输出，同时重置位置/速度环控制器参数（如清空 PID 积分器）等等*/
    axis_->mechanical_brake_.release();

    CRITICAL_SECTION() {
        control_law_ = control_law;

        // Reset controller states, integrators, setpoints, etc.
        axis_->controller_.reset();
        axis_->acim_estimator_.rotor_flux_ = 0.0f;
        if (control_law_) {
            control_law_->reset();
        }

        if (!odrv.config_.enable_brake_resistor || brake_resistor_armed) {
            armed_state_ = 1;
            is_armed_ = true;
        } else {
            error_ |= Motor::ERROR_BRAKE_RESISTOR_DISARMED;
        }
    }

    return true;
}

/**
 * @brief Updates the phase PWM timings unless the motor is disarmed.
 *
 * If the motor is armed, the PWM timings come into effect at the next update
 * event (and are enabled if they weren't already), unless the motor is disarmed
 * prior to that.
 * 
 * @param tentative: If true, the update is not counted as "refresh".
 */
void Motor::apply_pwm_timings(uint16_t timings[3], bool tentative) {
    CRITICAL_SECTION() {
        if (odrv.config_.enable_brake_resistor && !brake_resistor_armed) {
            disarm_with_error(ERROR_BRAKE_RESISTOR_DISARMED);
        }

        TIM_HandleTypeDef* htim = timer_;
        TIM_TypeDef* tim = htim->Instance;

        /**CCR 捕获/比较寄存器，该寄存器总共有 4 个 (TIMx_CCR1~4)，对应 4 个输通道 CH1~4 。
        这 4 个寄存器都差不多，以 TIMx_CCR1说明。在 PWM 模式下，CCR1 的值决定了 PWM 信号的占空比。
        当计数器（TIMx_CNT）的值小于 CCR1 时，输出为高电平。当计数器的值大于或等于 CCR1 时，
        输出为低电平。通过动态修改 CCR1 的值，可以实现占空比的实时调整。*/

        tim->CCR1 = timings[0]; /*TIM capture/compare register 1*/
        tim->CCR2 = timings[1]; /*TIM capture/compare register 2*/
        tim->CCR3 = timings[2]; /*TIM capture/compare register 3*/
        
        /*把 PWM 参数应用到 PWM 驱动*/
        if (!tentative) {
            if (is_armed_) {
                // Set the Automatic Output Enable so that the Master Output Enable
                // bit will be automatically enabled on the next update event.
                /*如果 AOE 位置 1，在下一个更新事件 UEV 时，MOE 位被自动置 1。*/
                tim->BDTR |= TIM_BDTR_AOE; /*TIM break and dead-time register*/
            }
        }

        // If a timer update event occurred just now while we were updating the
        // timings, we can't be sure what values the shadow registers now contain,
        // so we must disarm the motor.
        // (this also protects against the case where the update interrupt has too
        // low priority, but that should not happen)
        // if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)) {
        //     disarm_with_error(ERROR_CONTROL_DEADLINE_MISSED);
        // }

        /**
         * [MOE] (Main Output Enable)：当刹车输入有效时，该位会被硬件异步清 0。
         * 1：若设置了相应的使能位（即TIMX_CCER的CCxE、CCxNE）, 则开启 OC 和 OCN 输出。
         * 0：禁止 OC 和 OCN 输出或强制为空闲状态。
         * [AOE] (Automatic Output Enable)：TIM_AutomaticOutput，该位的设置与MOE相关。
         * 0：MOE 位只能被软件置1。
         * 1：MOE 被软件置1或者当刹车信号无效后被更新时间自动置1。也就是说当刹车信号有效时，
         * MOE 位被硬件清 0，该位若为 0，则 MOE 位只能由软件置 1（即使刹车信号出现一次，之后无效）
         * 若 AOE 位为 1，则 MOE 除了软件置 1 外，当刹车信号失效时，在下一个更新事件时，
         * MOE 会被自动置 1。
         */
    }
}

/**
 * @brief Disarms the motor PWM.
 * 
 * After this function returns, it is guaranteed that all three
 * motor phases are floating and will not be enabled again until
 * arm() is called.
 */
bool Motor::disarm(bool* p_was_armed) {
    bool was_armed;
    
    CRITICAL_SECTION() {
        was_armed = is_armed_;
        if (is_armed_) { /*解除电机 PWM 输出*/
            gate_driver_.set_enabled(false);
        }
        is_armed_ = false;
        armed_state_ = 0;
        TIM_HandleTypeDef* timer = timer_;
        timer->Instance->BDTR &= ~TIM_BDTR_AOE; // prevent the PWMs from automatically enabling at the next update
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timer);
        control_law_ = nullptr;
    }

    // Check necessary to prevent infinite recursion
    if (was_armed) {
        update_brake_current();
    }

    if (p_was_armed) {
        *p_was_armed = was_armed;
    }

    return true;
}

/**
 * 电流环控制器参数（PID 增益）的自动调谐逻辑，它基于电机的相电阻和相电感动态实时更新电流
 * 环的比例增益（P-gain）和积分增益（I-gain），这是实现高性能电流控制的关键部分。
 * 当电机相电阻 resistance 或相电感 inductance 发生变化时需要调用。
 * 详细查看：https://blog.csdn.net/MOS_JBET/article/details/147070157
 */
// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
void Motor::update_current_controller_gains() {
    // Calculate current control gains

    /*根据电机 phase_inductance 和 phase_resistance 和电流控制带宽 current_control_bandwidth 
    计算电流环的 P 增益 和 I 增益。这是基于模型的控制器调参（Model-Based Tuning），确保电流环的动态响应最优。*/
    float p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.pi_gains_ = {p_gain, plant_pole * p_gain};
}

bool Motor::apply_config() {
    config_.parent = this;
    is_calibrated_ = config_.pre_calibrated;
    update_current_controller_gains();
    return true;
}

/**
 * 这是 ODrive 对门级驱动的配置，以及关于电流采样和增益设置的相关操作。
 * 
 * 首先计算精确的增益，然后调整它以获得等于或大于请求的范围，否则使用可能的最大范围。
 * 
 * 常量定义：
 * constexpr float kMargin = 0.90f; 定义了一个常量kMargin，其值为0.90。这个常量用作一个安全裕量或缩放因子。
 * constexpr float max_output_swing = 1.35f; // [V] out of amplifier 定义了一个常量max_output_swing，表示放大器输出的最大摆幅为1.35伏特。
 * 
 * 计算最大单位增益电流：
 * float max_unity_gain_current = kMargin * max_output_swing * shunt_conductance_; // [A]
 * 这里计算了最大单位增益电流。它使用了之前定义的kMargin、max_output_swing和shunt_conductance_（分流电导，单位为西门子）。
 * 通过这个计算，我们得到了一个电流值，该值表示在最大输出摆幅和给定的分流电导下，放大器可以处理的最大电流。
 * 
 * 计算请求的增益：
 * float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]
 * 这里计算了请求的增益。增益是输出变化与输入变化的比率。在这里，它是电压比，所以单位是伏特/伏特（V/V）。
 * config_.requested_current_range很可能是用户或配置文件中设置的期望电流范围。
 * 通过将最大单位增益电流除以请求的电流范围，我们得到了一个增益值，该值表示在给定的电流范围内，输出电压如何随输入电压变化。
 * 
 * 总的来说，这段代码的目的是根据ODrive的配置和硬件限制（如放大器的最大输出摆幅和分流电阻的电导）
 * 来计算一个合适的增益值。这个增益值用于确保在期望的电流范围内，放大器可以正常工作，并且不会超出其规格或导致任何损坏。
 */
// @brief Set up the gate drivers
bool Motor::setup() {
    fet_thermistor_.update();
    motor_thermistor_.update();

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    constexpr float kMargin = 0.90f;
    /*1.35f 即最大输出摆幅 1.35V，也就是说经过运放放大 x 倍后的最大输出电压因该在 1.35V 之间。
    详细来说就是当采样电阻流过最大设计电流（例如 60A）时在采样电阻两端可以产生最大分压，
    而这个分压经过运放放大 x 倍后的最大输出电压因该在 1.35V 之间。*/
    constexpr float max_output_swing = 1.35f; // [V] out of amplifier
    float max_unity_gain_current = kMargin * max_output_swing * shunt_conductance_; // [A]
    float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]
    
    /**
     * 为什么 float max_unity_gain_current = kMargin * max_output_swing * shunt_conductance_; 会得到电流值？
     * 
     * 在这段代码中，max_unity_gain_current 的计算实际上是在确定放大器输出在最大摆幅时，通过分流电阻（shunt resistor）能够产生的最大电流。这里的“单位增益”（unity gain）并不是直接指放大器的增益为1，而是用于描述一种情况，即放大器的输出在其最大摆幅时，对应到分流电阻上的电流情况。
     * 然而，这里的命名可能会有些误导。通常，单位增益指的是放大器增益为1的情况，但在这里，它更像是用来描述在不超出放大器最大输出能力的前提下，通过分流电阻可能达到的最大电流。
     * 具体来说，max_unity_gain_current 的计算过程是这样的：
     * (1) max_output_swing 是放大器的最大输出电压摆幅，单位是伏特（V）。
     * (2) shunt_conductance_ 是分流电阻的电导，单位是西门子（S），它是电阻（单位 Ω）的倒数。电导越大，电阻越小，导电能力越强。
     * (3) kMargin 是一个安全系数，用来确保放大器输出不会达到其极限值，从而留出一些余量来处理可能的瞬态过载或其他非理想情况。
     * 
     * 因此，当我们将 max_output_swing（最大输出电压摆幅）乘以shunt_conductance_（分流电导）时，我们得到的是通过分流电阻的最大可能电流，单位是安培（A）。这是因为根据欧姆定律的变种形式，电流（I）等于电压（V）除以电阻（R），或者等于电压乘以电导（G）：
     * [I= U/R = U*G]
     * 
     * 在这里，我们用最大输出电压摆幅乘以电导来得到最大电流。然后，我们再乘以 kMargin 来降低这个最大值，以确保系统有一定的安全裕量。
     * 不过，需要注意的是，这里的命名和计算方式可能会让不熟悉这段代码的人感到困惑。更准确的命名可能是 
     * max_safe_current_through_shunt 或类似的名称，以更清晰地表明这个变量的含义。
     */

    float actual_gain;
    if (!gate_driver_.config(requested_gain, &actual_gain))
        return false;

    // Values for current controller
    phase_current_rev_gain_ = 1.0f / actual_gain;
    // Clip all current control to actual usable range
    max_allowed_current_ = max_unity_gain_current * phase_current_rev_gain_;

    max_dc_calib_ = 0.1f * max_allowed_current_;

    if (!gate_driver_.init())
        return false;

    return true;
}

void Motor::disarm_with_error(Motor::Error error){
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
    last_error_time_ = odrv.n_evt_control_loop_ * current_meas_period;
    disarm();
}

/*电机状态检查*/
bool Motor::do_checks(uint32_t timestamp) {
    gate_driver_.do_checks();

    if (!gate_driver_.is_ready()) {
        disarm_with_error(ERROR_DRV_FAULT);
        return false;
    }
    if (!motor_thermistor_.do_checks()) {
        disarm_with_error(ERROR_MOTOR_THERMISTOR_OVER_TEMP);
        return false;
    }
    if (!fet_thermistor_.do_checks()) {
        disarm_with_error(ERROR_FET_THERMISTOR_OVER_TEMP);
        return false;
    }
    return true;
}

/**
 * 计算电机运行时允许的实时最大电流值 effective current limit（单位：A），用于后续控制过程中处理电流限制。
 * 详细可查看：https://blog.csdn.net/MOS_JBET/article/details/147070310
 */
float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim; /*获取用户设定的电流限制（如30A）*/
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        /*综合配置值、硬件能力、电压限制等多个限制因素，确保电流控制既满足用户需求，又不会超出系统安全范围。*/
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage); //gimbal motor is voltage control
    } else {
        current_lim = std::min(current_lim, axis_->motor_.max_allowed_current_);
    }

    // Apply thermistor current limiters
    current_lim = std::min(current_lim, motor_thermistor_.get_current_limit(config_.current_lim));
    current_lim = std::min(current_lim, fet_thermistor_.get_current_limit(config_.current_lim));
    effective_current_lim_ = current_lim; /*更新电流限制用于后续控制过程中处理电流限制*/

    return effective_current_lim_;
}

/*计算电机的最大可用扭矩，用于三环闭环控制过程中处理扭矩限制，注意对于 ACIM 电机可用扭矩允许为 0。*/
//return the maximum available torque for the motor.
//Note - for ACIM motors, available torque is allowed to be 0.
float Motor::max_available_torque() {
    /*effective_current_lim_ 有效电流限制 (A) 通常是 config_.current_lim 经过弱磁、电压限制等调整后的值*/
    /*torque_constant 扭矩常数 (N·m/A) 与电机设计相关，Kt=3/2*p*Ψ（p: 极对数，Ψ: 磁链）对于 PMSM 电机通常是固定值*/
    /*rotor_flux_ 转子磁通 (Weber, Wb) ACIM 电机的磁通是可调节的*/
    if (config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        /*ACIM 电机 Torque_Max=Ieff*Kt*Ψ*/
        float max_torque = effective_current_lim_ * config_.torque_constant * axis_->acim_estimator_.rotor_flux_;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    } else {
        /*PMSM 或步进电机 Torque_Max=Ieff*Kt*/
        float max_torque = effective_current_lim_ * config_.torque_constant;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    }
}

/*把在电流采样电路采样到的分压电压对应的 ADC 原始值转换为电机相电流（单位：安培 A）*/
std::optional<float> Motor::phase_current_from_adcval(uint32_t ADCValue) {
    // Make sure the measurements don't come too close to the current sensor's hardware limitations
    if (ADCValue < CURRENT_ADC_LOWER_BOUND || ADCValue > CURRENT_ADC_UPPER_BOUND) {
        error_ |= ERROR_CURRENT_SENSE_SATURATION;
        return std::nullopt;
    }

    /*去除 ADC 偏置（硬件零漂）1<<11=2048 将原始 ADC 值转换为有符号数 (-2048~+2047)，代表电流方向（正/负）。*/
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal; /*转换为放大器输出电压（采样电阻电流分压在进入 ADC 前还经过运算放大器放大）*/
    float shunt_volt = amp_out_volt * phase_current_rev_gain_;
    float current = shunt_volt * shunt_conductance_;
    return current;
}

//--------------------------------
// Measurement and calibration
//--------------------------------

/**
 * 用于测量电机相电阻（phase resistance）的函数，通过注入测试电流并测量电压降来计算电阻值。
 * 动态测量电机相电阻（单位：Ω），用于自动校准或故障检测。
 * 输入参数：test_current 测试电流幅值（如5A）。max_voltage 允许的最大测试电压（防止过压损坏）。
 * 输出：true 测量成功，结果存入 config_.phase_resistance。false 测量失败（错误码记录在 axis_->error_）。
 */
// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    ResistanceMeasurementControlLaw control_law;
    control_law.target_current_ = test_current;
    control_law.max_voltage_ = max_voltage;

    arm(&control_law);

    for (size_t i = 0; i < 3000; ++i) {
        if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_)) {
            break;
        }
        osDelay(1);
    }

    bool success = is_armed_;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    disarm();

    config_.phase_resistance = control_law.get_resistance();
    if (is_nan(config_.phase_resistance)) {
        // TODO: the motor is already disarmed at this stage. This is an error
        // that only pretains to the measurement and its result so it should
        // just be a return value of this function.
        disarm_with_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
        success = false;
    }

    float I_beta = control_law.get_Ibeta();
    if (is_nan(I_beta) || (abs(I_beta) / test_current) > 0.2f) {
        disarm_with_error(ERROR_UNBALANCED_PHASES);
        success = false;
    }

    return success;
}

/**
 * 测量电机相电感（phase inductance）函数，通过施加交变电压并测量电流变化率来计算电感值。
 * 动态测量电机相电感（单位：H），用于电机参数自动辨识。
 * 输入参数：voltage_low 低电平测试电压（如-5V）voltage_high 高电平测试电压（如+5V）。
 * 输出：true 测量成功，结果存入 config_.phase_inductance，false 测量失败（错误码记录在 axis_->error_）
 */
bool Motor::measure_phase_inductance(float test_voltage) {
    InductanceMeasurementControlLaw control_law;
    control_law.test_voltage_ = test_voltage;

    arm(&control_law);

    for (size_t i = 0; i < 1250; ++i) {
        if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_)) {
            break;
        }
        osDelay(1);
    }

    bool success = is_armed_;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    disarm();

    config_.phase_inductance = control_law.get_inductance();
    
    // TODO arbitrary values set for now
    if (!(config_.phase_inductance >= 2e-6f && config_.phase_inductance <= 4000e-6f)) {
        /*问题可能因为电流采样电阻值定义错误，电源供电不稳定，硬件电路连接异常导致采集不到有效的电流导致电机相电感测量无效*/
        error_ |= ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        success = false;
    }

    return success;
}


/*注意：电机校准并不是校准电机的旋转角度误差，而是在该过程中测量出电机的相电阻和相电感，
同时保存这两个参数，所以在电机校准这个过程中电机并不会也不需要旋转*/
// TODO: motor calibration should only be a utility function that's called from
// the UI on explicit user request. It should take its parameters as input
// arguments and return the measured results without modifying any config values.
bool Motor::run_calibration() {
    float R_calib_max_voltage = config_.resistance_calib_max_voltage;
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT
        || config_.motor_type == MOTOR_TYPE_ACIM) {
        if (!measure_phase_resistance(config_.calibration_current, R_calib_max_voltage))
            return false;
        if (!measure_phase_inductance(R_calib_max_voltage))
            return false;
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        // no calibration needed
    } else {
        return false;
    }

    update_current_controller_gains();
    
    is_calibrated_ = true;
    return true;
}

/**该函数实现了扭矩到电流的转换、限幅、前馈补偿，为后续 FOC 控制环准备好 Idq 电流 Vdq 电压设定值*/
void Motor::update(uint32_t timestamp) {
    // Load torque setpoint, convert to motor direction（获取来自 Controller 的目标扭矩）
    std::optional<float> maybe_torque = torque_setpoint_src_.present();
    if (!maybe_torque.has_value()) {
        error_ |= ERROR_UNKNOWN_TORQUE;
        return;
    }
    float torque = direction_ * *maybe_torque;

    /*读取上一次的 Id（直轴电流）和 Iq（交轴电流）设定值。*/
    // Load setpoints from previous iteration.
    auto [id, iq] = Idq_setpoint_.previous()
                     .value_or(float2D{0.0f, 0.0f});

    /*获取允许最大电流 effective_current_lim_ 参数*/
    // Load effective current limit
    float ilim = axis_->motor_.effective_current_lim_;

    /*Autoflux 跟踪的是上一次的 Iq (此时的 Iq 还没有更新，需要到下面才会进行更新), 以确保我们追踪的是一个可行的电流。*/
    // Autoflux tracks old Iq (that may be 2-norm clamped last cycle) to make sure we are chasing a feasable current.
    if ((axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) && config_.acim_autoflux_enable) {
        float abs_iq = std::abs(iq);
        float gain = abs_iq > id ? config_.acim_autoflux_attack_gain : config_.acim_autoflux_decay_gain;
        id += gain * (abs_iq - id) * current_meas_period;
        id = std::clamp(id, config_.acim_autoflux_min_Id, 0.9f * ilim); // 10% space reserved for Iq
    } else {
        id = std::clamp(id, -ilim * 0.99f, ilim * 0.99f); // 1% space reserved for Iq to avoid numerical issues
    }

    /*扭矩转电流,对 BLDC, PMSM 电机而言直接除以力矩常数即可，ACIM 电机分母还要乘以磁链 (rotor_flux)。*/
    // Convert requested torque to current
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        iq = torque / (axis_->motor_.config_.torque_constant * std::max(axis_->acim_estimator_.rotor_flux_, config_.acim_gain_min_flux));
    } else {
        iq = torque / axis_->motor_.config_.torque_constant;
    }

    // 2-norm clamping where Id takes priority（当 d 轴电流 id 确定后，iq 的最大允许值由 
    // Iqmax=sqrt(Ilim^2-Id^2) 确定指空间上两个向量矩阵的直线距离，类似于求棋盘上两点间的直线距离）
    float iq_lim_sqr = SQ(ilim) - SQ(id); /*计算约束方程的平方项差值*/
    float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr); /*开平方得到 q 轴电流上限*/
    iq = std::clamp(iq, -Iq_lim, Iq_lim); /*将实际 iq 钳制到 [-Iq_lim, Iq_lim] 区间*/

    /*更新电流设定值，除 GIMBAL 电机外，更新 Idq_setpoint_*/
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) {
        Idq_setpoint_ = {id, iq};
    }

    /*ACIM 磁链估算器更新，调用 acim_estimator_.update(timestamp)。*/
    // This update call is in bit a weird position because it depends on the
    // Id,q setpoint but outputs the phase velocity that we depend on later
    // in this function.
    // A cleaner fix would be to take the feedforward calculation out of here
    // and turn it into a separate component.
    MEASURE_TIME(axis_->task_times_.acim_estimator_update)
        axis_->acim_estimator_.update(timestamp);

    float vd = 0.0f;
    float vq = 0.0f;

    std::optional<float> phase_vel = phase_vel_src_.present();

    /**
     * 前馈控制主要应用于该控制系统本身存在较大的滞后作用，如果单单是在系统产生误差之后去控制的话 (PID)，
     * 那么系统总是跟在干扰后面波动，系统无法稳定。前馈的基本观点就是建立按扰动量进行补偿的开环控制，
     * 也就是说当影响系统的扰动出现时，按照扰动量的大小直接产生相应的矫正作用，抵消扰动的影响。
     * 当控制算式选的恰当时，可以达到很高的控制精度。
     * 反馈控制：也就是输出受到扰动影响后采取校正措施，因此反馈控制是闭环控制系统。
     * 前馈控制：也就是在输出受到干扰之前采取纠正措施，前馈控制并不依赖于反馈信号。
     */

    /**
     * 例如考虑一台热水器，要求在输入水温和水量变化时，维持输出水温不变。
     * 反馈控制：出水水温低于设定值，则加大热源功率，出水水温高于设定值，则减小热源功率。
     * 前馈控制：加装进水温度传感器和水量传感器，移除出水温度传感器，根据进水温度与目标温度的温差和水量，
     * 直接计算得到所需的加热功率，然后直接调整热源。
     */

    /*根据 R-wL 开启/关闭前馈电压计算，根据电机相电感配置，得出反电动势扰动来计算前馈，调整更新 Vd, Vq 电压*/
    /*前馈，反馈两者结合，前馈作为先导控制，反馈作为后随控制，保证控制系统极高的稳定性*/
    if (config_.R_wL_FF_enable) { /*电阻 R 和电感 L 相关的前馈补偿*/
        if (!phase_vel.has_value()) {
            error_ |= ERROR_UNKNOWN_PHASE_VEL;
            return;
        }

        vd -= *phase_vel * config_.phase_inductance * iq;
        vq += *phase_vel * config_.phase_inductance * id;
        vd += config_.phase_resistance * id;
        vq += config_.phase_resistance * iq;
    }

    if (config_.bEMF_FF_enable) { /*用于启用/禁用了反电动势前馈补偿*/
        if (!phase_vel.has_value()) {
            error_ |= ERROR_UNKNOWN_PHASE_VEL;
            return;
        }

        vq += *phase_vel * (2.0f/3.0f) * (config_.torque_constant / config_.pole_pairs);
    }
    
    /*GIMBAL 电机特殊处理，GIMBAL 电机将电流直接解释为电压。*/
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        // reinterpret current as voltage
        Vdq_setpoint_ = {vd + id, vq + iq};
    } else {
        Vdq_setpoint_ = {vd, vq};
    }
}


/**
 * @brief Called when the underlying hardware timer triggers an update event.
 */
void Motor::current_meas_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current) {
    // TODO: this is platform specific
    //const float current_meas_period = static_cast<float>(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;
    TaskTimerContext tmr{axis_->task_times_.current_sense};

    n_evt_current_measurement_++;

    /*要求校准时间 ≥ 7.5 倍时间常数（一阶系统在 5~7τ 后基本稳定），要求各相偏移值不能太大（如不超过 1A）*/
    bool dc_calib_valid = (dc_calib_running_since_ >= config_.dc_calib_tau * 7.5f)
                       && (abs(DC_calib_.phA) < max_dc_calib_)
                       && (abs(DC_calib_.phB) < max_dc_calib_)
                       && (abs(DC_calib_.phC) < max_dc_calib_);

    if (armed_state_ == 1 || armed_state_ == 2) {
        current_meas_ = {0.0f, 0.0f, 0.0f};
        armed_state_ += 1;
    } else if (current.has_value() && dc_calib_valid) {
        /*在正常运行时，获取真实的三相电流，并减去已校准的 DC 偏移，得到准确的电流值。*/
        current_meas_ = {
            current->phA - DC_calib_.phA,
            current->phB - DC_calib_.phB,
            current->phC - DC_calib_.phC
        };
    } else {
        current_meas_ = std::nullopt;
    }

    // Run system-level checks (e.g. overvoltage/undervoltage condition)
    // The motor might be disarmed in this function. In this case the
    // handler will continue to run until the end but it won't have an
    // effect on the PWM.
    odrv.do_fast_checks();

    if (current_meas_.has_value()) {
        // Check for violation of current limit
        // If Ia + Ib + Ic == 0 holds then we have:
        // Inorm^2 = Id^2 + Iq^2 = Ialpha^2 + Ibeta^2 = 2/3 * (Ia^2 + Ib^2 + Ic^2)
        float Itrip = effective_current_lim_ + config_.current_lim_margin;
        float Inorm_sq = 2.0f / 3.0f * (SQ(current_meas_->phA)
                                      + SQ(current_meas_->phB)
                                      + SQ(current_meas_->phC));

        /*SQ(x) 求 x 平方的函数*/

        // Hack: we disable the current check during motor calibration because
        // it tends to briefly overshoot when the motor moves to align flux with I_alpha
        if (Inorm_sq > SQ(Itrip)) {
            disarm_with_error(ERROR_CURRENT_LIMIT_VIOLATION);
        }
    } else if (is_armed_) {
        // Since we can't check current limits, be safe for now and disarm.
        // Theoretically we could continue to operate if there is no active
        // current limit.
        disarm_with_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
    }

    if (control_law_) {
        /*on_measurement 内部调用 Clarke transform（克拉克变换）更新 Ialpha, Ibeta*/
        Error err = control_law_->on_measurement(vbus_voltage,
                            current_meas_.has_value() ?
                                std::make_optional(std::array<float, 3>{current_meas_->phA, current_meas_->phB, current_meas_->phC})
                                : std::nullopt,
                            timestamp);
        if (err != ERROR_NONE) {
            disarm_with_error(err);
        }
    }
}

/**
 * 持续采集获取三相电流采样电路的 “零点偏移” 并进行一阶低通滤波。
 * 当底层硬件计时器触发更新事件时调用（此时恰好 PWM 关闭，电路中没有任何电流经过），此时获取的电流值可以认为是 “零点偏移”。
 * @brief Called when the underlying hardware timer triggers an update event.
 */
void Motor::dc_calib_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current) {
    const float dc_calib_period = static_cast<float>(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;
    TaskTimerContext tmr{axis_->task_times_.dc_calib};
    /*电机驱动中，使用电流传感器（采样电阻+运放）测量三相电流（Ia, Ib, Ic）。
    这些电流采样电路存在硬件偏移 (DC Offset)，运放有输入失调电压，ADC 有偏移误差，
    导致即使没有电流（0A），传感器输出可能不是 0V。*/
    if (current.has_value()) {
        /**DC_calib_ 存储的是三相电流采样电路的 “零点偏移”（Zero-Offset 或 DC Bias），
        用于在电流测量时进行实时校准，消除硬件带来的静态误差。*/
        const float calib_filter_k = std::min(dc_calib_period / config_.dc_calib_tau, 1.0f);
        /*一阶低通滤波，计算等价于 Y = Y * (1 - k) + x * k*/
        /*时间常数 k 由 min(dc_calib_period / tau, 1.0) 控制*/
        DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
        DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
        DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
        dc_calib_running_since_ += dc_calib_period;
    } else {
        DC_calib_.phA = 0.0f;
        DC_calib_.phB = 0.0f;
        DC_calib_.phC = 0.0f;
        dc_calib_running_since_ = 0.0f;
    }
}


void Motor::pwm_update_cb(uint32_t output_timestamp) {
    TaskTimerContext tmr{axis_->task_times_.pwm_update};
    n_evt_pwm_update_++;

    Error control_law_status = ERROR_CONTROLLER_FAILED;
    float pwm_timings[3] = {NAN, NAN, NAN};
    std::optional<float> i_bus;

    if (control_law_) {
        /*调用矢量控制（SVM 状态机）更新 PWM 控制参数到 pwm_timings*/
        control_law_status = control_law_->get_output(
            output_timestamp, pwm_timings, &i_bus);
    }

    // Apply control law to calculate PWM duty cycles
    if (is_armed_ && control_law_status == ERROR_NONE) {
        /*根据最终 pwm_timings 更新 PWM 驱动，调节矢量电流*/
        uint16_t next_timings[] = {
            (uint16_t)(pwm_timings[0] * (float)TIM_1_8_PERIOD_CLOCKS),
            (uint16_t)(pwm_timings[1] * (float)TIM_1_8_PERIOD_CLOCKS),
            (uint16_t)(pwm_timings[2] * (float)TIM_1_8_PERIOD_CLOCKS)
        };
        /*根据 next_timings 最终更新 PWM 驱动，调节电流矢量*/
        apply_pwm_timings(next_timings, false);
    } else if (is_armed_) {
        if (!(timer_->Instance->BDTR & TIM_BDTR_MOE) && (control_law_status == ERROR_CONTROLLER_INITIALIZING)) {
            // If the PWM output is armed in software but not yet in
            // hardware we tolerate the "initializing" error.
            i_bus = 0.0f;
        } else {
            disarm_with_error(control_law_status);
        }
    }

    if (!is_armed_) {
        // If something above failed, reset I_bus to 0A.
        i_bus = 0.0f;
    } else if (is_armed_ && !i_bus.has_value()) {
        // If the motor is armed then i_bus must be known
        disarm_with_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
        i_bus = 0.0f;
    }

    I_bus_ = *i_bus;

    if (*i_bus < config_.I_bus_hard_min || *i_bus > config_.I_bus_hard_max) {
        disarm_with_error(ERROR_I_BUS_OUT_OF_RANGE);
    }

    update_brake_current();
}
