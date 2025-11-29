
#include "foc.hpp"
#include <board.h>

/**Ia,Ib,Ic 这三个电流基向量是非正交的，学过线性代数的同学可能会想到，
可以做一个很简单的基变换将其正交化为一个直角坐标系，新的直角坐标就是
Alpha-Beta 坐标系，变换公式如下。*/
Motor::Error AlphaBetaFrameController::on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::array<float, 3>> currents, /*接收 Ia, Ib, Ic 三相电流*/
            uint32_t input_timestamp) {

    std::optional<float2D> Ialpha_beta;

    if (currents.has_value()) {
        /*Clarke transform（克拉克变换），one_by_sqrt3 为 1/√3 的近似值。*/
        /*输入三相电流 Ia, Ib, Ic 对应 currents[0], currents[1], currents[2]，
        输出两相静止坐标系下的电流 Ialpha 和 Ibeta*/
        Ialpha_beta = {
            (*currents)[0],
            one_by_sqrt3 * ((*currents)[1] - (*currents)[2])
        };

        /*Clarke 变换公式：Ialpha = Ia*/
        /*Ibeta = (Ib - Ic) / sqrt(3)*/
        /*Ibeta 计算是使用对称变换矩阵时简化得到的，实际不是这样的*/
    }
    
    /*注意调用的是 FOC 类对象的 on_measurement 函数（通过第二个形参可知一个是接收三相电流，
    一个是接收 Ialpha, Ibeta）来更新 FOC 对象要用的 Ialpha, Ibeta 参数*/
    return on_measurement(vbus_voltage, Ialpha_beta, input_timestamp);
}

/**
 * 三相电压给定所合成的电压矢量旋转角速度为 ω=2πf，旋转一周所需的时间（三相正弦波周期）为 T=1/f,
 * 若载波频率是 fs，则频率比为 R=fs/f 即 T/Ts。这样将电压旋转平面等切割成 R 个小增量，
 * 亦即设定电压矢量每次增量的角度是 2π/R，即 γ=2π/R=2π/fs/f=2π/T/Ts=2πTs/T。
 * 
 * - fs: 开关频率或称载波频率（Switching Frequency），单位 Hz（例如 PWM 的频率，如 10 kHz）。
 * - f: 基波频率（Fundamental Frequency），单位 Hz（例如三相交流电的频率，如 50 Hz 或 400 Hz）。
 * - R: 频率比。
 * 
 * 想象画一个圆：理想情况下，电压矢量应连续平滑地旋转一圈（对应一个正弦周期）。但在数字控制系统中（如 STM32、DSP），
 * 只能每隔一段时间更新一次角度。这个 “更新间隔” 由 PWM 周期决定，即每 (Ts=1/fs) 秒更新一次。
 * 所以，在一个完整的基波周期 T=1/f 内，你能更新多少次？是不是 T/Ts 次，即 1/f/1/fs=fs/f。
 * 这就是 频率比表示：每个基波周期内，控制器执行更新的次数。
 * 
 * 在一个 50Hz 正弦波周期（20ms）内，系统会以 10kHz 的频率更新 200 次 电压矢量或 PWM 占空比。
 * 这就像用 200 个小线段 去逼近一个圆，越多次数，波形越接近理想正弦波。
 */
Motor::Error AlphaBetaFrameController::get_output(
            uint32_t output_timestamp, float (&pwm_timings)[3],
            std::optional<float>* ibus) {
    std::optional<float2D> mod_alpha_beta;
    Motor::Error status = get_alpha_beta_output(output_timestamp, &mod_alpha_beta, ibus);
    
    if (status != Motor::ERROR_NONE) {
        return status;
    } else if (!mod_alpha_beta.has_value() || is_nan(mod_alpha_beta->first) || is_nan(mod_alpha_beta->second)) {
        return Motor::ERROR_MODULATION_IS_NAN;
    }

    auto [tA, tB, tC, success] = SVM(mod_alpha_beta->first, mod_alpha_beta->second);
    if (!success) {/*调制幅度错误*/
        return Motor::ERROR_MODULATION_MAGNITUDE;
    }

    pwm_timings[0] = tA;
    pwm_timings[1] = tB;
    pwm_timings[2] = tC;

    return Motor::ERROR_NONE;
}

void FieldOrientedController::reset() {
    v_current_control_integral_d_ = 0.0f;
    v_current_control_integral_q_ = 0.0f;
    vbus_voltage_measured_ = std::nullopt;
    Ialpha_beta_measured_ = std::nullopt;
    power_ = 0.0f;
}

Motor::Error FieldOrientedController::on_measurement(
        std::optional<float> vbus_voltage, std::optional<float2D> Ialpha_beta,
        uint32_t input_timestamp) {
    // Store the measurements for later processing.
    i_timestamp_ = input_timestamp;
    vbus_voltage_measured_ = vbus_voltage;
    Ialpha_beta_measured_ = Ialpha_beta;

    return Motor::ERROR_NONE;
}

/*Field-Oriented Control (FOC) 负责将 Vdq 指令（旋转坐标系下的电压）转换为 mod_alpha_beta（静止坐标系下的 PWM 调制信号）*/
/*将 Motor 控制器计算出的 Vd, Vq 电压指令，经过一系列变换，最终生成 α-β 坐标系下的 PWM 调制信号，用于驱动三相逆变器。*/
ODriveIntf::MotorIntf::Error FieldOrientedController::get_alpha_beta_output(
        uint32_t output_timestamp, std::optional<float2D>* mod_alpha_beta,
        std::optional<float>* ibus) {

    if (!vbus_voltage_measured_.has_value() || !Ialpha_beta_measured_.has_value()) {
        // FOC didn't receive a current measurement yet.
        return Motor::ERROR_CONTROLLER_INITIALIZING;
    } else if (abs((int32_t)(i_timestamp_ - ctrl_timestamp_)) > MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA) {
        // Data from control loop and current measurement are too far apart.
        return Motor::ERROR_BAD_TIMING;
    }

    // TODO: improve efficiency in case PWM updates are requested at a higher
    // rate than current sensor updates. In this case we can reuse mod_d and
    // mod_q from a previous iteration.

    if (!Vdq_setpoint_.has_value()) {
        return Motor::ERROR_UNKNOWN_VOLTAGE_COMMAND;
    } else if (!phase_.has_value() || !phase_vel_.has_value()) {
        return Motor::ERROR_UNKNOWN_PHASE_ESTIMATE;
    } else if (!vbus_voltage_measured_.has_value()) {
        return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
    }

    auto [Vd, Vq] = *Vdq_setpoint_; /*获取 Vd, Vq（Motor 控制器输出的 d-q 轴电压指令）*/
    float phase = *phase_; /*获取当前电角度（弧度，由编码器或观测器估计）*/
    float phase_vel = *phase_vel_; /*获取电角速度*/
    float vbus_voltage = *vbus_voltage_measured_; /*获取母线电压（用于 PWM 归一化）*/

    std::optional<float2D> Idq;

    // Park transform (帕克变换 Park Transform)
    if (Ialpha_beta_measured_.has_value()) {
        auto [Ialpha, Ibeta] = *Ialpha_beta_measured_;
        /*当前相位+角速度×时间差，即用线性预测法估算在当前采样时刻的电机角度（这是为了延迟补偿，补偿了电流采样与控制指令之间的时间延迟，提高当前相位准确度）*/
        /*_timestamp_ - ctrl_timestamp_ 为当前采样时刻与控制器上一次更新时间的时间差（单位是定时器计数）。*/
        float I_phase = phase + phase_vel * ((float)(int32_t)(i_timestamp_ - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
        /*计算该角度的余弦和正弦值，通常用于生成三相电流控制信号。*/
        float c_I = our_arm_cos_f32(I_phase);
        float s_I = our_arm_sin_f32(I_phase);

        /*Park 变换公式：Id = Ialpha*cos(θ)+Ibeta*sin(θ)*/
        /*Iq = -Ialpha*sin(θ)+Ibeta*cos(θ)*/
        Idq = { /*将测量的 Ialpha, Ibeta 经 Park 转换为 Id, Iq*/
            c_I * Ialpha + s_I * Ibeta,
            c_I * Ibeta - s_I * Ialpha
        };
        /*这个操作是可行的，因为我们会通过编码器输入转子的实时旋转角度，所以这个角度 θ 始终是一个已知数。经过这一步的变换，
        我们会发现，一个匀速旋转向量在这个坐标系下变成了一个定值！（显然的嘛，因为参考系相对于该向量静止了），
        这个坐标系下两个控制变量都被线性化了*/
        Id_measured_ += I_measured_report_filter_k_ * (Idq->first - Id_measured_);
        Iq_measured_ += I_measured_report_filter_k_ * (Idq->second - Iq_measured_);
        /*相电流测量 Id_measured_, Iq_measured_ 使用 Kalman Estimates 滤波平滑后提供
        给外部查询，例如实现电流/力矩反馈，从而达到力控效果。*/
    } else {
        Id_measured_ = 0.0f;
        Iq_measured_ = 0.0f;
    }


    float mod_to_V = (2.0f / 3.0f) * vbus_voltage; /*将调制量转换为实际电压的比例*/
    float V_to_mod = 1.0f / mod_to_V; /*将电压转换为 PWM 占空比 的比例*/
    /*对于三相逆变器，最大线电压幅值为 2/3 * Vbus所以 V = mod * (2/3 * Vbus) 即 mod = V / (2/3 * Vbus)*/
    float mod_d;
    float mod_q;

    /*电流控制模式*/
    if (enable_current_control_) {
        // Current control mode

        if (!pi_gains_.has_value()) {
            return Motor::ERROR_UNKNOWN_GAINS;
        } else if (!Idq.has_value()) {
            return Motor::ERROR_UNKNOWN_CURRENT_MEASUREMENT;
        } else if (!Idq_setpoint_.has_value()) {
            return Motor::ERROR_UNKNOWN_CURRENT_COMMAND;
        }

        auto [p_gain, i_gain] = *pi_gains_;
        auto [Id, Iq] = *Idq;
        auto [Id_setpoint, Iq_setpoint] = *Idq_setpoint_; /*来自 Motor 类对象的设定参数*/

        float Ierr_d = Id_setpoint - Id; /*期望扭矩电流 Id_setpoint 和实测电流 Id 之差*/
        float Ierr_q = Iq_setpoint - Iq; /*期望扭矩电流 Iq_setpoint 和实测电流 Iq 之差*/

        // 电流闭环控制，这部分计算是 PI 电流控制器的输出，基于 Id 和 Iq 的 PI 控制（这里才是真正的电流/力矩环实现）。
        // Apply PI control (V{d,q}_setpoint act as feed-forward terms in this mode)
        mod_d = V_to_mod * (Vd + v_current_control_integral_d_ + Ierr_d * p_gain);
        mod_q = V_to_mod * (Vq + v_current_control_integral_q_ + Ierr_q * p_gain);

        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        // 0.80f * sqrt3_by_2: 设置调制上限（80% 的六边形极限）
        float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / std::sqrt(mod_d * mod_d + mod_q * mod_q);
        if (mod_scalefactor < 1.0f) {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
            // TODO make decayfactor configurable（积分项衰减，防积分饱和）
            v_current_control_integral_d_ *= 0.99f;
            v_current_control_integral_q_ *= 0.99f;
        } else {
            v_current_control_integral_d_ += Ierr_d * (i_gain * current_meas_period);
            v_current_control_integral_q_ += Ierr_q * (i_gain * current_meas_period);
        }

    /*电压控制模式*/
    } else {
        // Voltage control mode
        mod_d = V_to_mod * Vd;
        mod_q = V_to_mod * Vq;
    }

    // Inverse park transform (反帕克变换 Inverse Park Transform)
    float pwm_phase = phase + phase_vel * ((float)(int32_t)(output_timestamp - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c_p * mod_d - s_p * mod_q;
    float mod_beta = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorless estimator)
    final_v_alpha_ = mod_to_V * mod_alpha;
    final_v_beta_ = mod_to_V * mod_beta;

    /*Park 变换完成后，接下来如果我们以 Id, Iq 这两个值作为反馈控制的对象，
    那么显然就可以使用一些线性控制器来进行控制了，比如 PID 没错工业界还是偏爱 PID，
    尽管学术界有很多炫酷的高级控制方法*/
    *mod_alpha_beta = {mod_alpha, mod_beta};

    if (Idq.has_value()) {
        auto [Id, Iq] = *Idq;
        /*额外计算 Ibus 母线电流估计，用于功率计算、效率分析、过流保护*/
        *ibus = mod_d * Id + mod_q * Iq;
        power_ = vbus_voltage * (*ibus).value();
    }
    
    return Motor::ERROR_NONE;
}

void FieldOrientedController::update(uint32_t timestamp) {
    CRITICAL_SECTION() {
        ctrl_timestamp_ = timestamp;
        enable_current_control_ = enable_current_control_src_;
        Idq_setpoint_ = Idq_setpoint_src_.present();
        Vdq_setpoint_ = Vdq_setpoint_src_.present();
        phase_ = phase_src_.present();
        phase_vel_ = phase_vel_src_.present();
    }
}
