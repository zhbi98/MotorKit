
#include "odrive_main.h"
#include <algorithm>
#include <numeric>

bool Controller::apply_config() {
    config_.parent = this;
    update_filter_gains();
    return true;
}

void Controller::reset() {
    // pos_setpoint is initialized in start_closed_loop_control
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
    mechanical_power_ = 0.0f;
    electrical_power_ = 0.0f;
}

void Controller::set_error(Error error) {
    error_ |= error;
    last_error_time_ = odrv.n_evt_control_loop_ * current_meas_period;
}

//--------------------------------
// Command Handling
//--------------------------------

/**规划梯形加减速运动轨迹，这个函数会使用梯形速度轨迹（Trapezoidal Trajectory）规划器，
 * 从当前位置移动到目标点 goal_point，并设置状态让控制器开始执行轨迹。*/
void Controller::move_to_pos(float goal_point) {
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

/**
 * 这个函数的作用是：相对于当前位置或当前设定值 pos_setpoint_，
 * 增量移动一个 displacement 距离，然后调用 input_pos_updated()，
 * 让控制器根据新的目标位置，开始执行新的轨迹或控制过程。*/
void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.calib_anticogging = true;
    }
}

float Controller::remove_anticogging_bias()
{
    auto& cogmap = config_.anticogging.cogging_map;
    
    auto sum = std::accumulate(std::begin(cogmap), std::end(cogmap), 0.0f);
    auto average = sum / std::size(cogmap);

    for(auto& val : cogmap) {
        val -= average;
    }

    return average;
}


/**
 * 这段是 ODrive 用于抗齿槽效应（Anti-Cogging）校准，是高级电机控制中常用的补偿机制之一。
 * 抗齿槽效应（Anti-Cogging）是在电机静止或低速时，转子有细微卡滞，会导致运动不平滑，特别是在低速精密控制时很明显。
 * 这种防齿槽实现迭代每个编码器位置，等待零速度和位置误差，然后对保持该位置所需的电流进行采样。
 * 该保持电流作为前馈项添加到控制环路中。*/
/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    /**pos_err 计算当前电机位置和设定位置之间的误差。希望它越接近 0 越好，表示电机已经稳定在期望位置。*/
    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / (float)axis_->encoder_.config_.cpr &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / (float)axis_->encoder_.config_.cpr) {
        /**把当前位置所需的积分电流（即维持当前位置的扭矩）保存到 `cogging_map` 中，
        每执行一次这个函数，就测量一个点（`index` 代表当前点）,`std::clamp` 防止越界（限制 index 在 0~3600 之间）*/
        config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 3600)] = vel_integrator_torque_;
    }
    if (config_.anticogging.index < 3600) {
        /*接下来设置下一个采样点，让电机移动到新的位置点继续采样*/
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * axis_->encoder_.getCoggingRatio();
        /*清空速度、力矩输入（表示只靠位置控制）*/
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        /*当所有点采样完毕：重置 index,把电机回零（归位）,标记 `anticogging_valid_ = true` 表示数据可用,关闭校准模式标志*/
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = 0.0f;  // Send the motor home
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

/**位置控制的核心逻辑，负责将目标位置（浮点数）转换为步进电机的“步数”。*/
void Controller::set_input_pos_and_steps(float const pos) {
    input_pos_ = pos;
    if (config_.circular_setpoints) {
        float const range = config_.circular_setpoint_range;
        /**fmodf_pos(pos, range)：将 pos 映射到 [0, range) 区间*/
        axis_->steps_ = (int64_t)(fmodf_pos(pos, range) / range * config_.steps_per_circular_range);
        /**转换为步数：steps_ = (pos_in_range / range) * total_steps*/
    } else {
        axis_->steps_ = (int64_t)(pos * config_.steps_per_circular_range);
    }
}

bool Controller::control_mode_updated() {
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        std::optional<float> estimate = (config_.circular_setpoints ?
                                pos_estimate_circular_src_ :
                                pos_estimate_linear_src_).any();
        if (!estimate.has_value()) {
            return false;
        }

        pos_setpoint_ = *estimate;
        set_input_pos_and_steps(*estimate);
    }
    return true;
}

/**
 * 更新输入滤波器的参数（带宽 → PID 系数），
 * 根据设定的带宽更新输入滤波器的 kp 和 ki 系数，以便输入信号平滑处理。*/
void Controller::update_filter_gains() {
    /*设定一个最大允许的滤波带宽，不能超过采样率的四分之一（Nyquist 准则的变种）,
    current_meas_hz 是当前电流采样频率（控制回路频率，单位 Hz）,
    input_filter_bandwidth 是配置中允许的最大滤波带宽*/
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    /*离散时间下的积分增益：假设使用一阶低通滤波器设计形式，2× 带宽是常用近似*/
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    /*等价于设置滤波器为无振荡、快速响应的平稳系统*/
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

/**速度限制函数，用于限制速度相关的扭矩指令大小，防止速度过冲或抖动。*/
static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    /*动态计算的扭矩上下限:当 vel_estimate 靠近 vel_limit 时，Tmax 减小，防
    止继续加速，当速度超限时，Tmax 甚至为负，形成刹车力矩*/
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    /*把输入 torque 限制在 Tmin ~ Tmax 范围内*/
    return std::clamp(torque, Tmin, Tmax);
}

/**
 * Controller::update 是电机位置闭环，速度闭环，电流闭环三环控制核心逻辑。
 * 注意：位置环只用 P，速度环用 PI，电流环用 PI。这样层层递进，响应快且易于调试（实际控制常见做法）。
 * 位置环输出的是速度设定值（vel_des），速度环再用 PI 控制，电流环用 PI 或更复杂控制，这样可以避免积分环节导致的漂移和不稳定。
 * 如果位置环加积分，可能会因编码器噪声或机械死区导致积分“风暴”，影响系统稳定性。
 */
bool Controller::update() {
    /**从 Encoder 对象处实时获取电机位置估计值，pos_estimate_linear_src_，
    pos_estimate_circular_src_ 通过 connect_to 函数连接到 Encoder，可以从 Encoder 获取位置值）*/
    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    std::optional<float> pos_wrap = pos_wrap_src_.present();

    /**从 Encoder 对象处实时获取电机速度估计值，vel_estimate_src_，
    通过 connect_to 函数连接到 Encoder，可以从 Encoder 获取速度值）*/
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    std::optional<float> anticogging_pos_estimate = axis_->encoder_.pos_estimate_.present();
    std::optional<float> anticogging_vel_estimate = axis_->encoder_.vel_estimate_.present();

    /*Circular-Position-Control 模式用于连续增量位置运动是有用的，例如转轴无限滚动。
    在正态位置模式下，input_pos 将增长到非常大的值，并且由于浮点取整而失去精度。*/
    if (axis_->step_dir_active_) { /*step_dir_active_ 指的是脉冲步进控制方式（此时才需要 steps_ 参数）*/
        if (config_.circular_setpoints) {
            /**pos_wrap 用于环形运动 (circular-setpoints) 将位置值包裹在固定范围内，实现位置环的循环范围或周期长度，
            从而避免数值直线累加溢出或漂移。例如旋转轴位置在 0~360 无限循环，本质就是把位置值限制在 [0, pos_wrap) 区间*/
            if (!pos_wrap.has_value()) {
                set_error(ERROR_INVALID_CIRCULAR_RANGE);
                return false;
            }
            input_pos_ = (float)(axis_->steps_ % config_.steps_per_circular_range) * (*pos_wrap / (float)(config_.steps_per_circular_range));
        } else {
            input_pos_ = (float)(axis_->steps_) / (float)(config_.steps_per_circular_range);
        }
    }

    if (config_.anticogging.calib_anticogging) {
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        if (!pos_wrap.has_value()) {
            set_error(ERROR_INVALID_CIRCULAR_RANGE);
            return false;
        }
        /*pos_wrap 使 input_pos_ 保持在 0~pos_wrap（或指定范围）以内，实现位置循环范围，避免角度漂移*/
        input_pos_ = fmodf_pos(input_pos_, *pos_wrap);
    }

    // Update inputs, 输入模式处理
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        /*直接传入控制指令指定的目标位置值 input_pos_ 作为目标位置*/
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        /*速度斜坡计算，限制速度的变化率*/
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        /*力矩斜坡计算，限制扭矩变化率*/
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        /*二阶位置跟踪滤波器，平滑输入位置*/
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            if (config_.circular_setpoints) {
                if (!pos_wrap.has_value()) {
                    set_error(ERROR_INVALID_CIRCULAR_RANGE);
                    return false;
                }
                delta_pos = wrap_pm(delta_pos, *pos_wrap);
            }
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        /*镜像另一个轴的状态，即跟随另一个轴的状态*/
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                std::optional<float> other_pos = axes[config_.axis_to_mirror].encoder_.pos_estimate_.present();
                std::optional<float> other_vel = axes[config_.axis_to_mirror].encoder_.vel_estimate_.present();
                std::optional<float> other_torque = axes[config_.axis_to_mirror].controller_.torque_output_.present();

                if (!other_pos.has_value() || !other_vel.has_value() || !other_torque.has_value()) {
                    set_error(ERROR_INVALID_ESTIMATE);
                    return false;
                }

                pos_setpoint_ = *other_pos * config_.mirror_ratio;
                vel_setpoint_ = *other_vel * config_.mirror_ratio;
                torque_setpoint_ = *other_torque * config_.torque_mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;

        /*未实现的模式（无效模式）*/
        /*case INPUT_MODE_MIX_CHANNELS: {
            NOT YET IMPLEMENTED
        } break;*/

        /*梯形轨迹规划控制（加减速规划）*/
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = axis_->trap_traj_.Xf_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos_estimate = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        /*调试/自动调谐，用于系统辨识*/
        case INPUT_MODE_TUNING: {
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));
            float c = our_arm_cos_f32(autotuning_phase_);
            float s = our_arm_sin_f32(autotuning_phase_);
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    /*输入模式和控制模式的区别：输入模式决定 “目标怎么来”（目标值如何生成和处理。），
    控制模式决定 “怎么控制到目标”（决定控制器实际执行哪种闭环环路）。*/

    // Never command a setpoint beyond its limit
    if(config_.enable_vel_limit) {
        vel_setpoint_ = std::clamp(vel_setpoint_, -config_.vel_limit, config_.vel_limit);
    }
    const float Tlim = axis_->motor_.max_available_torque();
    torque_setpoint_ = std::clamp(torque_setpoint_, -Tlim, Tlim);

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;

    /**
     * gain_scheduling_multiplier 作用是根据位置误差动态调整控制器的各个增益 (vel_gain, vel_integrator_gain)，实现 “增益调度” (gain scheduling)。
     * 乘上它这会让增益（如 pos_gain, vel_gain 等）随着误差变小而线性减小，误差为零时增益为零，误差最大时增益为 1.0。
     * 这样做的目的是防止在目标附近时增益过高导致抖动或超调，而在误差较大时保持较高响应速度。
     * 后续所有用到 gain_scheduling_multiplier 的地方（如速度环、积分环）都会乘以它，实现各个增益根据目标误差自动动态调节。
     */

    /*control_mode >= MODE_XX 使用 >= 这种写法用于判断当前控制模式是否包含或高于某个环节（如位置环）。
    可以让同一段代码适用于更高层的模式。例如，位置环相关代码在位置控制及更高模式下都需要执行，
    即控制模式是递进的，数值越高，包含的控制环节越多。*/
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;

        if (config_.circular_setpoints) {
            if (!pos_estimate_circular.has_value() || !pos_wrap.has_value()) {
                /*编码器校准参数没有被保存到 Flash，或启动时未读取到编码器校准参数，导致编码器未启动*/
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            
            /**wrap_pm 是用于环形坐标 (circular setpoints) 下的位置误差计算，确保误差始终在一个合理的区间内
            （根据 wrap_pm 的计算逻辑可知，区间范围通常是 [-range/2, +range/2]），避免因为角度跨越零点导致的数值跳变。
            例如：当位置是环形（比如旋转轴 360 无限循环），直接相减可能得到很大的误差（比如从 359 到 1，直接相减是 -358，但实际只差 2）。
            wrap_pm(x, range) 会把误差 x 包裹到 [-range/2, +range/2] 区间，保证环形运动的误差计算始终是最短路径，
            避免跨界跳变。这样控制器就能正确地控制到目标位置，而不会因为数值跳变导致控制异常。*/
            pos_err = wrap_pm(pos_err, *pos_wrap);
        } else {
            if (!pos_estimate_linear.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        /**vel_des 即将位置误差乘以比例增益 (pos_gain)，并加到期望速度 vel_des 上。这里是典型的比例控制*/
        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        /*当 enable_gain_scheduling 使能，并且位置误差 abs_pos_err 在 
        gain_scheduling_width 范围内时更新 gain_scheduling_multiplier 参数*/
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting（限速，超速检测，检测限制位置变化过程中的速度）
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)（如果速度估计超过容忍阈值则报错中断）
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain; /*获取速度积分增益*/
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->acim_estimator_.rotor_flux_;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (std::abs(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    /**Anti-cogging: 校准后启用防齿槽，我们获取当前位置并应用当前前馈，
    确保我们正确处理负编码器位置 (-1 == motor->encoder.encoder_cpr - 1)*/

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        if (!anticogging_pos_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float anticogging_pos = *anticogging_pos_estimate / axis_->encoder_.getCoggingRatio();
        torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 3600), 0, 3600)];
    }

    /**转速闭环控制*/
    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate.has_value()) {
            /*编码器校准参数没有被保存到 Flash，或启动时未读取到编码器校准参数，导致编码器未启动*/
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        /**torque 即将位置误差乘以比例增益 (vel_gain)，并加到期望力矩 torque 上。这里是典型的比例控制*/
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting（速度环积分控制）
        torque += vel_integrator_torque_; /*vel_integrator_torque_ 的更新逻辑在下面部分*/
    }

    /**电流闭环控制（即力矩控制环）*/
    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_torque_mode_vel_limit) {
        if (!vel_estimate.has_value()) {
            /*编码器校准参数没有被保存到 Flash，或启动时未读取到编码器校准参数，导致编码器未启动*/
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate, vel_gain, torque);
    }

    /*注意：实际电路电流大小没有传递到这里的所谓电流闭环控制参与计算，这里的电流闭环控制只负责计算出目标扭矩/电流，
    最终目标扭矩会传递给电流控制器（通常在 FieldOrientedController 相关模块，采集实际电流值如 Iq_measured_, Id_measured_），
    电流控制器会根据这个目标扭矩和获取实际电路电流大小，执行 PI 闭环计算，基于计算结果调整驱动信号，实现电流环闭环。*/

    // Torque limiting
    bool limited = false;
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting), 速度积分
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            /*根据速度误差 v_err，用速度积分增益 vel_integrator_gain 进行积分累加*/
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
        // integrator limiting to prevent windup（做防积分饱和处理）
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    float ideal_electrical_power = 0.0f;
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) {
        ideal_electrical_power = axis_->motor_.current_control_.power_ - \
            SQ(axis_->motor_.current_control_.Iq_measured_) * 1.5f * axis_->motor_.config_.phase_resistance - \
            SQ(axis_->motor_.current_control_.Id_measured_) * 1.5f * axis_->motor_.config_.phase_resistance;
        /*SQ(x) 求 x 平方的函数*/
    }
    else {
        ideal_electrical_power = axis_->motor_.current_control_.power_;
    }
    mechanical_power_ += config_.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

    // Spinout check(即防飞车检测 Spinout detected，通过比较机械功率和电功率阈值来判断电机失步)
    // If mechanical power is negative (braking) and measured power is positive, something is wrong
    // This indicates that the controller is trying to stop, but torque is being produced.
    // Usually caused by an incorrect encoder offset
    if (mechanical_power_ < config_.spinout_mechanical_power_threshold && electrical_power_ > config_.spinout_electrical_power_threshold) {
        set_error(ERROR_SPINOUT_DETECTED);
        return false;
    }

    /*经过上面的三个控制环计算 Controller 对象向外部输出合成的最终扭矩（电流）*/
    torque_output_ = torque;

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
