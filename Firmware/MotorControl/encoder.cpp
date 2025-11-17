
#include "odrive_main.h"
#include <Drivers/STM32/stm32_system.h>
#include <bitset>

Encoder::Encoder(TIM_HandleTypeDef* timer, Stm32Gpio index_gpio,
                 Stm32Gpio hallA_gpio, Stm32Gpio hallB_gpio, Stm32Gpio hallC_gpio,
                 Stm32SpiArbiter* spi_arbiter, Stm32Gpio abs_spi_cs_gpio) :
        timer_(timer), index_gpio_(index_gpio),
        hallA_gpio_(hallA_gpio), hallB_gpio_(hallB_gpio), hallC_gpio_(hallC_gpio),
        spi_arbiter_(spi_arbiter), abs_spi_cs_gpio_(abs_spi_cs_gpio)
{
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

bool Encoder::apply_config(ODriveIntf::MotorIntf::MotorType motor_type) {
    config_.parent = this;

    update_pll_gains();

    /*pre_calibrated 如果为 True，这意味着配置中存储的偏移量是有效的，无需通过 run_offset_calibration 来确定。
    在这种情况下，编码器将在找到索引（index）后立即进入准备状态。*/
    if (config_.pre_calibrated) {
        if (config_.mode == Encoder::MODE_HALL && config_.hall_polarity_calibrated)
            is_ready_ = true;
        if (config_.mode == Encoder::MODE_SINCOS)
            is_ready_ = true;
        if (motor_type == Motor::MOTOR_TYPE_ACIM)
            is_ready_ = true;
    }

    return true;
}

void Encoder::setup() {
    /*TIM_CHANNEL_ALL 指代开启 TIM_CHANNEL_1, TIM_CHANNEL_2*/
    HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);
    set_idx_subscribe();

    mode_ = config_.mode;

    if (mode_ == MODE_SPI_ABS_MA732) {
        abs_spi_dma_tx_[0] = 0x0000;
    }

    /*MODE_FLAG_ABS 是一类编码器的掩码，表示绝对编码器模式（如 SPI、SSI 等）*/
    if(mode_ & MODE_FLAG_ABS){
        abs_spi_cs_pin_init();

        if (axis_->controller_.config_.anticogging.pre_calibrated) {
            axis_->controller_.anticogging_valid_ = true;
        }
    }
}

void Encoder::set_error(Error error) {
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

/**
 * 在 ODrive 的 Encoder::enc_index_cb() 函数是一个回调函数，
 * 当编码器检测到索引（Index）信号时会被调用。这个函数主要处理与编码器索引信号
 * 相关的计数器和状态更新。
 */
// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// (maybe by attaching the interrupt on start search, synergistic with following)
void Encoder::enc_index_cb() {
    /*检查是否启用了 Z 索引信号*/
    if (config_.use_index) {
        /*捕获到 Index 脉冲，将编码器的 "循环计数" circular count 清零。
        false 表示不启用自动偏移校准功能（如果有的话）。通过重置循环计数，
        系统可以基于索引脉冲来重新确定电机位置的基准点。*/
        set_circular_count(0, false);
        if (config_.use_index_offset)
            /*捕获到 Index 脉冲，将编码器的 "循环计数" linear count 清零。
            计算方式是将配置中的索引偏移值 index_offset 乘以编码器每转脉冲数 CPR。
            对于线性计数每次捕获到索引信号时，把计数设置为预设的偏移量，
            有偏移量那么零点就是预设的固定偏移量。*/
            set_linear_count((int32_t)(config_.index_offset * config_.cpr));
        if (config_.pre_calibrated) {
            is_ready_ = true;
            if(axis_->controller_.config_.anticogging.pre_calibrated){
                axis_->controller_.anticogging_valid_ = true;
            }
        } else {
            /*我们不能在 set_circular_count 使用 update_offset，
            因为我们还在有机会更新之前设置线性计数。因此使 Index 搜索之前可能发生的偏移校准失效*/
            // We can't use the update_offset facility in set_circular_count because
            // we also set the linear count before there is a chance to update. Therefore:
            // Invalidate offset calibration that may have happened before idx search
            is_ready_ = false;
        }
        index_found_ = true;
    }

    /*为什么要 Disable interrupt，这样不就没法再捕获 Index 信号了吗？
    因为索引（Index）通常只需要开机时作为 "单次参考点" 来同步/校准编码器位置。
    enc_index_cb 在找到索引后就不需要使用了，关闭中断可以避免再次触发索引归零。*/

    // Disable interrupt
    index_gpio_.unsubscribe();
}

/*检查是否启用了 Z 索引信号，如果启用了索引信号则开启索引信号管脚中断，用于圈数计算/计数*/
/**override_enable == true（强制启用）,find_idx_on_lockin_only == false（表示允许在任意时刻订阅）*/
void Encoder::set_idx_subscribe(bool override_enable) {
    if (config_.use_index && (override_enable || !config_.find_idx_on_lockin_only)) {
        if (!index_gpio_.subscribe(true, false, enc_index_cb_wrapper, this)) {
            odrv.misconfigured_ = true;
        }
    } else if (!config_.use_index || config_.find_idx_on_lockin_only) {
        index_gpio_.unsubscribe();
    }
}

void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time（比例项，控制响应速度）
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped（积分项，增强低频响应，0.25 倍平方来自临界阻尼（critically damped）二阶系统设计）

    /*防止数值不稳定。由于是离散系统，T * Kp < 1 近似等价于采样时间太大或带宽太高将导致离散控制器发散。*/
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

/*检查设备是否已经校准*/
void Encoder::check_pre_calibrated() {
    // TODO: restoring config from python backup is fragile here (ACIM motor type must be set first)
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_ACIM) {
        if (!is_ready_)
            config_.pre_calibrated = false;
        if (mode_ == MODE_INCREMENTAL && !index_found_)
            config_.pre_calibrated = false;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;
    /*tim_cnt_sample_：指的是利用定时中断定时更新采集的编码器统计数*/
    tim_cnt_sample_ = count;

    // 最后将值更新到硬件中：因为 tim_cnt_sample_ 和 timer_->Instance->CNT 是同步的，所以要同时置为相同值 count
    //Write hardware last
    timer_->Instance->CNT = count;

    cpu_exit_critical(prim);
}

/**
 * ODrive 为何需要循环计数？
 * 
 * 在电机控制中，循环计数（circular count）是一个重要的概念，主要用于跟踪电机编码器输出的位置信息。
 * 编码器每转会产生一定数量的脉冲，通过累计这些脉冲的数量，可以精确地确定电机轴的绝对位置。
 * 在 ODrive 中，循环计数的作用包括：1. 位置追踪： 编码器的循环计数记录了电机从一个索引（Index）信号出现
 * 到另一个索引信号出现之间转过的圈数。这样就可以实时知道电机转子相对于初始位置的具体位置，
 * 这对于闭环控制系统至关重要。2. 防止数值溢出： 随着电机转动，编码器产生的脉冲数会不断累积。
 * 为了避免32位或更高位宽的计数器因超过其最大值而导致溢出问题，使用循环计数方法可以在达到最大值时自动回绕至最小值，
 * 并继续累加，从而维持连续且准确的位置跟踪。3. 绝对定位与参考点更新： 当编码器检测到索引信号（通常是一次性的高电平脉冲）时，
 * 系统可以通过重置循环计数并结合索引偏移量进行绝对位置校准。这有助于确保每次找到索引信号时，
 * 都能够恢复到已知的基准位置，这对于实现精确的零点回归、多圈定位以及初始化阶段的位置识别都极为重要。
 * 4. 抗干扰与稳定性： 循环计数还可以帮助系统在受到噪声、丢包等干扰时保持稳定的电机位置估计，
 * 通过算法处理可以纠正短时间内的位置误差，提高系统的整体性能和可靠性。
 */

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        /*count - count_in_cpr_ 表示当前位置与上一次位置的差值,加到当前 offset 上去，就相当于把原始测量值重新对齐到这个 count 上*/
        config_.phase_offset += count - count_in_cpr_;
        config_.phase_offset = mod(config_.phase_offset, config_.cpr); /*最后对 offset 做模运算，确保它始终在 [0, cpr) 范围内。*/
    }

    // Update states
    /*限值，将 count 限制到 config_.cpr 范围之内，例如限值为 4000*/
    count_in_cpr_ = mod(count, config_.cpr);
    /*已旋转一圈，清除 count_in_cpr_ 以及 pos_cpr_counts_*/
    pos_cpr_counts_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

bool Encoder::run_index_search() {
    config_.use_index = true;
    index_found_ = false;
    set_idx_subscribe(); /*实际就是开启 GPIO 中断，订阅 Index 脉冲（也就是 enc_index_cb()）*/

    /*调用电机控制逻辑，让电机按 lockin 策略缓慢加速旋转，直到触发 Index 或达到预设条件。*/
    bool success = axis_->run_lockin_spin(axis_->config_.calibration_lockin, false);
    return success;
}

bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;

    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;
    bool success = axis_->run_lockin_spin(lockin_config, false);

    if (success) {
        /*shadow_count_ 会持续更新，shadow_count_  大于暂存值说明在持续正转*/
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            config_.direction = 1;
        /*shadow_count_ 会持续更新，shadow_count_  小于于暂存值说明在持续反转*/
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            config_.direction = -1;
        } else {
            config_.direction = 0;
        }
    }

    return success;
}

/**运行霍尔极性校准*/
bool Encoder::run_hall_polarity_calibration() {
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;

    auto loop_cb = [this](bool const_vel) {
        if (const_vel)
            sample_hall_states_ = true;
        // No need to cancel early
        return true;
    };

    config_.hall_polarity_calibrated = false;
    states_seen_count_.fill(0);
    bool success = axis_->run_lockin_spin(lockin_config, false, loop_cb);
    sample_hall_states_ = false;

    if (success) {
        std::bitset<8> state_seen;
        std::bitset<8> state_confirmed;
        for (int i = 0; i < 8; i++) {
            if (states_seen_count_[i] > 0)
                state_seen[i] = true;
            if (states_seen_count_[i] > 50)
                state_confirmed[i] = true;
        }
        if (!(state_seen == state_confirmed)) {
            set_error(ERROR_ILLEGAL_HALL_STATE);
            return false;
        }

        // Hall effect sensors can be arranged at 60 or 120 electrical degrees.
        // Out of 8 possible states, 120 and 60 deg arrangements each miss 2 states.
        // ODrive assumes 120 deg separation - if a 60 deg setup is used, it can
        // be converted to 120 deg states by flipping the polarity of one sensor.
        uint8_t states = state_seen.to_ulong();
        uint8_t hall_polarity = 0;
        auto flip_detect = [](uint8_t states, unsigned int idx)->bool {
            return (~states & 0xFF) == (1<<(0+idx) | 1<<(7-idx));
        };
        if (flip_detect(states, 0)) {
            hall_polarity = 0b000;
        } else if (flip_detect(states, 1)) {
            hall_polarity = 0b001;
        } else if (flip_detect(states, 2)) {
            hall_polarity = 0b010;
        } else if (flip_detect(states, 3)) {
            hall_polarity = 0b100;
        } else {
            set_error(ERROR_ILLEGAL_HALL_STATE);
            return false;
        }
        config_.hall_polarity = hall_polarity;
        config_.hall_polarity_calibrated = true;
    }

    return success;
}

// 运行霍尔相位校准
bool Encoder::run_hall_phase_calibration() {
    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 30.0f; // run for 30 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_enc_idx = false;
    lockin_config.finish_on_vel = false;

    auto loop_cb = [this](bool const_vel) {
        if (const_vel)
            sample_hall_phase_ = true;
        // No need to cancel early
        return true;
    };

    // TODO: There is a race condition here with the execution in Encoder::update.
    // We should evaluate making thread execution synchronous with the control loops
    // at least optionally.
    // Perhaps the new loop_sync feature will give a loose timing guarantee that may be sufficient
    calibrate_hall_phase_ = true;
    config_.hall_edge_phcnt.fill(0.0f);
    hall_phase_calib_seen_count_.fill(0);
    bool success = axis_->run_lockin_spin(lockin_config, false, loop_cb);
    if (error_ & ERROR_ILLEGAL_HALL_STATE)
        success = false;

    if (success) {
        // Check deltas to dicern rotation direction
        float delta_phase = 0.0f;
        for (int i = 0; i < 6; i++) {
            int next_i = (i == 5) ? 0 : i+1;
            delta_phase += wrap_pm_pi(config_.hall_edge_phcnt[next_i] - config_.hall_edge_phcnt[i]);
        }
        // Correct reverse rotation
        if (delta_phase < 0.0f) {
            config_.direction = -1;
            for (int i = 0; i < 6; i++)
                config_.hall_edge_phcnt[i] = wrap_pm_pi(-config_.hall_edge_phcnt[i]);
        } else {
            config_.direction = 1;
        }
        // Normalize edge timing to 1st edge in sequence, and change units to counts
        float offset = config_.hall_edge_phcnt[0];
        for (int i = 0; i < 6; i++) {
            float& phcnt = config_.hall_edge_phcnt[i];
            phcnt = fmodf_pos((6.0f / (2.0f * M_PI)) * (phcnt - offset), 6.0f);
        }
    } else {
        config_.hall_edge_phcnt = hall_edge_defaults;
    }

    calibrate_hall_phase_ = false;
    return success;
}

/**
 * 将电机朝一个方向转动一会儿，然后朝另一个方向转动方向，
 * 以求电气相位之间的偏移量 0 和编码器状态 0。
 */
// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;

    // Require index found if enabled（编码器校准前没有先执行搜索定位编码器 Index 信号命令）
    if (config_.use_index && !index_found_) {
        set_error(ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }

    if (config_.mode == MODE_HALL && !config_.hall_polarity_calibrated) {
        set_error(ERROR_HALL_NOT_CALIBRATED_YET);
        return false;
    }

    /*我们使用 shadow_count_ 进行偏移校准，shadow_count_ 实际上会在编码器更新函数中更新，但为了及时性，在这里立即更新*/
    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    CRITICAL_SECTION() {
        // Reset state variables
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;

        float max_current_ramp = axis_->motor_.config_.calibration_current / start_lock_duration * 2.0f;
        axis_->open_loop_controller_.max_current_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_voltage_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;
        axis_->open_loop_controller_.target_current_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? axis_->motor_.config_.calibration_current : 0.0f;
        axis_->open_loop_controller_.target_voltage_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : axis_->motor_.config_.calibration_current;
        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(0 - config_.calib_scan_distance / 2.0f);

        axis_->motor_.current_control_.enable_current_control_src_ = (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);
        axis_->acim_estimator_.rotor_phase_src_.connect_to(&axis_->open_loop_controller_.phase_);

        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->acim_estimator_.rotor_phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
    }
    axis_->wait_for_control_iteration();

    axis_->motor_.arm(&axis_->motor_.current_control_);

    // go to start position of forward scan for start_lock_duration to get ready to scan
    for (size_t i = 0; i < (size_t)(start_lock_duration * 1000.0f); ++i) {
        if (!axis_->motor_.is_armed_) {
            return false; // TODO: return "disarmed" error code
        }
        if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
            axis_->motor_.disarm();
            return false; // TODO: return "aborted" error code
        }
        osDelay(1);
    }


    int32_t init_enc_val = shadow_count_;
    uint32_t num_steps = 0;
    int64_t encvaluesum = 0;

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = config_.calib_scan_omega;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
    }

    // scan forward（正向电角扫描:在电角度空间匀速旋转一次（扇区），采样 encoder 输出并累加）
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(-INFINITY) >= config_.calib_scan_distance;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }
    /*由于定时中断更新编码器值的原因，所以 shadow_count_ 会持续更新，即使前面有死循环*/

    /*shadow_count_ 会持续更新，shadow_count_  大于进入死循环前的暂存值说明在持续正转*/
    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        config_.direction = 1;
    /*shadow_count_ 会持续更新，shadow_count_  小于于进入死循环前的暂存值说明在持续反转*/
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        axis_->motor_.disarm();
        return false;
    }

    /*CPR 验证，确认驱动器配置时设定的 CPR 和电机实际脉冲数是否一致*/
    /*防止脉冲数设置错误，比如把编码器 CPR 设成了 2048 实际是 8192*/

    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = std::abs(shadow_count_ - init_enc_val);
    if (std::abs(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        axis_->motor_.disarm();
        return false;
    }

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = -config_.calib_scan_omega;
    }

    // scan backwards
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(INFINITY) <= 0.0f;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Motor disarmed because of an error
    if (!axis_->motor_.is_armed_) {
        return false;
    }

    axis_->motor_.disarm();

    /*计算编码器值相对电角度中心的固定偏移值，取中值作为编码器偏移位置，得出一个固定偏移值（偏移校准参数）*/
    config_.phase_offset = encvaluesum / num_steps;
    int32_t residual = encvaluesum - ((int64_t)config_.phase_offset * (int64_t)num_steps);
    /*offset_float 是子采样补偿（防止小误差导致 jitter）*/
    config_.phase_offset_float = (float)residual / (float)num_steps + 0.5f;  // add 0.5 to center-align state to phase
    /*这两个最终计算结果都是编码器偏移校准参数，需要存储到 Flash*/

    is_ready_ = true;
    return true;
}

/*解码霍尔传感器的三位二进制数*/
static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}

/*定时中断轮询采样编码器数据*/
void Encoder::sample_now() {
    switch (mode_) {
        /*增量式编码器，通过定时器计数更新位置*/
        case MODE_INCREMENTAL: {
            /*tim_cnt_sample_ 指的是利用定时中断定时更新采集的编码器统计数*/
            tim_cnt_sample_ = (int16_t)timer_->Instance->CNT;
        } break;

        /*霍尔编码器*/
        case MODE_HALL: {
            // do nothing: samples already captured in general GPIO capture
        } break;

        case MODE_SINCOS: { /*正弦/余弦模拟信号编码器，填写 ADC 通道号*/
            sincos_sample_s_ = get_adc_relative_voltage_ch(2); /*GPIOA, GPIO_PIN_2*/
            sincos_sample_c_ = get_adc_relative_voltage_ch(3); /*GPIOA, GPIO_PIN_3*/
        } break;

        /*多种 SPI 接口的绝对式编码器（AMS、CUI、RLS、AEAT）*/
        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI:
        case MODE_SPI_ABS_AEAT:
        case MODE_SPI_ABS_RLS:
        case MODE_SPI_ABS_MA732:
        {
            abs_spi_start_transaction();
            // Do nothing
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }

    // Sample all GPIO digital input data registers, used for HALL sensors for example.
    for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); ++i) {
        port_samples_[i] = ports_to_sample[i]->IDR;
    }
}

bool Encoder::read_sampled_gpio(Stm32Gpio gpio) {
    for (size_t i = 0; i < sizeof(ports_to_sample) / sizeof(ports_to_sample[0]); ++i) {
        if (ports_to_sample[i] == gpio.port_) {
            return port_samples_[i] & gpio.pin_mask_;
        }
    }
    return false;
}

/*读取霍尔传感器采样的三位二进制数*/
void Encoder::decode_hall_samples() {
    hall_state_ = (read_sampled_gpio(hallA_gpio_) ? 1 : 0)
                | (read_sampled_gpio(hallB_gpio_) ? 2 : 0)
                | (read_sampled_gpio(hallC_gpio_) ? 4 : 0);
}

bool Encoder::abs_spi_start_transaction() {
    if (mode_ & MODE_FLAG_ABS){
        if (Stm32SpiArbiter::acquire_task(&spi_task_)) {
            spi_task_.ncs_gpio = abs_spi_cs_gpio_;
            spi_task_.tx_buf = (uint8_t*)abs_spi_dma_tx_;
            spi_task_.rx_buf = (uint8_t*)abs_spi_dma_rx_;
            spi_task_.length = 1;
            spi_task_.end_callback = [](void* ctx, bool success) 
            { 
                ((Encoder*)ctx)->abs_spi_cb(success); 
            };
            spi_task_.parm = this;
            spi_task_.next = nullptr;
            
            spi_arbiter_->transfer_async(&spi_task_);
        } else {
            return false;
        }
    }
    return true;
}

/**
 * 绝对值编码器 SPI 数据读取与解析 的核心逻辑。它涵盖了对 AMS, CUI, 和 RLS 三种 SPI 绝对编码器的支持，
 * 并在 DMA 读取完成回调中解析位置值、校验奇偶性、更新内部状态。
 * 支持的编码器类型：
 * AMS（如 AS5047P）：需要偶校验 + 错误标志判断
 * CUI（如 AMT21）：需要特殊方式奇偶校验（返回 ~v & 3）
 * RLS（如 RM44）：无校验，右移2位获得有效数据
 */

/*经典的位运算方式，用于快速求 16 位值的奇偶校验（返回 1 表示奇数个 1）*/
uint8_t ams_parity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

/*CUI 编码器奇偶校验方式特殊，返回值不是 1 位，而是 2 位*/
uint8_t cui_parity(uint16_t v) {
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    return ~v & 3;
}

void Encoder::abs_spi_cb(bool success) {
    uint16_t pos;

    if (!success) {
        goto done;
    }

    switch (mode_) {
        case MODE_SPI_ABS_AMS: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            // check if parity is correct (even) and error flag clear
            if (ams_parity(rawVal) || ((rawVal >> 14) & 1)) {
                goto done;
            }
            /*第 14 位是 错误标志（1 = error) 第 15 位是 偶校验 低 14 位是实际角度*/
            pos = rawVal & 0x3fff;
        } break;

        case MODE_SPI_ABS_CUI: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            // check if parity is correct
            if (cui_parity(rawVal)) {
                goto done;
            }
            /*低 14 位为角度值，其余为奇偶校验*/
            pos = rawVal & 0x3fff;
        } break;

        case MODE_SPI_ABS_RLS: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            /*数据右移两位，直接取中间 14 位为角度*/
            pos = (rawVal >> 2) & 0x3fff;
        } break;

        case MODE_SPI_ABS_MA732: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            pos = (rawVal >> 2) & 0x3fff;
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           goto done;
        } break;
    }

    pos_abs_ = pos;
    abs_spi_pos_updated_ = true;
    if (config_.pre_calibrated) {
        is_ready_ = true;
    }

done:
    Stm32SpiArbiter::release_task(&spi_task_);
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode and init cs pin
    /*abs_spi_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);*/

    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

/*霍尔传感器模式*/
// Note that this may return counts +1 or -1 without any wrapping
int32_t Encoder::hall_model(float internal_pos) {
    int32_t base_cnt = (int32_t)std::floor(internal_pos);

    float pos_in_range = fmodf_pos(internal_pos, 6.0f);
    int pos_idx = (int)pos_in_range;
    if (pos_idx == 6) pos_idx = 5; // in case of rounding error
    int next_i = (pos_idx == 5) ? 0 : pos_idx+1;

    float below_edge = config_.hall_edge_phcnt[pos_idx];
    float above_edge = config_.hall_edge_phcnt[next_i];

    // if we are blow the "below" edge, we are the count under
    if (wrap_pm(pos_in_range - below_edge, 6.0f) < 0.0f)
        return base_cnt - 1;
    // if we are above the "above" edge, we are the count over
    else if (wrap_pm(pos_in_range - above_edge, 6.0f) > 0.0f)
        return base_cnt + 1;
    // otherwise we are in the nominal count (or completely lost)
    return base_cnt;
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; //LATCH

    switch (mode_) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit

            /*tim_cnt_sample_ 指的是利用定时中断定时更新采集的编码器统计数*/

            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;

        case MODE_HALL: {
            decode_hall_samples();
            if (sample_hall_states_) {
                states_seen_count_[hall_state_]++;
            }
            if (config_.hall_polarity_calibrated) {
                int32_t hall_cnt;
                if (decode_hall((hall_state_ ^ config_.hall_polarity), &hall_cnt)) {
                    if (calibrate_hall_phase_) {
                        if (sample_hall_phase_ && last_hall_cnt_.has_value()) {
                            int mod_hall_cnt = mod(hall_cnt - last_hall_cnt_.value(), 6);
                            size_t edge_idx;
                            if (mod_hall_cnt == 0) { goto skip; } // no count - do nothing
                            else if (mod_hall_cnt == 1) { // counted up
                                edge_idx = hall_cnt;
                            } else if (mod_hall_cnt == 5) { // counted down
                                edge_idx = last_hall_cnt_.value();
                            } else {
                                set_error(ERROR_ILLEGAL_HALL_STATE);
                                return false;
                            }

                            auto maybe_phase = axis_->open_loop_controller_.phase_.any();
                            if (maybe_phase) {
                                float phase = maybe_phase.value();
                                // Early increment to get the right divisor in recursive average
                                hall_phase_calib_seen_count_[edge_idx]++;
                                float& edge_phase = config_.hall_edge_phcnt[edge_idx];
                                if (hall_phase_calib_seen_count_[edge_idx] == 1)
                                    edge_phase = phase;
                                else {
                                    // circularly wrapped recursive average
                                    edge_phase += (phase - edge_phase) / hall_phase_calib_seen_count_[edge_idx];
                                    edge_phase = wrap_pm_pi(edge_phase);
                                }
                            }
                        }
                    skip:
                        last_hall_cnt_ = hall_cnt;

                        return true; // Skip all velocity and phase estimation
                    }

                    delta_enc = hall_cnt - count_in_cpr_;
                    delta_enc = mod(delta_enc, 6);
                    if (delta_enc > 3)
                        delta_enc -= 6;
                } else {
                    if (!config_.ignore_illegal_hall_state) {
                        set_error(ERROR_ILLEGAL_HALL_STATE);
                        return false;
                    }
                }
            }
        } break;

        case MODE_SINCOS: {
            float phase = fast_atan2(sincos_sample_s_, sincos_sample_c_);
            int fake_count = (int)(1000.0f * phase);
            //CPR = 6283 = 2pi * 1k

            delta_enc = fake_count - count_in_cpr_;
            delta_enc = mod(delta_enc, 6283);
            if (delta_enc > 6283/2)
                delta_enc -= 6283;
        } break;
        
        case MODE_SPI_ABS_RLS:
        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI: 
        case MODE_SPI_ABS_AEAT:
        case MODE_SPI_ABS_MA732: {
            if (abs_spi_pos_updated_ == false) {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.05f) {
                    set_error(ERROR_ABS_SPI_COM_FAIL);
                    return false;
                }
            } else {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }

            abs_spi_pos_updated_ = false;
            delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr/2) {
                delta_enc -= config_.cpr;
            }

        }break;
        default: {
            set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return false;
        } break;
    }

    /*这里并不是将 tim_cnt_sample_ 值直接赋予给 count_in_cpr_ 变量，而是通过增量的形式更新*/
    /*shadow_count_ 以及 count_in_cpr_ 在此同步更新*/
    shadow_count_ += delta_enc; /*当前编码器值相比上一次编码器值的变化量*/
    count_in_cpr_ += delta_enc; /*当前编码器值相比上一次编码器值的变化量*/
    /*限值，将 count_in_cpr_ 限制到 config_.cpr 范围之内，例如限值为 4000*/
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if(mode_ & MODE_FLAG_ABS)
        count_in_cpr_ = pos_abs_latched;

    /*详细查看以下内容解读：*/
    /*https://blog.csdn.net/loop222/article/details/133788966*/
    /*https://zhuanlan.zhihu.com/p/665365512*/
    /*https://blog.csdn.net/weixin_43824941/article/details/118739397*/
    /*https://blog.csdn.net/MOS_JBET/article/details/147153320*/

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    /*使用一个 简化的一阶数字 PLL（锁相环）结构，核心任务是：利用 编码器读数（离散跳变）估算 连续相位，利用估算相位变化量，推算 电机的角速度。*/

    /*锁相环 PLL 滤波器：用于平滑位置和速度估计，关键参数为 pll_kp_、pll_ki_。*/
    /*如果速度估计很小，还会自动 snap 到 0，防止抖动。*/
    // run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_; /*整体位置估计（可能 > CPR，适合全局定位）*/
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_; /*仅在 CPR 范围内的位置估计（周期性），用于电角度和控制*/

    /*注意上面 vel_estimate_counts_ 没有被赋予初始值，所以默认为 0，
    不赋予初始值是因为它可以随着时间的推移通过和实际值的误差积分逐渐的逼近正确值*/

    // Encoder model
    auto encoder_model = [this](float internal_pos)->int32_t {
        if (config_.mode == MODE_HALL)
            return hall_model(internal_pos);
        else
            /*std::floor 对给定的浮点数（通常是 float 或 double 类型）执行向下取整操作，
            即将数字舍去小数部分，返回小于或等于该数的最大整数值。*/
            return (int32_t)std::floor(internal_pos);
    };
    // discrete phase detector 即离散相位检波器（计算实际 shadow_count_，count_in_cpr_ 和各自估计值的差值）或理解为计算位置误差（离散检测器）:比较预测位置与实际编码器值的差值。
    float delta_pos_counts = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));

    /*计算位置误差（离散检测器）:比较预测位置与实际编码器值的差值。*/
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    /**wrap_pm() 使用 ARM 汇编语言实现的单精度浮点数到 32 位有符号整数的转换。它利用了ARM处理器中的向量浮点单元(VFP)提供的VCVT指令来进行类型转换。
    在这里将 delta_pos_cpr_counts / (float)(config_.cpr) 的浮点结果转化为带符号的 32 位整数*/

    // pll feedback（PLL 反馈：修正位置和速度）
    /**这是典型的 PI 控制结构，模仿锁相环的思想：位置偏差->修正估计位置（P），
    位置偏差积分->修正速度估计（I）这样既能保证跟踪精度，也具备一定滤波特性，抗抖动。*/
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    /*慢速防抖 snap-to-zero*/
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller（编码器输出的可供使用的位置数据）
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;
    
    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_.any().value_or(0.0f);
    pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular = fmodf_pos(pos_circular, axis_->controller_.config_.circular_setpoint_range);
    pos_circular_ = pos_circular;

    /*编码器插值（用于高分辨率控制)，对于增量编码器，跳变是离散的（如每 1 tick 才有更新）
    插值 = 利用估计速度进行线性推测，补足跳变之间的空白*/
    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f; /*静止*/
    // reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f; /*刚跳变（+1）*/
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f; /*刚跳变（-1）*/
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_; /*经过插值最后得到一个更平滑的 interpolated_enc 用于电角度等计算*/

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    //用于在传感器模式下 SVM 对转子所在扇区的判断（编码器还用于闭环控制模式，闭环控制编码器值不要求弧度制）

    /*elec_rad_per_enc 表示每个编码器计数对应的电角度（弧度），M_PI 用于将角度转换为弧度制表示，2*M_PI表示一圈的弧度（360°）。*/
    /*电机每转一圈，电角度会转 pole_pairs 圈。例如，极对数为 7，则电机轴转一圈，电角度转 7 圈（转子转一圈，会经过 7 个 N 极 和 7 个 S 极）。
    因为电机内部有定子绕组产生旋转磁场，转子有 N 个磁极（N-S 成对出现），每一对磁极（一个 N 和一个 S）构成一个完整的磁场周期，
    所以当转子旋转时，每经过一对磁极，磁场就完成一个完整的周期变化（相当于电角度走完 360°）。*/
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    /*插值后的编码器计数减去相位偏移计数，乘以每个编码器计数对应的电角度（弧度），得到当前电角度（弧度）。*/
    float ph = elec_rad_per_enc * (interpolated_enc - config_.phase_offset_float);
    
    if (is_ready_) {
        /**wrap_pm_pi 是用于环形参数限制，把角度限制在 [-PI, PI] 区间，防止溢出，还有乘以方向（正反转）。*/
        phase_ = wrap_pm_pi(ph) * config_.direction;
        phase_vel_ = (2*M_PI) * *vel_estimate_.present() * axis_->motor_.config_.pole_pairs * config_.direction; /*弧度/秒*/
    }

    return true;
}
