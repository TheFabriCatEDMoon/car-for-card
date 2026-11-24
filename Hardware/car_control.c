#include "car_control.h"
#include "config.h"
#include "app_params.h"
#include "track.h"

static float Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static float Abs(float value)
{
    return value < 0.0f ? -value : value;
}

static void ResetWheel(CarWheel *wheel)
{
    PID_Init(&wheel->pid);
    wheel->pid.Kp = g_params.wheel_kp;
    wheel->pid.Ki = g_params.wheel_ki;
    wheel->pid.OutMax = g_params.pwm_max;
    wheel->pid.OutMin = 0.0f;
    wheel->pid.ILimit = g_params.wheel_i_limit;
    wheel->reverse_frames = 0;
    wheel->stalled_frames = 0;
}

static void StopDrive(CarControl *car)
{
    car->left_pwm = 0;
    car->right_pwm = 0;
    car->left_target = 0.0f;
    car->right_target = 0.0f;
    car->ramp_speed = 0.0f;
    car->line_initialized = false;
    ResetWheel(&car->left);
    ResetWheel(&car->right);
}

/*
 * 正常停车：先回 READY，然后（如果配了 back_time）进入倒转阶段。
 * 倒转由 CarControl_Step 每帧推进，PWM 的正负号在 main 里按下发处理。
 * 故障锁停不走这里 —— 卡住的时候再倒转只会把电机憋坏。
 */
static void StopWithBack(CarControl *car)
{
    car->status = CAR_READY;
    car->start_pending = false;
    car->backing = false;

    if (car->back_time_ms > 0U) {
        car->backing = true;
        car->back_until_ms = HAL_GetTick() + car->back_time_ms;
        car->left_pwm = (uint8_t)CAR_BACK_PWM;    /* 只是显示用，实际方向由 backing 决定 */
        car->right_pwm = (uint8_t)CAR_BACK_PWM;
        car->left_target = 0.0f;
        car->right_target = 0.0f;
        car->ramp_speed = 0.0f;
        car->line_initialized = false;
        ResetWheel(&car->left);
        ResetWheel(&car->right);
        return;
    }

    StopDrive(car);
}

/* TIM2/3 free-run at ARR=65535. Subtract in the 16-bit counter domain. */
static int32_t EncoderDelta(CarWheel *wheel, uint16_t count)
{
    uint16_t difference = (uint16_t)(count - wheel->previous_count);
    wheel->previous_count = count;
    return difference < 0x8000U ? (int32_t)difference : (int32_t)difference - 65536;
}

static bool StartPressed(CarControl *car, bool pressed)
{
    if (!pressed) {
        car->key_down_frames = 0;
        if (car->key_up_frames < CAR_KEY_FRAMES) {
            ++car->key_up_frames;
        }
        if (car->key_up_frames == CAR_KEY_FRAMES) {
            car->key_armed = true;
        }
        return false;
    }

    car->key_up_frames = 0;
    if (car->key_armed && ++car->key_down_frames == CAR_KEY_FRAMES) {
        car->key_armed = false;
        return true;
    }
    return false;
}

#if CAR_LINE_LOST_STOP
/* 锁停后必须松手再按，避免一按到底反复触发 */
static void RequireKeyRelease(CarControl *car)
{
    car->key_armed = false;
    car->key_up_frames = 0;
    car->key_down_frames = 0;
}
#endif

static void RunBench(CarControl *car, bool key1, bool key2)
{
    uint8_t selected = key1 ? 1U : 2U;

    car->left_pwm = 0;
    car->right_pwm = 0;
    if (!key1 && !key2) {
        if (car->bench_wheel != 0U) {
            car->bench_armed = false;
        }
        car->bench_wheel = 0;
        car->bench_active_frames = 0;
        car->bench_down_frames = 0;
        if (car->bench_up_frames < CAR_KEY_FRAMES) {
            ++car->bench_up_frames;
        }
        if (car->bench_up_frames == CAR_KEY_FRAMES) {
            car->bench_armed = true;
        }
        return;
    }

    car->bench_up_frames = 0;
    if ((key1 && key2) || (car->bench_wheel != 0U && car->bench_wheel != selected)) {
        car->bench_armed = false;
        return;
    }
    if (!car->bench_armed) {
        return;
    }

    car->bench_wheel = selected;
    if (car->bench_down_frames < CAR_KEY_FRAMES) {
        ++car->bench_down_frames;
    }
    if (car->bench_down_frames < CAR_KEY_FRAMES) {
        return;
    }
    if (car->bench_active_frames == CAR_BENCH_FRAMES) {
        car->bench_armed = false;
        return;
    }

    ++car->bench_active_frames;
    if (selected == 1U) {
        car->left_pwm = CAR_BENCH_PWM;
    } else {
        car->right_pwm = CAR_BENCH_PWM;
    }
}

static void UpdateEncoderCheck(CarWheel *wheel, int32_t ticks, uint8_t pwm)
{
    if (pwm > 0U && ticks < 0) {
        if (wheel->reverse_frames < g_params.reverse_frames) {
            ++wheel->reverse_frames;
        }
    } else if (pwm == 0U || ticks > 0) {
        wheel->reverse_frames = 0;
    }

    if (pwm >= g_params.stall_pwm && ticks == 0) {
        if (wheel->stalled_frames < 255U) {
            ++wheel->stalled_frames;
        }
    } else {
        wheel->stalled_frames = 0;
    }
}

static CarStatus EncoderStatus(CarControl *car)
{
    UpdateEncoderCheck(&car->left, car->left_ticks, car->left_pwm);
    UpdateEncoderCheck(&car->right, car->right_ticks, car->right_pwm);
    if (car->left.reverse_frames >= g_params.reverse_frames) {
        return CAR_LEFT_REVERSE;
    }
    if (car->right.reverse_frames >= g_params.reverse_frames) {
        return CAR_RIGHT_REVERSE;
    }
    if (car->left.stalled_frames >= g_params.stall_frames) {
        return CAR_LEFT_NO_ENCODER;
    }
    if (car->right.stalled_frames >= g_params.stall_frames) {
        return CAR_RIGHT_NO_ENCODER;
    }
    return CAR_RUNNING;
}

static uint8_t DriveWheel(CarWheel *wheel, float target, int32_t ticks, bool is_left)
{
    float start_pwm = is_left ? g_params.left_start_pwm : g_params.right_start_pwm;
    float per_count = is_left ? g_params.left_pwm_per_count : g_params.right_pwm_per_count;

    if (target < 0.25f) {
        ResetWheel(wheel);
        return 0;
    }

    wheel->pid.Target = target;
    wheel->pid.OutMax = g_params.pwm_max;
    wheel->pid.Actual += g_params.wheel_filter_alpha * ((float)ticks - wheel->pid.Actual);
    wheel->pid.FeedForward = start_pwm + per_count * target;
    PID_Update(&wheel->pid);
    return (uint8_t)(wheel->pid.Out + 0.5f);
}

/* 把差速修正变成两轮目标速度和 PWM，限幅都在这里做。 */
static void DriveWithCorrection(CarControl *car, float correction)
{
    float turn_limit = Clamp(g_params.max_turn, 0.0f, car->ramp_speed);

    correction = Clamp(correction, -turn_limit, turn_limit);

    /* A line on the right calls for a faster left wheel. */
    car->left_target = Clamp(car->ramp_speed + correction, 0.0f, g_params.max_wheel_target);
    car->right_target = Clamp(car->ramp_speed - correction, 0.0f, g_params.max_wheel_target);
    car->left_pwm = DriveWheel(&car->left, car->left_target, car->left_ticks, true);
    car->right_pwm = DriveWheel(&car->right, car->right_target, car->right_ticks, false);
}

/* 正常循迹：误差低通 + 微分，再按弯道降速算差速。 */
static void FollowLine(CarControl *car, float error)
{
    float previous_error = car->filtered_line;
    float base;
    float correction;

    /* 可选：先串一级 3 点滑动平均（SET LINE_AVG 1 打开） */
    if (g_params.line_avg != 0U) {
        error = Error_Moving_Average(error);
    }

    if (!car->line_initialized) {
        car->filtered_line = error;
        car->line_derivative = 0.0f;
        car->line_initialized = true;
    } else {
        car->filtered_line += g_params.line_filter_alpha * (error - car->filtered_line);
        car->line_derivative += g_params.line_d_alpha *
            (car->filtered_line - previous_error - car->line_derivative);
    }

    base = Clamp(g_params.base_speed - g_params.curve_slowdown * Abs(car->filtered_line),
                 g_params.min_base_speed, g_params.base_speed);
    car->ramp_speed = Clamp(car->ramp_speed + g_params.speed_ramp, 0.0f, base);

    correction = g_params.line_kp * car->filtered_line +
                 g_params.line_kd * car->line_derivative;
    if (Abs(car->filtered_line) <= g_params.line_deadband) {
        correction = g_params.line_kd * car->line_derivative;
    }
    DriveWithCorrection(car, correction);
}

/*
 * 丢线后主动找线（只在 CAR_LINE_LOST_STOP == 0 时走这里）：
 *   1) 从中间丢的：先直行 SEARCH_MID 帧，跨过断线、路口的小缺口；
 *      从边上丢的：只直行 SEARCH_SIDE 帧，然后立刻朝那一侧找；
 *   2) 朝最后看到线的一侧打方向（力度 SEARCH_TURN_RATIO），最多 SEARCH_TURN 帧；
 *   3) 还没找到就每 SEARCH_SWEEP 帧左右换一次方向，一直扫到线回来；
 *   4) 线一进探头，下一帧就走 FollowLine，滤波器从头初始化。
 */
static void SearchLine(CarControl *car)
{
    float side = car->last_line_error >= 0.0f ? 1.0f : -1.0f;
    unsigned straight = Abs(car->last_line_error) < 0.5f
                            ? (unsigned)g_params.search_mid_frames
                            : (unsigned)g_params.search_side_frames;
    uint32_t lost = car->lost_frames;
    unsigned sweep = (unsigned)g_params.search_sweep_frames;
    float direction = 0.0f;
    float correction;
    uint32_t phase;

    if (sweep == 0U) {
        sweep = 1U;
    }

    /* 找线时降到 SEARCH_SPEED，转弯半径更小，更容易重新压到线。 */
    if (car->ramp_speed > g_params.search_speed) {
        car->ramp_speed -= g_params.speed_ramp;
        if (car->ramp_speed < g_params.search_speed) {
            car->ramp_speed = g_params.search_speed;
        }
    }

    if (lost <= straight) {
        direction = 0.0f;                                   /* ① 先直行跨缺口 */
    } else if (lost <= straight + (uint32_t)g_params.search_turn_frames) {
        direction = side;                                   /* ② 朝最后方向找 */
    } else {
        phase = (lost - straight - (uint32_t)g_params.search_turn_frames) / sweep;
        direction = (phase % 2U) == 0U ? side : -side;      /* ③ 左右交替扫线 */
    }

    /* 找线期间不用 PD；线回来时由 FollowLine 重新初始化滤波器。 */
    car->line_initialized = false;
    correction = direction * g_params.search_turn_ratio *
                 Clamp(g_params.max_turn, 0.0f, car->ramp_speed);
    DriveWithCorrection(car, correction);
}

/* 看到线就正常循迹；丢线按 CAR_LINE_LOST_STOP 决定停车还是主动找线。 */
static void DriveAlongLine(CarControl *car, TrackState line)
{
    if (line == TRACK_LINE) {
        car->status = CAR_RUNNING;
        FollowLine(car, car->line_error);
        return;
    }

#if CAR_LINE_LOST_STOP
    /* 原保护：第一帧丢线先撤 PWM，连续坏帧后锁停，必须重按 KEY1。 */
    StopDrive(car);
    if (car->invalid_frames >= g_params.invalid_frames) {
        car->status = line == TRACK_LOST ? CAR_NO_LINE : CAR_BAD_LINE;
        RequireKeyRelease(car);
    }
#else
    /* 关闭保护：不停车，边跑边把线找回来。 */
    car->status = CAR_SEARCH;
    SearchLine(car);
#endif
}

void CarControl_Init(CarControl *car, uint16_t left_count, uint16_t right_count,
                     bool bench_mode)
{
    *car = (CarControl){CAR_READY};
    car->status = bench_mode ? CAR_BENCH : CAR_READY;
    car->cross_count = -1;          /* 按需求：上电初始值 -1 */
    car->back_time_ms = CAR_BACK_MS;  /* 停车后倒转时长，0 = 不倒转 */
    car->cross_latched = false;
    car->left.previous_count = left_count;
    car->right.previous_count = right_count;
    StopDrive(car);
}

/* 外部强制停车（模式逻辑用）：回 READY，按 KEY1 可再启动 */
void CarControl_Stop(CarControl *car)
{
    StopWithBack(car);
}

void CarControl_Step(CarControl *car, uint8_t gray, uint16_t left_count,
                     uint16_t right_count, bool key1_pressed, bool key2_pressed)
{
    bool start = StartPressed(car, key1_pressed);
    float decoded_error = 0.0f;
    TrackState line = Track_Decode(gray, &decoded_error);

    car->gray = gray;
    car->left_ticks = EncoderDelta(&car->left, left_count) * CAR_LEFT_ENCODER_SIGN;
    car->right_ticks = EncoderDelta(&car->right, right_count) * CAR_RIGHT_ENCODER_SIGN;

    /* 压线计数：3 路及以上同时压黑，按"没压线 → 压线"的跳变加 1，
       一次压线只加 1（10 ms 一帧，否则一次过线会加十几次）。 */
    if (Track_BlackCount(gray) >= CAR_CROSS_MIN_CHANNELS) {
        if (!car->cross_latched) {
            car->cross_latched = true;
            if (car->cross_count < INT16_MAX) {
                ++car->cross_count;
            }
        }
    } else {
        car->cross_latched = false;
    }

    /* 丢线时仍然显示、也继续使用最后一次有效误差。 */
    if (line == TRACK_LINE) {
        car->last_line_error = decoded_error;
        car->line_error = decoded_error;
    } else {
        car->line_error = car->last_line_error;
    }

    if (car->status == CAR_BENCH) {
        RunBench(car, key1_pressed, key2_pressed);
        return;
    }

    /* 停车后的倒转阶段：两轮后退 back_time_ms，时间到才真正停住。
       实际下发方向在 main 的 TIM4 回调里按 car->backing 取反。 */
    if (car->backing) {
        if ((int32_t)(HAL_GetTick() - car->back_until_ms) < 0) {
            car->left_pwm = (uint8_t)CAR_BACK_PWM;    /* 显示用 */
            car->right_pwm = (uint8_t)CAR_BACK_PWM;
            return;
        }
        car->backing = false;
        StopDrive(car);
        return;
    }

    /* 延时起步（mode 2）：KEY1 已按下，等 start_delay_ms 再真正开跑，
       这段时间上层在读视觉卡片、算目标圈数。 */
    if (car->start_pending) {
        if ((uint32_t)(HAL_GetTick() - car->start_request_ms) < car->start_delay_ms) {
            return;
        }
        car->start_pending = false;
        if (car->status == CAR_READY) {
            StopDrive(car);
            car->status = CAR_RUNNING;
            DriveAlongLine(car, line);
        }
        return;
    }

    /* 连续有效帧数：起步需要攒够 frames 才允许开跑 */
    if (line == TRACK_LINE) {
        if (car->valid_frames < g_params.valid_frames) {
            ++car->valid_frames;
        }
        car->invalid_frames = 0;
        car->lost_frames = 0;
    } else {
        car->valid_frames = 0;
        if (car->invalid_frames < g_params.invalid_frames) {
            ++car->invalid_frames;
        }
        if (car->lost_frames < UINT32_MAX) {
            ++car->lost_frames;
        }
    }

    /* KEY2 去抖：数有效按键（上层切 mode），顺手把压线计数清零 */
    if (key2_pressed) {
        if (car->stop_down_frames < 255U) {
            ++car->stop_down_frames;
        }
        if (car->stop_down_frames == g_params.stop_frames) {
            if (car->key2_presses < UINT16_MAX) {
                ++car->key2_presses;   /* 一次有效按键只加 1 */
            }
            /* 按 KEY2 = 重新开始计数：回到上电初值 -1 */
            car->cross_count = -1;
            car->cross_latched = false;
        }
    } else {
        car->stop_down_frames = 0;
    }

    if (car->status == CAR_RUNNING || car->status == CAR_SEARCH) {
        CarStatus encoder = EncoderStatus(car);
#if CAR_ENCODER_PROTECT
        if (encoder != CAR_RUNNING) {
            car->status = encoder;
            StopDrive(car);
            return;
        }
#else
        (void)encoder;   /* 临时关掉堵转保护：只跑不停车 */
#endif
        /* KEY1 再按一次：停车（先倒转 back_time_ms 再停住） */
        if (start) {
            StopWithBack(car);
            return;
        }
        DriveAlongLine(car, line);
        return;
    }

    if (!start) {
        return;
    }

    /* 报警锁存状态：这一次按键先解除报警，回到 READY */
    if (car->status != CAR_READY) {
        car->status = CAR_READY;
        StopDrive(car);
        return;
    }

    /* READY：起步 */
    if (car->valid_frames < g_params.valid_frames) {
        car->status = line == TRACK_AMBIGUOUS ? CAR_BAD_LINE : CAR_NO_LINE;
        return;
    }

    /* 配了延时就先挂起，倒计时结束才真正起步 */
    if (car->start_delay_ms > 0U) {
        if (!car->start_pending) {
            car->start_pending = true;
            car->start_request_ms = HAL_GetTick();
        }
        return;
    }

    StopDrive(car);
    car->status = CAR_RUNNING;
    DriveAlongLine(car, line);
}
