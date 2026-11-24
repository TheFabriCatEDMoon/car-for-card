#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "PID.h"

typedef enum {
    CAR_READY,
    CAR_RUNNING,
    CAR_NO_LINE,
    CAR_BAD_LINE,
    CAR_LEFT_REVERSE,
    CAR_RIGHT_REVERSE,
    CAR_LEFT_NO_ENCODER,
    CAR_RIGHT_NO_ENCODER,
    CAR_BENCH,
    CAR_SEARCH
} CarStatus;

typedef struct {
    PID_t pid;
    uint16_t previous_count;
    uint8_t reverse_frames;
    uint8_t stalled_frames;
} CarWheel;

typedef struct {
    CarStatus status;
    uint8_t gray;
    int32_t left_ticks;
    int32_t right_ticks;
    float line_error;
    float left_target;
    float right_target;
    uint8_t left_pwm;
    uint8_t right_pwm;

    CarWheel left;
    CarWheel right;
    float filtered_line;
    float line_derivative;
    float ramp_speed;
    float last_line_error;      /* 最后一次有效循迹误差，丢线找线时用来定方向 */
    uint32_t lost_frames;       /* 已连续丢线多少帧（找线状态机用） */
    int16_t cross_count;        /* 压线计数：上电 -1，每压一次 +1 */
    bool cross_latched;         /* 这次压线已经加过，离开才重新武装 */
    uint16_t key2_presses;      /* KEY2 有效按下的次数（去抖后），给上层切模式用 */
    uint16_t start_delay_ms;    /* 延时起步：KEY1 之后等这么久再开跑（0 = 立即） */
    uint32_t start_request_ms;  /* 延时起步的起始时刻 */
    bool start_pending;         /* true = 正在延时等待起步（上层用它读卡片） */
    uint16_t back_time_ms;      /* 停车后两轮倒转多久（0 = 不倒转） */
    uint32_t back_until_ms;     /* 倒转结束时刻（HAL_GetTick） */
    bool backing;               /* true = 正在倒转，上层据此把 PWM 取反 */
    uint8_t valid_frames;
    uint8_t invalid_frames;
    uint8_t key_down_frames;
    uint8_t key_up_frames;
    uint8_t stop_down_frames;   /* KEY2 连续按下帧数（切 mode 的去抖） */
    uint8_t bench_up_frames;
    uint8_t bench_down_frames;
    uint8_t bench_active_frames;
    uint8_t bench_wheel;
    bool line_initialized;
    bool key_armed;
    bool bench_armed;
} CarControl;

void CarControl_Init(CarControl *car, uint16_t left_count, uint16_t right_count,
                     bool bench_mode);
void CarControl_Step(CarControl *car, uint8_t gray, uint16_t left_count,
                     uint16_t right_count, bool key1_pressed, bool key2_pressed);

/* 外部强制停车（模式逻辑用），会回到 READY 并清掉电机输出 */
void CarControl_Stop(CarControl *car);

#endif
