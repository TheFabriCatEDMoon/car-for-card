#ifndef APP_PARAMS_H
#define APP_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 运行期参数表。
 *
 * 以前所有可调量都是 config.h 里的宏，改一次要重新编译烧录。
 * 现在把它们搬进这个结构体：上电时用宏里的值初始化，运行中可以通过
 * 串口 2 的 "SET <名字> <值>" 直接改，控制环每一帧都读这里，立刻生效。
 */
typedef struct {
    /* 速度环 */
    float base_speed;           /* 直线基础速度（计数/10ms） */
    float min_base_speed;       /* 弯道基础速度下限 */
    float speed_ramp;           /* 每周期加减速量 */
    float curve_slowdown;       /* 误差每 1 个探头间距降低多少基础速度 */
    float max_wheel_target;     /* 单轮目标速度上限 */
    float max_turn;             /* 单侧差速修正上限 */

    /* 循迹 PD */
    float line_kp;
    float line_kd;
    float line_filter_alpha;    /* 误差一阶低通系数 */
    float line_d_alpha;         /* 微分项低通系数 */
    float line_deadband;        /* 误差死区（探头间距） */

    /* 轮速 PI */
    float wheel_kp;
    float wheel_ki;
    float wheel_i_limit;
    float wheel_filter_alpha;

    /* 电机前馈与限幅 */
    float left_start_pwm;
    float right_start_pwm;
    float left_pwm_per_count;
    float right_pwm_per_count;
    float pwm_max;

    /* 丢线找线 */
    float search_speed;
    float search_turn_ratio;
    uint16_t search_mid_frames;
    uint16_t search_side_frames;
    uint16_t search_turn_frames;
    uint16_t search_sweep_frames;

    /* 保护阈值 */
    uint16_t stall_frames;
    uint16_t stall_pwm;
    uint16_t reverse_frames;
    uint16_t valid_frames;
    uint16_t invalid_frames;
    uint16_t stop_frames;

    /* 日志与遥测 */
    uint16_t telemetry_ms;      /* [T] 全量数据打印周期，0 = 关闭 */
    uint16_t params_ms;         /* [P] 参数打印周期，0 = 关闭 */
    uint16_t log_enable;        /* 0 = 静默（只影响串口日志） */

    /* 滤波 */
    uint16_t line_avg;          /* 1 = 循迹误差再串一级 3 点滑动平均 */
} CarParams;

extern CarParams g_params;

/* 用 config.h 里的宏恢复默认值 */
void Params_Init(void);

/* 按名字设置/读取，名字不区分大小写；越界会自动夹到合法区间 */
bool Params_Set(const char *name, float value);
bool Params_Get(const char *name, float *value);

/* 打印 */
void Params_Dump(void);
void Params_Help(void);

int Params_Count(void);
const char *Params_NameAt(int index);
float Params_ValueAt(int index);
float Params_MinAt(int index);
float Params_MaxAt(int index);
int Params_DecimalsAt(int index);

#endif
