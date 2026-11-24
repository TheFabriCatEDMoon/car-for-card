#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Target;       // 目标值
    float Actual;       // 实际值
    float Actual1;      // 上一次实际值
    float Out;          // 输出值

    float Kp, Ki, Kd;   // 三个参数

    float Error0;       // 当前误差
    float Error1;       // 上次误差

    float POut;         // 比例项输出
    float IOut;         // 积分项输出
    float DOut;         // 微分项输出
    float DOut1;        // 上次微分项

    float DAlpha;       // 一阶滤波系数 (通常填 0.1 ~ 0.5)
    float DeadBand;     // 死区 (通常填个小数，比如 0.5)

    float OutMax;       // 输出限幅上限 (比如 99)
    float OutMin;       // 输出限幅下限 (比如 -99)
    float OutOffset;    // 输出偏移 (电机死区补偿，比如 10)
    float FeedForward;  // Additive feedforward, in output units.
    float ILimit;       // Symmetric integral limit. Zero uses output limits.
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif
