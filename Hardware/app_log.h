#ifndef APP_LOG_H
#define APP_LOG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 日志端口注册层。
 *
 * 日志核心是 Hardware/log.c（log_info / log_warn / log_error / log_vofa），
 * 它只认一个 LogPortOps.write 回调。这个文件负责把该回调绑到串口 2，
 * 并统一管理日志开关。
 *
 * 用法：MX_USART2_UART_Init() 之后调用一次 AppLog_Init()。
 *
 * 注意：
 *   1. 同步阻塞发送（HAL_UART_Transmit），只能在主循环里打印，
 *      不要在 TIM4 / USART 中断里调用。
 *   2. 浮点格式依赖 printf 的 %f，链接时必须带 -u _printf_float，
 *      否则 newlib-nano 会把 %f 打印成空。
 */
void AppLog_Init(void);

/* 日志总开关：false 时 log_info/log_vofa 都不输出（error 也不输出） */
void AppLog_SetEnabled(bool enabled);
bool AppLog_IsEnabled(void);

/*
 * 直接往串口 2 写一段原始字节，不经过 log 的格式化和等级过滤。
 * VOFA+ 的 FireWater 帧（"a,b,c\n"）就是这么发出去的。
 * 只能在主循环里调用（同步阻塞发送）。
 */
void AppLog_WriteRaw(const char *data, uint32_t length);

/*
 * VOFA 模式：打开后停掉所有文本日志，只留 VOFA 数据帧。
 * 否则日志行会被 VOFA 当成一帧数据画进去，曲线全是毛刺。
 */
void AppLog_SetVofaMode(bool on);
bool AppLog_VofaMode(void);

#endif
