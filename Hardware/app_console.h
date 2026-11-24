#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 串口 2 命令行。
 *
 *   接收在 USART2 中断里做（只做行缓冲），解析在主循环里做。
 *   命令：
 *     SET <名字> <值>        例：SET LINE_KP 2.2
 *     SET /C <名字> <值>     例：SET /C LINE_KP 2.2（/C 会被忽略）
 *     SET <名字>=<值>        例：SET LINE_KP=2.2
 *     GET <名字>            读一个
 *     DUMP                  打印全部参数
 *     RESET                 恢复上电默认值
 *     HELP                  帮助
 *     TELEM <ms>            设置 [T] 打印周期（0 关闭）
 *     LOG <0|1>             关闭/打开日志
 */
void Console_Init(void);

/* 在 USART2 接收中断里调用：取走刚收到的字节并重新挂接收 */
void Console_HandleIrq(void);

/* UART 出错后重新挂接收（在 HAL_UART_ErrorCallback 里调用） */
void Console_Rearm(void);

/* 主循环里调用，处理已经收到的一整行 */
void Console_Poll(void);

#endif
