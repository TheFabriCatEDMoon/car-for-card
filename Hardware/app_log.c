#include "app_log.h"
#include "log.h"
#include "usart.h"

/*
 * newlib-nano 默认不链接浮点 printf，%f 会打印成空。
 * 常见做法是链接时加 -u _printf_float，但 EIDE 在保存工程时会把它覆盖掉，
 * 所以在源码里强引用一下这个符号：效果等价，而且不依赖构建配置。
 * 没有这行，串口里的 [T] 遥测和 VOFA 数据所有浮点都会是空的。
 */
extern char _printf_float;
static char *volatile s_force_float_printf __attribute__((used)) = &_printf_float;

static bool s_initialized = false;
static bool s_enabled = false;
static bool s_vofa = false;

/* log.c 要求的底层写接口：把一段字节送出去，返回是否成功 */
static bool AppLog_Write(const char *data, uint32_t length)
{
    if (length == 0U) {
        return true;
    }
    /* 115200 下 256 字节约 22 ms，给 100 ms 足够余量 */
    return HAL_UART_Transmit(&huart2, (uint8_t *)data, (uint16_t)length, 100U) == HAL_OK;
}

static const LogPortOps s_port_ops = {
    .write = AppLog_Write
};

void AppLog_Init(void)
{
    LogConfig config;

    config.ops = &s_port_ops;
    config.level = LOG_LEVEL_INFO;
    config.enable_color = false;   /* 日志要存文件，别混 ANSI 转义 */
    config.async_write = false;    /* 同步阻塞，主循环里打印就够了 */

    if (log_init(&config) == LOG_STATUS_OK) {
        s_initialized = true;
        s_enabled = true;
    } else {
        s_initialized = false;
        s_enabled = false;
    }
}

void AppLog_SetEnabled(bool enabled)
{
    if (!s_initialized) {
        return;   /* 端口没注册成功，开关没有意义 */
    }
    (void)log_set_level(enabled ? LOG_LEVEL_INFO : LOG_LEVEL_NONE);
    s_enabled = enabled;
}

bool AppLog_IsEnabled(void)
{
    return s_enabled;
}

void AppLog_WriteRaw(const char *data, uint32_t length)
{
    (void)AppLog_Write(data, length);
}

void AppLog_SetVofaMode(bool on)
{
    s_vofa = on;
    if (!s_initialized) {
        return;
    }
    /* VOFA 模式：文本日志全关，避免日志行混进数据帧 */
    (void)log_set_level(on ? LOG_LEVEL_NONE : LOG_LEVEL_INFO);
    s_enabled = !on;
}

bool AppLog_VofaMode(void)
{
    return s_vofa;
}
