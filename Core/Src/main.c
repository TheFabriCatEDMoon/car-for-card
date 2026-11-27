/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "OLED.h"
#include "motor.h"
#include "track.h"
#include "car_control.h"
#include "log.h"
#include "app_log.h"
#include "app_params.h"
#include "app_console.h"
#include "config.h"   /* 所有可调参数与结构常量 */
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* TIM4 中断里填、主循环里读的一份快照（OLED 和串口遥测共用）。 */
typedef struct {
    CarStatus status;
    uint8_t gray;
    int32_t left_ticks;
    int32_t right_ticks;
    int16_t error_x10;
    uint8_t left_pwm;
    uint8_t right_pwm;
    float ramp_speed;
    float left_target;
    float right_target;
    uint32_t lost_frames;
    int16_t cross_count;         /* 压线计数：上电 -1，每次压线 +1 */
    uint16_t key2_presses;       /* KEY2 有效按下次数，用来切 mode */
} DisplaySnapshot;

/*
 * 视觉板（MaixPy）从 USART1 发来的一行，两种格式：
 *   <标签>\n                         例如：STOP
 *   <标签>,<置信度>,<x>,<y>,<w>,<h>\n 例如：NONE,0.00,0,0,0,0
 * 上电时还会先发一行 "Hello MaixPy"。
 * 置信度按 ×100 存成整数（0.95 → 95），避免在中断里碰浮点。
 */
typedef struct {
    char label[10];          /* 标签，和 OLED 上显示的一致 */
    int32_t score_x100;      /* 置信度 ×100 */
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    bool valid;              /* false = 收到 NONE */
    uint32_t frames;         /* 收到并解析成功的行数 */
} VisionFrame;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 2.0 的整车参数（起步斜坡、循迹 PID、轮速环、前馈）都是按这两个时间尺度标定的：
 *   PWM 载波 10 kHz，控制环 100 Hz。
 * 2.1 的主频从 64 MHz（HSI/2 × 16）换成了 72 MHz（HSE × 9），而 .ioc 里的
 * 预分频还是 2.0 那组数（TIM1=63、TIM4=6399），直接跑会变成 11.25 kHz / 112.5 Hz。
 * Timers_Match20Timing() 按实际主频重算 PSC，把时间尺度拉回 2.0。
 */
#define CAR_PWM_TICK_HZ    1000000U   /* TIM1 计数时钟，ARR=99 → 10 kHz PWM */
#define CAR_CTRL_TICK_HZ     10000U   /* TIM4 计数时钟，ARR=99 → 100 Hz 控制环 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static CarControl car;
static volatile DisplaySnapshot telemetry;
static uint8_t rx_byte;              /* USART1：视觉模块 */
static char rx_line[CAR_VISION_LINE_MAX];   /* 中断里正在收的一行 */
static uint8_t rx_line_len;
static char rx_ready[CAR_VISION_LINE_MAX];  /* 收满一行，交给主循环解析 */
static volatile bool rx_ready_flag;
static bool rx_discard;
static volatile char card_name[10];
static VisionFrame vision;           /* 解析后的视觉帧 */
static uint32_t last_telemetry_ms;
static uint32_t last_params_ms;
static CarStatus logged_status;
static bool logged_backing;          /* 倒转阶段开始/结束的日志边沿检测 */
static bool self_test_ok;
static bool s_log_enabled = true;
/* 工作模式：1..4 循环，每次按 KEY2 加 1，到 5 时回 1 */
static volatile uint8_t mode = 1;
static uint16_t last_key2_presses;
/* 目标压线次数（cross_count 达到它就停车）：mode1 固定 1，mode2 从卡片取 */
static volatile int16_t cross_target;
static volatile bool cross_target_valid;
static bool cross_done;              /* 已经因为到达目标停过车 */
static CarStatus prev_status;

#define APPLICATION_TASK_STACK_WORDS 768U
#define DISPLAY_TASK_STACK_WORDS     512U

static StaticTask_t application_task_tcb;
static StaticTask_t display_task_tcb;
static StackType_t application_task_stack[APPLICATION_TASK_STACK_WORDS];
static StackType_t display_task_stack[DISPLAY_TASK_STACK_WORDS];
static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static const char *StatusName(CarStatus status);
static const char *TrackStateName(TrackState state);
static void ShowRow(uint8_t row, char text[32]);
static void RefreshDisplay(void);
static void PrintTelemetry(void);
static void SelfTest_Run(void);
static void LogInitLine(const char *name, bool ok, const char *detail);
static void Vision_Poll(void);
static void PrintVofaFrame(void);
static int16_t CardNumber(const volatile char *name);
static void ApplyMode(uint8_t m);
static void Timers_Match20Timing(void);
static void ApplicationTask(void *argument);
static void DisplayTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *StatusName(CarStatus status)
{
    switch (status) {
    case CAR_READY:            return "READY";
    case CAR_RUNNING:          return "RUN";
    case CAR_NO_LINE:          return "NO LINE";
    case CAR_BAD_LINE:         return "BAD LINE";
    case CAR_LEFT_REVERSE:     return "LEFT REV";
    case CAR_RIGHT_REVERSE:    return "RIGHT REV";
    case CAR_LEFT_NO_ENCODER:  return "LEFT NO ENC";
    case CAR_RIGHT_NO_ENCODER: return "RIGHT NO ENC";
    case CAR_BENCH:            return "BENCH";
    case CAR_SEARCH:           return "SEARCH";
    default:                   return "?";
    }
}

static const char *TrackStateName(TrackState state)
{
    switch (state) {
    case TRACK_LINE:      return "LINE";
    case TRACK_LOST:      return "LOST";
    case TRACK_AMBIGUOUS: return "AMBIG";
    default:              return "?";
    }
}

static void LogInitLine(const char *name, bool ok, const char *detail)
{
    if (ok) {
        log_info("[INIT] %-26s OK   %s", name, detail ? detail : "");
    } else {
        log_error("[INIT] %-26s FAIL %s", name, detail ? detail : "");
    }
}

static void ShowRow(uint8_t row, char text[32])
{
    size_t length = 0;
    while (text[length] != '\0') {
        ++length;
    }
    while (length < 21U) {
        text[length++] = ' ';
    }
    text[21] = '\0';
    OLED_ShowString(0, row, text, OLED_6X8);
}

/*
 * 定时器重定标：按实际主频重算 TIM1（PWM）和 TIM4（控制环）的预分频，
 * 保证 2.0 标定过的参数在 72 MHz 下仍然是 10 kHz PWM / 100 Hz 控制环。
 *
 * 直接写 PSC/ARR 再触发一次 UG（更新事件），让影子寄存器立刻装载；
 * 不走 HAL_TIM_Base_Init，是为了不去碰 TIM1 已经配好的通道、BDTR 和 MOE。
 */
static void Timers_Match20Timing(void)
{
    uint32_t pwm_psc = (SystemCoreClock / CAR_PWM_TICK_HZ) - 1U;
    uint32_t ctrl_psc = (SystemCoreClock / CAR_CTRL_TICK_HZ) - 1U;
    uint32_t pwm_hz;
    uint32_t ctrl_hz;

    htim1.Init.Prescaler = pwm_psc;
    htim1.Init.Period = 99U;
    htim1.Instance->PSC = pwm_psc;
    htim1.Instance->ARR = 99U;
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    htim4.Init.Prescaler = ctrl_psc;
    htim4.Init.Period = 99U;
    htim4.Instance->PSC = ctrl_psc;
    htim4.Instance->ARR = 99U;
    htim4.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    pwm_hz = (SystemCoreClock / (pwm_psc + 1U)) / 100U;
    ctrl_hz = (SystemCoreClock / (ctrl_psc + 1U)) / 100U;
    log_info("[INIT] TIM1 PWM %lu Hz (PSC=%lu), TIM4 ctrl %lu Hz (PSC=%lu)",
             (unsigned long)pwm_hz, (unsigned long)pwm_psc,
             (unsigned long)ctrl_hz, (unsigned long)ctrl_psc);
}

/* ===================== 视觉板串口接收 =====================================
 * 发送端：serial_svc.py（MaixPy）
 *   上电：   "Hello MaixPy\n"
 *   有目标： "<标签>\n"                        例：STOP
 *   无目标： "NONE,0.00,0,0,0,0\n"             标签,NONE 置信度,x,y,w,h
 *
 * 中断里只做行缓冲，解析放在主循环，避免在中断里做重活。
 * ======================================================================== */

/* 取一个整数，遇到非数字停下，指针停在分隔符上 */
static int32_t Vision_ParseInt(const char **pp)
{
    const char *p = *pp;
    int32_t sign = 1;
    int32_t value = 0;

    while (*p == ' ') {
        ++p;
    }
    if (*p == '-') {
        sign = -1;
        ++p;
    }
    while (*p >= '0' && *p <= '9') {
        value = value * 10 + (*p - '0');
        ++p;
    }
    *pp = p;
    return sign * value;
}

/* 取一个 "0.95" 这样的小数，返回 ×100 的整数（0.95 → 95），不用浮点 */
static int32_t Vision_ParseHundredths(const char **pp)
{
    const char *p = *pp;
    int32_t sign = 1;
    int32_t value = 0;
    int32_t fraction = 0;
    unsigned digits = 0;

    while (*p == ' ') {
        ++p;
    }
    if (*p == '-') {
        sign = -1;
        ++p;
    }
    while (*p >= '0' && *p <= '9') {
        value = value * 10 + (*p - '0');
        ++p;
    }
    if (*p == '.') {
        ++p;
        while (*p >= '0' && *p <= '9' && digits < 2U) {
            fraction = fraction * 10 + (*p - '0');
            ++digits;
            ++p;
        }
        while (*p >= '0' && *p <= '9') {
            ++p;
        }
    }
    while (digits < 2U) {
        fraction *= 10;
        ++digits;
    }
    *pp = p;
    return sign * (value * 100 + fraction);
}

/* 解析一整行：标签[,置信度,x,y,w,h] */
static void Vision_Parse(const char *line)
{
    char label[10];
    unsigned n = 0;
    const char *p = line;
    int32_t score = 0;
    int32_t x = 0;
    int32_t y = 0;
    int32_t w = 0;
    int32_t h = 0;
    unsigned i;

    while (*p != '\0' && *p != ',' && n < sizeof(label) - 1U) {
        label[n++] = *p++;
    }
    label[n] = '\0';

    if (*p == ',') {
        ++p;
        score = Vision_ParseHundredths(&p);
        if (*p == ',') { ++p; x = Vision_ParseInt(&p); }
        if (*p == ',') { ++p; y = Vision_ParseInt(&p); }
        if (*p == ',') { ++p; w = Vision_ParseInt(&p); }
        if (*p == ',') { ++p; h = Vision_ParseInt(&p); }
    }

    for (i = 0; i < sizeof(vision.label); ++i) {
        vision.label[i] = label[i];
    }
    vision.score_x100 = score;
    vision.x = (int16_t)x;
    vision.y = (int16_t)y;
    vision.w = (int16_t)w;
    vision.h = (int16_t)h;
    vision.valid = !(label[0] == 'N' && label[1] == 'O' &&
                     label[2] == 'N' && label[3] == 'E' && label[4] == '\0');
    ++vision.frames;

    /* OLED 上显示的就是这个标签（保持原来的行为） */
    for (i = 0; i < sizeof(card_name); ++i) {
        card_name[i] = '\0';
    }
    for (i = 0; i < n && i < sizeof(card_name) - 1U; ++i) {
        card_name[i] = label[i];
    }
}

/* 主循环调用：把中断收到的一整行解析掉，标签变化时打一条日志 */
static void Vision_Poll(void)
{
    static char last_label[10];
    static bool last_valid;
    char line[CAR_VISION_LINE_MAX];
    uint32_t primask;

    if (!rx_ready_flag) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(line, rx_ready, sizeof line);
    rx_ready_flag = false;
    __set_PRIMASK(primask);

    Vision_Parse(line);

    if (strcmp(vision.label, last_label) != 0 || vision.valid != last_valid) {
        memcpy(last_label, vision.label, sizeof last_label);
        last_valid = vision.valid;
        if (vision.valid) {
            log_info("[VIS] %s score=%ld.%02ld box=%d,%d,%d,%d",
                     vision.label,
                     (long)(vision.score_x100 / 100),
                     (long)(vision.score_x100 % 100),
                     (int)vision.x, (int)vision.y,
                     (int)vision.w, (int)vision.h);
        } else {
            log_info("[VIS] NONE");
        }
    }
}

/* 从视觉标签里取数字：10C → 10，2C → 2，AC → 1，JC → 11，QC → 12，KC → 13。
   取不到（None、空串、未知标签）返回 0。 */
static int16_t CardNumber(const volatile char *name)
{
    int16_t value = 0;
    unsigned digits = 0;
    unsigned i = 0;

    while (i < 9U && name[i] >= '0' && name[i] <= '9' && digits < 3U) {
        value = (int16_t)(value * 10 + (name[i] - '0'));
        ++i;
        ++digits;
    }
    if (digits > 0U) {
        return value;
    }

    switch (name[0]) {
    case 'A': return 1;
    case 'J': return 11;
    case 'Q': return 12;
    case 'K': return 13;
    default:  return 0;
    }
}

/* 切换 mode 时设定：目标压线次数 + 启动延时 */
static void ApplyMode(uint8_t m)
{
    uint32_t start_delay_ms = (m == 2U) ? 1000U : 0U;
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    car.start_pending = false;
    car.start_delay_ms = start_delay_ms;
    cross_done = false;

    if (m == 2U) {
        cross_target = 0;             /* mode2：等 KEY1 之后从卡片取 */
        cross_target_valid = false;
    } else {
        cross_target = 1;             /* mode1 / 3 / 4：固定 1 次 */
        cross_target_valid = true;
    }
    __set_PRIMASK(irq_state);

    log_info("[MODE] %u  target=%d valid=%d delay=%u ms",
             (unsigned)m, (int)cross_target, (int)cross_target_valid,
             (unsigned)start_delay_ms);
}

static void RefreshDisplay(void)
{
    static const char *const status_text[] = {
        "READY", "RUN", "NO LINE", "BAD LINE",
        "LEFT REV", "RIGHT REV", "LEFT NO ENC", "RIGHT NO ENC", "BENCH",
        "SEARCH"
    };
    DisplaySnapshot data;
    char card[10];
    char bits[9];
    char text[32];
    unsigned i;
    uint8_t display_mode;
    int16_t display_cross_target;
    bool display_cross_target_valid;
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    data = telemetry;
    for (i = 0; i < sizeof(card); ++i) {
        card[i] = card_name[i];
    }
    display_mode = mode;
    display_cross_target = cross_target;
    display_cross_target_valid = cross_target_valid;
    __set_PRIMASK(irq_state);

    for (i = 0; i < 8U; ++i) {
        bits[i] = (data.gray & (1U << i)) ? '1' : '0';
    }
    bits[8] = '\0';

    (void)snprintf(text, sizeof(text), "%s",
                   status_text[(unsigned)data.status <
                               (sizeof status_text / sizeof status_text[0])
                                   ? (unsigned)data.status
                                   : 0U]);
    ShowRow(0, text);
    /* 右上角显示 mode：6x8 字体每字符 6 px，"M:n" 占 18 px */
    (void)snprintf(text, sizeof(text), "M:%u", (unsigned)display_mode);
    OLED_ShowString(CAR_OLED_MODE_X, 0, text, OLED_6X8);
    (void)snprintf(text, sizeof(text), "G:%s E:%+d", bits, (int)data.error_x10);
    ShowRow(2, text);
    (void)snprintf(text, sizeof(text), "L:%+ld R:%+ld",
                   (long)data.left_ticks, (long)data.right_ticks);
    ShowRow(3, text);
    (void)snprintf(text, sizeof(text), "PWM L:%u R:%u",
                   (unsigned)data.left_pwm, (unsigned)data.right_pwm);
    ShowRow(4, text);
    (void)snprintf(text, sizeof(text), "Cross: %d", (int)data.cross_count);
    ShowRow(5, text);
    /* Cross 行最右侧显示目标圈数 cross_target：右对齐，6x8 字体每字符 6 px，
       屏幕宽 128 px，所以起点 = 128 - 6 * 字符数。没设目标时显示 "T:-"。 */
    if (display_cross_target_valid) {
        (void)snprintf(text, sizeof(text), "T:%d", (int)display_cross_target);
    } else {
        (void)snprintf(text, sizeof(text), "T:-");
    }
    OLED_ShowString((uint8_t)(128U - 6U * (unsigned)strlen(text)), 5, text, OLED_6X8);
    (void)snprintf(text, sizeof(text), "Card: %s", card);
    ShowRow(6, text);
}

/* 100 ms 一次的遥测：一行人读、一行 VOFA 画图 */
static void PrintTelemetry(void)
{
    DisplaySnapshot data;
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    data = telemetry;
    __set_PRIMASK(irq_state);

    log_info("[T] st=%-11s g=%02X e=%+.2f v=%.2f tl=%.2f tr=%.2f pwm=%u/%u tk=%ld/%ld lf=%lu cc=%d md=%u",
             StatusName(data.status), data.gray,
             (double)data.error_x10 / 10.0,
             (double)data.ramp_speed, (double)data.left_target, (double)data.right_target,
             (unsigned)data.left_pwm, (unsigned)data.right_pwm,
             (long)data.left_ticks, (long)data.right_ticks,
             (unsigned long)data.lost_frames, (int)data.cross_count, (unsigned)mode);

    /*
     * 这里原来还有一条 log_vofa(...)：它的 names 参数是 #__VA_ARGS__ 展开的整串
     * 表达式，实际打出来是 "(int)data.status, data.gray, ...: 0, 249, -2.0, ..."，
     * 又长又没法解析，100 ms 一次白占约 1.8 KB/s 带宽。
     * 真正的画图数据由 VOFA 模式（PrintVofaFrame）输出纯 CSV，所以删掉它。
     */
}

/*
 * VOFA+ 的 FireWater 帧：一行 10 个通道，逗号分隔，'\n' 结尾。
 * 通道顺序（VOFA 里按这个顺序加控件）：
 *   0 e       = 循迹偏差
 *   1 v       = 当前限速（斜坡输出）
 *   2 tl      = 左轮目标速度
 *   3 tr      = 右轮目标速度
 *   4 pl      = 左轮 PWM
 *   5 pr      = 右轮 PWM
 *   6 gray    = 灰度原始字节
 *   7 cc      = 压线计数
 *   8 st      = 状态机编号（0=READY 1=RUN ... 9=SEARCH）
 *   9 md      = 当前 mode
 */
static void PrintVofaFrame(void)
{
    DisplaySnapshot data;
    uint32_t irq_state = __get_PRIMASK();
    char buffer[CAR_VOFA_LINE_MAX];
    int length;

    __disable_irq();
    data = telemetry;
    __set_PRIMASK(irq_state);

    length = snprintf(buffer, sizeof buffer,
                      "%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%d,%u,%u\n",
                      (double)data.error_x10 / 10.0,
                      (double)data.ramp_speed,
                      (double)data.left_target,
                      (double)data.right_target,
                      (unsigned)data.left_pwm,
                      (unsigned)data.right_pwm,
                      (unsigned)data.gray,
                      (int)data.cross_count,
                      (unsigned)data.status,
                      (unsigned)mode);

    if (length > 0) {
        if (length > (int)sizeof buffer - 1) {
            length = (int)sizeof buffer - 1;
        }
        AppLog_WriteRaw(buffer, (uint32_t)length);
    }
}

/*
 * 上电自检：每个模块初始化完都打一行，最后汇总。
 * 顺序和 main() 里的初始化顺序一致。
 */
static void SelfTest_Run(void)
{
    uint16_t left_before;
    uint16_t right_before;
    uint16_t left_after;
    uint16_t right_after;
    int32_t left_delta;
    int32_t right_delta;
    uint8_t gray_a;
    uint8_t gray_b;
    TrackState state;
    float error = 0.0f;
    bool key1;
    bool key2;

    log_info("======== self test begin ========");

    LogInitLine("clock 72 MHz (HSE)", SystemCoreClock == 72000000U, "system clock");

    LogInitLine("TIM1 PWM (PA8/PA11)", htim1.State == HAL_TIM_STATE_BUSY, "");
    LogInitLine("TIM2 encoder (PA0/PA1)", htim2.State == HAL_TIM_STATE_BUSY, "left wheel");
    LogInitLine("TIM3 encoder (PA6/PA7)", htim3.State == HAL_TIM_STATE_BUSY, "right wheel");
    LogInitLine("TIM4 100 Hz tick", htim4.State == HAL_TIM_STATE_BUSY, "control loop");
    LogInitLine("USART1 vision", huart1.gState != HAL_UART_STATE_RESET, "115200 8N1");
    LogInitLine("USART2 console", huart2.gState != HAL_UART_STATE_RESET, "115200 8N1");
    LogInitLine("line sensor", true, "8 channels CLK/DAT");
    LogInitLine("OLED", true, "128x64 software I2C");
    LogInitLine("params table", Params_Count() > 0, "runtime SET/GET ready");

    /* 编码器静默检查：车不动时 50 ms 内计数应该完全不变 */
    left_before = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    right_before = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    HAL_Delay(50);
    left_after = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    right_after = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    left_delta = (int16_t)(left_after - left_before);
    right_delta = (int16_t)(right_after - right_before);
    log_info("[TEST] encoder idle 50ms  L=%ld R=%ld %s",
             (long)left_delta, (long)right_delta,
             (left_delta == 0 && right_delta == 0) ? "OK" : "WARN (wheels moving?)");

    /* 灰度两帧一致性：环境光抖动大时会直接看出来 */
    gray_a = Track_Read_Serial();
    HAL_Delay(2);
    gray_b = Track_Read_Serial();
    state = Track_Decode(gray_b, &error);
    log_info("[TEST] line sensor  gray=%02X/%02X state=%-5s err=%+.2f %s",
             gray_a, gray_b, TrackStateName(state), (double)error,
             (gray_a == gray_b) ? "OK" : "WARN (unstable)");

    key1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET;
    key2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET;
    log_info("[TEST] keys  KEY1=%s KEY2=%s %s",
             key1 ? "DOWN" : "UP", key2 ? "DOWN" : "UP",
             (!key1 && !key2) ? "OK" : "NOTE (held at boot)");

    self_test_ok = (SystemCoreClock == 72000000U) &&
                   (htim1.State == HAL_TIM_STATE_BUSY) &&
                   (htim2.State == HAL_TIM_STATE_BUSY) &&
                   (htim3.State == HAL_TIM_STATE_BUSY) &&
                   (htim4.State == HAL_TIM_STATE_BUSY) &&
                   (huart2.gState != HAL_UART_STATE_RESET);

    if (self_test_ok) {
        log_info("======== self test PASS ========");
    } else {
        log_error("======== self test FAIL ========");
    }
    log_info("[SM] initial state = %s (bench=%u)", StatusName(car.status),
             (unsigned)(car.status == CAR_BENCH));
    logged_status = car.status;
}

static void ApplicationTask(void *argument)
{
    (void)argument;

    for (;;) {
        uint32_t now;
        uint32_t irq_state;
        bool backing;
        bool start_pending;
        bool log_wanted;

        Console_Poll();
        Vision_Poll();

        log_wanted = (g_params.log_enable != 0U) && !AppLog_VofaMode();
        if (log_wanted != s_log_enabled) {
            s_log_enabled = log_wanted;
            AppLog_SetEnabled(log_wanted);
        }

        now = HAL_GetTick();

        if (g_params.telemetry_ms > 0U &&
            (uint32_t)(now - last_telemetry_ms) >= g_params.telemetry_ms) {
            last_telemetry_ms = now;
            if (AppLog_VofaMode()) {
                PrintVofaFrame();
            } else {
                PrintTelemetry();
            }
        }

        if (g_params.params_ms > 0U &&
            (uint32_t)(now - last_params_ms) >= g_params.params_ms) {
            last_params_ms = now;
            Params_Dump();
        }

        if (telemetry.status != logged_status) {
            log_info("[SM] %s -> %s (lost=%lu, pwm=%u/%u)",
                     StatusName(logged_status), StatusName(telemetry.status),
                     (unsigned long)telemetry.lost_frames,
                     (unsigned)telemetry.left_pwm, (unsigned)telemetry.right_pwm);
            logged_status = telemetry.status;
        }

        /* 停车倒转阶段的开始/结束各打一条，方便确认 back_time 生效 */
        irq_state = __get_PRIMASK();
        __disable_irq();
        backing = car.backing;
        __set_PRIMASK(irq_state);
        if (backing != logged_backing) {
            logged_backing = backing;
            if (backing) {
                log_info("[BACK] reverse %u ms at pwm %u", (unsigned)CAR_BACK_MS,
                         (unsigned)CAR_BACK_PWM);
            } else {
                log_info("[BACK] done, wheels stopped");
            }
        }

        /* 每按一次 KEY2，mode 加 1；到 5 回到 1 */
        if (telemetry.key2_presses != last_key2_presses) {
            last_key2_presses = telemetry.key2_presses;
            ++mode;
            if (mode >= 5U) {
                mode = 1U;
            }
            ApplyMode(mode);
        }

        /*
         * mode 2：延时等待起步的这 1 秒里，看视觉卡片决定目标圈数。
         * 牌面是 2 时目标为 2 圈，其余情况目标为 3 圈。
         */
        irq_state = __get_PRIMASK();
        __disable_irq();
        start_pending = car.start_pending;
        __set_PRIMASK(irq_state);
        if (mode == 2U && start_pending) {
            cross_target = (CardNumber(card_name) == 2) ? 2 : 3;
            cross_target_valid = true;
        }

        if ((telemetry.status == CAR_RUNNING || telemetry.status == CAR_SEARCH) &&
            cross_target_valid && !cross_done &&
            telemetry.cross_count == cross_target) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            CarControl_Stop(&car);
            __set_PRIMASK(primask);
            cross_done = true;
            log_info("[TARGET] cross_count=%d reached target %d, car stopped",
                     (int)telemetry.cross_count, (int)cross_target);
        }

        if (telemetry.status == CAR_RUNNING &&
            prev_status != CAR_RUNNING && prev_status != CAR_SEARCH) {
            if (cross_done) {
                cross_done = false;
                log_info("[TARGET] new run, counter re-armed");
            }
            if (mode == 2U) {
                log_info("[M2] target = %d laps (card 2 -> 2, otherwise -> 3)",
                         (int)cross_target);
            }
        }
        prev_status = telemetry.status;

        vTaskDelay(pdMS_TO_TICKS(CAR_LOOP_DELAY_MS));
    }
}

static void DisplayTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_refresh_ms = HAL_GetTick() - CAR_DISPLAY_MS;

    (void)argument;
    for (;;) {
        uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - last_refresh_ms) >= CAR_DISPLAY_MS) {
            last_refresh_ms = now;
            if (!AppLog_VofaMode()) {
                RefreshDisplay();
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10U));
    }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **task_tcb_buffer,
                                   StackType_t **task_stack_buffer,
                                   uint32_t *stack_size)
{
    *task_tcb_buffer = &idle_task_tcb;
    *task_stack_buffer = idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    Error_Handler();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  bool bench_mode;

  /*
   * USART1（视觉板）的 NVIC 中断在 2.1 的 .ioc 里没打开 —— CubeMX 的 NVIC 表
   * 只有 TIM4 和 USART2，所以 stm32f1xx_it.c 里也没有 USART1_IRQHandler。
   * 视觉数据靠 HAL_UART_Receive_IT 收，中断不开就一个字节都进不来，这里补上；
   * 对应的中断入口写在文件末尾的 USER CODE 4 里。
   */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* 日志先起来，后面每一步初始化都能打出来 */
  AppLog_Init();
  Params_Init();
  Console_Init();

  log_info("====================");
  log_info("STM32F103C8T6 line-following car");
  log_info("build %s %s", __DATE__, __TIME__);
  log_info("sysclk=%lu Hz, params=%d",
           (unsigned long)SystemCoreClock, Params_Count());
  log_info("====================");

  /*
   * 2.1 是 72 MHz，2.0 是 64 MHz：先把 TIM1/TIM4 的预分频按实际主频重算，
   * 否则 PWM 和控制环都快 12.5%，2.0 调好的参数全部失准。
   * 必须赶在 HAL_TIM_PWM_Start / HAL_TIM_Base_Start_IT 之前。
   * （放在 AppLog_Init 之后，这一行日志才打得出来。）
   */
  Timers_Match20Timing();

  bench_mode = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET;
  log_info("[INIT] bench mode %s (KEY2 %s at boot)", bench_mode ? "ON" : "OFF",
           bench_mode ? "held" : "released");

  Track_Init();
  log_info("[INIT] line sensor ready");
  OLED_Init();
  log_info("[INIT] OLED ready");

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    log_error("[INIT] TIM1/TIM2/TIM3 start FAILED");
    Error_Handler();
  }
  log_info("[INIT] PWM + encoders started");
  SetMotorSpeed(0, 0);

  CarControl_Init(&car, (uint16_t)__HAL_TIM_GET_COUNTER(&htim2),
                  (uint16_t)__HAL_TIM_GET_COUNTER(&htim3), bench_mode);
  ApplyMode(mode);              /* 上电按默认 mode（1）设好目标圈数和启动延时 */
  telemetry.status = car.status;
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  /* The timer was updated during initialization. Wait one full period. */
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    log_error("[INIT] TIM4 start FAILED");
    Error_Handler();
  }
  log_info("[INIT] control loop started (100 Hz)");

  SelfTest_Run();
  Params_Dump();
  log_info("type HELP for commands, DUMP lists parameters");

  /* USER CODE END 2 */

  if (xTaskCreateStatic(ApplicationTask, "application",
                        APPLICATION_TASK_STACK_WORDS, NULL,
                        tskIDLE_PRIORITY + 2U,
                        application_task_stack, &application_task_tcb) == NULL ||
      xTaskCreateStatic(DisplayTask, "display",
                        DISPLAY_TASK_STACK_WORDS, NULL,
                        tskIDLE_PRIORITY + 1U,
                        display_task_stack, &display_task_tcb) == NULL)
  {
    log_error("[INIT] FreeRTOS task creation failed");
    Error_Handler();
  }

  log_info("[INIT] FreeRTOS scheduler starting");
  vTaskStartScheduler();
  Error_Handler();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
 * 视觉板串口的中断入口。
 * 2.1 的 .ioc 没有给 USART1 开 NVIC，CubeMX 因此没在 stm32f1xx_it.c 里生成它，
 * 而 HAL_UART_Receive_IT 必须有这个入口才能收数据，所以手写一个。
 * 以后若在 CubeMX 里把 USART1 的 NVIC 打开，请删掉这一段，否则链接时重定义。
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* USART1：视觉板一行一行发，这里只做行缓冲，解析交给主循环的 Vision_Poll() */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        Console_HandleIrq();
        return;
    }

    if (huart->Instance != USART1) {
        return;
    }
    if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        rx_discard = true;
        return;  /* The HAL error callback rearms after handling the error. */
    }

    if (rx_byte == '\n') {
        if (!rx_discard && rx_line_len > 0U && !rx_ready_flag) {
            memcpy(rx_ready, rx_line, rx_line_len);
            rx_ready[rx_line_len] = '\0';
            rx_ready_flag = true;
        }
        rx_line_len = 0;
        rx_discard = false;
    } else if (rx_byte != '\r' && rx_line_len < sizeof(rx_line) - 1U) {
        /* The OLED font handles printable ASCII only. */
        rx_line[rx_line_len++] = (rx_byte >= 32U && rx_byte <= 126U) ? (char)rx_byte : '?';
    }

    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        rx_discard = true;
        (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
    } else if (huart->Instance == USART2) {
        Console_Rearm();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint16_t left_count;
    uint16_t right_count;
    uint8_t gray;
    bool key1_pressed;
    bool key2_pressed;

    if (htim->Instance != TIM4) {
        return;
    }

    left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    key1_pressed = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET;
    key2_pressed = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET;
    gray = Track_Read_Serial();

    CarControl_Step(&car, gray, left_count, right_count, key1_pressed, key2_pressed);

    /* 停车后的倒转阶段：两轮给负 PWM（后退），时间到由控制层清掉 backing */
    if (car.backing) {
        SetMotorSpeed(-(int16_t)CAR_BACK_PWM, -(int16_t)CAR_BACK_PWM);
    } else {
        SetMotorSpeed(car.left_pwm, car.right_pwm);
    }

    telemetry.status = car.status;
    telemetry.gray = car.gray;
    telemetry.left_ticks = car.left_ticks;
    telemetry.right_ticks = car.right_ticks;
    telemetry.error_x10 = (int16_t)(car.line_error * 10.0f);
    telemetry.left_pwm = car.left_pwm;
    telemetry.right_pwm = car.right_pwm;
    telemetry.ramp_speed = car.ramp_speed;
    telemetry.left_target = car.left_target;
    telemetry.right_target = car.right_target;
    telemetry.lost_frames = car.lost_frames;
    telemetry.cross_count = car.cross_count;
    telemetry.key2_presses = car.key2_presses;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  if (htim1.Instance == TIM1)
  {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
