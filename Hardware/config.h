#ifndef CAR_CONFIG_ALL_H
#define CAR_CONFIG_ALL_H

/*
 * ============================================================================
 *  config.h —— 全部可调量的唯一入口
 * ============================================================================
 *
 *  第一部分（A）是**参数**：就是串口 `SET 名字 值` 能改的那 36 个量，
 *  这里定义的是它们的**上电默认值**。运行中改只影响本次，复位后回到这里。
 *
 *  第二部分（B）是**结构常量**：方向标定、保护开关、缓冲区、波特率、
 *  刷新周期等，改完必须重新编译。
 *
 *  工作流：
 *    改 A 区 → 只在调试时试算新值时，可以先 `SET 名字 值` 在线试，
 *             满意了再写回 A 区并重新编译（否则断电就丢）
 *    改 B 区 → 必须 Build + flash.ps1
 *
 *  谁用到参数，就在谁的 .c 文件里显式 include 本文件，不要依赖传递包含。
 */

/* ==========================================================================
 * A. 参数默认值（36 项）—— 与运行参数表一一对应，顺序同 `DUMP` 输出
 *
 *    格式： #define CAR_<名字>  <默认值>   行尾注释写 SET 名 | 范围 | 说明
 *    单位：速度为"每 10 ms 控制周期的编码器计数"，PWM 为 0~99 的占空比百分比
 * ========================================================================== */

/* ---- A1. 速度与转向 ------------------------------------------------------ */

#define CAR_BASE_SPEED             7.0f   /* BASE_SPEED        | 0~40    | 直道基础速度 */
#define CAR_MIN_BASE_SPEED         4.0f   /* MIN_BASE_SPEED    | 0~40    | 弯道降速下限 */
#define CAR_SPEED_RAMP             0.5f   /* SPEED_RAMP        | 0.05~5  | 加减速斜坡步长 */
#define CAR_CURVE_SLOWDOWN         1.5f   /* CURVE_SLOWDOWN    | 0~5     | 偏差越大降速越多 */
#define CAR_MAX_WHEEL_TARGET      20.0f   /* MAX_WHEEL_TARGET  | 0~100   | 单轮目标速度上限 */
#define CAR_MAX_TURN               6.0f   /* MAX_TURN          | 0~30    | 差速转向最大增量 */

/* ---- A2. 循迹 PD（外环）------------------------------------------------- */

#define CAR_LINE_KP                1.1f   /* LINE_KP           | 0~20    | 比例：跟线紧度 */
#define CAR_LINE_KD                10.0f   /* LINE_KD           | 0~20    | 微分：抑制过冲 */
#define CAR_LINE_FILTER_ALPHA      0.6f   /* LINE_FILTER_ALPHA | 0.05~1  | 偏差 EMA 滤波，τ=-10/ln(α) ms：0.9→95ms,0.6→20ms */
#define CAR_LINE_D_ALPHA          0.35f   /* LINE_D_ALPHA      | 0.05~1  | 微分项滤波 */
#define CAR_LINE_DEADBAND         0.15f   /* LINE_DEADBAND     | 0~4     | 死区（台阶比它大时无效） */
#define CAR_LINE_AVG_DEFAULT         0U   /* LINE_AVG          | 0~1     | 1=启用滑动平均滤波 */

/* ---- A3. 轮速环 PI（内环）----------------------------------------------- */

#define CAR_WHEEL_KP              1.05f   /* WHEEL_KP          | 0~10    | 速度环比例 */
#define CAR_WHEEL_KI              0.70f   /* WHEEL_KI          | 0~2     | 速度环积分 */
#define CAR_WHEEL_I_LIMIT         10.0f   /* WHEEL_I_LIMIT     | 0~40    | 积分限幅 */
#define CAR_WHEEL_FILTER_ALPHA    0.45f   /* WHEEL_FILTER_ALPHA| 0.05~1  | 测速滤波 */
#define CAR_LEFT_START_PWM        13.0f   /* LEFT_START_PWM    | 0~60    | 左轮起步死区补偿 */
#define CAR_RIGHT_START_PWM       13.0f   /* RIGHT_START_PWM   | 0~60    | 右轮起步死区补偿 */
#define CAR_LEFT_PWM_PER_COUNT     0.8f   /* LEFT_PWM_PER_COUNT | 0~3    | 左轮 PWM/计数 前馈 */
#define CAR_RIGHT_PWM_PER_COUNT    0.8f   /* RIGHT_PWM_PER_COUNT| 0~3    | 右轮 PWM/计数 前馈 */
#define CAR_PWM_MAX                 38    /* PWM_MAX           | 1~99    | PWM 上限 */

/* ---- A4. 找线（丢线后自救）--------------------------------------------- */

#define CAR_LINE_SEARCH_SPEED      6.0f   /* SEARCH_SPEED      | 0~20    | 找线速度 */
#define CAR_LINE_SEARCH_TURN_RATIO 0.8f   /* SEARCH_TURN_RATIO | 0~1     | 找线打方向力度 */
#define CAR_LINE_SEARCH_MID_FRAMES   30U  /* SEARCH_MID        | 0~500   | 中间丢线先直行帧数 */
#define CAR_LINE_SEARCH_SIDE_FRAMES   8U  /* SEARCH_SIDE       | 0~500   | 边上丢线先等待帧数 */
#define CAR_LINE_SEARCH_TURN_FRAMES  40U  /* SEARCH_TURN       | 0~500   | 朝一个方向找多久 */
#define CAR_LINE_SEARCH_SWEEP_FRAMES 25U  /* SEARCH_SWEEP      | 1~500   | 左右交替周期 */

/* ---- A5. 保护阈值 ------------------------------------------------------- */

#define CAR_STALL_FRAMES           25U    /* STALL_FRAMES      | 0~250   | 堵转判定帧数 */
#define CAR_STALL_PWM              24U    /* STALL_PWM         | 0~99    | 堵转判定 PWM 阈值 */
#define CAR_REVERSE_FRAMES          4U    /* REVERSE_FRAMES    | 0~200   | 反转判定帧数 */
#define CAR_VALID_FRAMES            3U    /* VALID_FRAMES      | 1~50    | 起步要求的连续有效帧 */
#define CAR_INVALID_FRAMES          3U    /* INVALID_FRAMES    | 1~50    | 判坏帧数 */
#define CAR_STOP_FRAMES             3U    /* STOP_FRAMES       | 1~50    | KEY2 去抖帧数 */

/* ---- A6. 通信与日志 ----------------------------------------------------- */

#define CAR_TELEMETRY_MS_DEFAULT   100U   /* TELEMETRY_MS      | 0~5000  | 遥测周期 ms，0=关 */
#define CAR_PARAMS_MS_DEFAULT     1000U   /* PARAMS_MS         | 0~60000 | 参数打印周期 ms（很吵） */
#define CAR_LOG_ENABLE_DEFAULT       1U   /* LOG_ENABLE        | 0~1     | 文本日志开关 */

/* ==========================================================================
 * B. 结构常量（改完必须重新编译）
 * ========================================================================== */

/* ---- B1. 物理标定：车往前推时轮子向前、计数增加；反了就改 -1 ---------- */

#define CAR_LEFT_MOTOR_SIGN        1
#define CAR_RIGHT_MOTOR_SIGN       1
#define CAR_LEFT_ENCODER_SIGN      1
#define CAR_RIGHT_ENCODER_SIGN     1

/* ---- B2. 灰度探头极性 --------------------------------------------------- */

#define CAR_BLACK_BIT              0      /* 0 = 该位为 0 表示压黑线 */
#define CAR_BIT0_IS_LEFT           1      /* 1 = bit0 是最左边那路 */
#define CAR_MAX_LINE_SENSORS      6U      /* 一帧最多几路同时压黑，超过判 AMBIGUOUS */
#define CAR_CROSS_MIN_CHANNELS    4U      /* 几路同时压黑算"一次压线"（OLED Cross 行） */

/* ---- B3. 保护策略开关 --------------------------------------------------- */

#define CAR_LINE_LOST_STOP         0      /* 1=丢线锁停；0=不停车，自动找线（当前） */
#define CAR_ENCODER_PROTECT        0      /* 1=堵转锁停；0=只跑不停（当前，注意电机发热） */

/* ---- B3b. 停车后的倒转（back_time）-------------------------------------
 * 正常停车（按 KEY1 / 到达 cross_target）时，两轮先倒转 CAR_BACK_MS 毫秒，
 * 让车从线上退一点、避免停在压线处影响下一轮计数，然后才真正停住。
 * 故障锁停（编码器反转/堵转）不会倒转。0 = 关闭倒转。
 * ---------------------------------------------------------------------- */

#define CAR_BACK_MS              250U     /* 倒转时长 ms（用户要的 back_time = 250） */
#define CAR_BACK_PWM              30U     /* 倒转时的 PWM（0~99） */

/* ---- B4. 按键与点动测试 ------------------------------------------------- */

#define CAR_KEY_FRAMES             3U     /* KEY1 去抖帧数 */
#define CAR_BENCH_PWM             26U     /* 上电按住 KEY2 进入点动的 PWM */
#define CAR_BENCH_FRAMES          50U     /* 每次点动最长帧数 */

/* ---- B5. 显示与主循环 --------------------------------------------------- */

#define CAR_DISPLAY_MS           150U     /* OLED 刷新周期 ms（VOFA 模式自动停刷） */
#define CAR_LOOP_DELAY_MS          1U     /* 主循环每圈延时 ms */
#define CAR_OLED_MODE_X          110U     /* "M:n" 右上角 x 坐标 */
#define CAR_OLED_I2C_HZ       400000U     /* 软件 I2C 频率（400 kHz 是 SSD1306 上限） */
#define CAR_OLED_INIT_DELAY_MS   200U     /* OLED 上电初始化等待 ms */

/* ---- B6. 串口 ----------------------------------------------------------- */

#define CAR_VISION_BAUD       115200U     /* USART1 视觉板波特率 */
#define CAR_CONSOLE_BAUD      115200U     /* USART2 调试串口波特率（改这里即可） */

/* ---- B7. 缓冲区大小（改小省 RAM，总量只有 20 KB）------------------------ */

#define CAR_VISION_LINE_MAX       48U     /* 视觉板一行最大长度 */
#define CAR_CONSOLE_LINE_MAX      72U     /* 串口命令一行最大长度 */
#define CAR_VOFA_LINE_MAX        112U     /* VOFA CSV 帧缓冲 */

/* ---- B8. 滤波窗口 ------------------------------------------------------- */

#define CAR_LINE_AVG_WINDOW        3U     /* LINE_AVG=1 时的滑动平均窗口帧数 */

/* ---- B9. mode 3 视觉卡片识别 ------------------------------------------- */

#define CAR_CARD_SLOTS             6U     /* card1..card6，最多记 6 张 */
#define CAR_CARD_TEXT_MAX         10U     /* 卡名缓冲，和视觉标签 card_name 一样长 */
#define CAR_CARD_WAIT_MS        2000U     /* 记完一张卡后 detect 关多久再打开 */
#define CAR_CARD_STOP_MS        2000U     /* 识别到 S/C 后停车多久再自动起步 */
#define CAR_CARD_BEEP_MS        1000U     /* 停车期间蜂鸣器响多久 */

/* ---- B10. mode 4 视觉卡片匹配 ------------------------------------------ */

#define CAR_MATCH_SLOTS           12U     /* card1..card12：前 6 张算目标，后 6 张比对 */
#define CAR_MATCH_LAP_TARGET       2U     /* mode 4 的 cross_target：跑 2 圈后停车 */
#define CAR_MATCH_STOP_MS       2000U     /* 命中目标后停车多久再自动起步 */
#define CAR_MATCH_PICK             3U     /* 每组取 3 张卡求和 */
#define CAR_MATCH_COMBOS          20U     /* C(6,3) = 20 组 */

/* ==========================================================================
 * C. 编译期检查：配错了直接报错，不用等上车才发现
 * ========================================================================== */

#if (CAR_LEFT_MOTOR_SIGN != 1 && CAR_LEFT_MOTOR_SIGN != -1) || \
    (CAR_RIGHT_MOTOR_SIGN != 1 && CAR_RIGHT_MOTOR_SIGN != -1) || \
    (CAR_LEFT_ENCODER_SIGN != 1 && CAR_LEFT_ENCODER_SIGN != -1) || \
    (CAR_RIGHT_ENCODER_SIGN != 1 && CAR_RIGHT_ENCODER_SIGN != -1)
#error "Motor/encoder signs must be +1 or -1"
#endif

#if (CAR_BLACK_BIT != 0 && CAR_BLACK_BIT != 1) || \
    (CAR_BIT0_IS_LEFT != 0 && CAR_BIT0_IS_LEFT != 1) || \
    (CAR_MAX_LINE_SENSORS < 1 || CAR_MAX_LINE_SENSORS > 7) || \
    (CAR_CROSS_MIN_CHANNELS < 1 || CAR_CROSS_MIN_CHANNELS > 8)
#error "Invalid line sensor configuration"
#endif

#if CAR_PWM_MAX < CAR_STALL_PWM || CAR_PWM_MAX > 99 || CAR_BENCH_PWM > 45
#error "Invalid PWM limit (PWM_MAX 0..99, must be >= STALL_PWM, BENCH_PWM <= 45)"
#endif

#if (CAR_LINE_LOST_STOP != 0 && CAR_LINE_LOST_STOP != 1) || \
    (CAR_ENCODER_PROTECT != 0 && CAR_ENCODER_PROTECT != 1)
#error "Protection switches must be 0 or 1"
#endif

#if (CAR_LINE_AVG_WINDOW < 1) || (CAR_LINE_AVG_WINDOW > 8)
#error "CAR_LINE_AVG_WINDOW must be 1..8"
#endif

#if (CAR_VISION_LINE_MAX < 8) || (CAR_CONSOLE_LINE_MAX < 16) || (CAR_VOFA_LINE_MAX < 64)
#error "Buffer sizes too small"
#endif

#if (CAR_CARD_SLOTS < 6) || (CAR_CARD_TEXT_MAX < 2) || (CAR_CARD_BEEP_MS > CAR_CARD_STOP_MS)
#error "Invalid card detection configuration (need 6 slots, BEEP <= STOP)"
#endif

#if (CAR_MATCH_SLOTS < 12) || (CAR_MATCH_COMBOS != 20) || \
    (CAR_MATCH_PICK != 3) || (CAR_CARD_BEEP_MS > CAR_MATCH_STOP_MS)
#error "Invalid mode 4 card matching configuration"
#endif

#endif /* CAR_CONFIG_ALL_H */
