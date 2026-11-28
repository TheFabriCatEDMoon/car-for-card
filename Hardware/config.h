#ifndef CAR_CONFIG_ALL_H
#define CAR_CONFIG_ALL_H

/*
 * ============================================================================
 *  config.h —— 调参面板
 * ============================================================================
 *
 *  初次调车建议先只改 A1「速度与转向」，其他参数保持默认：
 *    - 车太慢：小幅增加 BASE_SPEED；若已经到 MAX_WHEEL_TARGET 上限，
 *      再小幅增加 MAX_WHEEL_TARGET。
 *      BASE_SPEED 高于 MAX_WHEEL_TARGET 时，单轮目标会被上限截住。
 *    - 弯道太快：降低 MIN_BASE_SPEED，或增加 CURVE_SLOWDOWN。
 *    - 起步太冲：降低 SPEED_RAMP；只是起步太慢时再增加它。
 *    - 左右摆动：先降低 BASE_SPEED，再小幅调整 LINE_KP / LINE_KD。
 *    - 保持 MIN_BASE_SPEED 不大于 BASE_SPEED；提高 PWM_MAX 前确认硬件额定值。
 *
 *  在线试参数（USART2，115200，命令后按回车）：
 *    SET BASE_SPEED 8       临时设置，数值会限制在合法范围内
 *    GET BASE_SPEED         查询当前值
 *    DUMP                   查看全部参数及范围
 *    RESET                  恢复下方 CAR_* 默认值
 *
 *  A 区是串口 SET 支持的运行参数；这里的 CAR_* 是上电默认值。
 *  在线 SET 只在本次运行有效，确认效果后把值写回 A 区并重新编译烧录。
 *
 *  B 区是硬件/功能结构常量：方向标定、保护开关、缓冲区、波特率等；
 *  修改后需要重新编译烧录，不支持串口 SET。
 *
 *  单位：速度是每个 10 ms 控制周期的编码器计数；PWM 是占空比百分比。
 *  参数范围以 DUMP 输出为准；不要一次大幅改多个参数，便于比较效果。
 */

/* ==========================================================================
 * A. 运行参数默认值（36 项）—— 与串口 DUMP 参数表对应
 *
 *    调参顺序建议：速度/转向 → 循迹 PD → 轮速 PI/前馈 → 找线/保护。
 *    先调一项、低速试跑、确认稳定后再调下一项。
 * ========================================================================== */

/* ---- A1. 常用：速度与转向 ------------------------------------------------ */

#define CAR_BASE_SPEED             25.0f   /* BASE_SPEED：直道目标速度；增大可提速。范围 0~40 */
#define CAR_MIN_BASE_SPEED         4.0f    /* MIN_BASE_SPEED：弯道最低速度；太快就降低。范围 0~40，需 <= BASE_SPEED */
#define CAR_SPEED_RAMP             1.5f    /* SPEED_RAMP：每 10 ms 加速步长；越大起步越快。范围 0.05~5 */
#define CAR_CURVE_SLOWDOWN         1.5f    /* CURVE_SLOWDOWN：偏差越大降速越多；范围 0~5 */
#define CAR_MAX_WHEEL_TARGET      40.0f    /* MAX_WHEEL_TARGET：单轮速度上限；BASE_SPEED 被卡住时检查此项。范围 0~100 */
#define CAR_MAX_TURN               20.0f    /* MAX_TURN：转向差速上限；太迟钝可加，摆动大可减。范围 0~30 */

/* ---- A2. 循迹：PD 外环 --------------------------------------------------- */

#define CAR_LINE_KP                1.0f    /* LINE_KP：比例纠偏力度；太迟钝可加，左右摆动可减。范围 0~20 */
#define CAR_LINE_KD               20.0f    /* LINE_KD：抑制转向过冲；太敏感/抖动可减。范围 0~20 */
#define CAR_LINE_FILTER_ALPHA      0.6f    /* LINE_FILTER_ALPHA：偏差滤波；越大响应越快、越小越平滑。范围 0.05~1 */
#define CAR_LINE_D_ALPHA           0.35f   /* LINE_D_ALPHA：微分滤波；越大响应越快，也更容易受噪声影响。范围 0.05~1 */
#define CAR_LINE_DEADBAND          0.15f   /* LINE_DEADBAND：小于此偏差时减弱比例纠偏；范围 0~4 */
#define CAR_LINE_AVG_DEFAULT       1U      /* LINE_AVG：0 关闭、1 开启额外滑动平均。范围 0~1 */

/* ---- A3. 轮速：PI 内环与电机前馈 ---------------------------------------- */

#define CAR_WHEEL_KP              1.05f   /* WHEEL_KP：轮速比例；范围 0~10 */
#define CAR_WHEEL_KI              0.70f   /* WHEEL_KI：轮速积分；持续有速度误差时才考虑调整。范围 0~2 */
#define CAR_WHEEL_I_LIMIT        10.0f    /* WHEEL_I_LIMIT：积分限幅；范围 0~40 */
#define CAR_WHEEL_FILTER_ALPHA    0.45f   /* WHEEL_FILTER_ALPHA：测速滤波；大=响应快，小=更平滑。范围 0.05~1 */
#define CAR_LEFT_START_PWM       13.0f    /* LEFT_START_PWM：左轮起步补偿；单侧起步无力时小幅增加。范围 0~60 */
#define CAR_RIGHT_START_PWM      13.0f    /* RIGHT_START_PWM：右轮起步补偿；单侧起步无力时小幅增加。范围 0~60 */
#define CAR_LEFT_PWM_PER_COUNT    0.8f    /* LEFT_PWM_PER_COUNT：左轮速度前馈；范围 0~3 */
#define CAR_RIGHT_PWM_PER_COUNT   0.8f    /* RIGHT_PWM_PER_COUNT：右轮速度前馈；范围 0~3 */
#define CAR_PWM_MAX                99     /* PWM_MAX：电机输出上限；提高前确认电机/驱动板额定值。范围 1~99 */

/* ---- A4. 丢线后找线 ------------------------------------------------------ */

#define CAR_LINE_SEARCH_SPEED       6.0f  /* SEARCH_SPEED：找线时的速度；范围 0~20 */
#define CAR_LINE_SEARCH_TURN_RATIO  0.8f  /* SEARCH_TURN_RATIO：找线转向力度比例；范围 0~1 */
#define CAR_LINE_SEARCH_MID_FRAMES 30U    /* SEARCH_MID：中间丢线后先直行的帧数；每帧 10 ms。范围 0~500 */
#define CAR_LINE_SEARCH_SIDE_FRAMES 8U    /* SEARCH_SIDE：边缘丢线后先直行的帧数；每帧 10 ms。范围 0~500 */
#define CAR_LINE_SEARCH_TURN_FRAMES 40U   /* SEARCH_TURN：向最后见线方向搜索的帧数；每帧 10 ms。范围 0~500 */
#define CAR_LINE_SEARCH_SWEEP_FRAMES 25U  /* SEARCH_SWEEP：左右交替搜索的周期帧数；每帧 10 ms。范围 1~500 */

/* ---- A5. 保护与起步判定 -------------------------------------------------- */

#define CAR_STALL_FRAMES           25U    /* STALL_FRAMES：堵转判定帧数；每帧 10 ms。范围 0~250 */
#define CAR_STALL_PWM              24U    /* STALL_PWM：高于此 PWM 且无编码器计数时累计堵转。范围 0~99 */
#define CAR_REVERSE_FRAMES          4U    /* REVERSE_FRAMES：编码器反向判定帧数；每帧 10 ms。范围 0~200 */
#define CAR_VALID_FRAMES            3U    /* VALID_FRAMES：起步前要求连续看到有效线的帧数。范围 1~50 */
#define CAR_INVALID_FRAMES          3U    /* INVALID_FRAMES：连续无效线判定帧数。范围 1~50 */
#define CAR_STOP_FRAMES             3U    /* STOP_FRAMES：KEY2 去抖帧数；范围 1~50 */

/* ---- A6. 串口输出与日志 -------------------------------------------------- */

#define CAR_TELEMETRY_MS_DEFAULT  100U    /* TELEMETRY_MS：运行遥测间隔 ms；0=关闭。范围 0~5000 */
#define CAR_PARAMS_MS_DEFAULT    1000U    /* PARAMS_MS：重复打印参数间隔 ms；0=关闭，减少串口占用。范围 0~60000 */
#define CAR_LOG_ENABLE_DEFAULT      1U    /* LOG_ENABLE：默认文本日志开关；0=关，1=开。范围 0~1 */

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
