#include "app_params.h"
#include "config.h"
#include "log.h"

#include <stddef.h>
#include <string.h>

CarParams g_params;

typedef enum {
    P_F32,
    P_U16
} ParamType;

typedef struct {
    const char *name;
    ParamType type;
    void *addr;
    float min;
    float max;
    unsigned decimals;
} ParamDesc;

#define F32(n, f, lo, hi, d) { n, P_F32, &g_params.f, lo, hi, d }
#define U16(n, f, lo, hi)    { n, P_U16, &g_params.f, lo, hi, 0 }

static const ParamDesc s_table[] = {
    F32("BASE_SPEED",        base_speed,        0.0f,   40.0f, 2),
    F32("MIN_BASE_SPEED",    min_base_speed,    0.0f,   40.0f, 2),
    F32("SPEED_RAMP",        speed_ramp,        0.05f,   5.0f, 3),
    F32("CURVE_SLOWDOWN",    curve_slowdown,    0.0f,    5.0f, 3),
    F32("MAX_WHEEL_TARGET",  max_wheel_target,  0.0f,  100.0f, 2),
    F32("MAX_TURN",          max_turn,          0.0f,   30.0f, 2),

    F32("LINE_KP",           line_kp,           0.0f,   20.0f, 3),
    F32("LINE_KD",           line_kd,           0.0f,   20.0f, 3),
    F32("LINE_FILTER_ALPHA", line_filter_alpha, 0.05f,   1.0f, 3),
    F32("LINE_D_ALPHA",      line_d_alpha,      0.05f,   1.0f, 3),
    F32("LINE_DEADBAND",     line_deadband,     0.0f,    4.0f, 3),

    F32("WHEEL_KP",          wheel_kp,          0.0f,   10.0f, 3),
    F32("WHEEL_KI",          wheel_ki,          0.0f,    2.0f, 4),
    F32("WHEEL_I_LIMIT",     wheel_i_limit,     0.0f,   40.0f, 2),
    F32("WHEEL_FILTER_ALPHA",wheel_filter_alpha,0.05f,   1.0f, 3),

    F32("LEFT_START_PWM",    left_start_pwm,    0.0f,   60.0f, 2),
    F32("RIGHT_START_PWM",   right_start_pwm,   0.0f,   60.0f, 2),
    F32("LEFT_PWM_PER_COUNT",left_pwm_per_count,0.0f,    3.0f, 4),
    F32("RIGHT_PWM_PER_COUNT",right_pwm_per_count,0.0f,  3.0f, 4),
    F32("PWM_MAX",           pwm_max,           1.0f,   99.0f, 1),

    F32("SEARCH_SPEED",      search_speed,      0.0f,   20.0f, 2),
    F32("SEARCH_TURN_RATIO", search_turn_ratio, 0.0f,    1.0f, 3),
    U16("SEARCH_MID",        search_mid_frames, 0.0f,  500.0f),
    U16("SEARCH_SIDE",       search_side_frames,0.0f,  500.0f),
    U16("SEARCH_TURN",       search_turn_frames,0.0f,  500.0f),
    U16("SEARCH_SWEEP",      search_sweep_frames,1.0f, 500.0f),

    U16("STALL_FRAMES",      stall_frames,      0.0f,  250.0f),
    U16("STALL_PWM",         stall_pwm,         0.0f,   99.0f),
    U16("REVERSE_FRAMES",    reverse_frames,    0.0f,  200.0f),
    U16("VALID_FRAMES",      valid_frames,      1.0f,   50.0f),
    U16("INVALID_FRAMES",    invalid_frames,    1.0f,   50.0f),
    U16("STOP_FRAMES",       stop_frames,       1.0f,   50.0f),

    U16("TELEMETRY_MS",      telemetry_ms,      0.0f, 5000.0f),
    U16("PARAMS_MS",         params_ms,         0.0f, 60000.0f),
    U16("LOG_ENABLE",        log_enable,        0.0f,    1.0f),

    U16("LINE_AVG",          line_avg,          0.0f,    1.0f)
};

#undef F32
#undef U16

#define PARAM_COUNT ((int)(sizeof s_table / sizeof s_table[0]))

static float ReadValue(const ParamDesc *d);
static void WriteValue(const ParamDesc *d, float value);

void Params_Init(void)
{
    g_params.base_speed          = CAR_BASE_SPEED;
    g_params.min_base_speed      = CAR_MIN_BASE_SPEED;
    g_params.speed_ramp          = CAR_SPEED_RAMP;
    g_params.curve_slowdown      = CAR_CURVE_SLOWDOWN;
    g_params.max_wheel_target    = CAR_MAX_WHEEL_TARGET;
    g_params.max_turn            = CAR_MAX_TURN;

    g_params.line_kp             = CAR_LINE_KP;
    g_params.line_kd             = CAR_LINE_KD;
    g_params.line_filter_alpha   = CAR_LINE_FILTER_ALPHA;
    g_params.line_d_alpha        = CAR_LINE_D_ALPHA;
    g_params.line_deadband       = CAR_LINE_DEADBAND;

    g_params.wheel_kp            = CAR_WHEEL_KP;
    g_params.wheel_ki            = CAR_WHEEL_KI;
    g_params.wheel_i_limit       = CAR_WHEEL_I_LIMIT;
    g_params.wheel_filter_alpha  = CAR_WHEEL_FILTER_ALPHA;

    g_params.left_start_pwm      = CAR_LEFT_START_PWM;
    g_params.right_start_pwm     = CAR_RIGHT_START_PWM;
    g_params.left_pwm_per_count  = CAR_LEFT_PWM_PER_COUNT;
    g_params.right_pwm_per_count = CAR_RIGHT_PWM_PER_COUNT;
    g_params.pwm_max             = (float)CAR_PWM_MAX;

    g_params.search_speed        = CAR_LINE_SEARCH_SPEED;
    g_params.search_turn_ratio   = CAR_LINE_SEARCH_TURN_RATIO;
    g_params.search_mid_frames   = CAR_LINE_SEARCH_MID_FRAMES;
    g_params.search_side_frames  = CAR_LINE_SEARCH_SIDE_FRAMES;
    g_params.search_turn_frames  = CAR_LINE_SEARCH_TURN_FRAMES;
    g_params.search_sweep_frames = CAR_LINE_SEARCH_SWEEP_FRAMES;

    g_params.stall_frames        = CAR_STALL_FRAMES;
    g_params.stall_pwm           = CAR_STALL_PWM;
    g_params.reverse_frames      = CAR_REVERSE_FRAMES;
    g_params.valid_frames        = CAR_VALID_FRAMES;
    g_params.invalid_frames      = CAR_INVALID_FRAMES;
    g_params.stop_frames         = CAR_STOP_FRAMES;

    g_params.telemetry_ms        = CAR_TELEMETRY_MS_DEFAULT;
    g_params.params_ms           = CAR_PARAMS_MS_DEFAULT;
    g_params.log_enable          = CAR_LOG_ENABLE_DEFAULT;

    g_params.line_avg            = CAR_LINE_AVG_DEFAULT;

    /* 编译期宏可能被人手改到非法区间（例如 LINE_FILTER_ALPHA > 1），
       这里统一夹紧并告警，避免带着发散参数上车。 */
    {
        int i;
        for (i = 0; i < PARAM_COUNT; ++i) {
            const ParamDesc *d = &s_table[i];
            float raw = ReadValue(d);
            float fixed = raw;
            if (fixed < d->min) {
                fixed = d->min;
            }
            if (fixed > d->max) {
                fixed = d->max;
            }
            if (fixed != raw) {
                WriteValue(d, fixed);
                log_warn("default %s=%.3f out of [%.3f..%.3f], clamped to %.3f",
                         d->name, (double)raw, (double)d->min, (double)d->max,
                         (double)fixed);
            }
        }
    }
}

static char Upper(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static bool NameEqual(const char *a, const char *b)
{
    while (*a && *b) {
        if (Upper(*a) != Upper(*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

static const ParamDesc *Find(const char *name)
{
    int i;

    if (!name || !*name) {
        return NULL;
    }
    for (i = 0; i < PARAM_COUNT; ++i) {
        if (NameEqual(s_table[i].name, name)) {
            return &s_table[i];
        }
    }
    return NULL;
}

static float ReadValue(const ParamDesc *d)
{
    if (d->type == P_F32) {
        return *(const float *)d->addr;
    }
    return (float)*(const uint16_t *)d->addr;
}

static void WriteValue(const ParamDesc *d, float value)
{
    if (value < d->min) {
        value = d->min;
    }
    if (value > d->max) {
        value = d->max;
    }
    if (d->type == P_F32) {
        *(float *)d->addr = value;
    } else {
        *(uint16_t *)d->addr = (uint16_t)(value + 0.5f);
    }
}

bool Params_Set(const char *name, float value)
{
    const ParamDesc *d = Find(name);

    if (!d) {
        return false;
    }
    if (value != value) {          /* NaN 保护 */
        return false;
    }
    WriteValue(d, value);
    return true;
}

bool Params_Get(const char *name, float *value)
{
    const ParamDesc *d = Find(name);

    if (!d || !value) {
        return false;
    }
    *value = ReadValue(d);
    return true;
}

int Params_Count(void)
{
    return PARAM_COUNT;
}

const char *Params_NameAt(int index)
{
    return (index >= 0 && index < PARAM_COUNT) ? s_table[index].name : "";
}

float Params_ValueAt(int index)
{
    return (index >= 0 && index < PARAM_COUNT) ? ReadValue(&s_table[index]) : 0.0f;
}

float Params_MinAt(int index)
{
    return (index >= 0 && index < PARAM_COUNT) ? s_table[index].min : 0.0f;
}

float Params_MaxAt(int index)
{
    return (index >= 0 && index < PARAM_COUNT) ? s_table[index].max : 0.0f;
}

int Params_DecimalsAt(int index)
{
    return (index >= 0 && index < PARAM_COUNT) ? (int)s_table[index].decimals : 3;
}

void Params_Dump(void)
{
    int i;

    log_info("---- parameters (%d) ----", PARAM_COUNT);
    for (i = 0; i < PARAM_COUNT; ++i) {
        log_info("%-20s = %8.*f   [%.*f .. %.*f]",
                 Params_NameAt(i),
                 Params_DecimalsAt(i), Params_ValueAt(i),
                 Params_DecimalsAt(i), Params_MinAt(i),
                 Params_DecimalsAt(i), Params_MaxAt(i));
    }
}

void Params_Help(void)
{
    log_info("SET <name> <value>   e.g. SET LINE_KP 2.2");
    log_info("SET /C <name> <val>  same, /C is ignored");
    log_info("GET <name>           print one parameter");
    log_info("DUMP                 print every parameter");
    log_info("RESET                restore compile-time defaults");
    log_info("TELEM <ms>           telemetry period, 0 = off");
    log_info("LOG <0|1>            enable/disable all logging");
    log_info("HELP                 this list");
}
