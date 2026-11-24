#include "track.h"
#include "config.h"

/*
 * 微秒延时：用 DWT 周期计数器，跟主频无关，也不会被中断打断后算错。
 * （原来的 for(i<72) __NOP() 每次都 4~8 个周期，实际“1 µs”有 5~7 µs。）
 */
static void Track_DelayUs(uint32_t us)
{
    static uint8_t  inited = 0;
    uint32_t start, ticks;

    if (!inited)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
        inited = 1;
    }

    start = DWT->CYCCNT;
    ticks = us * (SystemCoreClock / 1000000U);   /* 每 µs 的周期数随主频自动变：64MHz→64，72MHz→72 */
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

/*
 * 初始化：引脚 + 上电对齐
 *   1. CLK 先写 0 再切输出，避免切换瞬间产生一个假上升沿；
 *   2. DAT 浮空输入（辅助板是推挽驱动，不需要内部上下拉）；
 *   3. 等 100 ms，让传感器板和辅助板（MSPM0）上电稳定；
 *   4. 丢掉第一帧，等辅助板的位计数对齐到 bit0。
 */
void Track_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);

    gpio.Pin   = GRAY_CLK_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GRAY_CLK_PORT, &gpio);

    gpio.Pin  = GRAY_DAT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GRAY_DAT_PORT, &gpio);

    HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);

    HAL_Delay(100);
    (void)Track_Read_Serial();
}

/**
 * @brief  串行读取 8 路灰度
 * @return bit0=CH1 … bit7=CH8，1=白、0=黑（辅助板原始语义）
 *
 * 辅助板时序（和你手上那份 MSPM0G3507 串行例程一致）：
 *   上升沿 → 辅助板把第 i 位放到 DAT
 *   等 5 µs
 *   下降沿 → 数据稳定，采样
 * 一帧固定 8 个时钟，LSB 在前；两帧之间 CLK 保持低，间隔 ≥1 ms。
 */
uint8_t Track_Read_Serial(void)
{
    uint8_t data = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_SET);    /* 上升沿：辅助板放第 i 位 */
        Track_DelayUs(5);

        HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);  /* 下降沿后数据稳定 */
        Track_DelayUs(1);

        if (HAL_GPIO_ReadPin(GRAY_DAT_PORT, GRAY_DAT_PIN) == GPIO_PIN_SET)
        {
            data |= (uint8_t)(1u << i);   /* LSB 在前 */
        }
    }

    HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);      /* 帧结束保持低 */

    return data;
}

TrackState Track_Decode(uint8_t gray, float *error)
{
    int first = 8;
    int last = -1;
    int active = 0;
    int position_sum = 0;
    int position;

    *error = 0.0f;
    for (position = 0; position < 8; ++position) {
        int bit = CAR_BIT0_IS_LEFT ? position : 7 - position;
        if (((gray >> bit) & 1U) == CAR_BLACK_BIT) {
            if (active == 0) {
                first = position;
            }
            last = position;
            position_sum += position;
            ++active;
        }
    }
    if (active == 0) {
        return TRACK_LOST;
    }
    if ((unsigned)active > CAR_MAX_LINE_SENSORS || last - first + 1 != active) {
        return TRACK_AMBIGUOUS;
    }
    *error = (float)position_sum / (float)active - 3.5f;
    return TRACK_LINE;
}

float Get_Line_Error(uint8_t gray)
{
    float error;
    (void)Track_Decode(gray, &error);
    return error;
}

/* 统计这一帧有几路探头压黑（bit=0 为黑时，统计 0 的位数） */
uint8_t Track_BlackCount(uint8_t gray)
{
    uint8_t count = 0;
    unsigned i;

    for (i = 0; i < 8U; ++i) {
        if (((gray >> i) & 1U) == CAR_BLACK_BIT) {
            ++count;
        }
    }
    return count;
}

/*
 * 误差滑动平均滤波（取最近 3 次的有效误差求平均）
 */
float Error_Moving_Average(float new_error)
{
    static float error_buffer[CAR_LINE_AVG_WINDOW];
    static uint8_t index = 0; 
    float sum = 0.0f;
    uint8_t i;

    error_buffer[index] = new_error;
    
    ++index;
    if (index >= CAR_LINE_AVG_WINDOW) {
        index = 0;
    }

    for (i = 0; i < CAR_LINE_AVG_WINDOW; ++i) {
        sum += error_buffer[i];
    }

    return sum / (float)CAR_LINE_AVG_WINDOW;
}
