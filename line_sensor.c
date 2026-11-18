/*
 * line_sensor.c - 八路灰度读数与黑线位置计算
 *
 * 位置算法：把 8 路看成等间距（CAR_LINE_PITCH_MM）的采样点，
 * 对压线的通道做加权平均，得到黑线中心相对传感器板中心的偏移。
 * 比“只判断左右最外侧压线”精度高，过弯时输出是连续的，PD 好调。
 */
#include "ti_msp_dl_config.h"
#include "line_sensor.h"
#include "car_config.h"
#include "car_time.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} line_pin_t;

/* 顺序：CH1(最左) -> CH8(最右)，具体见 car_config.h 的 REVERSE 开关 */
static const line_pin_t s_pins[LINE_SENSOR_COUNT] = {
    { GRAY_CH1_PORT, GRAY_CH1_PIN },
    { GRAY_CH2_PORT, GRAY_CH2_PIN },
    { GRAY_CH3_PORT, GRAY_CH3_PIN },
    { GRAY_CH4_PORT, GRAY_CH4_PIN },
    { GRAY_CH5_PORT, GRAY_CH5_PIN },
    { GRAY_CH6_PORT, GRAY_CH6_PIN },
    { GRAY_CH7_PORT, GRAY_CH7_PIN },
    { GRAY_CH8_PORT, GRAY_CH8_PIN },
};

static uint8_t  s_mask;
static uint8_t  s_black;
static float    s_pos_mm;
static int8_t   s_edge;
static int8_t   s_last_edge = 1;
static uint32_t s_last_seen_ms;

static uint8_t count_bits(uint8_t value)
{
    uint8_t n = 0;

    while (value != 0U) {
        n = (uint8_t)(n + (value & 0x01U));
        value = (uint8_t)(value >> 1);
    }
    return n;
}

void line_sensor_update(void)
{
    uint8_t mask = 0U;
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        uint32_t level = (DL_GPIO_readPins(s_pins[i].port, s_pins[i].pin) != 0U) ? 1U : 0U;

#if (CAR_LINE_BLACK_LEVEL != 0)
        level ^= 1U;   /* 模块是“白线输出高”的接法时改这里 */
#endif

        if (level != 0U) {
            mask = (uint8_t)(mask | (uint8_t)(1U << i));
        }
    }

    s_mask  = mask;
    s_black = count_bits(mask);

    if (s_black > 0U) {
        float sum = 0.0f;

        for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
            if ((mask & (uint8_t)(1U << i)) != 0U) {
                sum += ((float)i - ((float)(LINE_SENSOR_COUNT) - 1.0f) * 0.5f);
            }
        }

        s_pos_mm = (sum / (float)s_black) * CAR_LINE_PITCH_MM;

#if CAR_LINE_SENSOR_REVERSE
        s_pos_mm = -s_pos_mm;
#endif

        if (s_pos_mm > (CAR_LINE_PITCH_MM * 0.5f)) {
            s_edge = 1;
        } else if (s_pos_mm < (-CAR_LINE_PITCH_MM * 0.5f)) {
            s_edge = -1;
        } else {
            s_edge = 0;
        }

        if (s_edge != 0) {
            s_last_edge = s_edge;
        }
        s_last_seen_ms = car_ms();
    } else {
        s_pos_mm = 0.0f;
        s_edge   = 0;
    }
}

uint8_t line_sensor_mask(void)
{
    return s_mask;
}

uint8_t line_sensor_black_count(void)
{
    return s_black;
}

bool line_sensor_on_line(void)
{
    return s_black > 0U;
}

bool line_sensor_marker(void)
{
    return s_black >= CAR_LINE_MARKER_MIN;
}

float line_sensor_position_mm(void)
{
    return s_pos_mm;
}

int8_t line_sensor_edge(void)
{
    return s_edge;
}

int8_t line_sensor_last_edge(void)
{
    return s_last_edge;
}

uint32_t line_sensor_last_seen_ms(void)
{
    return s_last_seen_ms;
}
