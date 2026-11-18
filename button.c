/*
 * button.c - 按键消抖与长短按识别
 *
 * 采样周期 10 ms，连续 3 次一致才算稳定（30 ms 消抖）。
 * 长按在“按住满 CAR_KEY_LONG_MS”时立刻触发一次，不等松手，
 * 这样松手时不会再补一个短按事件。
 *
 * 引脚：PA16。SysConfig 里这一组只有一根脚，生成的是组端口
 * KEY_PORT + KEY_START_PIN（不像 GRAY 那样跨端口给每脚一个 PORT 宏）。
 */
#include "ti_msp_dl_config.h"
#include "button.h"
#include "car_config.h"
#include "car_time.h"

static bool     s_raw;
static bool     s_stable_state;
static uint8_t  s_stable_cnt;
static uint32_t s_press_ms;
static bool     s_long_fired;
static bool     s_short_event;
static bool     s_long_event;

void button_init(void)
{
    s_raw          = (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U);
    s_stable_state = s_raw;
    s_stable_cnt   = 0;
    s_press_ms     = car_ms();
    s_long_fired   = false;
    s_short_event  = false;
    s_long_event   = false;
}

void button_poll(void)
{
    bool raw = (DL_GPIO_readPins(KEY_PORT, KEY_START_PIN) == 0U);

    if (raw != s_raw) {
        s_raw = raw;
        s_stable_cnt = 0;
        return;
    }

    if (s_stable_cnt < 255U) {
        s_stable_cnt++;
    }

    if ((s_stable_cnt == CAR_KEY_DEBOUNCE_TICKS) && (raw != s_stable_state)) {
        s_stable_state = raw;

        if (raw) {
            s_press_ms   = car_ms();
            s_long_fired = false;
        } else {
            uint32_t held = car_ms() - s_press_ms;

            if (!s_long_fired &&
                (held >= CAR_KEY_SHORT_MIN_MS) &&
                (held < CAR_KEY_LONG_MS)) {
                s_short_event = true;
            }
        }
    }

    /* 按住满长按时间，立刻报一次长按 */
    if (s_stable_state && !s_long_fired &&
        ((car_ms() - s_press_ms) >= CAR_KEY_LONG_MS)) {
        s_long_fired = true;
        s_long_event = true;
    }
}

bool button_take_short(void)
{
    bool v = s_short_event;

    s_short_event = false;
    return v;
}

bool button_take_long(void)
{
    bool v = s_long_event;

    s_long_event = false;
    return v;
}

bool button_pressed(void)
{
    return s_stable_state;
}
