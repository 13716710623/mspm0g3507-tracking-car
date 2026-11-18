/*
 * line_follow.c - 循迹 PD
 *
 * 差速小车的循迹本质是“用左右轮速差造转向”：
 *   corr > 0（黑线偏右）-> 左轮快右轮慢 -> 车头右摆
 * 偏差越大越减速，防止进弯速度过高直接冲出去。
 */
#include "line_follow.h"
#include "car_config.h"
#include "line_sensor.h"

static float    s_kp = CAR_LINE_KP;
static float    s_kd = CAR_LINE_KD;
static int16_t  s_cruise_mm_s = CAR_SPEED_ITEM1_MM_S;
static int16_t  s_min_mm_s = CAR_SPEED_MIN_MM_S;
static float    s_prev_pos;
static float    s_dpos;          /* 一阶低通后的偏差变化率，mm/s */
static uint16_t s_lost_ticks;
static int16_t  s_corr_pct;
static float    s_last_pos;

static float abs_f(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float clamp_pct_f(float v)
{
    if (v > 100.0f) {
        return 100.0f;
    }
    if (v < -100.0f) {
        return -100.0f;
    }
    return v;
}

void line_follow_init(void)
{
    line_follow_reset();
    s_kp = CAR_LINE_KP;
    s_kd = CAR_LINE_KD;
    s_cruise_mm_s = CAR_SPEED_ITEM1_MM_S;
    s_min_mm_s    = CAR_SPEED_MIN_MM_S;
}

void line_follow_reset(void)
{
    s_prev_pos  = 0.0f;
    s_dpos      = 0.0f;
    s_lost_ticks = 0;
    s_corr_pct  = 0;
    s_last_pos  = 0.0f;
}

void line_follow_set_speed(int16_t cruise_mm_s, int16_t min_mm_s)
{
    if (cruise_mm_s > 10) {
        s_cruise_mm_s = cruise_mm_s;
    }
    if (min_mm_s > 5) {
        s_min_mm_s = min_mm_s;
    }
    if (s_min_mm_s > s_cruise_mm_s) {
        s_min_mm_s = s_cruise_mm_s;
    }
}

void line_follow_set_gains(float kp, float kd)
{
    if ((kp >= 0.0f) && (kp <= 20.0f)) {
        s_kp = kp;
    }
    if ((kd >= 0.0f) && (kd <= 5.0f)) {
        s_kd = kd;
    }
}

bool line_follow_step(int16_t *left_pct, int16_t *right_pct)
{
    const float dt = (float)CAR_CTRL_PERIOD_MS / 1000.0f;
    const float max_speed = CAR_MAX_SPEED_MM_S;
    float pos;
    float base_mm;
    float base_pct;
    float corr;
    float lv;
    float rv;

    if (!line_sensor_on_line()) {
        float search = (float)CAR_LINE_SEARCH_PCT;

        if (s_lost_ticks < 0xFFFFU) {
            s_lost_ticks++;
        }

        /* 朝最后一次看到黑线的方向原地转，转回线上 */
        if (line_sensor_last_edge() >= 0) {
            lv = search;
            rv = -search;
        } else {
            lv = -search;
            rv = search;
        }

        s_corr_pct = (int16_t)((lv - rv) * 0.5f);
        *left_pct  = (int16_t)lv;
        *right_pct = (int16_t)rv;
        return false;
    }

    s_lost_ticks = 0;

    pos = line_sensor_position_mm();
    {
        float raw = (pos - s_prev_pos) / dt;

        s_dpos += 0.4f * (raw - s_dpos);
        s_prev_pos = pos;
    }
    s_last_pos = pos;

    /* 偏差大说明在弯里，主动降速，最低不低于 min */
    base_mm = (float)s_cruise_mm_s - CAR_LINE_SLOWDOWN * abs_f(pos);
    if (base_mm > (float)s_cruise_mm_s) {
        base_mm = (float)s_cruise_mm_s;
    }
    if (base_mm < (float)s_min_mm_s) {
        base_mm = (float)s_min_mm_s;
    }

    base_pct = base_mm * 100.0f / max_speed;

    corr = (s_kp * pos) + (s_kd * s_dpos);
    if (corr > (float)CAR_LINE_CORR_MAX_PCT) {
        corr = (float)CAR_LINE_CORR_MAX_PCT;
    }
    if (corr < -(float)CAR_LINE_CORR_MAX_PCT) {
        corr = -(float)CAR_LINE_CORR_MAX_PCT;
    }

    lv = base_pct + corr;
    rv = base_pct - corr;

    s_corr_pct = (int16_t)corr;
    *left_pct  = (int16_t)clamp_pct_f(lv);
    *right_pct = (int16_t)clamp_pct_f(rv);

    return true;
}

int16_t line_follow_correction(void)
{
    return s_corr_pct;
}

uint16_t line_follow_lost_ms(void)
{
    return (uint16_t)(s_lost_ticks * CAR_CTRL_PERIOD_MS);
}

float line_follow_position_mm(void)
{
    return s_last_pos;
}
