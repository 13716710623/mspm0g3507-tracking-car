/*
 * mission.c - C 题 6 个比赛项目的状态机
 *
 * 共用一条控制链：
 *   灰度 8 路 -> line_sensor -> line_follow(PD) -> control(速度环) -> motor
 *   编码器     -> odom（里程/转角）-> 定点停车、90° 转向
 *
 * 关键判据：
 *   - 圈数用“启停线”识别：启停线 5 cm 宽，8 路灰度会同时压到 4 路以上，
 *     正常 1.8 cm 线宽只有 1~2 路，两者区分度很大。
 *   - A 点用里程模周长判定：A 在 0、6.14、12.28 m 处，B 在 1.5 m（顺时针）
 *     或 4.64 m（逆时针）处，两者不会混淆。
 *   - 停车前 26 cm 先降到爬行速度，靠 5 cm 的判定窗口把偏差压到 1~2 cm，
 *     巡航速度直接刹停会冲出去 5 cm 以上。
 */
#include "mission.h"
#include "car_config.h"
#include "car_time.h"
#include "car_uart.h"
#include "control.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "odom.h"

/* 阶段编号，各项目只用自己的那一组 */
enum {
    PH_LAUNCH = 0,
    PH_TRACK,
    PH_SLOW,
    PH_BRAKE,
    PH_HOLD,
    PH_STOP_B,
    PH_BACK,
    PH_BACK_SLOW,
    PH_BACK_STOP,
    PH_TURN1,
    PH_TURN_HOLD,
    PH_TURN2,
    PH_SEEK_ROT,
    PH_SEEK_FWD,
    PH_APPROACH
};

typedef struct {
    uint16_t cruise_mm_s;
    uint8_t  laps;
    uint32_t timeout_ms;
} item_cfg_t;

static const item_cfg_t s_items[6] = {
    { CAR_SPEED_ITEM1_MM_S, 1U, CAR_TIMEOUT_ITEM1_MS },
    { CAR_SPEED_ITEM2_MM_S, 1U, CAR_TIMEOUT_ITEM2_MS },
    { CAR_SPEED_ITEM3_MM_S, 3U, CAR_TIMEOUT_ITEM3_MS },
    { CAR_SPEED_ITEM4_MM_S, 0U, CAR_TIMEOUT_ITEM4_MS },
    { 0,                    0U, CAR_TIMEOUT_ITEM5_MS },   /* 第 5 项原地转，不用巡航速度 */
    { CAR_SPEED_ITEM6_MM_S, 1U, CAR_TIMEOUT_ITEM6_MS },
};

static mission_state_t s_state = MISSION_IDLE;
static uint8_t  s_item = 1U;
static uint8_t  s_phase = PH_LAUNCH;
static uint32_t s_t0_ms;
static uint32_t s_phase_ms;
static uint32_t s_timeout_ms;
static bool     s_tracking;

/* 里程/停车目标 */
static int32_t  s_target_mm;
static int32_t  s_back_target_mm;
static uint8_t  s_lap_target;
static uint8_t  s_lap_count;

/* 启停线识别 */
static uint8_t  s_marker_count;
static uint8_t  s_marker_ticks;
static bool     s_marker_active;
static int32_t  s_last_marker_mm;

/* 找线/找线后的稳定计数 */
static uint8_t  s_settle;
static uint8_t  s_seek_tries;
static float    s_seek_yaw0;
static int32_t  s_seek_dist;
static float    s_yaw_ref;
static float    s_prev_yaw;      /* 上一拍转角，用来算角速度 */
static float    s_rate_f;        /* 低通后的角速度，deg/s */

/* 结果 */
static bool     s_ok;
static uint8_t  s_why;
static uint32_t s_result_ms;
static int32_t  s_result_mm;
static int32_t  s_dev_mm;
static float    s_final_yaw;

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

static float mm_s_to_pct(int16_t mm_s)
{
    return ((float)mm_s * 100.0f) / CAR_MAX_SPEED_MM_S;
}

static bool stopped(void)
{
    return (abs_f(control_left_speed_mm_s()) < CAR_STOP_SPEED_MM_S) &&
           (abs_f(control_right_speed_mm_s()) < CAR_STOP_SPEED_MM_S);
}

/* 距离是否落在“A 点”附近（周长整数倍） */
static bool near_lap_grid(int32_t dist)
{
    int32_t m = dist % (int32_t)CAR_LAP_LENGTH_MM;
    int32_t d;

    if (m < 0) {
        m += (int32_t)CAR_LAP_LENGTH_MM;
    }
    d = (m < ((int32_t)CAR_LAP_LENGTH_MM / 2)) ? m : ((int32_t)CAR_LAP_LENGTH_MM - m);

    return d <= (int32_t)CAR_MARKER_WINDOW_MM;
}

/*
 * 启停线检测：连续 CAR_MARKER_CONFIRM_TICKS 个节拍压到 4 路以上才算，
 * 加上 150 mm 的防抖距离，避免过弯时把线看成启停线。
 */
static bool marker_poll(int32_t dist)
{
    bool m = line_sensor_marker();
    bool edge = false;

    if (m) {
        if (s_marker_ticks < 255U) {
            s_marker_ticks++;
        }
    } else {
        s_marker_ticks = 0;
    }

    if (!s_marker_active && (s_marker_ticks >= CAR_MARKER_CONFIRM_TICKS)) {
        if ((s_marker_count == 0U) ||
            ((uint32_t)(dist - s_last_marker_mm) >= CAR_MARKER_REFRACT_MM)) {
            s_marker_active  = true;
            s_last_marker_mm = dist;
            s_marker_count++;
            edge = true;
        }
    }

    if (s_marker_ticks == 0U) {
        s_marker_active = false;
    }

    return edge;
}

/* 巡线一步，速度单位 mm/s */
static void follow(int16_t cruise_mm_s)
{
    int16_t left;
    int16_t right;
    int16_t lo = CAR_SPEED_MIN_MM_S;

    if (cruise_mm_s < lo) {
        lo = cruise_mm_s;
    }

    line_follow_set_speed(cruise_mm_s, lo);
    (void)line_follow_step(&left, &right);
    control_set_command(left, right);
    s_tracking = true;
}

/* 倒车直行：编码器算的航向做保持，直线段比倒着巡线稳 */
static void reverse_straight(int16_t cruise_mm_s)
{
    float base_pct = mm_s_to_pct(cruise_mm_s);
    float err = odom_yaw_deg() - s_yaw_ref;
    float u = CAR_BACK_YAW_KP * err;
    float lv;
    float rv;

    if (u > CAR_BACK_CORR_MAX_PCT) {
        u = CAR_BACK_CORR_MAX_PCT;
    }
    if (u < -CAR_BACK_CORR_MAX_PCT) {
        u = -CAR_BACK_CORR_MAX_PCT;
    }

    lv = -base_pct + u;
    rv = -base_pct - u;

    control_set_command((int16_t)clamp_pct_f(lv), (int16_t)clamp_pct_f(rv));
    s_tracking = false;
}

static void mission_print_result(void);

static void mission_finish(bool ok)
{
    control_set_command(0, 0);
    s_tracking  = false;
    s_ok        = ok;
    s_result_ms = car_ms() - s_t0_ms;
    s_result_mm = odom_distance_mm();
    s_dev_mm    = s_result_mm - s_target_mm;
    s_final_yaw = odom_yaw_deg();
    s_state     = ok ? MISSION_DONE : MISSION_FAIL;
    mission_print_result();
}

static void mission_fail(uint8_t why)
{
    s_why = why;
    mission_finish(false);
}

/* ---------------- 第 1/2/3 项：沿环路跑 N 圈，停在 A 点 ---------------- */

static void run_laps(void)
{
    int32_t dist   = odom_distance_mm();
    int32_t remain = s_target_mm - dist;
    int16_t cruise = (int16_t)s_items[s_item - 1U].cruise_mm_s;

    /* 起步时车就压在 A 点启停线上，先跑 30 cm 再开始数 */
    if (dist >= (int32_t)CAR_MARKER_BLANK_MM) {
        if (marker_poll(dist) && near_lap_grid(dist)) {
            if (s_lap_count < 250U) {
                s_lap_count++;
            }
        }
    }

    switch (s_phase) {
        case PH_LAUNCH:
            follow(cruise);
            if (dist >= (int32_t)CAR_MARKER_BLANK_MM) {
                s_phase = PH_TRACK;
            }
            break;

        case PH_TRACK:
            follow(cruise);
            if ((remain <= (int32_t)CAR_STOP_SLOW_ZONE_MM) ||
                (s_lap_count >= s_lap_target)) {
                s_phase    = PH_SLOW;
                s_phase_ms = car_ms();
            }
            break;

        case PH_SLOW:
            follow(CAR_SPEED_CREEP_MM_S);
            if ((s_lap_count >= s_lap_target) || (remain <= 0) ||
                car_elapsed(s_phase_ms, 3000U)) {
                control_set_command(0, 0);
                s_phase    = PH_BRAKE;
                s_phase_ms = car_ms();
                s_tracking = false;
            }
            break;

        case PH_BRAKE:
            control_set_command(0, 0);
            if (stopped() && car_elapsed(s_phase_ms, 80U)) {
                mission_finish(true);
            }
            break;

        default:
            s_phase = PH_LAUNCH;
            break;
    }
}

/* ---------------- 第 4 项：A→B 停 3 s→倒回 A ---------------- */

static void run_shuttle(void)
{
    int32_t dist    = odom_distance_mm();
    int32_t remain_b = s_target_mm - dist;

    switch (s_phase) {
        case PH_LAUNCH:
        case PH_TRACK:
            follow(CAR_SPEED_ITEM4_MM_S);
            if (remain_b <= (int32_t)CAR_STOP_SLOW_ZONE_MM) {
                s_phase    = PH_SLOW;
                s_phase_ms = car_ms();
            }
            break;

        case PH_SLOW:
            follow(CAR_SPEED_CREEP_MM_S);
            if (remain_b <= 0) {
                control_set_command(0, 0);
                s_phase    = PH_STOP_B;
                s_phase_ms = car_ms();
                s_tracking = false;
            }
            break;

        case PH_STOP_B:
            control_set_command(0, 0);
            if (stopped() && car_elapsed(s_phase_ms, 80U)) {
                s_phase    = PH_HOLD;
                s_phase_ms = car_ms();
            }
            break;

        case PH_HOLD:
            control_set_command(0, 0);
            if (car_elapsed(s_phase_ms, CAR_HOLD_MS)) {
                s_yaw_ref  = odom_yaw_deg();
                s_target_mm = s_back_target_mm;
                s_phase    = PH_BACK;
                s_phase_ms = car_ms();
            }
            break;

        case PH_BACK:
            reverse_straight(CAR_SPEED_ITEM4_MM_S);
            if ((dist - s_back_target_mm) <= (int32_t)CAR_STOP_SLOW_ZONE_MM) {
                s_phase = PH_BACK_SLOW;
            }
            break;

        case PH_BACK_SLOW:
            reverse_straight(CAR_SPEED_CREEP_MM_S);
            if (dist <= s_back_target_mm) {
                control_set_command(0, 0);
                s_phase    = PH_BACK_STOP;
                s_phase_ms = car_ms();
            }
            break;

        case PH_BACK_STOP:
            control_set_command(0, 0);
            if (stopped() && car_elapsed(s_phase_ms, 80U)) {
                mission_finish(true);
            }
            break;

        default:
            s_phase = PH_LAUNCH;
            break;
    }
}

/* ---------------- 第 5 项：原地转 90°，停 3 s，再转回 ----------------
 *
 * 角度环 = 比例 + 角速度阻尼：
 *   u = kp * 角度误差 - kd * 当前角速度
 * 只有比例项时，12% 的最小占空比会让车在目标角两侧来回摆，
 * 永远满足不了“连续 15 拍都在 ±2.5° 内”的判据；加上阻尼后
 * 到位前会先反向制动，能真正停住。
 */
static void run_turn(void)
{
    float yaw = odom_yaw_deg();
    float rate;
    float err;
    float u;
    float target;

    s_tracking = false;

    if ((s_phase == PH_TURN1) || (s_phase == PH_TURN2)) {
        target = (s_phase == PH_TURN1) ? CAR_TURN_ITEM5_DEG : 0.0f;

        /* 角速度做一阶低通，编码器量化噪声直接微分太吵 */
        rate = (yaw - s_prev_yaw) * (1000.0f / (float)CAR_CTRL_PERIOD_MS);
        s_rate_f += 0.35f * (rate - s_rate_f);
        s_prev_yaw = yaw;

        err = target - yaw;

        if (abs_f(err) <= CAR_TURN_TOL_DEG) {
            control_set_command(0, 0);
            s_settle++;
            if (s_settle >= CAR_TURN_SETTLE_TICKS) {
                s_settle = 0;
                if (s_phase == PH_TURN1) {
                    s_phase    = PH_TURN_HOLD;
                    s_phase_ms = car_ms();
                    control_set_command(0, 0);
                } else {
                    mission_finish(true);
                }
            }
            return;
        }

        s_settle = 0;

        u = (CAR_TURN_KP * err) - (CAR_TURN_KD * s_rate_f);

        if (u > (float)CAR_TURN_MAX_PCT) {
            u = (float)CAR_TURN_MAX_PCT;
        }
        if (u < -(float)CAR_TURN_MAX_PCT) {
            u = -(float)CAR_TURN_MAX_PCT;
        }

        /* 还在往目标方向使劲但力度不够时，补到最小占空比克服静摩擦；
         * 阻尼算出来的反向指令不能被补，否则等于把刹车又踩成油门。 */
        if ((err > 0.0f) && (u >= 0.0f) && (u < (float)CAR_TURN_MIN_PCT)) {
            u = (float)CAR_TURN_MIN_PCT;
        }
        if ((err < 0.0f) && (u <= 0.0f) && (u > -(float)CAR_TURN_MIN_PCT)) {
            u = -(float)CAR_TURN_MIN_PCT;
        }

        /* u > 0 需要逆时针：左轮后退、右轮前进 */
        control_set_command((int16_t)(-u), (int16_t)u);
        return;
    }

    if (s_phase == PH_TURN_HOLD) {
        control_set_command(0, 0);
        if (car_elapsed(s_phase_ms, CAR_HOLD_MS)) {
            s_phase    = PH_TURN2;
            s_settle   = 0;
            s_prev_yaw = odom_yaw_deg();
            s_rate_f   = 0.0f;
        }
        return;
    }

    s_phase    = PH_TURN1;
    s_prev_yaw = yaw;
    s_rate_f   = 0.0f;
}

/* ---------------- 第 6 项：白区找黑线 ---------------- */

static void run_seek(void)
{
    float yaw   = odom_yaw_deg();
    int32_t dist = odom_distance_mm();

    switch (s_phase) {
        case PH_LAUNCH:
            s_seek_yaw0 = yaw;
            s_seek_dist = dist;
            s_phase     = PH_SEEK_ROT;
            break;

        case PH_SEEK_ROT:
            s_tracking = false;
            if (line_sensor_on_line()) {
                control_set_command(0, 0);
                s_phase    = PH_APPROACH;
                s_phase_ms = car_ms();
                s_settle   = 0;
                break;
            }
            control_set_command(-CAR_SEEK_ROT_PCT, CAR_SEEK_ROT_PCT);
            if (abs_f(yaw - s_seek_yaw0) >= CAR_SEEK_SWEEP_DEG) {
                s_seek_tries++;
                if (s_seek_tries > CAR_SEEK_MAX_TRIES) {
                    mission_fail(MISSION_WHY_SEEK);
                } else {
                    s_seek_dist = dist;
                    s_phase     = PH_SEEK_FWD;
                    s_phase_ms  = car_ms();
                }
            }
            break;

        case PH_SEEK_FWD:
            s_tracking = false;
            control_set_command(CAR_SEEK_FWD_PCT, CAR_SEEK_FWD_PCT);
            if ((dist - s_seek_dist) >= (int32_t)CAR_SEEK_FWD_MM) {
                s_seek_yaw0 = yaw;
                s_phase     = PH_SEEK_ROT;
            }
            break;

        case PH_APPROACH:
            follow(CAR_SPEED_CREEP_MM_S);
            if (line_sensor_on_line() &&
                (abs_f(line_sensor_position_mm()) <= CAR_ALIGN_TOL_MM)) {
                s_settle++;
                if (s_settle >= CAR_APPROACH_SETTLE_TICKS) {
                    s_settle    = 0;
                    s_seek_dist = dist;
                    s_phase     = PH_TRACK;
                    s_phase_ms  = car_ms();
                }
            } else {
                s_settle = 0;
                if (line_follow_lost_ms() > CAR_APPROACH_LOST_MS) {
                    s_seek_yaw0 = yaw;
                    s_phase     = PH_SEEK_ROT;
                }
            }
            break;

        case PH_TRACK:
            follow(CAR_SPEED_ITEM6_MM_S);
            if (s_marker_count < CAR_ITEM6_STOP_MARKERS) {
                if ((dist - s_seek_dist) >= (int32_t)CAR_MARKER_BLANK_MM) {
                    (void)marker_poll(dist);
                }
            }
            if (s_marker_count >= CAR_ITEM6_STOP_MARKERS) {
                control_set_command(0, 0);
                s_phase    = PH_BRAKE;
                s_phase_ms = car_ms();
                s_tracking = false;
            }
            break;

        case PH_BRAKE:
            control_set_command(0, 0);
            if (stopped() && car_elapsed(s_phase_ms, 80U)) {
                mission_finish(true);
            }
            break;

        default:
            s_phase = PH_LAUNCH;
            break;
    }
}

static void run_item(void)
{
    switch (s_item) {
        case 1U:
        case 2U:
        case 3U:
            run_laps();
            break;
        case 4U:
            run_shuttle();
            break;
        case 5U:
            run_turn();
            break;
        case 6U:
            run_seek();
            break;
        default:
            mission_fail(MISSION_WHY_NONE);
            break;
    }
}

/* ---------------- 对外接口 ---------------- */

void mission_init(void)
{
    s_state  = MISSION_IDLE;
    s_item   = 1U;
    s_phase  = PH_LAUNCH;
    s_t0_ms  = 0;
    s_ok     = false;
    s_why    = MISSION_WHY_NONE;
    s_tracking = false;
}

void mission_update(void)
{
    line_sensor_update();
    odom_update();

    if (s_state != MISSION_RUN) {
        return;
    }

    if (car_elapsed(s_t0_ms, s_timeout_ms)) {
        mission_fail(MISSION_WHY_TIMEOUT);
        return;
    }
    if (control_fault()) {
        mission_fail(MISSION_WHY_FAULT);
        return;
    }

    run_item();

    if ((s_state == MISSION_RUN) && s_tracking &&
        (line_follow_lost_ms() > CAR_LINE_FAIL_MS)) {
        mission_fail(MISSION_WHY_LOST);
    }
}

bool mission_select(uint8_t item)
{
    if ((item < 1U) || (item > 6U)) {
        return false;
    }
    if (s_state == MISSION_RUN) {
        return false;
    }

    s_item  = item;
    s_state = MISSION_READY;
    s_phase = PH_LAUNCH;
    s_ok    = false;
    s_why   = MISSION_WHY_NONE;
    mission_print_status();
    return true;
}

void mission_next(void)
{
    uint8_t next = (uint8_t)((s_item >= 6U) ? 1U : (s_item + 1U));

    (void)mission_select(next);
}

uint8_t mission_item(void)
{
    return s_item;
}

bool mission_start(void)
{
    if (s_state == MISSION_RUN) {
        return false;
    }

    control_clear_fault();
    odom_reset();
    line_follow_reset();

    s_phase          = PH_LAUNCH;
    s_phase_ms       = car_ms();
    s_t0_ms          = car_ms();
    s_lap_count      = 0U;
    s_marker_count   = 0U;
    s_marker_ticks   = 0U;
    s_marker_active  = false;
    s_last_marker_mm = 0;
    s_settle         = 0U;
    s_seek_tries     = 0U;
    s_seek_yaw0      = 0.0f;
    s_seek_dist      = 0;
    s_yaw_ref        = 0.0f;
    s_prev_yaw       = 0.0f;
    s_rate_f         = 0.0f;
    s_ok             = false;
    s_why            = MISSION_WHY_NONE;
    s_final_yaw      = 0.0f;
    s_tracking       = false;

    s_timeout_ms = s_items[s_item - 1U].timeout_ms;
    s_lap_target = s_items[s_item - 1U].laps;
    s_back_target_mm = CAR_STOP_TRIM_MM;

    if (s_item == 4U) {
        s_target_mm = (int32_t)CAR_STRAIGHT_AB_MM + CAR_STOP_TRIM_MM;
    } else if ((s_item == 5U) || (s_item == 6U)) {
        s_target_mm = 0;
    } else {
        s_target_mm = ((int32_t)s_lap_target * (int32_t)CAR_LAP_LENGTH_MM) +
                      CAR_STOP_TRIM_MM;
    }

    s_state = MISSION_RUN;
    return true;
}

void mission_abort(void)
{
    if (s_state == MISSION_RUN) {
        mission_fail(MISSION_WHY_ABORT);
    }
}

bool mission_running(void)
{
    return s_state == MISSION_RUN;
}

mission_state_t mission_state(void)
{
    return s_state;
}

uint32_t mission_elapsed_ms(void)
{
    if (s_state == MISSION_RUN) {
        return car_ms() - s_t0_ms;
    }
    return s_result_ms;
}

int32_t mission_distance_mm(void)
{
    return odom_distance_mm();
}

int16_t mission_correction(void)
{
    return line_follow_correction();
}

int32_t mission_deviation_mm(void)
{
    return s_dev_mm;
}

float mission_yaw_deg(void)
{
    return odom_yaw_deg();
}

/* ---------------- 串口输出 ---------------- */

static void mission_print_result(void)
{
    static char buf[160];
    char *p = buf;

    p = car_uart_append_str(p, "$R,item=");
    p = car_uart_append_uint(p, (uint32_t)s_item);
    p = car_uart_append_str(p, ",ok=");
    p = car_uart_append_uint(p, s_ok ? 1U : 0U);
    p = car_uart_append_str(p, ",why=");
    p = car_uart_append_uint(p, (uint32_t)s_why);
    p = car_uart_append_str(p, ",t=");
    p = car_uart_append_uint(p, s_result_ms);
    p = car_uart_append_str(p, ",d=");
    p = car_uart_append_int(p, s_result_mm);
    p = car_uart_append_str(p, ",dev=");
    p = car_uart_append_int(p, s_dev_mm);
    p = car_uart_append_str(p, ",yaw10=");
    p = car_uart_append_int(p, (int32_t)(s_final_yaw * 10.0f));
    p = car_uart_append_str(p, ",lap=");
    p = car_uart_append_uint(p, (uint32_t)s_lap_count);
    p = car_uart_append_str(p, ",mk=");
    p = car_uart_append_uint(p, (uint32_t)s_marker_count);
    p = car_uart_append_str(p, "\r\n");
    *p = '\0';

    car_uart_text(buf);
}

void mission_print_status(void)
{
    static char buf[160];
    char *p = buf;

    p = car_uart_append_str(p, "$D,item=");
    p = car_uart_append_uint(p, (uint32_t)s_item);
    p = car_uart_append_str(p, ",st=");
    p = car_uart_append_uint(p, (uint32_t)s_state);
    p = car_uart_append_str(p, ",ph=");
    p = car_uart_append_uint(p, (uint32_t)s_phase);
    p = car_uart_append_str(p, ",t=");
    p = car_uart_append_uint(p, mission_elapsed_ms());
    p = car_uart_append_str(p, ",d=");
    p = car_uart_append_int(p, odom_distance_mm());
    p = car_uart_append_str(p, ",pos=");
    p = car_uart_append_int(p, (int32_t)line_sensor_position_mm());
    p = car_uart_append_str(p, ",blk=");
    p = car_uart_append_uint(p, (uint32_t)line_sensor_black_count());
    p = car_uart_append_str(p, ",corr=");
    p = car_uart_append_int(p, (int32_t)line_follow_correction());
    p = car_uart_append_str(p, ",lap=");
    p = car_uart_append_uint(p, (uint32_t)s_lap_count);
    p = car_uart_append_str(p, ",mk=");
    p = car_uart_append_uint(p, (uint32_t)s_marker_count);
    p = car_uart_append_str(p, "\r\n");
    *p = '\0';

    car_uart_text(buf);
}

void mission_print_menu(void)
{
    car_uart_text("$M,1=CW-1lap 2=CCW-1lap 3=CW-3laps 4=A-B-back 5=spin90 6=seek-line\r\n");
    car_uart_text("$M,serial: 1-6 select, P start, X stop, M next, S status\r\n");
    car_uart_text("$M,key: short=start/stop, long=next-item\r\n");
}
