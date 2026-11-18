/*
 * control.c - 轮速闭环（编码器测速 + 左右轮 PID + 软启动 + 堵转保护）
 *
 * 这一层只干三件事：
 *   1. 用编码器差分算轮速（mm/s）；
 *   2. 每个轮子一个 PID 把速度跟到目标；
 *   3. 限幅、斜坡、堵转判断，然后交给 motor.c。
 *
 * 没有开环降级、没有电池检测：C 题要定距停车和转 90°，
 * 编码器坏了本来也跑不了，与其降级不如直接报故障。
 */
#include "ti_msp_dl_config.h"
#include "control.h"
#include "car_config.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

#define CONTROL_PI  (3.14159265f)

static pid_t   s_left_pid;
static pid_t   s_right_pid;
static int32_t s_prev_left;
static int32_t s_prev_right;
static int16_t s_target_left;
static int16_t s_target_right;
static int16_t s_duty_left;
static int16_t s_duty_right;
static float   s_speed_left;
static float   s_speed_right;
static bool    s_stall_fault;
static uint16_t s_left_stall_ticks;
static uint16_t s_right_stall_ticks;

static float clamp_pct(float value)
{
    if (value > 100.0f) {
        return 100.0f;
    }
    if (value < -100.0f) {
        return -100.0f;
    }
    return value;
}

/* 每个周期最多变 CAR_RAMP_STEP_PCT 个百分点，起步不打滑、停车不点头 */
static int16_t ramp_step(int16_t current, float target)
{
    if (target > (float)current + (float)CAR_RAMP_STEP_PCT) {
        return (int16_t)(current + CAR_RAMP_STEP_PCT);
    }
    if (target < (float)current - (float)CAR_RAMP_STEP_PCT) {
        return (int16_t)(current - CAR_RAMP_STEP_PCT);
    }
    return (int16_t)target;
}

static bool command_on(int16_t pct)
{
    return (pct >= CAR_STALL_MIN_PCT) || (pct <= -CAR_STALL_MIN_PCT);
}

static bool near_zero(float speed)
{
    return (speed > -CAR_STALL_SPEED_MM_S) && (speed < CAR_STALL_SPEED_MM_S);
}

void control_init(void)
{
    pid_init(&s_left_pid, CAR_SPEED_KP, CAR_SPEED_KI, CAR_SPEED_KD, -100.0f, 100.0f);
    pid_init(&s_right_pid, CAR_SPEED_KP, CAR_SPEED_KI, CAR_SPEED_KD, -100.0f, 100.0f);

    s_prev_left  = encoder_left_counts();
    s_prev_right = encoder_right_counts();

    s_target_left  = 0;
    s_target_right = 0;
    s_duty_left    = 0;
    s_duty_right   = 0;
    s_speed_left   = 0.0f;
    s_speed_right  = 0.0f;
    s_stall_fault  = false;
    s_left_stall_ticks  = 0;
    s_right_stall_ticks = 0;
}

void control_set_command(int16_t left_pct, int16_t right_pct)
{
    s_target_left  = (int16_t)clamp_pct((float)left_pct);
    s_target_right = (int16_t)clamp_pct((float)right_pct);
}

void control_clear_fault(void)
{
    s_stall_fault = false;
    s_left_stall_ticks  = 0;
    s_right_stall_ticks = 0;
    pid_reset(&s_left_pid);
    pid_reset(&s_right_pid);
}

void control_update(void)
{
    int32_t left  = encoder_left_counts();
    int32_t right = encoder_right_counts();
    int32_t delta_left  = left - s_prev_left;
    int32_t delta_right = right - s_prev_right;
    const float dt = (float)CAR_CTRL_PERIOD_MS / 1000.0f;
    const float counts_per_rev = CAR_ENCODER_PPR * CAR_GEAR_RATIO * 4.0f;
    const float circumference  = CONTROL_PI * CAR_WHEEL_DIAMETER_MM;
    float target_left_mm_s;
    float target_right_mm_s;
    float duty_left;
    float duty_right;

    /* 1. 编码器差分测速（mm/s） */
    s_speed_left  = ((float)delta_left)  / counts_per_rev * circumference / dt;
    s_speed_right = ((float)delta_right) / counts_per_rev * circumference / dt;
    s_prev_left  = left;
    s_prev_right = right;

#if CAR_LEFT_ENCODER_INVERT
    s_speed_left = -s_speed_left;
#endif
#if CAR_RIGHT_ENCODER_INVERT
    s_speed_right = -s_speed_right;
#endif

    /* 2. 堵转：目标在动、轮速却接近 0，持续 1.2 s 判故障 */
    if (command_on(s_target_left) && near_zero(s_speed_left)) {
        if (s_left_stall_ticks < 0xFFFFU) {
            s_left_stall_ticks++;
        }
    } else {
        s_left_stall_ticks = 0;
    }

    if (command_on(s_target_right) && near_zero(s_speed_right)) {
        if (s_right_stall_ticks < 0xFFFFU) {
            s_right_stall_ticks++;
        }
    } else {
        s_right_stall_ticks = 0;
    }

    if (((uint32_t)s_left_stall_ticks * CAR_CTRL_PERIOD_MS >= CAR_STALL_TIMEOUT_MS) ||
        ((uint32_t)s_right_stall_ticks * CAR_CTRL_PERIOD_MS >= CAR_STALL_TIMEOUT_MS)) {
        s_stall_fault = true;
    }

    /* 3. 故障时目标清零 */
    if (s_stall_fault) {
        s_target_left  = 0;
        s_target_right = 0;
    }

    /* 4. 百分比 -> mm/s -> PID -> 占空比 */
    target_left_mm_s  = (float)s_target_left  * CAR_MAX_SPEED_MM_S / 100.0f;
    target_right_mm_s = (float)s_target_right * CAR_MAX_SPEED_MM_S / 100.0f;

    duty_left  = pid_update(&s_left_pid,  target_left_mm_s,  s_speed_left,  dt);
    duty_right = pid_update(&s_right_pid, target_right_mm_s, s_speed_right, dt);

    /* 5. 斜坡 + 限幅，然后交给电机驱动 */
    s_duty_left  = ramp_step(s_duty_left,  clamp_pct(duty_left));
    s_duty_right = ramp_step(s_duty_right, clamp_pct(duty_right));

    if (s_stall_fault) {
        motor_stop();
        s_duty_left  = 0;
        s_duty_right = 0;
    } else {
        motor_set(s_duty_left, s_duty_right);
    }
}

float control_left_speed_mm_s(void)
{
    return s_speed_left;
}

float control_right_speed_mm_s(void)
{
    return s_speed_right;
}

bool control_fault(void)
{
    return s_stall_fault;
}
