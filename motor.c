/*
 * motor.c - TB6612 双路电机驱动（地猛星小车主板）
 *
 * 实际接线（来自《地猛星引脚分配2.0》/原理图）：
 *   PA12 TIMG0_CCP0 -> PWMA（左）
 *   PA13 TIMG0_CCP1 -> PWMB（右）
 *   PA8  AIN1        PA9  AIN2
 *   PB18 BIN1        PA7  BIN2
 *   PB24 STBY
 *
 * 注意：方向脚跨 GPIOA/GPIOB，不能再用单一端口宏。
 */
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "car_config.h"

static int16_t s_left = 0;
static int16_t s_right = 0;
static int8_t  s_left_dir = 0;
static int8_t  s_right_dir = 0;

static int16_t clamp_speed(int16_t v)
{
    if (v > 100) {
        return 100;
    }
    if (v < -100) {
        return -100;
    }
    return v;
}

/* 设置某一路 PWM 占空比，pct 取绝对值 0-100 */
static void set_duty(uint32_t cc_index, int16_t pct)
{
    uint32_t compare;

    if (pct < 0) {
        pct = (int16_t)-pct;
    }
    if (pct > 100) {
        pct = 100;
    }

    compare = (CAR_PWM_PERIOD_LD * (uint32_t)(100 - pct)) / 100U;
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, cc_index);
}

/*
 * 方向控制：正转 IN1=1 IN2=0，反转 IN1=0 IN2=1，停止两脚都 0。
 * 只有方向变化时才动 IO，并且先把 PWM 清零。
 */
static void apply_direction(GPIO_Regs *in1_port, uint32_t in1_pin,
                            GPIO_Regs *in2_port, uint32_t in2_pin,
                            uint32_t cc_index, int16_t pct, int8_t *cached_dir)
{
    int8_t dir = (pct > 0) ? 1 : ((pct < 0) ? -1 : 0);

    if (dir == *cached_dir) {
        return;
    }

    set_duty(cc_index, 0);

    if (dir > 0) {
        DL_GPIO_setPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    } else if (dir < 0) {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_setPins(in2_port, in2_pin);
    } else {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }

    *cached_dir = dir;
}

void motor_init(void)
{
    /* 先关使能，保证上电时电机不动 */
    DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);

    set_duty(DL_TIMER_CC_0_INDEX, 0);
    set_duty(DL_TIMER_CC_1_INDEX, 0);

    DL_TimerG_startCounter(MOTOR_PWM_INST);

    s_left = 0;
    s_right = 0;
    s_left_dir = 0;
    s_right_dir = 0;
}

void motor_set(int16_t left_pct, int16_t right_pct)
{
    left_pct  = clamp_speed(left_pct);
    right_pct = clamp_speed(right_pct);

#if CAR_LEFT_INVERT
    left_pct = (int16_t)-left_pct;
#endif
#if CAR_RIGHT_INVERT
    right_pct = (int16_t)-right_pct;
#endif

    s_left  = left_pct;
    s_right = right_pct;

    if ((left_pct == 0) && (right_pct == 0)) {
        motor_stop();
        return;
    }

    /* 使能驱动芯片，再给方向和占空比 */
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    apply_direction(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN,
                    MOTOR_AIN2_PORT, MOTOR_AIN2_PIN,
                    DL_TIMER_CC_0_INDEX, left_pct, &s_left_dir);
    apply_direction(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN,
                    MOTOR_BIN2_PORT, MOTOR_BIN2_PIN,
                    DL_TIMER_CC_1_INDEX, right_pct, &s_right_dir);

    set_duty(DL_TIMER_CC_0_INDEX, left_pct);
    set_duty(DL_TIMER_CC_1_INDEX, right_pct);
}

void motor_stop(void)
{
    DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);

    set_duty(DL_TIMER_CC_0_INDEX, 0);
    set_duty(DL_TIMER_CC_1_INDEX, 0);

    s_left = 0;
    s_right = 0;
    s_left_dir = 0;
    s_right_dir = 0;
}

void motor_brake(void)
{
    set_duty(DL_TIMER_CC_0_INDEX, 0);
    set_duty(DL_TIMER_CC_1_INDEX, 0);

    DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    s_left = 0;
    s_right = 0;
    s_left_dir = 0;
    s_right_dir = 0;
}

int16_t motor_get_left(void)
{
    return s_left;
}

int16_t motor_get_right(void)
{
    return s_right;
}
