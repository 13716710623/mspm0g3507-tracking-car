/*
 * odom.c - 编码器里程计
 *
 * 距离：左右轮平均位移（轮速环已经在跑，滑移误差靠停车标定补）
 * 转角：差速 / 轮距，D 字型转弯时比陀螺仪差一点，但 90° 够用。
 */
#include "odom.h"
#include "car_config.h"
#include "encoder.h"

#define ODOM_PI          (3.14159265f)
#define ODOM_RAD_TO_DEG  (57.2957795f)

static int32_t s_prev_left;
static int32_t s_prev_right;
static float   s_distance_mm;
static float   s_yaw_deg;
static float   s_speed_mm_s;
static float   s_mm_per_count;

void odom_init(void)
{
    const float counts_per_rev = CAR_ENCODER_PPR * CAR_GEAR_RATIO * 4.0f;

    s_mm_per_count = (ODOM_PI * CAR_WHEEL_DIAMETER_MM) / counts_per_rev;

    s_prev_left  = encoder_left_counts();
    s_prev_right = encoder_right_counts();
    s_distance_mm = 0.0f;
    s_yaw_deg     = 0.0f;
    s_speed_mm_s  = 0.0f;
}

void odom_update(void)
{
    int32_t left  = encoder_left_counts();
    int32_t right = encoder_right_counts();
    int32_t delta_left  = left - s_prev_left;
    int32_t delta_right = right - s_prev_right;
    float   mm_left;
    float   mm_right;

    s_prev_left  = left;
    s_prev_right = right;

    mm_left  = (float)delta_left * s_mm_per_count;
    mm_right = (float)delta_right * s_mm_per_count;

    s_distance_mm += 0.5f * (mm_left + mm_right);

    if (CAR_WHEEL_BASE_MM > 1.0f) {
        s_yaw_deg += ((mm_right - mm_left) / CAR_WHEEL_BASE_MM) * ODOM_RAD_TO_DEG;
    }

    s_speed_mm_s = (0.5f * (mm_left + mm_right) * 1000.0f) / (float)CAR_CTRL_PERIOD_MS;
}

void odom_reset(void)
{
    s_prev_left  = encoder_left_counts();
    s_prev_right = encoder_right_counts();
    s_distance_mm = 0.0f;
    s_yaw_deg     = 0.0f;
    s_speed_mm_s  = 0.0f;
}

int32_t odom_distance_mm(void)
{
    return (int32_t)s_distance_mm;
}

float odom_yaw_deg(void)
{
    return s_yaw_deg;
}

float odom_speed_mm_s(void)
{
    return s_speed_mm_s;
}

float odom_mm_per_count(void)
{
    return s_mm_per_count;
}
