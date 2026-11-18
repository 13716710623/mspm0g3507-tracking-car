/*
 * pid.c - 位置式 PID
 *
 * 微分项作用在测量值上（derivative on measurement），避免目标突变时踢一脚。
 * 积分项带反算抗饱和：输出被限幅时把积分拉回边界。
 */
#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measure = 0.0f;
    pid->initialized = 0;
}

float pid_update(pid_t *pid, float target, float measure, float dt)
{
    float error;
    float derivative = 0.0f;
    float output;

    if (dt <= 0.0f) {
        return 0.0f;
    }

    if (pid->initialized == 0) {
        pid->prev_measure = measure;
        pid->initialized = 1;
    }

    error = target - measure;
    derivative = (measure - pid->prev_measure) / dt;
    pid->prev_measure = measure;

    pid->integral += error * dt;

    output = (pid->kp * error) + (pid->ki * pid->integral) - (pid->kd * derivative);

    if (output > pid->out_max) {
        output = pid->out_max;
        if (pid->ki != 0.0f) {
            /* 反算积分，防止积分饱和 */
            pid->integral = (output - (pid->kp * error) + (pid->kd * derivative)) / pid->ki;
        }
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (pid->ki != 0.0f) {
            pid->integral = (output - (pid->kp * error) + (pid->kd * derivative)) / pid->ki;
        }
    }

    return output;
}
