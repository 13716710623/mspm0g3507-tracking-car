/*
 * pid.h - 通用位置式 PID（带积分限幅和输出限幅）
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_measure;
    float out_min;
    float out_max;
    int   initialized;
} pid_t;

void  pid_init(pid_t *pid, float kp, float ki, float kd,
               float out_min, float out_max);
void  pid_reset(pid_t *pid);
float pid_update(pid_t *pid, float target, float measure, float dt);

#endif /* PID_H */
