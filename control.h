/*
 * control.h - 轮速闭环
 *
 * 上层只给 -100~+100 的百分比，内部换算成 mm/s 目标速度，
 * 编码器测速后做 PID，输出占空比给 motor_set()。
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void control_init(void);

/* 设置左右轮目标速度（-100 ~ +100），每 10 ms 调一次 control_update 生效 */
void control_set_command(int16_t left_pct, int16_t right_pct);

/* 每 10 ms 调一次 */
void control_update(void);

/* 清堵转故障并复位 PID（开始新项目时调用） */
void control_clear_fault(void);

float control_left_speed_mm_s(void);
float control_right_speed_mm_s(void);

/* 1 = 堵转（目标在动但轮子不转），任务会据此停车 */
bool control_fault(void);

#endif /* CONTROL_H */
