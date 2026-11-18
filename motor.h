/*
 * motor.h - TB6612 类双路直流电机驱动
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 初始化：关使能、占空比清零、启动 PWM 定时器 */
void motor_init(void);

/*
 * 设置左右轮速度，范围 -100 ~ +100：
 *   正数前进，负数后退，0 滑行
 * 内部会做限幅、方向切换和 STBY 使能控制。
 */
void motor_set(int16_t left_pct, int16_t right_pct);

/* 停车：拉低 STBY，方向脚清零，占空比清零 */
void motor_stop(void);

/* 刹车：方向脚同时拉高（短路刹车），占空比清零 */
void motor_brake(void);

int16_t motor_get_left(void);
int16_t motor_get_right(void);

#endif /* MOTOR_H */
