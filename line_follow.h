/*
 * line_follow.h - 循迹控制（位置式 PD + 弯道自动降速 + 丢线找回）
 *
 * 输入：line_sensor 的黑线横向偏差（mm）
 * 输出：左右轮目标百分比，直接喂给 control_set_command()，所以
 *       速度环、软启动、堵转保护全部沿用原有底盘代码。
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

void line_follow_init(void);

/* 新一轮任务开始前调用：清微分和丢线状态 */
void line_follow_reset(void);

/* 设定巡航速度与弯道最低速度（mm/s） */
void line_follow_set_speed(int16_t cruise_mm_s, int16_t min_mm_s);

/* 现场调 PD；kp 单位是“百分比/mm”，kd 是“百分比/(mm/s)” */
void line_follow_set_gains(float kp, float kd);

/*
 * 跑一步：调用前必须先 line_sensor_update()。
 * 返回 true 表示黑线在视野内；false 表示正在丢线找回（此时输出的
 * 是原地找线指令，仍应下发）。
 */
bool line_follow_step(int16_t *left_pct, int16_t *right_pct);

int16_t  line_follow_correction(void);   /* 最近一次的转向修正量（%） */
uint16_t line_follow_lost_ms(void);      /* 连续丢线时长 */
float    line_follow_position_mm(void);  /* 最近一次黑线偏差 */

#endif /* LINE_FOLLOW_H */
