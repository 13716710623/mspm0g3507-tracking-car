/*
 * odom.h - 里程计：用编码器算行驶距离和车头转角
 *
 * 竞赛题要的是“定距停车”和“原地转 90°”，没有陀螺仪时用左右轮
 * 差速积分算角度完全够用（轮距已知，编码器 0.13 mm/计数）。
 */
#ifndef ODOM_H
#define ODOM_H

#include <stdbool.h>
#include <stdint.h>

/* 上电调用一次：算每计数对应多少毫米，记录编码器基线 */
void odom_init(void);

/* 每 10 ms 调一次（必须和 control_update 同一个节拍） */
void odom_update(void);

/*
 * 把当前点当作原点：距离清零、转角清零。
 * 注意：只清本模块自己的基线，不动 encoder 的全局计数，
 * 否则 control.c 的速度环会看到一次巨大的计数跳变。
 */
void odom_reset(void);

int32_t odom_distance_mm(void);   /* 相对上次 reset 的里程，倒车为负 */
float   odom_yaw_deg(void);       /* 相对上次 reset 的转角，逆时针为正 */
float   odom_speed_mm_s(void);
float   odom_mm_per_count(void);

#endif /* ODOM_H */
