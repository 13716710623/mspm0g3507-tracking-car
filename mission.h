/*
 * mission.h - 2026 电赛校内赛 C 题：自主循迹与定点停车运动控制系统
 *
 * 一个状态机管 6 个比赛项目，全部通过按键/串口触发：
 *   1. 顺时针 1 圈，停在 A 点            （≤40 s，偏差 ≤5 cm）
 *   2. 逆时针 1 圈，停在 A 点            （≤20 s，偏差 ≤5 cm）
 *   3. 顺时针连续 3 圈，停在 A 点        （≤40 s，偏差 ≤5 cm）
 *   4. A→B 停车 3 s，倒车回 A 停车       （两处 ≤5 cm，≤30 s）
 *   5. 原地顺时针 90°，停 3 s，再逆时针 90° 转回（≤20 s）
 *   6. 从线外侧 20~40 cm 白区自主找线，顺时针到 A 停车（≤30 s）
 *
 * 具体每项的判定细节见 mission.c 里对应函数上方的注释。
 */
#ifndef MISSION_H
#define MISSION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MISSION_IDLE = 0,   /* 上电/待机 */
    MISSION_READY,      /* 已选项目，等按键启动 */
    MISSION_RUN,        /* 正在跑 */
    MISSION_DONE,       /* 正常完成 */
    MISSION_FAIL        /* 超时/丢线/故障中止 */
} mission_state_t;

/* 失败原因，出现在 $R 帧 why= 字段 */
#define MISSION_WHY_NONE        (0U)
#define MISSION_WHY_TIMEOUT     (1U)
#define MISSION_WHY_FAULT       (2U)   /* 电池低压/堵转 */
#define MISSION_WHY_LOST        (3U)   /* 巡线丢失太久 */
#define MISSION_WHY_SEEK        (4U)   /* 第 6 项找不到黑线 */
#define MISSION_WHY_NO_ENCODER  (5U)
#define MISSION_WHY_ABORT       (6U)   /* 人为中止 */

void mission_init(void);

/* 每 10 ms 调一次；内部会更新灰度采样和里程计 */
void mission_update(void);

/* 选中 1~6 项（正在跑时拒绝），返回是否成功 */
bool mission_select(uint8_t item);
void mission_next(void);
uint8_t mission_item(void);

/* 开始当前项目；编码器没信号时拒绝并打印 $X,no_encoder */
bool mission_start(void);

/* 中止（会立刻停车并打印结果帧） */
void mission_abort(void);

bool mission_running(void);
mission_state_t mission_state(void);

uint32_t mission_elapsed_ms(void);
int32_t  mission_distance_mm(void);
int16_t  mission_correction(void);
int32_t  mission_deviation_mm(void);
float    mission_yaw_deg(void);

/* 串口打印：项目菜单 / 当前状态 */
void mission_print_menu(void);
void mission_print_status(void);

#endif /* MISSION_H */
