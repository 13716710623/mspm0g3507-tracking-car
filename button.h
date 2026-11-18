/*
 * button.h - 启动按键（PA16，内部上拉，按下为低）
 *
 * 短按 = 开始/中止，长按 = 切换比赛项目。
 * 引脚分配表里 32 个 IO 只剩 PA16 没上车用，所以按键就接这儿。
 */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>

void button_init(void);

/* 每 10 ms 调一次 */
void button_poll(void);

/* 取走一次短按/长按事件（读后清零） */
bool button_take_short(void);
bool button_take_long(void);

/* 当前是否按住（消抖后） */
bool button_pressed(void);

#endif /* BUTTON_H */
