/*
 * oled.h - 0.96" SSD1306（128x64，I2C）显示
 *
 * 显存按行刷新：每次 oled_task() 最多重画一个字符（约 1 ms I2C），
 * 不会拖慢 10 ms 控制节拍。没接屏时初始化就会失败，之后所有绘制直接跳过。
 */
#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

/* 8x16 ASCII 字库，字符 32~126，每字符 16 字节（上 8 列 + 下 8 列） */
extern const uint8_t oled_font8x16[95][16];

/* 初始化（没接屏返回 false，之后所有绘制自动跳过） */
bool oled_init(void);

/* 每 10 ms 调一次：把待显示的字符慢慢画上去 */
void oled_task(void);

/*
 * 刷新四行状态（内容没变就不会重画）：
 *   ITEM 1  RUN
 *   T   26.7 s
 *   D   6140 mm
 *   DEV  -30 YAW 0
 */
void oled_status(uint8_t item, const char *state, uint32_t elapsed_ms,
                 int32_t dist_mm, int32_t dev_mm, int32_t yaw_deg10);

#endif /* OLED_H */
