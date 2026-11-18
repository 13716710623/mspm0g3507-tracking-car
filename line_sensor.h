/*
 * line_sensor.h - 八路红外灰度循迹传感器
 *
 * 硬件：地猛星小车 H2 接口，8 路数字量直接进 GPIO
 *       CH1 PA17 / CH2 PB8 / CH3 PB9 / CH4 PA24
 *       CH5 PA2  / CH6 PA0 / CH7 PA1 / CH8 PA23
 *       具体从哪一路算“最左”取决于模块怎么插，反了就把
 *       car_config.h 里的 CAR_LINE_SENSOR_REVERSE 改成 1。
 *
 * 约定：位置 pos 为正表示黑线在车体右侧（车要向右修）。
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT        (8U)

/* 每 10 ms 调一次：采样 8 路 GPIO，算出黑线位置 */
void line_sensor_update(void);

uint8_t line_sensor_mask(void);          /* bit0 = CH1 ... bit7 = CH8，1 = 压黑线 */
uint8_t line_sensor_black_count(void);   /* 压线的路数 */
bool    line_sensor_on_line(void);       /* 至少 1 路压线 */
bool    line_sensor_marker(void);        /* 启停线：多路同时压线（比 1.8 cm 线宽得多） */

/* 黑线中心相对传感器中心的横向偏差，单位 mm，+ 表示偏右 */
float   line_sensor_position_mm(void);

/* -1 = 线偏左，+1 = 偏右，0 = 居中；丢线时返回 0 */
int8_t  line_sensor_edge(void);

/* 最后一次看到黑线时它偏在哪一侧（丢线后仍然保留），用于丢线找回 */
int8_t  line_sensor_last_edge(void);

/* 最近一次看到黑线的时间（ms），用于判断丢线时长 */
uint32_t line_sensor_last_seen_ms(void);

#endif /* LINE_SENSOR_H */
