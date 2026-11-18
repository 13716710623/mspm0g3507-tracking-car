/*
 * encoder.h - 编码器计数（2 倍频：A 相双边沿，B 相判方向）
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 清计数并使能 GPIO 中断 */
void encoder_init(void);

/* 读取累计计数（带临界区保护） */
int32_t encoder_left_counts(void);
int32_t encoder_right_counts(void);

/* 清零累计计数 */
void encoder_reset(void);

#endif /* ENCODER_H */
