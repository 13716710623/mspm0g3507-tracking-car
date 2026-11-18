/*
 * car_time.h - 1 ms 系统节拍（定义在 main.c 的 SysTick_Handler 里）
 *
 * 所有模块统一用 car_ms() 取时间，避免各自开定时器。
 */
#ifndef CAR_TIME_H
#define CAR_TIME_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_ms;

static inline uint32_t car_ms(void)
{
    return g_ms;
}

/* since 之后是否已经过了 ms 毫秒（自动处理 32 位回绕） */
static inline bool car_elapsed(uint32_t since, uint32_t ms)
{
    return (uint32_t)(g_ms - since) >= ms;
}

#endif /* CAR_TIME_H */
