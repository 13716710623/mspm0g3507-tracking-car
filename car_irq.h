/*
 * car_irq.h - 临界区保护（Cortex-M0+ PRIMASK）
 *
 * TI clang 下 CMSIS 的 __disable_irq 不一定可用，这里直接用指令。
 */
#ifndef CAR_IRQ_H
#define CAR_IRQ_H

#include <stdint.h>

static inline uint32_t car_irq_save(void)
{
    uint32_t primask;
    __asm__ volatile("mrs %0, primask" : "=r"(primask)::"memory");
    __asm__ volatile("cpsid i" ::: "memory");
    return primask;
}

static inline void car_irq_restore(uint32_t primask)
{
    if (primask == 0U) {
        __asm__ volatile("cpsie i" ::: "memory");
    }
}

#endif /* CAR_IRQ_H */
