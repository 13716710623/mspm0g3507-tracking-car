/*
 * encoder.c - 编码器正交解码（4 倍频，跨 GPIOA/GPIOB）
 *
 * 实际接线（《地猛星引脚分配2.0》）：
 *   编码器1：A = PA21，B = PA22（左侧）
 *   编码器2：A = PB19，B = PB20（右侧）
 *
 * 两块端口的 GPIO 中断都归 GROUP1，所以要在同一个 GROUP1_IRQHandler 里
 * 按 pending group 分别处理。
 */
#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "car_config.h"
#include "car_irq.h"

static volatile int32_t s_left_counts = 0;
static volatile int32_t s_right_counts = 0;

static inline uint32_t read_level(GPIO_Regs *port, uint32_t pin)
{
    return DL_GPIO_readPins(port, pin) ? 1U : 0U;
}

/* A 相边沿：A 与 B 相同表示正转 */
static inline int32_t step_on_a(GPIO_Regs *a_port, uint32_t a_pin,
                                GPIO_Regs *b_port, uint32_t b_pin)
{
    return (read_level(a_port, a_pin) == read_level(b_port, b_pin)) ? 1 : -1;
}

/* B 相边沿：A 与 B 不同表示正转 */
static inline int32_t step_on_b(GPIO_Regs *a_port, uint32_t a_pin,
                                GPIO_Regs *b_port, uint32_t b_pin)
{
    return (read_level(a_port, a_pin) != read_level(b_port, b_pin)) ? 1 : -1;
}

void encoder_init(void)
{
    s_left_counts  = 0;
    s_right_counts = 0;

    NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIOB_INT_IRQN);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case ENCODER_GPIOA_INT_IIDX: {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(
                ENCODER_LEFT_A_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);

            if ((status & ENCODER_LEFT_A_PIN) != 0U) {
                s_left_counts += step_on_a(ENCODER_LEFT_A_PORT, ENCODER_LEFT_A_PIN,
                                           ENCODER_LEFT_B_PORT, ENCODER_LEFT_B_PIN);
            }
            if ((status & ENCODER_LEFT_B_PIN) != 0U) {
                s_left_counts += step_on_b(ENCODER_LEFT_A_PORT, ENCODER_LEFT_A_PIN,
                                           ENCODER_LEFT_B_PORT, ENCODER_LEFT_B_PIN);
            }

            DL_GPIO_clearInterruptStatus(
                ENCODER_LEFT_A_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);
            break;
        }
        case ENCODER_GPIOB_INT_IIDX: {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(
                ENCODER_RIGHT_A_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);

            if ((status & ENCODER_RIGHT_A_PIN) != 0U) {
                s_right_counts += step_on_a(ENCODER_RIGHT_A_PORT, ENCODER_RIGHT_A_PIN,
                                            ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN);
            }
            if ((status & ENCODER_RIGHT_B_PIN) != 0U) {
                s_right_counts += step_on_b(ENCODER_RIGHT_A_PORT, ENCODER_RIGHT_A_PIN,
                                            ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN);
            }

            DL_GPIO_clearInterruptStatus(
                ENCODER_RIGHT_A_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
            break;
        }
        default:
            break;
    }
}

int32_t encoder_left_counts(void)
{
    int32_t value;
    uint32_t primask = car_irq_save();

    value = s_left_counts;
    car_irq_restore(primask);

    return value;
}

int32_t encoder_right_counts(void)
{
    int32_t value;
    uint32_t primask = car_irq_save();

    value = s_right_counts;
    car_irq_restore(primask);

    return value;
}

void encoder_reset(void)
{
    uint32_t primask = car_irq_save();

    s_left_counts  = 0;
    s_right_counts = 0;
    car_irq_restore(primask);
}
