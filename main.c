/*
 * main.c - 2026 电赛校内赛 C 题：自主循迹与定点停车运动控制系统
 *
 * 只有一条控制链：
 *   八路灰度 -> line_sensor -> line_follow(PD) -> control(轮速环) -> motor
 *   编码器   -> odom（里程/转角）-> mission（6 项任务、定点停车、90° 转向）
 *
 * 启动方式：
 *   - 短按 PA16：开始 / 中止；长按：切换项目
 *   - 串口 5 个单字符：1~6 选项目、P 开始、X 停止、S 打印状态
 *     （没有上位机也能跑，串口只是现场选项目和看时间/里程用）
 */
#include "ti_msp_dl_config.h"
#include "car_config.h"
#include "car_time.h"
#include "car_uart.h"
#include "button.h"
#include "control.h"
#include "encoder.h"
#include "line_follow.h"
#include "mission.h"
#include "motor.h"
#include "odom.h"
#include "oled.h"

volatile uint32_t g_ms = 0;

/* 1ms 系统节拍 */
void SysTick_Handler(void)
{
    g_ms++;
}

/* 串口请求标志（中断里置位，主循环消费） */
static volatile uint8_t s_req_item;
static volatile bool    s_req_start;
static volatile bool    s_req_stop;
static volatile bool    s_req_status;

/* UART0 接收中断：只认 5 个字符 */
void CMD_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(CMD_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(CMD_UART_INST)) {
                uint8_t c = (uint8_t)DL_UART_Main_receiveData(CMD_UART_INST);

                if ((c >= (uint8_t)'1') && (c <= (uint8_t)'6')) {
                    s_req_item = (uint8_t)(c - (uint8_t)'0');
                } else if ((c == (uint8_t)'P') || (c == (uint8_t)'p')) {
                    s_req_start = true;
                } else if ((c == (uint8_t)'X') || (c == (uint8_t)'x')) {
                    s_req_stop = true;
                } else if ((c == (uint8_t)'S') || (c == (uint8_t)'s')) {
                    s_req_status = true;
                } else {
                    /* 其它字符忽略 */
                }
            }
            break;
        default:
            break;
    }
}

/* 任务状态 -> 屏幕上的四个字母 */
static const char *state_text(mission_state_t s)
{
    switch (s) {
        case MISSION_READY: return "READY";
        case MISSION_RUN:   return "RUN";
        case MISSION_DONE:  return "DONE";
        case MISSION_FAIL:  return "FAIL";
        default:            return "IDLE";
    }
}

int main(void)
{
    uint32_t last_ctrl_ms = 0;
    uint32_t last_led_ms  = 0;
    uint32_t last_tel_ms  = 0;
    uint32_t last_oled_ms = 0;
    uint32_t boot_ms;

    SYSCFG_DL_init();
    motor_init();
    encoder_init();
    control_init();
    line_follow_init();
    odom_init();
    mission_init();
    button_init();
    (void)oled_init();          /* 没接屏会自动放弃绘制，不影响其它功能 */
    boot_ms = car_ms();

    /* 1ms 节拍 + 串口接收中断 */
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    NVIC_EnableIRQ(CMD_UART_INST_INT_IRQN);

    car_uart_text("\r\nC-ti line-track fw " CAR_FW_VERSION "\r\n");
    mission_print_menu();

    while (1) {
        /* 1. 串口请求 */
        {
            uint8_t item = s_req_item;

            if (item != 0U) {
                s_req_item = 0U;
                (void)mission_select(item);
            }
        }
        if (s_req_start) {
            s_req_start = false;
            (void)mission_start();
        }
        if (s_req_stop) {
            s_req_stop = false;
            mission_abort();
            control_set_command(0, 0);
        }
        if (s_req_status) {
            s_req_status = false;
            mission_print_status();
        }

        /* 2. 10 ms 控制节拍：按键 -> 任务状态机 -> 轮速环 */
        if (car_elapsed(last_ctrl_ms, CAR_CTRL_PERIOD_MS)) {
            last_ctrl_ms = g_ms;
            button_poll();
            mission_update();
            control_update();
            oled_task();        /* 每拍最多画一个字符，约 1 ms I2C */
        }

        /* 3. 按键：短按开始/中止，长按换项目。
         *    上电后 CAR_KEY_MASK_MS 内丢弃按键事件：按键模块的 0.1 uF 电容冷上电时
         *    会被读成一次假按下，不加这个窗口车会自己开跑。 */
        if (car_elapsed(boot_ms, CAR_KEY_MASK_MS)) {
            if (button_take_short()) {
                if (mission_running()) {
                    mission_abort();
                    control_set_command(0, 0);
                } else {
                    (void)mission_start();
                }
            }
            if (button_take_long()) {
                if (!mission_running()) {
                    mission_next();
                }
            }
        } else {
            (void)button_take_short();
            (void)button_take_long();
        }

        /* 4. 心跳灯：待机 500 ms，待启动 250 ms，运行中 100 ms */
        {
            uint32_t period = 500U;

            if (mission_running()) {
                period = 100U;
            } else if (mission_state() == MISSION_READY) {
                period = 250U;
            }
            if (car_elapsed(last_led_ms, period)) {
                last_led_ms = g_ms;
                DL_GPIO_togglePins(MOTOR_LED_PORT, MOTOR_LED_PIN);
            }
        }

        /* 5. OLED 状态：每 200 ms 更新文本，实际绘制由 oled_task 分摊 */
        if (car_elapsed(last_oled_ms, CAR_OLED_PERIOD_MS)) {
            last_oled_ms = g_ms;
            oled_status(mission_item(), state_text(mission_state()),
                        mission_elapsed_ms(), mission_distance_mm(),
                        mission_deviation_mm(), (int32_t)(mission_yaw_deg() * 10.0f));
        }

        /* 6. 串口显示：运行中 2 Hz 打一次状态（时间、里程、黑线偏差） */
        if (mission_running() && car_elapsed(last_tel_ms, 500U)) {
            last_tel_ms = g_ms;
            mission_print_status();
        }
    }
}
