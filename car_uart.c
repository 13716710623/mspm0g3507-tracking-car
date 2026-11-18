/*
 * car_uart.c - 串口发送实现
 */
#include "ti_msp_dl_config.h"
#include "car_uart.h"

static void tx_byte(char c)
{
    while (DL_UART_Main_isTXFIFOFull(CMD_UART_INST)) {
        /* 等发送 FIFO 有空位 */
    }
    DL_UART_Main_transmitData(CMD_UART_INST, (uint8_t)c);
}

void car_uart_text(const char *text)
{
    while (*text != '\0') {
        tx_byte(*text);
        text++;
    }
}

char *car_uart_append_str(char *p, const char *s)
{
    while (*s != '\0') {
        *p = *s;
        p++;
        s++;
    }
    return p;
}

char *car_uart_append_uint(char *p, uint32_t value)
{
    char tmp[10];
    int n = 0;

    if (value == 0U) {
        *p = '0';
        return p + 1;
    }

    while ((value > 0U) && (n < (int)sizeof(tmp))) {
        tmp[n] = (char)('0' + (value % 10U));
        value /= 10U;
        n++;
    }
    while (n > 0) {
        n--;
        *p = tmp[n];
        p++;
    }
    return p;
}

char *car_uart_append_int(char *p, int32_t value)
{
    if (value < 0) {
        *p = '-';
        p++;
        return car_uart_append_uint(p, (uint32_t)(-value));
    }
    return car_uart_append_uint(p, (uint32_t)value);
}
