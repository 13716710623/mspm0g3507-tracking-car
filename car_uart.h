/*
 * car_uart.h - 串口 0（PA10/PA11，115200）的发送封装
 *
 * 只做“发字符串”这一件事，故意不用 printf：
 * 栈只有 512 字节，标准库 printf 家族一进来就溢出。
 * 遥测帧用 append_* 手工拼，长度可控。
 */
#ifndef CAR_UART_H
#define CAR_UART_H

#include <stdint.h>

/* 阻塞发送以 '\0' 结尾的字符串 */
void car_uart_text(const char *text);

/* 往缓冲区拼字符串，返回新的写指针（不检查越界，调用方自己留够空间） */
char *car_uart_append_str(char *p, const char *s);
char *car_uart_append_uint(char *p, uint32_t value);
char *car_uart_append_int(char *p, int32_t value);

#endif /* CAR_UART_H */
