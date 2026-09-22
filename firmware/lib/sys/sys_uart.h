#ifndef SYS_UART_H
#define SYS_UART_H

#include <stm32f411xe.h>

#include "uart.h"

void sys_uart_init(void)
{
    usart1_init(SystemCoreClock, 1115200);
}

#endif