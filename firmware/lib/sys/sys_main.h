#ifndef SYS_MAIN_H
#define SYS_MAIN_H

#include "sys_rcc.h"
#include "sys_uart.h"
#include "sys_lcd.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void sys_main_init(void)
    {
        // Inicializacao do clock em 50 MHz
        sys_rcc_init();

        // A USART deve ser iniciado depois
        // de inicializar o clock do sistema (RCC)
        sys_uart_init();

        // inicializacao do lcd
        sys_lcd_init();
    }

#ifdef __cplusplus
}
#endif

#endif
