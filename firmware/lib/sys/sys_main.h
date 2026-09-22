#ifndef SYS_MAIN_H
#define SYS_MAIN_H

#include "sys_rcc.h"
#include "sys_uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void sys_main_init(void)
    {
        sys_rcc_init();

        // A USART deve ser iniciado depois
        // de inicializar o clock do sistema (RCC)
        sys_uart_init();
    }

#ifdef __cplusplus
}
#endif

#endif
