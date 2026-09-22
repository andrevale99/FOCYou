#include <stm32f411xe.h>

#include "sys_main.h"

int main(void)
{
    /* Habilita acesso a FPU */
    SCB->CPACR |= (0xFU << 20);
    __DSB();
    __ISB();

    sys_main_init();

    while (1)
    {
    }
}