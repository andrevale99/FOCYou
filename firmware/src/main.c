#include <stm32f411xe.h>

#include "lcd16x2.h"

int main(void)
{
    /* Habilita acesso à FPU */
    SCB->CPACR |= (0xFU << 20);

    __DSB();
    __ISB();

    return 0;
}