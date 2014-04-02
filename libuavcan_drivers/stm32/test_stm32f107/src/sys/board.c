/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <ch.h>
#include <hal.h>

const PALConfig pal_default_config = {
    { VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH },
    { VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH },
    { VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH },
    { VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH },
    { VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH }
};

void __early_init(void)
{
    stm32_clock_init();
}

void boardInit(void)
{
    AFIO->MAPR |=
        AFIO_MAPR_CAN_REMAP_REMAP3 |
        AFIO_MAPR_CAN2_REMAP       |
        AFIO_MAPR_USART2_REMAP;
}
