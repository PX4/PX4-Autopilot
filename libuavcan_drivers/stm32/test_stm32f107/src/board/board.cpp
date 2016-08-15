/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "board.hpp"
#include <cstring>
#include <unistd.h>
#include <ch.h>
#include <ch.hpp>
#include <hal.h>
#include <zubax_chibios/sys/sys.h>

/**
 * GPIO config for ChibiOS PAL driver
 */
const PALConfig pal_default_config =
{
    { VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH },
    { VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH },
    { VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH },
    { VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH },
    { VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH }
};

namespace board
{

void init()
{
    halInit();

    chibios_rt::System::init();

    sdStart(&STDOUT_SD, nullptr);
}

__attribute__((noreturn))
void die(int error)
{
    lowsyslog("Fatal error %i\n", error);
    while (1)
    {
        setLed(false);
        ::sleep(1);
        setLed(true);
        ::sleep(1);
    }
}

void setLed(bool state)
{
    palWritePad(GPIO_PORT_LED, GPIO_PIN_LED, state);
}

void restart()
{
    NVIC_SystemReset();
}

void readUniqueID(std::uint8_t bytes[UniqueIDSize])
{
    std::memcpy(bytes, reinterpret_cast<const void*>(0x1FFFF7E8), UniqueIDSize);
}

}

/*
 * Early init from ChibiOS
 */
extern "C"
{

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

    /*
     * Enabling the CAN controllers, then configuring GPIO functions for CAN_TX.
     * Order matters, otherwise the CAN_TX pins will twitch, disturbing the CAN bus.
     * This is why we can't perform this initialization using ChibiOS GPIO configuration.
     */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    palSetPadMode(GPIOD, 1, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

#if UAVCAN_STM32_NUM_IFACES > 1
    RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
    palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
#endif
}

}
