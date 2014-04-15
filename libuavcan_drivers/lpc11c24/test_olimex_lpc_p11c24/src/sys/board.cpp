/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 * Board initialization for Olimex LPC11C24
 */

#include "board.hpp"
#include <chip.h>
#include <cstdlib>

#define PDRUNCFGUSEMASK 0x0000ED00
#define PDRUNCFGMASKTMP 0x000000FF

const uint32_t OscRateIn = 12000000; ///< External crystal
const uint32_t ExtRateIn = 0;

uint32_t SystemCoreClock = 12000000; ///< Initialized to default clock value, will be changed on init

namespace board
{
namespace
{

const unsigned ErrorLedPort = 1;
const unsigned ErrorLedPin  = 10;

const unsigned StatusLedPort = 1;
const unsigned StatusLedPin  = 11;

struct PinMuxGroup
{
    unsigned pin      : 8;
    unsigned modefunc : 24;
};

const PinMuxGroup pinmux[] =
{
    { IOCON_PIO1_10, IOCON_FUNC0 | IOCON_MODE_INACT }, // Error LED
    { IOCON_PIO1_11, IOCON_FUNC0 | IOCON_MODE_INACT }  // Status LED
};


void sysctlPowerDown(unsigned long powerdownmask)
{
    unsigned long pdrun = LPC_SYSCTL->PDRUNCFG & PDRUNCFGMASKTMP;
    pdrun |= (powerdownmask & PDRUNCFGMASKTMP);
    LPC_SYSCTL->PDRUNCFG = pdrun | PDRUNCFGUSEMASK;
}

void sysctlPowerUp(unsigned long powerupmask)
{
    unsigned long pdrun = LPC_SYSCTL->PDRUNCFG & PDRUNCFGMASKTMP;
    pdrun &= ~(powerupmask & PDRUNCFGMASKTMP);
    LPC_SYSCTL->PDRUNCFG = pdrun | PDRUNCFGUSEMASK;
}

void initWatchdog()
{
    sysctlPowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);   // Enable watchdog oscillator
    // TODO: init watchdog
}

void initClock()
{
    sysctlPowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);   // Enable system oscillator
    for (volatile int i = 0; i < 1000; i++) { }

    Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
    sysctlPowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

    /*
     * Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
     * MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
     * FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
     * FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range)
     */
    Chip_Clock_SetupSystemPLL(3, 1);
    sysctlPowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);
    while (!Chip_Clock_IsSystemPLLLocked()) { }

    Chip_Clock_SetSysClockDiv(1);

    Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

    SystemCoreClock = Chip_Clock_GetSystemClockRate();

    while (SystemCoreClock != 48000000) { }  // Loop forever if the clock failed to initialize properly
}

void initGpio()
{
    LPC_SYSCTL->SYSAHBCLKCTRL |= 1 << SYSCTL_CLOCK_IOCON;
    LPC_SYSCTL->SYSAHBCLKCTRL |= 1 << SYSCTL_CLOCK_GPIO;

    for (unsigned i = 0; i < (sizeof(pinmux) / sizeof(PinMuxGroup)); i++)
    {
        LPC_IOCON->REG[pinmux[i].pin] = pinmux[i].modefunc;
    }

    LPC_GPIO[ErrorLedPort].DIR  |= 1 << ErrorLedPin;
    LPC_GPIO[StatusLedPort].DIR |= 1 << StatusLedPin;
}

void init()
{
    Chip_SYSCTL_SetBODLevels(SYSCTL_BODRSTLVL_2_06V, SYSCTL_BODINTVAL_RESERVED1);
    Chip_SYSCTL_EnableBODReset();

    initWatchdog();
    initClock();
    initGpio();
}

} // namespace

void setStatusLed(bool state)
{
    LPC_GPIO[StatusLedPort].DATA[1 << StatusLedPin] = static_cast<unsigned long>(!state) << StatusLedPin;
}

void setErrorLed(bool state)
{
    LPC_GPIO[ErrorLedPort].DATA[1 << ErrorLedPin] = static_cast<unsigned long>(!state) << ErrorLedPin;
}

} // namespace board

extern "C"
{

void SystemInit()
{
    board::init();
}

}
