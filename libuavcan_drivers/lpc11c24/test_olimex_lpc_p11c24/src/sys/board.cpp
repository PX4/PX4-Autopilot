/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 * Board initialization for Olimex LPC11C24
 */

#include "board.hpp"
#include <chip.h>
#include <cstdlib>
#include <cstring>
#include <numeric>

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
    Chip_WWDT_Init(LPC_WWDT);                                   // Initialize watchdog
    sysctlPowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);                  // Enable watchdog oscillator
    Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 4);                   // WDT osc rate 0.6 MHz / 4 = 150 kHz
    Chip_Clock_SetWDTClockSource(SYSCTL_WDTCLKSRC_WDTOSC, 1);   // Clocking watchdog from its osc, div rate 1
    Chip_WWDT_SetTimeOut(LPC_WWDT, 37500);                      // 1 sec (hardcoded to reduce code size)
    Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);          // Mode: reset on timeout
    Chip_WWDT_Start(LPC_WWDT);                                  // Go
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

    resetWatchdog();
}

} // namespace

#if __GNUC__
__attribute__((optimize(0)))     // Optimization must be disabled lest it hardfaults in the IAP call
#endif
void readUniqueID(uint8_t out_uid[UniqueIDSize])
{
    unsigned aligned_array[4] = {};  // out_uid may be unaligned, so we need to use temp array
    unsigned iap_command = 58;
    reinterpret_cast<void(*)(void*, void*)>(0x1FFF1FF1)(&iap_command, aligned_array);
    std::memcpy(out_uid, aligned_array, 16);
}

void setStatusLed(bool state)
{
    LPC_GPIO[StatusLedPort].DATA[1 << StatusLedPin] = static_cast<unsigned long>(!state) << StatusLedPin;
}

void setErrorLed(bool state)
{
    LPC_GPIO[ErrorLedPort].DATA[1 << ErrorLedPin] = static_cast<unsigned long>(!state) << ErrorLedPin;
}

void resetWatchdog()
{
    Chip_WWDT_Feed(LPC_WWDT);
}

} // namespace board

extern "C"
{

void SystemInit();

void SystemInit()
{
    board::init();
}

}
