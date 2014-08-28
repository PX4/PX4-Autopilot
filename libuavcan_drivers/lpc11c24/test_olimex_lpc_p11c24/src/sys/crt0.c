/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 * ARM Cortex-M0(+)/M1/M3 startup file.
 */

typedef void (*funptr_t)(void);

#define fill32(start, end, filler) {   \
    unsigned *p1 = start;              \
    const unsigned * const p2 = end;   \
    while (p1 < p2)                    \
        *p1++ = filler;                \
}

extern const unsigned _etext;

extern unsigned _data;
extern unsigned _edata;

extern unsigned _bss;
extern unsigned _ebss;

extern funptr_t __init_array_start;
extern funptr_t __init_array_end;

__attribute__((noreturn))
extern int main(void);

extern void SystemInit(void);

#pragma GCC optimize 1

/**
 * Prototypes for the functions below
 */
void Reset_Handler(void);
void Default_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/**
 * Firmware entry point
 */
__attribute__((naked, noreturn))
void Reset_Handler(void)
{
    // Data section
    {
        const unsigned* tp = &_etext;
        unsigned* dp = &_data;
        while (dp < &_edata)
        {
            *dp++ = *tp++;
        }
    }

    // BSS section
    fill32(&_bss, &_ebss, 0);

    SystemInit();

    // Constructors
    {
        funptr_t* fpp = &__init_array_start;
        while (fpp < &__init_array_end)
        {
            (*fpp)();
            fpp++;
        }
    }

    (void)main();

    while (1) { }
}

/**
 * Default handlers
 */
__attribute__((weak))
void Default_Handler(void)
{
    while(1) { }
}

__attribute__((weak))
void NMI_Handler(void)
{
    while(1) { }
}

__attribute__((weak))
void HardFault_Handler(void)
{
    while(1) { }
}

__attribute__((weak))
void SVC_Handler(void)
{
    while(1) { }
}

__attribute__((weak))
void PendSV_Handler(void)
{
    while(1) { }
}

__attribute__((weak))
void SysTick_Handler(void)
{
    while(1) { }
}

/**
 * Default vectors for LPC11C24, to be overriden by the firmware as needed
 */
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

void CAN_IRQHandler(void) ALIAS(Default_Handler);
void SSP1_IRQHandler(void) ALIAS(Default_Handler);
void I2C_IRQHandler(void) ALIAS(Default_Handler);
void TIMER16_0_IRQHandler(void) ALIAS(Default_Handler);
void TIMER16_1_IRQHandler(void) ALIAS(Default_Handler);
void TIMER32_0_IRQHandler(void) ALIAS(Default_Handler);
void TIMER32_1_IRQHandler(void) ALIAS(Default_Handler);
void SSP0_IRQHandler(void) ALIAS(Default_Handler);
void UART_IRQHandler(void) ALIAS(Default_Handler);
void ADC_IRQHandler(void) ALIAS(Default_Handler);
void WDT_IRQHandler(void) ALIAS(Default_Handler);
void BOD_IRQHandler(void) ALIAS(Default_Handler);
void PIOINT3_IRQHandler(void) ALIAS(Default_Handler);
void PIOINT2_IRQHandler(void) ALIAS(Default_Handler);
void PIOINT1_IRQHandler(void) ALIAS(Default_Handler);
void PIOINT0_IRQHandler(void) ALIAS(Default_Handler);
void WAKEUP_IRQHandler(void) ALIAS(Default_Handler);

/**
 * Refer to the linker script
 */
extern void __stack_end(void);

/**
 * Vector table for LPC11Cxx
 * Must be explicitly defined 'used', otherwise LTO optimizer will discard it.
 */
__attribute__ ((used, section("vectors")))
void (* const VectorTable[64])(void) =
{
    __stack_end,                            // The initial stack pointer
    Reset_Handler,                          // The reset handler
    NMI_Handler,                            // The NMI handler
    HardFault_Handler,                      // The hard fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    SVC_Handler,                            // SVCall handler
    0,                                      // Reserved
    0,                                      // Reserved
    PendSV_Handler,                         // The PendSV handler
    SysTick_Handler,                        // The SysTick handler

    WAKEUP_IRQHandler,                      // PIO0_0  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_1  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_2  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_3  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_4  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_5  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_6  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_7  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_8  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_9  Wakeup
    WAKEUP_IRQHandler,                      // PIO0_10 Wakeup
    WAKEUP_IRQHandler,                      // PIO0_11 Wakeup
    WAKEUP_IRQHandler,                      // PIO1_0  Wakeup

    CAN_IRQHandler,                         // C_CAN Interrupt
    SSP1_IRQHandler,                        // SPI/SSP1 Interrupt
    I2C_IRQHandler,                         // I2C0
    TIMER16_0_IRQHandler,                   // CT16B0 (16-bit Timer 0)
    TIMER16_1_IRQHandler,                   // CT16B1 (16-bit Timer 1)
    TIMER32_0_IRQHandler,                   // CT32B0 (32-bit Timer 0)
    TIMER32_1_IRQHandler,                   // CT32B1 (32-bit Timer 1)
    SSP0_IRQHandler,                        // SPI/SSP0 Interrupt
    UART_IRQHandler,                        // UART0

    0,                                      // Reserved
    0,                                      // Reserved

    ADC_IRQHandler,                         // ADC   (A/D Converter)
    WDT_IRQHandler,                         // WDT   (Watchdog Timer)
    BOD_IRQHandler,                         // BOD   (Brownout Detect)
    0,                                      // Reserved
    PIOINT3_IRQHandler,                     // PIO INT3
    PIOINT2_IRQHandler,                     // PIO INT2
    PIOINT1_IRQHandler,                     // PIO INT1
    PIOINT0_IRQHandler,                     // PIO INT0

    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
};
