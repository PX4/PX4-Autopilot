/* File: startup_ARMCM0.S
 * Purpose: startup file for Cortex-M0 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.3
 * Date: 08 Feb 2012
 *
 * Copyright (c) 2012, ARM Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the ARM Limited nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
    .syntax unified
    .arch armv6-m

#ifdef NXP_CRP
    .section .NXP.CRP
    .align 2
    .globl __nxp_crp
__nxp_crp:
    .long NXP_CRP
    .size __nxp_crp, . - __nxp_crp
#endif

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0x400
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0xC00
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
#if defined(CHIP_LPC11AXX)
#define HAVE_LPC_IRQ_HANDLERS
    .long    PIN_INT0_IRQHandler           // 16+ 0: Pin interrupt
    .long    PIN_INT1_IRQHandler           // 16+ 1: Pin interrupt
    .long    PIN_INT2_IRQHandler           // 16+ 2: Pin interrupt
    .long    PIN_INT3_IRQHandler           // 16+ 3: Pin interrupt
    .long    PIN_INT4_IRQHandler           // 16+ 4: Pin interrupt
    .long    PIN_INT5_IRQHandler           // 16+ 5: Pin interrupt
    .long    PIN_INT6_IRQHandler           // 16+ 6: Pin interrupt
    .long    PIN_INT7_IRQHandler           // 16+ 7: Pin interrupt
    .long    GINT0_IRQHandler              // 16+ 8: Port interrupt
    .long    GINT1_IRQHandler              // 16+ 9: Port interrupt
    .long    ACMP_IRQHandler               // 16+10: Analog Comparator
    .long    DAC_IRQHandler                // 16+11: D/A Converter
    .long    0                             // 16+12: Reserved
    .long    0                             // 16+13: Reserved
    .long    SSP1_IRQHandler               // 16+14: SSP1
    .long    I2C_IRQHandler                // 16+15: I2C
    .long    TIMER_16_0_IRQHandler         // 16+16: 16-bit Timer0
    .long    TIMER_16_1_IRQHandler         // 16+17: 16-bit Timer1
    .long    TIMER_32_0_IRQHandler         // 16+18: 32-bit Timer0
    .long    TIMER_32_1_IRQHandler         // 16+19: 32-bit Timer1
    .long    SSP0_IRQHandler               // 16+20: SSP0
    .long    USART_IRQHandler              // 16+21: USART
    .long    0                             // 16+22: Reserved
    .long    0                             // 16+23: Reserved
    .long    ADC_IRQHandler                // 16+24: A/D Converter
    .long    WDT_IRQHandler                // 16+25: Watchdog Timer
    .long    BOD_IRQHandler                // 16+26: Brown Out Detect
    .long    FMC_IRQHandler                // 16+27: IP2111 Flash Memory Controller
    .long    0                             // 16+28: Reserved
    .long    0                             // 16+29: Reserved
    .long    0                             // 16+30: Reserved
    .long    0                             // 16+31: Reserved
#endif

#if defined(CHIP_LPC11CXX)
#define HAVE_LPC_IRQ_HANDLERS
    .long    WAKEUP_IRQHandler             // PIO0_0  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_1  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_2  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_3  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_4  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_5  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_6  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_7  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_8  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_9  Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_10 Wakeup
    .long    WAKEUP_IRQHandler             // PIO0_11 Wakeup
    .long    WAKEUP_IRQHandler             // PIO1_0  Wakeup
    .long    CAN_IRQHandler                // C_CAN Interrupt
    .long    SSP1_IRQHandler               // SPI/SSP1 Interrupt
    .long    I2C_IRQHandler                // I2C0
    .long    TIMER_16_0_IRQHandler         // CT16B0 (16-bit Timer 0)
    .long    TIMER_16_1_IRQHandler         // CT16B1 (16-bit Timer 1)
    .long    TIMER_32_0_IRQHandler         // CT32B0 (32-bit Timer 0)
    .long    TIMER_32_1_IRQHandler         // CT32B1 (32-bit Timer 1)
    .long    SSP0_IRQHandler               // SPI/SSP0 Interrupt
    .long    UART_IRQHandler               // UART0
    .long    0                             // Reserved
    .long    0                             // Reserved
    .long    ADC_IRQHandler                // ADC   (A/D Converter)
    .long    WDT_IRQHandler                // WDT   (Watchdog Timer)
    .long    BOD_IRQHandler                // BOD   (Brownout Detect)
    .long    0                             // Reserved
    .long    PIOINT3_IRQHandler            // PIO INT3
    .long    PIOINT2_IRQHandler            // PIO INT2
    .long    PIOINT1_IRQHandler            // PIO INT1
    .long    PIOINT0_IRQHandler            // PIO INT0
#endif

#if defined(CHIP_LPC11UXX)
#define HAVE_LPC_IRQ_HANDLERS
    .long    FLEX_INT0_IRQHandler             //  0 - GPIO pin interrupt 0
    .long    FLEX_INT1_IRQHandler             //  1 - GPIO pin interrupt 1
    .long    FLEX_INT2_IRQHandler             //  2 - GPIO pin interrupt 2
    .long    FLEX_INT3_IRQHandler             //  3 - GPIO pin interrupt 3
    .long    FLEX_INT4_IRQHandler             //  4 - GPIO pin interrupt 4
    .long    FLEX_INT5_IRQHandler             //  5 - GPIO pin interrupt 5
    .long    FLEX_INT6_IRQHandler             //  6 - GPIO pin interrupt 6
    .long    FLEX_INT7_IRQHandler             //  7 - GPIO pin interrupt 7
    .long    GINT0_IRQHandler                 //  8 - GPIO GROUP0 interrupt
    .long    GINT1_IRQHandler                 //  9 - GPIO GROUP1 interrupt
    .long    0                                // 10 - Reserved
    .long    0                                // 11 - Reserved
    .long    0                                // 12 - Reserved
    .long    0                                // 13 - Reserved
    .long    SSP1_IRQHandler                  // 14 - SPI/SSP1 Interrupt
    .long    I2C_IRQHandler                   // 15 - I2C0
    .long    TIMER_16_0_IRQHandler            // 16 - CT16B0 (16-bit Timer 0)
    .long    TIMER_16_1_IRQHandler            // 17 - CT16B1 (16-bit Timer 1)
    .long    TIMER_32_0_IRQHandler            // 18 - CT32B0 (32-bit Timer 0)
    .long    TIMER_32_1_IRQHandler            // 19 - CT32B1 (32-bit Timer 1)
    .long    SSP0_IRQHandler                  // 20 - SPI/SSP0 Interrupt
    .long    UART_IRQHandler                  // 21 - UART0
    .long    USB_IRQHandler                   // 22 - USB IRQ
    .long    USB_FIQHandler                   // 23 - USB FIQ
    .long    ADC_IRQHandler                   // 24 - ADC (A/D Converter)
    .long    WDT_IRQHandler                   // 25 - WDT (Watchdog Timer)
    .long    BOD_IRQHandler                   // 26 - BOD (Brownout Detect)
    .long    FMC_IRQHandler                   // 27 - IP2111 Flash Memory Controller
    .long    0                                // 28 - Reserved
    .long    0                                // 29 - Reserved
    .long    USBWakeup_IRQHandler             // 30 - USB wake-up interrupt
    .long    0                                // 31 - Reserved
#endif

#ifndef HAVE_LPC_IRQ_HANDLERS
#error "No LPC IRQ handlers defined for this chip type."
#endif

    .size    __isr_vector, . - __isr_vector

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble    .flash_to_ram_loop_end

    movs    r4, 0
.flash_to_ram_loop:
    ldr    r0, [r1,r4]
    str    r0, [r2,r4]
    adds    r4, 4
    cmp    r4, r3
    blt    .flash_to_ram_loop
.flash_to_ram_loop_end:

    ldr    r0, =__start
    bx     r0
    .pool
    .size Reset_Handler, . - Reset_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    Default_Handler

#if defined(CHIP_LPC11AXX)
    def_irq_handler    PIN_INT0_IRQHandler
    def_irq_handler    PIN_INT1_IRQHandler
    def_irq_handler    PIN_INT2_IRQHandler
    def_irq_handler    PIN_INT3_IRQHandler
    def_irq_handler    PIN_INT4_IRQHandler
    def_irq_handler    PIN_INT5_IRQHandler
    def_irq_handler    PIN_INT6_IRQHandler
    def_irq_handler    PIN_INT7_IRQHandler
    def_irq_handler    GINT0_IRQHandler
    def_irq_handler    GINT1_IRQHandler
    def_irq_handler    ACMP_IRQHandler
    def_irq_handler    DAC_IRQHandler
    def_irq_handler    SSP1_IRQHandler
    def_irq_handler    I2C_IRQHandler
    def_irq_handler    TIMER_16_0_IRQHandler
    def_irq_handler    TIMER_16_1_IRQHandler
    def_irq_handler    TIMER_32_0_IRQHandler
    def_irq_handler    TIMER_32_1_IRQHandler
    def_irq_handler    SSP0_IRQHandler
    def_irq_handler    USART_IRQHandler
    def_irq_handler    ADC_IRQHandler
    def_irq_handler    WDT_IRQHandler
    def_irq_handler    BOD_IRQHandler
    def_irq_handler    FMC_IRQHandler
#elif defined(CHIP_LPC11CXX)
    def_irq_handler    WAKEUP_IRQHandler
    def_irq_handler    CAN_IRQHandler
    def_irq_handler    SSP1_IRQHandler
    def_irq_handler    I2C_IRQHandler
    def_irq_handler    TIMER_16_0_IRQHandler
    def_irq_handler    TIMER_16_1_IRQHandler
    def_irq_handler    TIMER_32_0_IRQHandler
    def_irq_handler    TIMER_32_1_IRQHandler
    def_irq_handler    SSP0_IRQHandler
    def_irq_handler    UART_IRQHandler
    def_irq_handler    ADC_IRQHandler
    def_irq_handler    WDT_IRQHandler
    def_irq_handler    BOD_IRQHandler
    def_irq_handler    PIOINT3_IRQHandler
    def_irq_handler    PIOINT2_IRQHandler
    def_irq_handler    PIOINT1_IRQHandler
    def_irq_handler    PIOINT0_IRQHandler
#elif defined(CHIP_LPC11UXX)
    def_irq_handler    FLEX_INT0_IRQHandler
    def_irq_handler    FLEX_INT1_IRQHandler
    def_irq_handler    FLEX_INT2_IRQHandler
    def_irq_handler    FLEX_INT3_IRQHandler
    def_irq_handler    FLEX_INT4_IRQHandler
    def_irq_handler    FLEX_INT5_IRQHandler
    def_irq_handler    FLEX_INT6_IRQHandler
    def_irq_handler    FLEX_INT7_IRQHandler
    def_irq_handler    GINT0_IRQHandler
    def_irq_handler    GINT1_IRQHandler
    def_irq_handler    SSP1_IRQHandler
    def_irq_handler    I2C_IRQHandler
    def_irq_handler    TIMER_16_0_IRQHandler
    def_irq_handler    TIMER_16_1_IRQHandler
    def_irq_handler    TIMER_32_0_IRQHandler
    def_irq_handler    TIMER_32_1_IRQHandler
    def_irq_handler    SSP0_IRQHandler
    def_irq_handler    UART_IRQHandler
    def_irq_handler    USB_IRQHandler
    def_irq_handler    USB_FIQHandler
    def_irq_handler    ADC_IRQHandler
    def_irq_handler    WDT_IRQHandler
    def_irq_handler    BOD_IRQHandler
    def_irq_handler    FMC_IRQHandler
    def_irq_handler    USBWakeup_IRQHandler
#endif

    .end
