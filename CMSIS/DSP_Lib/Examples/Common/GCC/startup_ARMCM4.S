/* File: startup_ARMCM4.S
 * Purpose: startup file for Cortex-M4 devices. Should use with
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
    .arch armv7-m

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
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long    WDT_IRQHandler        /*  0:  Watchdog Timer            */
    .long    RTC_IRQHandler        /*  1:  Real Time Clock           */
    .long    TIM0_IRQHandler       /*  2:  Timer0 / Timer1           */
    .long    TIM2_IRQHandler       /*  3:  Timer2 / Timer3           */
    .long    MCIA_IRQHandler       /*  4:  MCIa                      */
    .long    MCIB_IRQHandler       /*  5:  MCIb                      */
    .long    UART0_IRQHandler      /*  6:  UART0 - DUT FPGA          */
    .long    UART1_IRQHandler      /*  7:  UART1 - DUT FPGA          */
    .long    UART2_IRQHandler      /*  8:  UART2 - DUT FPGA          */
    .long    UART4_IRQHandler      /*  9:  UART4 - not connected     */
    .long    AACI_IRQHandler       /* 10: AACI / AC97                */
    .long    CLCD_IRQHandler       /* 11: CLCD Combined Interrupt    */
    .long    ENET_IRQHandler       /* 12: Ethernet                   */
    .long    USBDC_IRQHandler      /* 13: USB Device                 */
    .long    USBHC_IRQHandler      /* 14: USB Host Controller        */
    .long    CHLCD_IRQHandler      /* 15: Character LCD              */
    .long    FLEXRAY_IRQHandler    /* 16: Flexray                    */
    .long    CAN_IRQHandler        /* 17: CAN                        */
    .long    LIN_IRQHandler        /* 18: LIN                        */
    .long    I2C_IRQHandler        /* 19: I2C ADC/DAC                */
    .long    0                     /* 20: Reserved                   */
    .long    0                     /* 21: Reserved                   */
    .long    0                     /* 22: Reserved                   */
    .long    0                     /* 23: Reserved                   */
    .long    0                     /* 24: Reserved                   */
    .long    0                     /* 25: Reserved                   */
    .long    0                     /* 26: Reserved                   */
    .long    0                     /* 27: Reserved                   */
    .long    CPU_CLCD_IRQHandler   /* 28: Reserved - CPU FPGA CLCD   */
    .long    0                     /* 29: Reserved - CPU FPGA        */
    .long    UART3_IRQHandler      /* 30: UART3    - CPU FPGA        */
    .long    SPI_IRQHandler        /* 31: SPI Touchscreen - CPU FPGA */

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

#if 1
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
.flash_to_ram_loop:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .flash_to_ram_loop
#else
    subs    r3, r2
    ble    .flash_to_ram_loop_end
.flash_to_ram_loop:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .flash_to_ram_loop
.flash_to_ram_loop_end:
#endif

#ifndef __NO_SYSTEM_INIT
    ldr    r0, =SystemInit
    blx    r0
#endif

    ldr    r0, =_start
    bx    r0
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
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    Default_Handler

    def_irq_handler    WDT_IRQHandler
    def_irq_handler    RTC_IRQHandler
    def_irq_handler    TIM0_IRQHandler
    def_irq_handler    TIM2_IRQHandler
    def_irq_handler    MCIA_IRQHandler
    def_irq_handler    MCIB_IRQHandler
    def_irq_handler    UART0_IRQHandler
    def_irq_handler    UART1_IRQHandler
    def_irq_handler    UART2_IRQHandler
    def_irq_handler    UART3_IRQHandler
    def_irq_handler    UART4_IRQHandler
    def_irq_handler    AACI_IRQHandler
    def_irq_handler    CLCD_IRQHandler
    def_irq_handler    ENET_IRQHandler
    def_irq_handler    USBDC_IRQHandler
    def_irq_handler    USBHC_IRQHandler
    def_irq_handler    CHLCD_IRQHandler
    def_irq_handler    FLEXRAY_IRQHandler
    def_irq_handler    CAN_IRQHandler
    def_irq_handler    LIN_IRQHandler
    def_irq_handler    I2C_IRQHandler
    def_irq_handler    CPU_CLCD_IRQHandler
    def_irq_handler    SPI_IRQHandler

    .end
