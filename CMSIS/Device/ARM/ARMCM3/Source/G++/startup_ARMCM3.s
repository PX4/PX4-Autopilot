/**************************************************************************//**
 * @file     startup_ARMCM3.s
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM3 Device Series
 * @version  V1.07
 * @date     30. January 2012
 *
 * @note     Version CodeSourcery Sourcery G++ Lite (with CS3)
 * Copyright (C) 2012 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/


/*
// <h> Stack Configuration
//   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Stack_Size, 0x00000400
    .section ".stack", "w"
    .align  3
    .globl  __cs3_stack_mem
    .globl  __cs3_stack_size
__cs3_stack_mem:
    .if     Stack_Size
    .space  Stack_Size
    .endif
    .size   __cs3_stack_mem,  . - __cs3_stack_mem
    .set    __cs3_stack_size, . - __cs3_stack_mem


/*
// <h> Heap Configuration
//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Heap_Size,  0x00000C00
    
    .section ".heap", "w"
    .align  3
    .globl  __cs3_heap_start
    .globl  __cs3_heap_end
__cs3_heap_start:
    .if     Heap_Size
    .space  Heap_Size
    .endif
__cs3_heap_end:


/* Vector Table */

    .section ".cs3.interrupt_vector"
    .globl  __cs3_interrupt_vector_cortex_m
    .type   __cs3_interrupt_vector_cortex_m, %object

__cs3_interrupt_vector_cortex_m:
    .long   __cs3_stack                 /* Top of Stack                 */
    .long   __cs3_reset                 /* Reset Handler                */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   MemManage_Handler           /* MPU Fault Handler            */
    .long   BusFault_Handler            /* Bus Fault Handler            */
    .long   UsageFault_Handler          /* Usage Fault Handler          */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   DebugMon_Handler            /* Debug Monitor Handler        */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    /* External Interrupts */
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

    .size   __cs3_interrupt_vector_cortex_m, . - __cs3_interrupt_vector_cortex_m


    .thumb


/* Reset Handler */

    .section .cs3.reset,"x",%progbits
    .thumb_func
    .globl  __cs3_reset_cortex_m
    .type   __cs3_reset_cortex_m, %function
__cs3_reset_cortex_m:
    .fnstart
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0,=_start
    BX      R0
    .pool
    .cantunwind
    .fnend
    .size   __cs3_reset_cortex_m,.-__cs3_reset_cortex_m

    .section ".text"

/* Exception Handlers */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler

    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler

    .weak   MemManage_Handler
    .type   MemManage_Handler, %function
MemManage_Handler:
    B       .
    .size   MemManage_Handler, . - MemManage_Handler

    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    B       .
    .size   BusFault_Handler, . - BusFault_Handler

    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    B       .
    .size   UsageFault_Handler, . - UsageFault_Handler

    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler

    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    B       .
    .size   DebugMon_Handler, . - DebugMon_Handler

    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler

    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  def_irq_handler handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

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
