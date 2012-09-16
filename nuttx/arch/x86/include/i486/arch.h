/****************************************************************************
 * arch/x86/include/i486/arch.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_X86_INCLUDE_I486_ARCH_H
#define __ARCH_X86_INCLUDE_I486_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* FLAGS bits */

#define X86_FLAGS_CF             (1 << 0)  /* Bit 0:  Carry Flag */
                                           /* Bit 1:  Reserved */
#define X86_FLAGS_PF             (1 << 2)  /* Bit 2:  Parity Flag */
                                           /* Bit 3:  Reserved */
#define X86_FLAGS_AF             (1 << 4)  /* Bit 4:  Auxillary carry Flag */
                                           /* Bit 5:  Reserved */
#define X86_FLAGS_ZF             (1 << 6)  /* Bit 6:  Zero Flag */
#define X86_FLAGS_SF             (1 << 7)  /* Bit 7:  Sign Flag */
#define X86_FLAGS_TF             (1 << 8)  /* Bit 8:  Trap Flag */
#define X86_FLAGS_IF             (1 << 9)  /* Bit 9:  Interrupt Flag */
#define X86_FLAGS_DF             (1 << 10) /* Bit 10: Direction Flag */
#define X86_FLAGS_OF             (1 << 11) /* Bit 11: Overflow Flag */
#define X86_FLAGS_IOPL_SHIFT     (12)      /* Bits 12-13: IOPL mask (286+ only)*/
#define X86_FLAGS_IOPL_MASK      (3 << X86_FLAGS_IOPL_SHIFT)
#define X86_FLAGS_NT             (1 << 14) /* Bit 14: Nested Task */
                                           /* Bit 15: Reserved */

/* EFLAGS bits (Extend the basic FLAGS bit definitions) */

#define X86_EFLAGS_RF            (1 << 16) /* Bit 16: Resume Flag (386+ only) */
#define X86_EFLAGS_VM            (1 << 17) /* Bit 17: Virtual Mode (386+ only) */
#define X86_EFLAGS_AC            (1 << 18) /* Bit 18: Alignment Check (486SX+ only) */
#define X86_EFLAGS_VIF           (1 << 19) /* Bit 19: Virtual Interrupt Flag (Pentium+) */
#define X86_EFLAGS_VIP           (1 << 20) /* Bit 20: Virtual Interrupt Pending (Pentium+) */
#define X86_EFLAGS_ID            (1 << 21) /* Bit 21: CPUID detection flag (Pentium+) */

/* Programmable Interrupt Controller (PIC) */

/* Operational Control Words
 *
 * The first instruction the Operation Control Word 1 (OCW1) to set which
 * IRQ's to mask and which IRQ's not to.
 */

#define PIC1_OCW1               0x20
#define PIC2_OCW1               0xa0

#  define PIC1_OCW1_IRQ0         (1 << 0) /* IRQ0  System Timer */
#  define PIC1_OCW1_IRQ1         (1 << 1) /* IRQ1  Keyboard */
#  define PIC1_OCW1_IRQ2         (1 << 2) /* IRQ2  PIC2 */
#  define PIC1_OCW1_IRQ3         (1 << 3) /* IRQ3  Serial Port */
#  define PIC1_OCW1_IRQ4         (1 << 4) /* IRQ4  Serial Port */
#  define PIC1_OCW1_IRQ5         (1 << 5) /* IRQ5  Reserved/Sound Card */
#  define PIC1_OCW1_IRQ6         (1 << 6) /* IRQ6  Floppy Disk Controller */
#  define PIC1_OCW1_IRQ7         (1 << 7) /* IRQ7  Parallel Port */
#  define PIC1_OCW1_ALL

#  define PIC2_OCW1_IRQ8         (1 << 0) /* IRQ8  Real Time Clock */
#  define PIC2_OCW1_IRQ9         (1 << 1) /* IRQ9  Redirected IRQ2 */
#  define PIC2_OCW1_IRQ10        (1 << 2) /* IRQ10 Reserved */
#  define PIC2_OCW1_IRQ11        (1 << 3) /* IRQ11 Reserved */
#  define PIC2_OCW1_IRQ12        (1 << 4) /* IRQ12 PS/2 Mouse */
#  define PIC2_OCW1_IRQ13        (1 << 5) /* IRQ13 Maths Co-Processor */
#  define PIC2_OCW1_IRQ14        (1 << 6) /* IRQ14 Hard Disk Drive */
#  define PIC2_OCW1_IRQ15        (1 << 7) /* IRQ15 Reserved */
#  define PIC2_OCW1_ALL

/* Operation Control Word 2 selects how the End of Interrupt (EOI) procedure
 * works. The only thing of interest to us in this register is the non-
 * specific EOI command, which we must send at the end of our ISR's.
 */

#define PIC1_OCW2                0x20
#define PIC2_OCW2                0xa0

#  define PIC_OCW2_ACT_SHIFT     (0)
#  define PIC_OCW2_ACT_MASK      (7 << PIC_OCW2_ACT_SHIFT)
#    define PIC1_OCW2_ACT_IRQ0   (0 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 0 */
#    define PIC1_OCW2_ACT_IRQ1   (1 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 1 */
#    define PIC1_OCW2_ACT_IRQ2   (2 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 2 */
#    define PIC1_OCW2_ACT_IRQ3   (3 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 3 */
#    define PIC1_OCW2_ACT_IRQ4   (4 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 4 */
#    define PIC1_OCW2_ACT_IRQ5   (5 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 5 */
#    define PIC1_OCW2_ACT_IRQ6   (6 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 6 */
#    define PIC1_OCW2_ACT_IRQ7   (7 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 7 */

#    define PIC2_OCW2_ACT_IRQ8   (0 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 8 */
#    define PIC2_OCW2_ACT_IRQ9   (1 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 9 */
#    define PIC2_OCW2_ACT_IRQ10  (2 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 10 */
#    define PIC2_OCW2_ACT_IRQ11  (3 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 11 */
#    define PIC2_OCW2_ACT_IRQ12  (4 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 12 */
#    define PIC2_OCW2_ACT_IRQ13  (5 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 13 */
#    define PIC2_OCW2_ACT_IRQ14  (6 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 14 */
#    define PIC2_OCW2_ACT_IRQ15  (7 << PIC_OCW2_ACT_SHIFT) /* Act on IRQ 15 */

#  define PIC_OCW2_EOI_SHIFT     (5)
#  define PIC_OCW2_EOI_MASK      (7 << PIC_OCW2_EOI_SHIFT)
#    define PIC_OCW2_EOI_AUTO    (0 << PIC_OCW2_EOI_SHIFT) /* Rotate in Auto EOI Mode (Clear) */
#    define PIC_OCW2_EOI_NONSPEC (1 << PIC_OCW2_EOI_SHIFT) /* Non Specific EOI */
#    define PIC_OCW2_EOI_SPEC    (3 << PIC_OCW2_EOI_SHIFT) /* Specific EOI */
#    define PIC_OCW2_EOI_RAUTO   (4 << PIC_OCW2_EOI_SHIFT) /* Rotate in Auto EOI Mode (Set) */
#    define PIC_OCW2_EOI_RNSPEC  (5 << PIC_OCW2_EOI_SHIFT) /* Rotate on Non-Specific EOI */
#    define PIC_OCW2_EOI_PRIO    (6 << PIC_OCW2_EOI_SHIFT) /* Set Priority Command (Use Bits 2:0) */
#    define PIC_OCW2_EOI_RSPEC   (7 << PIC_OCW2_EOI_SHIFT) /* Rotate on Specific EOI (Use Bits 2:0) */

/* Operation Control Word 3.  Bits 0 and 1 bitsenable us to read the status
 * of the Interrupt Request Register (IRR) and the In-Service Register (ISR).
 * This is done by setting the appropriate bits correctly and reading the
 * register at the Base Address.
 *
 * For example if we wanted to read the In-Service Register (ISR), then we
 * would set both bits 1 and 0 to 1. The next read to the base register,
 * (0x20 for PIC1 or 0xa0 for PIC2) will return the status of the In-Service
 * Register. 
 */

#define PIC1_OCW3                0x20
#define PIC2_OCW3                0xa0

#  define PIC_OCW3_PCMD_SHIFT    (0)      /* Poll command */
#  define PIC_OCW3_PCMD_MASK     (3 << PIC_OCW3_PCMD_SHIFT)
#    define PIC_OCW3_PCMD_IRR    (2 << PIC_OCW3_PCMD_SHIFT) /* Next Read Returns Interrupt Request Register */
#    define PIC_OCW3_PCMD_ISR    (3 << PIC_OCW3_PCMD_SHIFT) /* Next Read Returns In-Service Register */
#  define PIC_OCW3_POLLCMD       (1 << 2) /* Poll command */
#  define PIC_OCW3_ONE           (1 << 3) /* Must be set to 1 */
#  define PIC_OCW3_SM_SHIFT      (5)
#  define PIC_OCW3_SM_MASK       (3 << PIC_OCW3_SM_SHIFT)
#    define PIC_OCW3_RSM         (2 << PIC_OCW3_SM_SHIFT) /* Reset Special Mask */
#    define PIC_OCW3_SSM         (3 << PIC_OCW3_SM_SHIFT) /* Set Special Mask */

/* If the PIC has been reset, it must be initialized with 2 to 4 Initialization
 * Command Words (ICW) before it will accept and process Interrupt Requests. The
 * following outlines the four possible Initialization Command Words. 
 */

#define PIC1_ICW1                0x20
#define PIC2_ICW1                0xa0

#  define PIC_ICW1_ICW4          (1 << 0) /* Will be Sending ICW4 (no ICW4) */
#  define PIC_ICW1_SINGLE        (1 << 1) /* Single PIC (vs. Cascaded pics) */
#  define PIC_ICW1_INTERVAL      (1 << 2) /* Call Address Interval of 4 (vs 8) */
#  define PIC_ICW1_LEVEL         (1 << 3) /* Level Triggered Interrupts (vs Edge) */
#  define PIC_ICW1_ICW1          (1 << 4) /* Must be set to 1 for ICW1 */
#  define PIC_ICW1_VEC_SHIFT     (5)      /* Interrupt Vector Addresses for MCS-80/85 Mode */
#  define PIC_ICW1_VEC_MASK      (7 << PIC_ICW1_VEC_SHIFT)

/* Initialization Command Word 2 (ICW2) selects which vector information is
 * released onto the bus, during the 2nd INTA Pulse. Using the 8086 mode,
 * only bits 7:3 need to be used. This will be 00001000 (0x08) for PIC1 and
 * 01110000 (0x70) for PIC2. If you wish to relocate the IRQ Vector Table,
 * then you can use this register. 
 */

#define PIC1_ICW2                0x21
#define PIC2_ICW2                0xa1

/* There are two different Initialization Command Word 3's. One is used, if
 * the PIC is a master, while the other is used for slaves.
 */

#define PIC1_ICW3                0x21
#define PIC2_ICW3                0xa1

/* Master ICW3 */

#  define PIC1_ICW3_IRQ0         (1 << 0) /* IRQ0  is connected to a Slave */
#  define PIC1_ICW3_IRQ1         (1 << 1) /* IRQ1  is connected to a Slave */
#  define PIC1_ICW3_IRQ2         (1 << 2) /* IRQ2  is connected to a Slave */
#  define PIC1_ICW3_IRQ3         (1 << 3) /* IRQ3  is connected to a Slave */
#  define PIC1_ICW3_IRQ4         (1 << 4) /* IRQ4  is connected to a Slave */
#  define PIC1_ICW3_IRQ5         (1 << 5) /* IRQ5  is connected to a Slave */
#  define PIC1_ICW3_IRQ6         (1 << 6) /* IRQ6  is connected to a Slave */
#  define PIC1_ICW3_IRQ7         (1 << 7) /* IRQ7  is connected to a Slave */

/* And for the slave device, the ICW3 below is used. */

#  define PIC_ICW3_SID_MASK      (0)      /* Slave ID */
#  define PIC_ICW3_SID_SHIFT     (7 << PIC_ICW3_SID_MASK)
#    define PIC_ICW3_SID0        (0 << PIC_ICW3_SID_MASK) /* Slave 0 */
#    define PIC_ICW3_SID1        (1 << PIC_ICW3_SID_MASK) /* Slave 1 */
#    define PIC_ICW3_SID2        (2 << PIC_ICW3_SID_MASK) /* Slave 2 */
#    define PIC_ICW3_SID3        (3 << PIC_ICW3_SID_MASK) /* Slave 3 */
#    define PIC_ICW3_SID4        (4 << PIC_ICW3_SID_MASK) /* Slave 4 */
#    define PIC_ICW3_SID5        (5 << PIC_ICW3_SID_MASK) /* Slave 5 */
#    define PIC_ICW3_SID6        (6 << PIC_ICW3_SID_MASK) /* Slave 6 */
#    define PIC_ICW3_SID7        (7 << PIC_ICW3_SID_MASK) /* Slave 7 */

#define PIC1_ICW4                0x21
#define PIC2_ICW4                0xa1

#  define PIC_ICW4_FNM           (1 << 4) /* Special Fully Nested Mode */
#  define PIC_ICW4_BMODE_SHIFT   (2)      /* Bufferd mode */
#  define PIC_ICW4_BMODE_MASK    (3 << PIC_ICW4_BMODE_SHIFT)
#    define PIC_ICW4_BMODE_NON   (0 << PIC_ICW4_BMODE_SHIFT) /* Non - Buffered Mode */
#    define PIC_ICW4_BMODE_SLAVE (2 << PIC_ICW4_BMODE_SHIFT) /* Buffered Mode - Slave */
#    define PIC_ICW4_BMODE_MSTR  (3 << PIC_ICW4_BMODE_SHIFT) /* Buffered Mode - Master */
#  define PIC_ICW4_AEOI          (1 << 1) /* Auto EOI  */
#  define PIC_ICW4_808xMODE      (1 << 0) /* 8086/8080 Mode (vs MCS-80/85) */

/* Interrupt Mask Register */

#define PIC1_IMR                 0x21
#define PIC2_IMR                 0xa1

#  define PIC1_IMR_IRQ0         (1 << 0) /* IRQ0  System Timer */
#  define PIC1_IMR_IRQ1         (1 << 1) /* IRQ1  Keyboard */
#  define PIC1_IMR_IRQ2         (1 << 2) /* IRQ2  PIC2 */
#  define PIC1_IMR_IRQ3         (1 << 3) /* IRQ3  Serial Port */
#  define PIC1_IMR_IRQ4         (1 << 4) /* IRQ4  Serial Port */
#  define PIC1_IMR_IRQ5         (1 << 5) /* IRQ5  Reserved/Sound Card */
#  define PIC1_IMR_IRQ6         (1 << 6) /* IRQ6  Floppy Disk Controller */
#  define PIC1_IMR_IRQ7         (1 << 7) /* IRQ7  Parallel Port */
#  define PIC1_IMR_ALL          0xff

#  define PIC2_IMR_IRQ8         (1 << 0) /* IRQ8  Real Time Clock */
#  define PIC2_IMR_IRQ9         (1 << 1) /* IRQ9  Redirected IRQ2 */
#  define PIC2_IMR_IRQ10        (1 << 2) /* IRQ10 Reserved */
#  define PIC2_IMR_IRQ11        (1 << 3) /* IRQ11 Reserved */
#  define PIC2_IMR_IRQ12        (1 << 4) /* IRQ12 PS/2 Mouse */
#  define PIC2_IMR_IRQ13        (1 << 5) /* IRQ13 Maths Co-Processor */
#  define PIC2_IMR_IRQ14        (1 << 6) /* IRQ14 Hard Disk Drive */
#  define PIC2_IMR_IRQ15        (1 << 7) /* IRQ15 Reserved */
#  define PIC2_IMR_ALL          0xff

/* Programmable Interrupt Timer Definitions */

#define PIT_REG_COUNTER0        0x40
#define PIT_REG_COUNTER1        0x41
#define PIT_REG_COUNTER2        0x42
#define PIT_REG_COMMAND         0x43

/* PIT command bit defintions */

#  define PIT_OCW_BINCOUNT_BCD  (1 << 0) /* vs binary */
#  define PIT_OCW_MODE_SHIFT    (1)
#  define PIT_OCW_MODE_MASK     (7 << PIT_OCW_MODE_SHIFT)
#    define PIT_OCW_MODE_TMCNT  (0 << PIT_OCW_MODE_SHIFT)  /* Terminal count */
#    define PIT_OCW_MODE_ONESHOT (1 << PIT_OCW_MODE_SHIFT) /* One shot */
#    define PIT_OCW_MODE_RATEGEN (2 << PIT_OCW_MODE_SHIFT) /* Rate gen */
#    define PIT_OCW_MODE_SQUARE (3 << PIT_OCW_MODE_SHIFT)  /* Square wave generation */
#    define PIT_OCW_MODE_SWTRIG (4 << PIT_OCW_MODE_SHIFT)  /* Software trigger */
#    define PIT_OCW_MODE_HWTRIG (5 << PIT_OCW_MODE_SHIFT)  /* Hardware trigger */
#  define PIT_OCW_RL_SHIFT      (4)
#  define PIT_OCW_RL_MASK       (3 << PIT_OCW_RL_SHIFT)
#    define PIT_OCW_RL_LATCH    (0 << PIT_OCW_RL_SHIFT)
#    define PIT_OCW_RL_LSBONLY  (1 << PIT_OCW_RL_SHIFT)
#    define PIT_OCW_RL_MSBONLY  (2 << PIT_OCW_RL_SHIFT)
#    define PIT_OCW_RL_DATA     (3 << PIT_OCW_RL_SHIFT)
#  define PIT_OCW_COUNTER_SHIFT (6)
#  define PIT_OCW_COUNTER_MASK  (3 << PIT_OCW_COUNTER_SHIFT)
#    define PIT_OCW_COUNTER_0   (0 << PIT_OCW_COUNTER_SHIFT)
#    define PIT_OCW_COUNTER_1   (1 << PIT_OCW_COUNTER_SHIFT)
#    define PIT_OCW_COUNTER_2   (2 << PIT_OCW_COUNTER_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GDT data structures ******************************************************
 *
 * The Global Descriptor Table or GDT is a data structure used by Intel x86-
 * family processors starting with the 80286 in order to define the
 * characteristics of the various memory areas used during program execution, 
 * for example the base address, the size and access privileges like
 * executability and writability. These memory areas are called segments in
 * Intel terminology.
 */

/* This structure defines one segment */

struct gdt_entry_s
{
  uint16_t lowlimit;           /* The lower 16 bits of the limit */
  uint16_t lowbase;            /* The lower 16 bits of the base */
  uint8_t  midbase;            /* The next 8 bits of the base */
  uint8_t  access;             /* Access flags, determine ring segment can be used in */
  uint8_t  granularity;
  uint8_t  hibase;             /* The last 8 bits of the base */
} packed_struct;

/* This structure refers to the array of GDT entries, and is in the format
 * required by the lgdt instruction.
 */

struct gdt_ptr_s
{
  uint16_t limit;               /* The upper 16 bits of all selector limits */
  uint32_t base;                /* The address of the first GDT entry */
} packed_struct;

/* IDT data structures ******************************************************
 *
 * The Interrupt Descriptor Table (IDT) is a data structure used by the x86
 * architecture to implement an interrupt vector table. The IDT is used by the
 * processor to determine the correct response to interrupts and exceptions.
 */

struct idt_entry_s
{
  uint16_t lobase;           /* Lower 16-bits of vector address for interrupt */
  uint16_t sel;              /* Kernel segment selector */
  uint8_t  zero;             /* This must always be zero */
  uint8_t  flags;            /* (See documentation) */
  uint16_t hibase;           /* Upper 16-bits of vector address for interrupt */
} packed_struct;

/* A struct describing a pointer to an array of interrupt handlers.  This is
 * in a format suitable for giving to 'lidt'.
 */

struct idt_ptr_s
{
  uint16_t limit;
  uint32_t base;             /* The address of the first GDT entry */
} packed_struct;

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Return stack pointer */

static inline uint32_t up_getsp()
{
  uint32_t regval;

  asm volatile(
    "\tmovl %%esp, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

/* Get segment registers */

static inline uint32_t up_getds()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%ds, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getcs()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%cs, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

static inline uint32_t up_getss()
{
  uint32_t regval;

  asm volatile(
    "\tmov %%ss, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN void gdt_flush(uint32_t gdt_addr);
EXTERN void idt_flush(uint32_t idt_addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_INCLUDE_I486_ARCH_H */

