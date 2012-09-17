/****************************************************************************
 * arch/mips/include/mips32/syscall.h
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

/* This file should never be included directed but, rather, only indirectly
 * through include/syscall.h or include/sys/sycall.h
 */

#ifndef __ARCH_MIPS_INCLUDE_MIPS32_SYSCALL_H
#define __ARCH_MIPS_INCLUDE_MIPS32_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pro-processor Definitions
 ****************************************************************************/

#define SYS_syscall 0x00

/* Configuration ********************************************************************/
/* This logic uses three system calls {0,1,2} for context switching.  The first three
 * syscall values must be reserved.
 */

#ifdef CONFIG_NUTTX_KERNEL
#  ifndef CONFIG_SYS_RESERVED
#    error "CONFIG_SYS_RESERVED must be defined to the value 2"
#  elif CONFIG_SYS_RESERVED != 2
#    error "CONFIG_SYS_RESERVED must have the value 2"
#  endif
#endif

/* sys_call macros ******************************************************************/
/* System calls with 3 parameters and fewer are handled by sys_call0 (sys_call1,
 * sys_call2, and sys_call3 are aliases for sys_call0).  This is because the
 * parmeters are passed in a0-a3.  a0 is reserved for the syscall number leaving
 * up to thre additional parameters that can be passed in registers.  The remainder
 * would have to be pushed onto the stack.
 *
 * Instead, these macros are provided which handle parameters four, five and six in
 * a non-standard way:  The use s0 ($7), s1 ($8), and s2 ($9) to pass the additional
 * parameters.
 */

#ifndef __ASSEMBLY__

/* System call SYS_ argument and four additional parameters. */

#define sys_call4(nbr,parm1,parm2,parm3,parm4) __extension__({ \
  uintptr_t __result; \
  __asm__ __volatile__ (\
    "\tmove	$4, %0\n" \
    "\tmove	$5, %1\n" \
    "\tmove	$6, %2\n" \
    "\tmove	$7, %3\n" \
    "\tmove	$8, %4\n" \
    "\la	$12, sys_call3\n" \
    "\jalr	$12, $31\n" \
    "\tmove	%5, $r2\n" \
    : "=r" (nbr) "=r" (parm1) "=r" (parm2) "=r" (parm3) "=r" (parm4) \
    : " "r"(__result)\
    : "memory"\
  ); \
  __result; \
})

/* System call SYS_ argument and five additional parameters. */

#define sys_call5(nbr,parm1,parm2,parm3,parm4,parm5) __extension__({ \
  uintptr_t __result; \
  __asm__ __volatile__ (\
    "\tmove	$4, %0\n" \
    "\tmove	$5, %1\n" \
    "\tmove	$6, %2\n" \
    "\tmove	$7, %3\n" \
    "\tmove	$8, %4\n" \
    "\tmove	$9, %5\n" \
    "\la	$12, sys_call3\n" \
    "\jalr	$12, $31\n" \
    "\tmove	%6, $r2\n" \
    : "=r" (nbr) "=r" (parm1) "=r" (parm2) "=r" (parm3) "=r" (parm4) "=r" (parm5) \
    : " "r"(__result)\
    : "memory"\
  ); \
  __result; \
})

/* System call SYS_ argument and six additional parameters. */

#define sys_call5(nbr,parm1,parm2,parm3,parm4,parm5,parm6) __extension__({ \
  uintptr_t __result; \
  __asm__ __volatile__ (\
    "\tmove	$4, %0\n" \
    "\tmove	$5, %1\n" \
    "\tmove	$6, %2\n" \
    "\tmove	$7, %3\n" \
    "\tmove	$8, %4\n" \
    "\tmove	$9, %5\n" \
    "\tmove	$10, %5\n" \
    "\la	$12, sys_call3\n" \
    "\jalr	$12, $31\n" \
    "\tmove	%6, $r2\n" \
    : "=r" (nbr) "=r" (parm1) "=r" (parm2) "=r" (parm3) "=r" (parm4) "=r" (parm5) \
    : " "r"(__result)\
    : "memory"\
  ); \
  __result; \
})

/* Context switching system calls ***************************************************/

/* SYS call 1:
 *
 * void up_fullcontextrestore(uint32_t *restoreregs) __attribute__ ((noreturn));
 */

#define SYS_restore_context (1)
#define up_fullcontextrestore(restoreregs) \
  (void)sys_call1(SYS_restore_context, (uintptr_t)restoreregs)

/* SYS call 2:
 *
 * void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 */

#define SYS_switch_context (2)
#define up_switchcontext(saveregs, restoreregs) \
  (void)sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs)

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

/****************************************************************************
 * Name: up_syscall0
 *
 * Description:
 *   System call SYS_ argument and no additional parameters.
 *
 ****************************************************************************/

EXTERN uintptr_t sys_call0(unsigned int nbr);

/****************************************************************************
 * Name: up_syscall1
 *
 * Description:
 *   System call SYS_ argument and one additional parameter.
 *
 ****************************************************************************/

EXTERN uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1);

/****************************************************************************
 * Name: up_syscall2
 *
 * Description:
 *   System call SYS_ argument and two additional parameters.
 *
 ****************************************************************************/

EXTERN uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1, uintptr_t parm2);

/****************************************************************************
 * Name: up_syscall3
 *
 * Description:
 *   System call SYS_ argument and three additional parameters.
 *
 ****************************************************************************/

EXTERN uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                           uintptr_t parm2, uintptr_t parm3);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_MIPS32_SYSCALL_H */

