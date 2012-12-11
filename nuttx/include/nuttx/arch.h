/****************************************************************************
 * include/nuttx/arch.h
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_ARCH_H
#define __INCLUDE_NUTTX_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <arch/arch.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*sig_deliver_t)(FAR _TCB *tcb);

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * These are standard interfaces that must be exported to the
 * scheduler from architecture-specific code.
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS
 *   initialization after the basic OS services have been
 *   initialized.  The architecture specific details of
 *   initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the
 *   clock, and registering device drivers are some of the
 *   things that are different for each processor and hardware
 *   platform.
 *
 *   up_initialize is called after the OS initialized but
 *   before the init process has been started and before the
 *   libraries have been initialized.  OS services and driver
 *   services are available.
 *
 ****************************************************************************/

void up_initialize(void);

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed
 *   when their is no other ready-to-run task.  This is processor
 *   idle time and will continue until some interrupt occurs to
 *   cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g.,
 *   this is where power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void);

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(FAR _TCB *tcb);

/****************************************************************************
 * Name: up_create_stack
 *
 * Description:
 *   Allocate a stack for a new thread and setup
 *   up stack-related information in the TCB.
 *
 *   The following TCB fields must be initialized:
 *   adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   stack_alloc_ptr: Pointer to allocated stack
 *   adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Inputs:
 *   tcb: The TCB of new task
 *   stack_size:  The requested stack size.  At least this much
 *     must be allocated.
 *
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
int up_create_stack(FAR _TCB *tcb, size_t stack_size);
#endif

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup up stack-related information in the TCB
 *   using pre-allocated stack memory
 *
 *   The following TCB fields must be initialized:
 *   adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   stack_alloc_ptr: Pointer to allocated stack
 *   adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Inputs:
 *   tcb: The TCB of new task
 *   stack_size:  The allocated stack size.
 *
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
int up_use_stack(FAR _TCB *tcb, FAR void *stack, size_t stack_size);
#endif

/****************************************************************************
 * Name: up_release_stack
 *
 * Description:
 *   A task has been stopped. Free all stack
 *   related resources retained int the defunct TCB.
 *
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
void up_release_stack(FAR _TCB *dtcb);
#endif

/****************************************************************************
 * Name: up_unblock_task
 *
 * Description:
 *   A task is currently in an inactive task list
 *   but has been prepped to execute.  Move the TCB to the
 *   ready-to-run list, restore its context, and start execution.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Inputs:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run taks, executed.
 *
 ****************************************************************************/

void up_unblock_task(FAR _TCB *tcb);

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task at the head of
 *   the ready to run list must be stopped.  Save its context
 *   and move it to the inactive list specified by task_state.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Inputs:
 *   tcb: Refers to a task in the ready-to-run list (normally
 *     the task at the head of the list).  It most be
 *     stopped, its context saved and moved into one of the
 *     waiting task lists.  It it was the task at the head
 *     of the ready-to-run list, then a context to the new
 *     ready to run task must be performed.
 *   task_state: Specifies which waiting task list should be
 *     hold the blocked task TCB.
 *
 ****************************************************************************/

void up_block_task(FAR _TCB *tcb, tstate_t task_state);

/****************************************************************************
 * Name: up_release_pending
 *
 * Description:
 *   When tasks become ready-to-run but cannot run because
 *   pre-emption is disabled, they are placed into a pending
 *   task list.  This function releases and makes ready-to-run
 *   all of the tasks that have collected in the pending task
 *   list.  This can cause a context switch if a new task is
 *   placed at the head of the ready to run list.
 *
 *   This function is called only from the NuttX scheduling
 *   logic when pre-emptioni is re-enabled.  Interrupts will
 *   always be disabled when this function is called.
 *
 ****************************************************************************/

void up_release_pending(void);

/****************************************************************************
 * Name: up_reprioritize_rtr
 *
 * Description:
 *   Called when the priority of a running or
 *   ready-to-run task changes and the reprioritization will 
 *   cause a context switch.  Two cases:
 *
 *   1) The priority of the currently running task drops and the next
 *      task in the ready to run list has priority.
 *   2) An idle, ready to run task's priority has been raised above the
 *      the priority of the current, running task and it now has the
 *      priority.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Inputs:
 *   tcb: The TCB of the task that has been reprioritized
 *   priority: The new task priority
 *
 ****************************************************************************/

void up_reprioritize_rtr(FAR _TCB *tcb, uint8_t priority);

/****************************************************************************
 * Name: _exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 *   Unlike other UP APIs, this function may be called directly from user
 *   programs in various states.  The implementation of this function should
 *   disable interrupts before performing scheduling operations.
 *
 ****************************************************************************/
/* Prototype is in unistd.h */

/****************************************************************************
 * Name: up_assert and up_assert_code
 *
 * Description:
 *   Assertions may be handled in an architecture-specific
 *   way.
 *
 ****************************************************************************/
/* Prototype is in assert.h */

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'igdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
void up_schedule_sigaction(FAR _TCB *tcb, sig_deliver_t sigdeliver);
#endif

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   The heap may be statically allocated by defining CONFIG_HEAP_BASE and
 *   CONFIG_HEAP_SIZE.  If these are not defined, then this function will be
 *   called to dynamically set aside the heap region.
 *
 ****************************************************************************/

#ifndef CONFIG_HEAP_BASE
void up_allocate_heap(FAR void **heap_start, size_t *heap_size);
#endif

/****************************************************************************
 * Name: up_setpicbase, up_getpicbase
 *
 * Description:
 *   It NXFLAT external modules (or any other binary format that requires)
 *   PIC) are supported, then these macros must defined to (1) get or get
 *   the PIC base register value.  These must be implemented with in-line
 *   assembly.
 *
 ****************************************************************************/

#ifndef CONFIG_PIC
#  define up_setpicbase(picbase)
#  define up_getpicbase(ppicbase)
#endif

/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called from the binary loader logic when a new
 *   task is created in RAM in order to instantiate an address environment for
 *   the task.
 *
 * Input Parameters:
 *   tcb - The TCB of the task needing the address environment.
 *   envsize - The size (in bytes) of the address environment needed by the
 *     task.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_create(FAR _TCB *tcb, size_t envsize);
#endif

/****************************************************************************
 * Name: up_addrenv_share
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address ennvironment of its parent
 *   task.  In this case, the parent's address environment needs to be
 *   "cloned" for the child.
 *
 * Input Parameters:
 *   ptcb - The TCB of the parent task that has the address environment.
 *   ctcb - The TCB of the child thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_share(FAR const _TCB *ptcb, FAR _TCB *ctcb);
#endif

/****************************************************************************
 * Name: up_addrenv_instantiate
 *
 * Description:
 *   After an address environment has been established for a task (via
 *   up_addrenv_create().  This function may be called to to instantiate
 *   that address environment in the virtual address space.  this might be
 *   necessary, for example, to load the code for the task from a file or
 *   to access address environment private data.
 *
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be instantiated.
 *
 * Returned Value:
 *   A handle that may be used with up_addrenv_restore() to restore the
 *   address environment before up_addrenv_instantiate() was called.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
FAR void *up_addrenv_instantiate(FAR _TCB *tcb);
#endif

/****************************************************************************
 * Name: up_addrenv_restore
 *
 * Description:
 *   Restore an address environment using a handle previously returned by
 *   up_addrenv_instantiate().
 *
 * Input Parameters:
 *   handle - A handle previously returned by up_addrenv_instantiate.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_restore(FAR void *handle);
#endif

/****************************************************************************
 * Name: up_addrenv_release
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  When there are no further
 *   references to an address environment, that address environment should
 *   be destroyed.
 *
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
int up_addrenv_release(FAR _TCB *tcb);
#endif

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in
 *   the interrupt handler context.
 *
 ****************************************************************************/

bool up_interrupt_context(void);

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (irqrestore() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_enable_irq(int irq);
#endif

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (irqsave() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_disable_irq(int irq);
#endif

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority);
#endif

/****************************************************************************
 * Name: up_romgetc
 *
 * Description:
 *   In Harvard architectures, data accesses and instruction accesses occur
 *   on different busses, perhaps concurrently.  All data accesses are
 *   performed on the data bus unless special  machine instructions are
 *   used to read data from the instruction address space.  Also, in the
 *   typical MCU, the available SRAM data memory is much smaller that the
 *   non-volatile FLASH instruction memory.  So if the application requires
 *   many constant strings, the only practical solution may be to store
 *   those constant strings in FLASH memory where they can only be accessed
 *   using architecture-specific machine instructions.
 *
 *   A similar case is where strings are retained in "external" memory such
 *   as EEPROM or serial FLASH.  This case is similar only in that again
 *   special operations are required to obtain the string data; it cannot
 *   be accessed directly from a string pointer.
 *
 *   If CONFIG_ARCH_ROMGETC is defined, then the architecture logic must
 *   export the function up_romgetc().  up_romgetc() will simply read one
 *   byte of data from the instruction space.
 *
 *   If CONFIG_ARCH_ROMGETC, certain C stdio functions are effected: (1)
 *   All format strings in printf, fprintf, sprintf, etc. are assumed to
 *   lie in FLASH (string arguments for %s are still assumed to reside in
 *   SRAM). And (2), the string argument to puts and fputs is assumed to
 *   reside in FLASH.  Clearly, these assumptions may have to modified for
 *   the particular needs of your environment.  There is no "one-size-fits-all"
 *   solution for this problem.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ROMGETC
char up_romgetc(FAR const char *ptr);
#else
#  define up_romgetc(ptr) (*ptr)
#endif

/****************************************************************************
 * Name: up_mdelay and up_udelay
 *
 * Description:
 *   Some device drivers may require that the plaform-specific logic
 *   provide these timing loops for short delays.
 *
 ***************************************************************************/

void up_mdelay(unsigned int milliseconds);
void up_udelay(useconds_t microseconds);

/****************************************************************************
 * Name: up_cxxinitialize
 *
 * Description:
 *   If C++ and C++ static constructors are supported, then this function
 *   must be provided by board-specific logic in order to perform
 *   initialization of the static C++ class instances.
 *
 *   This function should then be called in the application-specific
 *   user_start logic in order to perform the C++ initialization.  NOTE
 *   that no component of the core NuttX RTOS logic is involved; This
 *   function definition only provides the 'contract' between application
 *   specific C++ code and platform-specific toolchain support
 *
 ***************************************************************************/

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
void up_cxxinitialize(void);
#endif

/****************************************************************************
 * These are standard interfaces that are exported by the OS
 * for use by the architecture specific logic
 ****************************************************************************/

/****************************************************************************
 * Name: sched_process_timer
 *
 * Description:
 *   This function handles system timer events.
 *   The timer interrupt logic itself is implemented in the
 *   architecture specific code, but must call the following OS
 *   function periodically -- the calling interval must be
 *   MSEC_PER_TICK.
 *
 ****************************************************************************/

void sched_process_timer(void);

/****************************************************************************
 * Name: irq_dispatch
 *
 * Description:
 *   This function must be called from the achitecture-
 *   specific logic in order to dispatch an interrupt to
 *   the appropriate, registered handling logic.
 *
 ***************************************************************************/

void irq_dispatch(int irq, FAR void *context);

/****************************************************************************
 * Board-specific button interfaces exported by the board-specific logic
 ****************************************************************************/

/****************************************************************************
 * Name: up_buttoninit
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports button interfaces, then CONFIG_ARCH_BUTTONS
 *   will be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void up_buttoninit(void);
#endif

/****************************************************************************
 * Name: up_buttons
 *
 * Description:
 *   After up_buttoninit() has been called, up_buttons() may be called to
 *   collect the state of all buttons.  up_buttons() returns an 8-bit bit set
 *   with each bit associated with a button.  A bit set to "1" means that the
 *   button is depressed; a bit set to "0" means that the button is released.
 *   The correspondence of the each button bit and physical buttons is board-
 *   specific.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports button interfaces, then CONFIG_ARCH_BUTTONS
 *   will be defined
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
uint8_t up_buttons(void);
#endif

/****************************************************************************
 * Name: up_irqbutton
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *   The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports any button interfaces, then
 *   CONFIG_ARCH_BUTTONS will be defined; If the board supports interrupt
 *   buttons, then CONFIG_ARCH_IRQBUTTONS will also be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif

/************************************************************************************
 * Relay control functions
 *
 * Description:
 *   Non-standard functions for relay control.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_RELAYS
void up_relaysinit(void);
void relays_setstat(int relays, bool stat);
bool relays_getstat(int relays);
void relays_setstats(uint32_t relays_stat);
uint32_t relays_getstats(void);
void relays_onoff(int relays, uint32_t mdelay);
void relays_onoffs(uint32_t relays_stat, uint32_t mdelay);
void relays_resetmode(int relays);
void relays_powermode(int relays);
void relays_resetmodes(uint32_t relays_stat);
void relays_powermodes(uint32_t relays_stat);
#endif

/****************************************************************************
 * Debug interfaces exported by the architecture-specific logic
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Output one character on the console
 *
 ****************************************************************************/

int up_putc(int ch);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ARCH_H */

