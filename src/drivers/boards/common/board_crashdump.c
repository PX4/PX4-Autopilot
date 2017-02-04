#include <px4_config.h>
#include <px4_tasks.h>

#include <stdio.h>
#include <string.h>

#include <nuttx/board.h>

#include "up_internal.h"
#include <systemlib/hardfault_log.h>

static void copy_reverse(stack_word_t *dest, stack_word_t *src, int size)
{
	while (size--) {
		*dest++ = *src--;
	}
}

__EXPORT void board_crashdump(uintptr_t currentsp, FAR void *tcb, FAR const uint8_t *filename, int lineno)
{
	/* We need a chunk of ram to save the complete context in.
	 * Since we are going to reboot we will use &_sdata
	 * which is the lowest memory and the amount we will save
	 * _should be_ below any resources we need herein.
	 * Unfortunately this is hard to test. See dead below
	 */

	fullcontext_s *pdump = (fullcontext_s *)&_sdata;

	(void)enter_critical_section();

	struct tcb_s *rtcb = (struct tcb_s *)tcb;

	/* Zero out everything */

	memset(pdump, 0, sizeof(fullcontext_s));

	/* Save Info */

	pdump->info.lineno = lineno;

	if (filename) {

		int offset = 0;
		unsigned int len = strlen((char *)filename) + 1;

		if (len > sizeof(pdump->info.filename)) {
			offset = len - sizeof(pdump->info.filename) ;
		}

		strncpy(pdump->info.filename, (char *)&filename[offset], sizeof(pdump->info.filename));
	}

	/* Save the value of the pointer for current_regs as debugging info.
	 * It should be NULL in case of an ASSERT and will aid in cross
	 * checking the validity of system memory at the time of the
	 * fault.
	 */

	pdump->info.current_regs = (uintptr_t) CURRENT_REGS;

	/* Save Context */


#if CONFIG_TASK_NAME_SIZE > 0
	strncpy(pdump->info.name, rtcb->name, CONFIG_TASK_NAME_SIZE);
#endif

	pdump->info.pid = rtcb->pid;


	/* If  current_regs is not NULL then we are in an interrupt context
	 * and the user context is in current_regs else we are running in
	 * the users context
	 */

	if (CURRENT_REGS) {
		pdump->info.stacks.interrupt.sp = currentsp;

		pdump->info.flags |= (eRegsPresent | eUserStackPresent | eIntStackPresent);
		memcpy(pdump->info.regs, (void *)CURRENT_REGS, sizeof(pdump->info.regs));
		pdump->info.stacks.user.sp = pdump->info.regs[REG_R13];

	} else {

		/* users context */
		pdump->info.flags |= eUserStackPresent;

		pdump->info.stacks.user.sp = currentsp;
	}

	if (pdump->info.pid == 0) {

		pdump->info.stacks.user.top = g_idle_topstack - 4;
		pdump->info.stacks.user.size = CONFIG_IDLETHREAD_STACKSIZE;

	} else {
		pdump->info.stacks.user.top = (uint32_t) rtcb->adj_stack_ptr;
		pdump->info.stacks.user.size = (uint32_t) rtcb->adj_stack_size;;
	}

#if CONFIG_ARCH_INTERRUPTSTACK > 3

	/* Get the limits on the interrupt stack memory */

	pdump->info.stacks.interrupt.top = (uint32_t)&g_intstackbase;
	pdump->info.stacks.interrupt.size  = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

	/* If In interrupt Context save the interrupt stack data centered
	 * about the interrupt stack pointer
	 */

	if ((pdump->info.flags & eIntStackPresent) != 0) {
		stack_word_t *ps = (stack_word_t *) pdump->info.stacks.interrupt.sp;
		copy_reverse(pdump->istack, &ps[arraySize(pdump->istack) / 2], arraySize(pdump->istack));
	}

	/* Is it Invalid? */

	if (!(pdump->info.stacks.interrupt.sp <= pdump->info.stacks.interrupt.top &&
	      pdump->info.stacks.interrupt.sp > pdump->info.stacks.interrupt.top - pdump->info.stacks.interrupt.size)) {
		pdump->info.flags |= eInvalidIntStackPrt;
	}

#endif

	/* If In interrupt context or User save the user stack data centered
	 * about the user stack pointer
	 */
	if ((pdump->info.flags & eUserStackPresent) != 0) {
		stack_word_t *ps = (stack_word_t *) pdump->info.stacks.user.sp;
		copy_reverse(pdump->ustack, &ps[arraySize(pdump->ustack) / 2], arraySize(pdump->ustack));
	}

	/* Is it Invalid? */

	if (!(pdump->info.stacks.user.sp <= pdump->info.stacks.user.top &&
	      pdump->info.stacks.user.sp > pdump->info.stacks.user.top - pdump->info.stacks.user.size)) {
		pdump->info.flags |= eInvalidUserStackPtr;
	}

	int rv = stm32_bbsram_savepanic(HARDFAULT_FILENO, (uint8_t *)pdump, sizeof(fullcontext_s));

	/* Test if memory got wiped because of using _sdata */

	if (rv == -ENXIO) {
		char *dead = "Memory wiped - dump not saved!";

		while (*dead) {
			up_lowputc(*dead++);
		}

	} else if (rv == -ENOSPC) {

		/* hard fault again */

		up_lowputc('!');
	}

#if defined(CONFIG_BOARD_RESET_ON_CRASH)
	board_reset(0);
#endif
}
