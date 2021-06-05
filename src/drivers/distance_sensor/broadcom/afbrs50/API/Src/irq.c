
#include <nuttx/irq.h>

/*! Global lock level counter value. */
static volatile int g_irq_lock_ct;

static volatile irqstate_t irqstate_flags;

/*!***************************************************************************
* @brief Enable IRQ Interrupts
*
* @details Enables IRQ interrupts by clearing the I-bit in the CPSR.
* Can only be executed in Privileged modes.
*
* @return -
*****************************************************************************/
void IRQ_UNLOCK(void)
{
	assert(g_irq_lock_ct > 0);

	if (--g_irq_lock_ct <= 0) {
		g_irq_lock_ct = 0;
		leave_critical_section(irqstate_flags);
	}
}

/*!***************************************************************************
* @brief Disable IRQ Interrupts
*
* @details Disables IRQ interrupts by setting the I-bit in the CPSR.
* Can only be executed in Privileged modes.
*
* @return -
*****************************************************************************/
void IRQ_LOCK(void)
{
	irqstate_flags = enter_critical_section();
	g_irq_lock_ct++;
}
