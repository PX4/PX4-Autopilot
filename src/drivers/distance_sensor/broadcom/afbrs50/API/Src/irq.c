#include <nuttx/arch.h>
#include <nuttx/irq.h>

static volatile irqstate_t irqstate_flags;

/*!***************************************************************************
* @brief Enable IRQ Interrupts
*
* @return -
*****************************************************************************/
void IRQ_UNLOCK(void)
{
	if (!up_interrupt_context()) {
		leave_critical_section(irqstate_flags);
	}
}

/*!***************************************************************************
* @brief Disable IRQ Interrupts
*
* @return -
*****************************************************************************/
void IRQ_LOCK(void)
{
	if (!up_interrupt_context()) {
		irqstate_flags = enter_critical_section();
	}
}
