
#include <nuttx/irq.h>

static volatile irqstate_t irqstate_flags;

/*!***************************************************************************
* @brief Enable IRQ Interrupts
*
* @return -
*****************************************************************************/
void IRQ_UNLOCK(void)
{
	leave_critical_section(irqstate_flags);
}

/*!***************************************************************************
* @brief Disable IRQ Interrupts
*
* @return -
*****************************************************************************/
void IRQ_LOCK(void)
{
	irqstate_flags = enter_critical_section();
}
