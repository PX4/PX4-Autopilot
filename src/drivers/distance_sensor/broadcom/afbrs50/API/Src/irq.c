
#include <nuttx/irq.h>

static volatile irqstate_t irqstate_flags;
static volatile size_t _lock_count = 0;

/*!***************************************************************************
* @brief Enable IRQ Interrupts
*
* @return -
*****************************************************************************/
void IRQ_UNLOCK(void)
{
	if (_lock_count > 0) {
		_lock_count--;

		if (_lock_count == 0) {
			leave_critical_section(irqstate_flags);
		}
	}
}

/*!***************************************************************************
* @brief Disable IRQ Interrupts
*
* @return -
*****************************************************************************/
void IRQ_LOCK(void)
{
	if (_lock_count == 0) {
		irqstate_flags = enter_critical_section();
	}

	_lock_count++;
}
