#include <stdint.h>

#include "libc.h"

extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _stack_top;

extern int main(void);
extern void mu_isr(void);

void reset_handler(void);

#define SCB_VTOR      (*(volatile uint32_t *)0xE000ED08u)
#define SCB_CPACR     (*(volatile uint32_t *)0xE000ED88u)
#define NVIC_ISER(n)  (*(volatile uint32_t *)(0xE000E100u + 4 * (n)))

#define MUB_IRQ       118
#define NVIC_IRQS     160

#include "status.h"
#ifdef CM4_FAULT_INJECT
#include "fault.h"
#endif

volatile uint32_t *const cm4_status = (volatile uint32_t *)CM4_STATUS_DA;
volatile char *const cm4_app_name = (volatile char *)CM4_APP_NAME_DA;

/* A fault must not escalate to lockup: on the RT1176 an M4 lockup resets
 * the whole chip, taking the CM7 down with it.
 */

static void default_handler(void)
{
	uint32_t ipsr;
	__asm volatile("mrs %0, ipsr" : "=r"(ipsr));
	cm4_status[0] = CM4_STATE_FAULT;
	cm4_status[1] = ipsr;

#ifdef CM4_FAULT_INJECT

	if (cm4_fault_escalate) {
		__asm volatile("udf #0");	/* fault in HardFault: lockup */
	}

#endif

	for (;;) {
		__asm volatile("wfi");
	}
}

__attribute__((section(".vectors"), used))
const void *const vectors[16 + NVIC_IRQS] = {
	[0] = &_stack_top,
	[1] = reset_handler,
	[2 ... 15] = default_handler,
	[16 ... 16 + NVIC_IRQS - 1] = default_handler,
	[16 + MUB_IRQ] = mu_isr,
};

void reset_handler(void)
{
	/* .data is loaded in place; bss is ours to clear. */
	memset(&_sbss, 0, (uintptr_t)&_ebss - (uintptr_t)&_sbss);

	cm4_status[0] = CM4_STATE_BOOT;
	cm4_status[1] = 0;
	cm4_status[2] = 0;
	cm4_status[3] = 0;

	for (unsigned i = 0; i < CM4_APP_NAME_LEN; i++) {
		cm4_app_name[i] = i < sizeof(CM4_APP_NAME) - 1 ? CM4_APP_NAME[i] : '\0';
	}

	SCB_VTOR = (uint32_t)vectors;
	SCB_CPACR |= 0xFu << 20;	/* CP10/CP11 full access: FPv4-SP, code is hard-float */
	__asm volatile("dsb; isb" ::: "memory");
	NVIC_ISER(MUB_IRQ / 32) = 1u << (MUB_IRQ % 32);
	__asm volatile("dsb; isb; cpsie i" ::: "memory");

	main();

	for (;;) {
		__asm volatile("wfi");
	}
}
