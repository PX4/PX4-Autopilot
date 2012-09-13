#include <stdint.h>
#include <nuttx/irq.h>
#include "up_internal.h"

void os_start(void)
{
	up_lowputc('X');
	up_lowputc('\n');
	for (;;);
}

void up_assert_code(const uint8_t *filename, int lineno, int errorcode)
{
	up_lowputc('?');
	up_lowputc('\n');
	for (;;);
}

int irq_attach(int irq, xcpt_t isr)
{
	up_lowputc('A');
	up_lowputc('\n');
	for (;;);
}

uint32_t *up_doirq(int irq, uint32_t *regs)
{
	up_lowputc('I');
	up_lowputc('\n');
	for (;;);
}