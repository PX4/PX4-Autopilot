#include "mu.h"

#define MUB_BASE      0x40C4C000u
#define MU_TR(n)      (*(volatile uint32_t *)(MUB_BASE + 0x00 + 4 * (n)))
#define MU_RR(n)      (*(volatile uint32_t *)(MUB_BASE + 0x10 + 4 * (n)))
#define MU_SR         (*(volatile uint32_t *)(MUB_BASE + 0x20))
#define MU_CR         (*(volatile uint32_t *)(MUB_BASE + 0x24))

#define MU_SR_TE0     (1u << 23)
#define MU_SR_RF0     (1u << 27)
#define MU_SR_GIP_MASK (0xFu << 28)
#define MU_CR_RIE0    (1u << 27)

volatile uint32_t mu_rx_pending;

void mu_init(void)
{
	/* The CM7 owns the MU reset and clock gates; only arm our RX interrupt. */
	MU_CR = MU_CR_RIE0;
}

void mu_send(uint32_t msg)
{
	/* Mailbox still full: a kick is pending and the CM7 rescans both rings
	 * when it takes it, so coalesce instead of spinning.
	 */
	if (MU_SR & MU_SR_TE0) {
		MU_TR(0) = msg;
	}
}

void mu_isr(void)
{
	uint32_t sr = MU_SR;

	if (sr & MU_SR_RF0) {
		(void)MU_RR(0);	/* reading clears RF0; the value is a hint we do not need */
		mu_rx_pending = 1;
	}

	if (sr & MU_SR_GIP_MASK) {
		MU_SR = sr & MU_SR_GIP_MASK;
	}
}
