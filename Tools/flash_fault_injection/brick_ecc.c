/*
 * brick_ecc.c - plant an uncorrectable flash ECC error (STM32H7)
 *
 * Plants an uncorrectable ECC error in the parameter flash word at ADDR,
 * reproducing the torn-write brick: the firmware's find_entry() scan reads the
 * word, takes an uncorrectable (double-bit) ECC error -> precise bus fault ->
 * hard fault on every boot, recoverable only by a mass erase or the
 * bootloader's ECC scrub. See README.md.
 *
 * Recipe (verified on H743, BFAR landed here in the sweep): program the flash
 * word twice with no erase between, clearing a DIFFERENT single bit each time
 * (0xFFFFFFFE then 0xFFFFFFFD). The stored data becomes 0xFFFFFFFC but the
 * stored ECC is ecc(P1)&ecc(P2), inconsistent with it -> uncorrectable. (A
 * complementary pair like 0x55/0xAA collapses to data 0 with valid ECC and
 * does NOT fault - that is why the sweep was needed.)
 *
 * Runs from RAM with interrupts masked, no OS, driving the bank-2 flash
 * controller directly. Defaults target the ARK_FPV parameter sector
 * (STM32H743, bank 2, sector 7 @ 0x081E0000).
 */

#define REG(a) (*(volatile unsigned int *)(a))

#define FLASH_KEYR2 0x52002104u
#define FLASH_CR2   0x5200210Cu
#define FLASH_SR2   0x52002110u
#define FLASH_CCR2  0x52002114u

#define KEY1 0x45670123u
#define KEY2 0xCDEF89ABu

#define CR_LOCK  (1u << 0)
#define CR_PG    (1u << 1)
#define CR_SER   (1u << 2)
#define CR_START (1u << 7)
#define CR_SNB(n) (((n) & 0x7u) << 8)

#define SR_BUSY  ((1u << 0) | (1u << 2))   /* BSY | QW */
#define CCR_CLEAR_ALL 0x0FEF0000u

#define ADDR 0x081E0000u   /* set to 0x081FD100 to match the original BFAR */
#define SNB  7u
#define P1   0xFFFFFFFEu   /* clear bit 0 */
#define P2   0xFFFFFFFDu   /* then clear bit 1 -> data 0xFFFFFFFC, bad ECC */

static void wait_idle(void)
{
	while (REG(FLASH_SR2) & SR_BUSY) { }
}

/* program the 256-bit flash word at ADDR with value v (8 x 32-bit) */
static void program_word(unsigned v)
{
	volatile unsigned int *w = (volatile unsigned int *)ADDR;
	REG(FLASH_CR2) = CR_PG;

	for (unsigned i = 0; i < 8; i++) { w[i] = v; }

	wait_idle();
	REG(FLASH_CR2) = 0;
}

void _start(void)
{
	__asm volatile ("cpsid i");

	/* unlock bank 2 and clear stale error flags */
	REG(FLASH_KEYR2) = KEY1;
	REG(FLASH_KEYR2) = KEY2;
	REG(FLASH_CCR2) = CCR_CLEAR_ALL;

	/* erase sector 7 for a known starting state */
	REG(FLASH_CR2) = CR_SER | CR_SNB(SNB);
	REG(FLASH_CR2) = CR_SER | CR_SNB(SNB) | CR_START;
	wait_idle();
	REG(FLASH_CR2) = 0;

	/* double-program the word with two different single-bit clears */
	program_word(P1);
	program_word(P2);

	/* lock and stop for the debugger */
	REG(FLASH_CR2) = CR_LOCK;
	__asm volatile ("bkpt #0");

	for (;;) { }
}
