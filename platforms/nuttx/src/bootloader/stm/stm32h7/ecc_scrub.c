/*
 * STM32H7 flash ECC scrub.
 *
 * A torn write to a flash row (power lost mid program/erase) can leave it with
 * inconsistent ECC. On H7 a CPU read of such a row raises an uncorrectable
 * (double-bit) ECC error -> bus fault, so the application hard-faults on every
 * boot before it can print anything, and only a mass erase recovers it -
 * reflashing does not, because neither the app nor the bootloader image
 * touches the parameter sector.
 *
 * Before booting we scan the application and parameter flash with DMA: a DMA
 * read of a corrupt row latches FLASH_SR DBECCERR instead of bus-faulting the
 * CPU. Any sector holding an uncorrectable error is erased, so a corrupt
 * parameter sector re-seeds to defaults on the next app boot, and corrupt app
 * flash fails the image check and stays in the bootloader for reflash.
 *
 */

#include <px4_platform_common/px4_config.h>

#include "hw_config.h"

#include <nuttx/progmem.h>
#include <stdbool.h>
#include <stdint.h>

#include "ecc_scrub.h"

/* Registers defined locally to avoid pulling in arch-internal headers. */
#define ECC_RCC_AHB1ENR   (*(volatile uint32_t *)0x580244d8u)
#define ECC_RCC_DMA1EN    (1u << 0)

#define ECC_DMA1          0x40020000u
#define ECC_DMA_LISR      (*(volatile uint32_t *)(ECC_DMA1 + 0x00u))
#define ECC_DMA_LIFCR     (*(volatile uint32_t *)(ECC_DMA1 + 0x08u))
#define ECC_DMA_S0CR      (*(volatile uint32_t *)(ECC_DMA1 + 0x10u))
#define ECC_DMA_S0NDTR    (*(volatile uint32_t *)(ECC_DMA1 + 0x14u))
#define ECC_DMA_S0PAR     (*(volatile uint32_t *)(ECC_DMA1 + 0x18u))
#define ECC_DMA_S0M0AR    (*(volatile uint32_t *)(ECC_DMA1 + 0x1cu))
#define ECC_DMA_S0FCR     (*(volatile uint32_t *)(ECC_DMA1 + 0x24u))
#define ECC_DMA_S0CR_EN   (1u << 0)
#define ECC_DMA_S0_FLAGS  (0x3du)   /* stream-0 FE/DME/TE/HT/TC flags in LISR/LIFCR */
#define ECC_DMA_S0_TCIF   (1u << 5)
/* DIR=mem-to-mem, PINC, MINC, PSIZE=32, MSIZE=32 */
#define ECC_DMA_S0CR_CFG  ((2u << 6) | (1u << 9) | (1u << 10) | (2u << 11) | (2u << 13))
#define ECC_DMA_S0FCR_CFG ((1u << 2) | (3u << 0))   /* direct mode off (FIFO required for M2M), threshold full */

#define ECC_FLASH         0x52002000u
#define ECC_FLASH_SR1     (*(volatile uint32_t *)(ECC_FLASH + 0x010u))
#define ECC_FLASH_CCR1    (*(volatile uint32_t *)(ECC_FLASH + 0x014u))
#define ECC_FLASH_SR2     (*(volatile uint32_t *)(ECC_FLASH + 0x110u))
#define ECC_FLASH_CCR2    (*(volatile uint32_t *)(ECC_FLASH + 0x114u))
#define ECC_FLASH_DBECCERR (1u << 26)
#define ECC_FLASH_CLR_ECC ((1u << 26) | (1u << 25))   /* clear DBECCERR + SNECCERR */

#define ECC_CHUNK_WORDS   256u   /* 1 KiB per DMA transfer */

/* Optional: define to an output pinset (e.g. a spare FMU channel pad) to
 * measure the scan time on a scope - driven high for the duration. */
// #define ECC_SCRUB_TIMING_GPIO   /* PI0 = FMU_CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN0)

/* DMA scratch: .bss lands in AXI SRAM (0x24000000), which DMA1 can reach. */
static uint32_t ecc_scan_buf[ECC_CHUNK_WORDS];

static void ecc_flags_clear(void)
{
	ECC_FLASH_CCR1 = ECC_FLASH_CLR_ECC;
	ECC_FLASH_CCR2 = ECC_FLASH_CLR_ECC;
}

/* bounded so a misbehaving DMA can never hang the bootloader (a 1 KiB
 * transfer completes in a few thousand cycles; this is orders more) */
#define ECC_DMA_TIMEOUT   2000000u

static bool ecc_wait_stream_idle(void)
{
	uint32_t timeout = ECC_DMA_TIMEOUT;

	while ((ECC_DMA_S0CR & ECC_DMA_S0CR_EN) && --timeout) { }

	return timeout != 0;
}

static bool flash_region_has_ecc_error(uintptr_t base, uint32_t size)
{
	for (uint32_t ofs = 0; ofs < size; ofs += ECC_CHUNK_WORDS * sizeof(uint32_t)) {
		ecc_flags_clear();

		ECC_DMA_S0CR = 0;

		if (!ecc_wait_stream_idle()) {
			ECC_DMA_S0CR = 0;
			return false;   /* DMA stuck: skip the scan, fall back to a plain boot */
		}

		ECC_DMA_LIFCR = ECC_DMA_S0_FLAGS;

		ECC_DMA_S0PAR  = base + ofs;                 /* source: flash */
		ECC_DMA_S0M0AR = (uint32_t)ecc_scan_buf;     /* destination: SRAM */
		ECC_DMA_S0NDTR = ECC_CHUNK_WORDS;
		ECC_DMA_S0FCR  = ECC_DMA_S0FCR_CFG;
		ECC_DMA_S0CR   = ECC_DMA_S0CR_CFG;
		ECC_DMA_S0CR  |= ECC_DMA_S0CR_EN;

		/* stream self-disables on completion or on a read error */
		if (!ecc_wait_stream_idle()) {
			ECC_DMA_S0CR = 0;
			return false;
		}

		if ((ECC_FLASH_SR1 & ECC_FLASH_DBECCERR) || (ECC_FLASH_SR2 & ECC_FLASH_DBECCERR)) {
			return true;
		}

		/* a clean verdict needs transfer-complete: the stream also stops on
		 * non-ECC bus errors, which must skip the scan rather than pass it */
		if (!(ECC_DMA_LISR & ECC_DMA_S0_TCIF)) {
			return false;
		}
	}

	return false;
}

void check_ecc_errors(void)
{
	ECC_RCC_AHB1ENR |= ECC_RCC_DMA1EN;
	(void)ECC_RCC_AHB1ENR;

#if defined(ECC_SCRUB_TIMING_GPIO)
	px4_arch_configgpio(ECC_SCRUB_TIMING_GPIO);
	px4_arch_gpiowrite(ECC_SCRUB_TIMING_GPIO, true);
#endif

	/* scan to the last physical sector: parameter sectors sit past
	 * BOARD_FLASH_SECTORS, and boards may have more than one */
	const unsigned last_sector = up_progmem_neraseblocks() - 1;

	for (unsigned sector = BOARD_FIRST_FLASH_SECTOR_TO_ERASE; sector <= last_sector; sector++) {
		uintptr_t base = APP_LOAD_ADDRESS + (sector - BOARD_FIRST_FLASH_SECTOR_TO_ERASE) * FLASH_SECTOR_SIZE;

		if (flash_region_has_ecc_error(base, FLASH_SECTOR_SIZE)) {
			ecc_flags_clear();
			up_progmem_eraseblock(sector);
		}
	}

	/* leave the stream idle and the ECC flags clear for the rest of boot */
	ECC_DMA_S0CR = 0;
	ecc_flags_clear();

#if defined(ECC_SCRUB_TIMING_GPIO)
	px4_arch_gpiowrite(ECC_SCRUB_TIMING_GPIO, false);
#endif
}
