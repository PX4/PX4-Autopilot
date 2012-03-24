#ifndef _CALYPSO_CLK_H
#define _CALYPSO_CLK_H

#include <stdint.h>

#define CALYPSO_PLL26_52_MHZ	((2 << 8) | 0)
#define CALYPSO_PLL26_86_7_MHZ	((10 << 8) | 2)
#define CALYPSO_PLL26_87_MHZ	((3 << 8) | 0)
#define CALYPSO_PLL13_104_MHZ	((8 << 8) | 0)

enum mclk_div {
	_ARM_MCLK_DIV_1		= 0,
	ARM_MCLK_DIV_1		= 1,
	ARM_MCLK_DIV_2		= 2,
	ARM_MCLK_DIV_3		= 3,
	ARM_MCLK_DIV_4		= 4,
	ARM_MCLK_DIV_5		= 5,
	ARM_MCLK_DIV_6		= 6,
	ARM_MCLK_DIV_7		= 7,
	ARM_MCLK_DIV_1_5	= 0x80 | 1,
	ARM_MCLK_DIV_2_5	= 0x80 | 2,
};

void calypso_clock_set(uint8_t vtcxo_div2, uint16_t inp, enum mclk_div mclk_div);
void calypso_pll_set(uint16_t inp);
void calypso_clk_dump(void);

/* CNTL_RST */
enum calypso_rst {
	RESET_DSP	= (1 << 1),
	RESET_EXT	= (1 << 2),
	RESET_WDOG	= (1 << 3),
};

void calypso_reset_set(enum calypso_rst calypso_rst, int active);
int calypso_reset_get(enum calypso_rst);

enum calypso_bank {
	CALYPSO_nCS0	= 0,
	CALYPSO_nCS1	= 2,
	CALYPSO_nCS2	= 4,
	CALYPSO_nCS3	= 6,
	CALYPSO_nCS7	= 8,
	CALYPSO_CS4	= 0xa,
	CALYPSO_nCS6	= 0xc,
};

enum calypso_mem_width {
	CALYPSO_MEM_8bit	= 0,
	CALYPSO_MEM_16bit	= 1,
	CALYPSO_MEM_32bit	= 2,
};

void calypso_mem_cfg(enum calypso_bank bank, uint8_t ws,
		     enum calypso_mem_width width, int we);

/* Enable or disable the internal bootrom mapped to 0x0000'0000 */
void calypso_bootrom(int enable);

/* Enable or disable the debug unit */
void calypso_debugunit(int enable);

/* configure the RHEA bus bridge[s] */
void calypso_rhea_cfg(uint8_t fac0, uint8_t fac1, uint8_t timeout,
		      uint8_t ws_h, uint8_t ws_l, uint8_t w_en0, uint8_t w_en1);

#endif /* _CALYPSO_CLK_H */
