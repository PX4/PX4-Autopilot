/*
 * @brief LPC11XX System clock control functions
 *
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Inprecise clock rates for the watchdog oscillator */
STATIC const uint32_t wdtOSCRate[WDTLFO_OSC_4_60 + 1] = {
	0,					/* WDT_OSC_ILLEGAL */
	600000,				/* WDT_OSC_0_60 */
	1050000,			/* WDT_OSC_1_05 */
	1400000,			/* WDT_OSC_1_40 */
	1750000,			/* WDT_OSC_1_75 */
	2100000,			/* WDT_OSC_2_10 */
	2400000,			/* WDT_OSC_2_40 */
	2700000,			/* WDT_OSC_2_70 */
	3000000,			/* WDT_OSC_3_00 */
	3250000,			/* WDT_OSC_3_25 */
	3500000,			/* WDT_OSC_3_50 */
	3750000,			/* WDT_OSC_3_75 */
	4000000,			/* WDT_OSC_4_00 */
	4200000,			/* WDT_OSC_4_20 */
	4400000,			/* WDT_OSC_4_40 */
	4600000				/* WDT_OSC_4_60 */
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Compute a WDT or LFO rate */
STATIC uint32_t Chip_Clock_GetWDTLFORate(uint32_t reg)
{
	uint32_t div;
	CHIP_WDTLFO_OSC_T clk;

	/* Get WDT oscillator settings */
	clk = (CHIP_WDTLFO_OSC_T) ((reg >> 5) & 0xF);
	div = reg & 0x1F;

	/* Compute clock rate and divided by divde value */
	return wdtOSCRate[clk] / ((div + 1) << 1);
}

/* Compute a PLL frequency */
STATIC uint32_t Chip_Clock_GetPLLFreq(uint32_t PLLReg, uint32_t inputRate)
{
	uint32_t msel = ((PLLReg & 0x1F) + 1);

	return inputRate * msel;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set System PLL clock source */
void Chip_Clock_SetSystemPLLSource(CHIP_SYSCTL_PLLCLKSRC_T src)
{
	LPC_SYSCTL->SYSPLLCLKSEL  = (uint32_t) src;
	LPC_SYSCTL->SYSPLLCLKUEN  = 0;
	LPC_SYSCTL->SYSPLLCLKUEN  = 1;
}

/* Bypass System Oscillator and set oscillator frequency range */
void Chip_Clock_SetPLLBypass(bool bypass, bool highfr)
{
	uint32_t ctrl = 0;

	if (bypass) {
		ctrl |= (1 << 0);
	}
	if (highfr) {
		ctrl |= (1 << 1);
	}

	LPC_SYSCTL->SYSOSCCTRL = ctrl;
}

#if defined(CHIP_LPC11UXX)
/* Set USB PLL clock source */
void Chip_Clock_SetUSBPLLSource(CHIP_SYSCTL_PLLCLKSRC_T src)
{
	LPC_SYSCTL->USBPLLCLKSEL  = (uint32_t) src;
	LPC_SYSCTL->USBPLLCLKUEN  = 0;
	LPC_SYSCTL->USBPLLCLKUEN  = 1;
}

#endif

/* Set main system clock source */
void Chip_Clock_SetMainClockSource(CHIP_SYSCTL_MAINCLKSRC_T src)
{
	LPC_SYSCTL->MAINCLKSEL  = (uint32_t) src;
	LPC_SYSCTL->MAINCLKUEN  = 0;
	LPC_SYSCTL->MAINCLKUEN  = 1;
}

#if defined(CHIP_LPC11UXX)
/* Set USB clock source and divider */
void Chip_Clock_SetUSBClockSource(CHIP_SYSCTL_USBCLKSRC_T src, uint32_t div)
{
	LPC_SYSCTL->USBCLKSEL = (uint32_t) src;
	LPC_SYSCTL->USBCLKUEN = 0;
	LPC_SYSCTL->USBCLKUEN = 1;
	LPC_SYSCTL->USBCLKDIV = div;
}

#endif /*CHIP_LPC11UXX*/

#if defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC1125)
/* Set WDT clock source and divider */
void Chip_Clock_SetWDTClockSource(CHIP_SYSCTL_WDTCLKSRC_T src, uint32_t div)
{
	LPC_SYSCTL->WDTCLKSEL = (uint32_t) src;
	LPC_SYSCTL->WDTCLKUEN = 0;
	LPC_SYSCTL->WDTCLKUEN = 1;
	LPC_SYSCTL->WDTCLKDIV = div;
}

#endif

#if !defined(CHIP_LPC110X)
/* Set CLKOUT clock source and divider */
void Chip_Clock_SetCLKOUTSource(CHIP_SYSCTL_CLKOUTSRC_T src, uint32_t div)
{
	LPC_SYSCTL->CLKOUTSEL = (uint32_t) src;
	LPC_SYSCTL->CLKOUTUEN = 0;
	LPC_SYSCTL->CLKOUTUEN = 1;
	LPC_SYSCTL->CLKOUTDIV = div;
}

#endif

/* Return estimated watchdog oscillator rate */
uint32_t Chip_Clock_GetWDTOSCRate(void)
{
	return Chip_Clock_GetWDTLFORate(LPC_SYSCTL->WDTOSCCTRL);
}

#if defined(CHIP_LPC11AXX)
/* Return estimated low frequency oscillator rate */
uint32_t Chip_Clock_GetLFOOSCRate(void)
{
	return Chip_Clock_GetWDTLFORate(LPC_SYSCTL->LFOSCCTRL);
}

#endif

/* Return System PLL input clock rate */
uint32_t Chip_Clock_GetSystemPLLInClockRate(void)
{
	uint32_t clkRate;

	switch ((CHIP_SYSCTL_PLLCLKSRC_T) (LPC_SYSCTL->SYSPLLCLKSEL & 0x3)) {
	case SYSCTL_PLLCLKSRC_IRC:
		clkRate = Chip_Clock_GetIntOscRate();
		break;

	case SYSCTL_PLLCLKSRC_MAINOSC:
		clkRate = Chip_Clock_GetMainOscRate();
		break;

#if defined(CHIP_LPC11AXX)
	case SYSCTL_PLLCLKSRC_EXT_CLKIN:
		clkRate = Chip_Clock_GetExtClockInRate();
		break;
#endif

	default:
		clkRate = 0;
	}

	return clkRate;
}

/* Return System PLL output clock rate */
uint32_t Chip_Clock_GetSystemPLLOutClockRate(void)
{
	return Chip_Clock_GetPLLFreq(LPC_SYSCTL->SYSPLLCTRL,
								 Chip_Clock_GetSystemPLLInClockRate());
}

#if defined(CHIP_LPC11UXX)
/* Return USB PLL input clock rate */
uint32_t Chip_Clock_GetUSBPLLInClockRate(void)
{
	uint32_t clkRate;

	switch ((CHIP_SYSCTL_PLLCLKSRC_T) (LPC_SYSCTL->USBPLLCLKSEL & 0x3)) {
	case SYSCTL_PLLCLKSRC_IRC:
		clkRate = Chip_Clock_GetIntOscRate();
		break;

	case SYSCTL_PLLCLKSRC_MAINOSC:
		clkRate = Chip_Clock_GetMainOscRate();
		break;

	default:
		clkRate = 0;
	}

	return clkRate;
}

/* Return USB PLL output clock rate */
uint32_t Chip_Clock_GetUSBPLLOutClockRate(void)
{
	return Chip_Clock_GetPLLFreq(LPC_SYSCTL->USBPLLCTRL,
								 Chip_Clock_GetUSBPLLInClockRate());
}

#endif

/* Return main clock rate */
uint32_t Chip_Clock_GetMainClockRate(void)
{
	uint32_t clkRate = 0;

	switch ((CHIP_SYSCTL_MAINCLKSRC_T) (LPC_SYSCTL->MAINCLKSEL & 0x3)) {
	case SYSCTL_MAINCLKSRC_IRC:
		clkRate = Chip_Clock_GetIntOscRate();
		break;

	case SYSCTL_MAINCLKSRC_PLLIN:
		clkRate = Chip_Clock_GetSystemPLLInClockRate();
		break;

#if defined(CHIP_LPC11AXX)
	case SYSCTL_MAINCLKSRC_LFOSC:
		clkRate = Chip_Clock_GetLFOOSCRate();
		break;

#else
	case SYSCTL_MAINCLKSRC_WDTOSC:
		clkRate = Chip_Clock_GetWDTOSCRate();
		break;
#endif

	case SYSCTL_MAINCLKSRC_PLLOUT:
		clkRate = Chip_Clock_GetSystemPLLOutClockRate();
		break;
	}

	return clkRate;
}

/* Return system clock rate */
uint32_t Chip_Clock_GetSystemClockRate(void)
{
	/* No point in checking for divide by 0 */
	return Chip_Clock_GetMainClockRate() / LPC_SYSCTL->SYSAHBCLKDIV;
}
