/*
 * @brief LPC11XX System Control registers and control functions
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
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
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __SYSCTL_11XX_H_
#define __SYSCTL_11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SYSCTL_11XX CHIP: LPC11xx System Control block driver
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

/**
 * @brief LPC11XX System Control block structure
 */
typedef struct {			/*!< SYSCTL Structure */
	__IO uint32_t  SYSMEMREMAP;		/*!< System Memory remap register */
	__IO uint32_t  PRESETCTRL;		/*!< Peripheral reset Control register */
	__IO uint32_t  SYSPLLCTRL;		/*!< System PLL control register */
	__I  uint32_t  SYSPLLSTAT;		/*!< System PLL status register */
	__IO uint32_t  USBPLLCTRL;		/*!< USB PLL control register, LPC11UXX only*/
	__I  uint32_t  USBPLLSTAT;		/*!< USB PLL status register, LPC11UXX only */
	__I  uint32_t  RESERVED1[2];
	__IO uint32_t  SYSOSCCTRL;		/*!< System Oscillator control register */
	__IO uint32_t  WDTOSCCTRL;		/*!< Watchdog Oscillator control register */
	__IO uint32_t  IRCCTRL;			/*!< IRC control register, not on LPC11UXX and LPC11EXX */
	__IO uint32_t  LFOSCCTRL;		/*!< LF oscillator control, LPC11AXX only */
	__IO uint32_t  SYSRSTSTAT;		/*!< System Reset Status register           */
	__I  uint32_t  RESERVED2[3];
	__IO uint32_t  SYSPLLCLKSEL;	/*!< System PLL clock source select register */
	__IO uint32_t  SYSPLLCLKUEN;	/*!< System PLL clock source update enable register*/
	__IO uint32_t  USBPLLCLKSEL;	/*!< USB PLL clock source select register, LPC11UXX only */
	__IO uint32_t  USBPLLCLKUEN;	/*!< USB PLL clock source update enable register, LPC11UXX only */
	__I  uint32_t  RESERVED3[8];
	__IO uint32_t  MAINCLKSEL;	/*!< Main clock source select register		*/
	__IO uint32_t  MAINCLKUEN;	/*!< Main clock source update enable register	*/
	__IO uint32_t  SYSAHBCLKDIV;	/*!< System Clock divider register */
	__I  uint32_t  RESERVED4;
	__IO uint32_t  SYSAHBCLKCTRL;	/*!< System clock control register */
	__I  uint32_t  RESERVED5[4];
	__IO uint32_t  SSP0CLKDIV;	/*!< SSP0 clock divider register   */
	__IO uint32_t  USARTCLKDIV;	/*!< UART clock divider register   */
	__IO uint32_t  SSP1CLKDIV;	/*!< SSP1 clock divider register, not on CHIP_LPC110X, CHIP_LPC11XXLV */
	__I  uint32_t  RESERVED6[8];
	__IO uint32_t  USBCLKSEL;	/*!< USB clock source select register, LPC11UXX only */
	__IO uint32_t  USBCLKUEN;	/*!< USB clock source update enable register, LPC11UXX only */
	__IO uint32_t  USBCLKDIV;	/*!< USB clock source divider register, LPC11UXX only */
	__I  uint32_t  RESERVED7;
	__IO uint32_t  WDTCLKSEL;	/*!< WDT clock source select register, some parts only */
	__IO uint32_t  WDTCLKUEN;	/*!< WDT clock source update enable register, some parts only */
	__IO uint32_t  WDTCLKDIV;	/*!< WDT clock divider register, some parts only */
	__I  uint32_t  RESERVED8;
	__IO uint32_t  CLKOUTSEL;	/*!< Clock out source select register, not on LPC1102/04 */
	__IO uint32_t  CLKOUTUEN;	/*!< Clock out source update enable register, not on LPC1102/04 */
	__IO uint32_t  CLKOUTDIV;	/*!< Clock out divider register, not on LPC1102/04 */
	__I  uint32_t  RESERVED9[5];
	__I  uint32_t  PIOPORCAP[2];/*!< POR captured PIO status registers, index 1 on LPC1102/04 */
	__I  uint32_t  RESERVED10[18];
	__IO uint32_t  BODCTRL;		/*!< Brown Out Detect register */
	__IO uint32_t  SYSTCKCAL;	/*!< System tick counter calibration register */
	__I  uint32_t  RESERVED11[6];
	__IO uint32_t  IRQLATENCY;	/*!< IRQ delay register, on LPC11UXX and LPC11EXX only */
	__IO uint32_t  NMISRC;		/*!< NMI source control register,some parts only */
	__IO uint32_t  PINTSEL[8];	/*!< GPIO pin interrupt select register 0-7, not on CHIP_LPC110X, CHIP_LPC11XXLV, CHIP_LPC11CXX */
	__IO uint32_t  USBCLKCTRL;	/*!< USB clock control register, LPC11UXX only */
	__I  uint32_t  USBCLKST;	/*!< USB clock status register, LPC11UXX only */
	__I  uint32_t  RESERVED12[24];
	__IO uint32_t  STARTAPRP0;	/*!< Start loigc 0 interrupt wake up enable register 0, on CHIP_LPC110X, CHIP_LPC11XXLV, CHIP_LPC11CXX */
	__IO uint32_t  STARTERP0;	/*!< Start loigc signal enable register 0, not on LPC11AXX */
	__IO uint32_t  STARTRSRP0CLR;	/*!< Start loigc reset register 0, on CHIP_LPC110X, CHIP_LPC11XXLV, CHIP_LPC11CXX */
	__IO uint32_t  STARTSRP0;		/*!< Start loigc status register 0, on CHIP_LPC110X, CHIP_LPC11XXLV, CHIP_LPC11CXX */
	__I  uint32_t  RESERVED13;
	__IO uint32_t  STARTERP1;	/*!< Start logic 1 interrupt wake up enable register 1, on LPC11UXX and LPC11EXX only */
	__I  uint32_t  RESERVED14[6];
	__IO uint32_t  PDSLEEPCFG;	/*!< Power down states in deep sleep mode register, not on LPC11AXX */
	__IO uint32_t  PDWAKECFG;	/*!< Power down states in wake up from deep sleep register, not on LPC11AXX */
	__IO uint32_t  PDRUNCFG;	/*!< Power configuration register*/
	__I  uint32_t  RESERVED15[110];
	__I  uint32_t  DEVICEID;	/*!< Device ID register */
} LPC_SYSCTL_T;

/**
 * System memory remap modes used to remap interrupt vectors
 */
typedef enum CHIP_SYSCTL_BOOT_MODE_REMAP {
	REMAP_BOOT_LOADER_MODE,	/*!< Interrupt vectors are re-mapped to Boot ROM */
	REMAP_USER_RAM_MODE,	/*!< Interrupt vectors are re-mapped to Static RAM */
	REMAP_USER_FLASH_MODE	/*!< Interrupt vectors are not re-mapped and reside in Flash */
} CHIP_SYSCTL_BOOT_MODE_REMAP_T;

/**
 * @brief	Re-map interrupt vectors
 * @param	remap	: system memory map value
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_Map(CHIP_SYSCTL_BOOT_MODE_REMAP_T remap)
{
	LPC_SYSCTL->SYSMEMREMAP = (uint32_t) remap;
}

/**
 * Peripheral reset identifiers, not available on all devices
 */
typedef enum {
	RESET_SSP0,			/*!< SSP0 reset control */
	RESET_I2C0,			/*!< I2C0 reset control */
	RESET_SSP1,			/*!< SSP1 reset control */
#if !defined(CHIP_LPC1125)
	RESET_CAN0,			/*!< CAN0 reset control */
	RESET_UART0,		/*!< UART0 reset control */
	RESET_TIMER0_16,	/*!< 16-bit Timer 0 reset control */
	RESET_TIMER1_16,	/*!< 16-bit Timer 1 reset control */
	RESET_TIMER0_32,	/*!< 32-bit Timer 0 reset control */
	RESET_TIMER1_32,	/*!< 32-bit Timer 1 reset control */
	RESET_ACMP,			/*!< Analog comparator reset control */
	RESET_DAC0,			/*!< DAC reset control */
	RESET_ADC0			/*!< ADC reset control */
#endif
} CHIP_SYSCTL_PERIPH_RESET_T;

/**
 * @brief	Assert reset for a peripheral
 * @param	periph	: Peripheral to assert reset for
 * @return	Nothing
 * @note	The peripheral will stay in reset until reset is de-asserted. Call
 * Chip_SYSCTL_DeassertPeriphReset() to de-assert the reset.
 */
STATIC INLINE void Chip_SYSCTL_AssertPeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	LPC_SYSCTL->PRESETCTRL &= ~(1 << (uint32_t) periph);
}

/**
 * @brief	De-assert reset for a peripheral
 * @param	periph	: Peripheral to de-assert reset for
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_DeassertPeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	LPC_SYSCTL->PRESETCTRL |= (1 << (uint32_t) periph);
}

/**
 * @brief	Resets a peripheral
 * @param	periph	:	Peripheral to reset
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_PeriphReset(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	Chip_SYSCTL_AssertPeriphReset(periph);
	Chip_SYSCTL_DeassertPeriphReset(periph);
}

/**
 * System reset status
 */
#define SYSCTL_RST_POR    (1 << 0)	/*!< POR reset status */
#define SYSCTL_RST_EXTRST (1 << 1)	/*!< External reset status */
#define SYSCTL_RST_WDT    (1 << 2)	/*!< Watchdog reset status */
#define SYSCTL_RST_BOD    (1 << 3)	/*!< Brown-out detect reset status */
#define SYSCTL_RST_SYSRST (1 << 4)	/*!< software system reset status */

/**
 * Non-Maskable Interrupt Enable/Disable value
 */
#define SYSCTL_NMISRC_ENABLE   ((uint32_t) 1 << 31)	/*!< Enable the Non-Maskable Interrupt (NMI) source */

/**
 * @brief	Get system reset status
 * @return	An Or'ed value of SYSCTL_RST_*
 * @note	This function returns the detected reset source(s).
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetSystemRSTStatus(void)
{
	return LPC_SYSCTL->SYSRSTSTAT;
}

/**
 * @brief	Clear system reset status
 * @param	reset	: An Or'ed value of SYSCTL_RST_* status to clear
 * @return	Nothing
 * @note	This function returns the detected reset source(s).
 */
STATIC INLINE void Chip_SYSCTL_ClearSystemRSTStatus(uint32_t reset)
{
	LPC_SYSCTL->SYSRSTSTAT = reset;
}

/**
 * @brief	Read POR captured PIO status
 * @param	index	: POR register index, 0 or 1
 * @return	captured POR PIO status
 * @note	Some devices only support index 0.
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetPORPIOStatus(int index)
{
	return LPC_SYSCTL->PIOPORCAP[index];
}

/**
 * Brown-out detector reset level
 */
typedef enum CHIP_SYSCTL_BODRSTLVL {
#if defined(CHIP_LPC11CXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC1125)
	SYSCTL_BODRSTLVL_1_46V,	/*!< Brown-out reset at 1.46v */
#else
	SYSCTL_BODRSTLVL_RESERVED1,	/*!< Only possible value for LPC11A/02/XXLV */
#endif
#if defined(CHIP_LPC11XXLV)
	SYSCTL_BODRSTLVL_RESERVED1,
	SYSCTL_BODRSTLVL_RESERVED2,
	SYSCTL_BODRSTLVL_RESERVED3,
#elif defined(CHIP_LPC11AXX)
	SYSCTL_BODRSTLVL_RESERVED2,
	SYSCTL_BODRSTLVL_2_52V,	/*!< Brown-out reset at 2.52v */
	SYSCTL_BODRSTLVL_2_80V,	/*!< Brown-out reset at 2.80v */
#else
	SYSCTL_BODRSTLVL_2_06V,	/*!< Brown-out reset at 2.06v */
	SYSCTL_BODRSTLVL_2_35V,	/*!< Brown-out reset at 2.35v */
	SYSCTL_BODRSTLVL_2_63V,	/*!< Brown-out reset at 2.63v */
#endif
} CHIP_SYSCTL_BODRSTLVL_T;

/**
 * Brown-out detector interrupt level
 */
typedef enum CHIP_SYSCTL_BODRINTVAL {
	SYSCTL_BODINTVAL_RESERVED1,
#if defined(CHIP_LPC110X) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC1125)
	SYSCTL_BODINTVAL_2_22V,	/*!< Brown-out interrupt at 2.22v */
	SYSCTL_BODINTVAL_2_52V,	/*!< Brown-out interrupt at 2.52v */
	SYSCTL_BODINTVAL_2_80V,	/*!< Brown-out interrupt at 2.8v */
#endif
} CHIP_SYSCTL_BODRINTVAL_T;

/**
 * @brief	Set brown-out detection interrupt and reset levels
 * @param	rstlvl	: Brown-out detector reset level
 * @param	intlvl	: Brown-out interrupt level
 * @return	Nothing
 * @note	Brown-out detection reset will be disabled upon exiting this function.
 * Use Chip_SYSCTL_EnableBODReset() to re-enable.
 */
STATIC INLINE void Chip_SYSCTL_SetBODLevels(CHIP_SYSCTL_BODRSTLVL_T rstlvl,
											CHIP_SYSCTL_BODRINTVAL_T intlvl)
{
	LPC_SYSCTL->BODCTRL = ((uint32_t) rstlvl) | (((uint32_t) intlvl) << 2);
}

#if defined(CHIP_LPC11AXX)
/**
 * @brief	Returns brown-out detection interrupt status
 * @return	true if the BOD interrupt is pending, otherwise false
 */
STATIC INLINE bool Chip_SYSCTL_GetBODIntStatus(void)
{
	return (bool) ((LPC_SYSCTL->BODCTRL & (1 << 6)) != 0);
}

#else
/**
 * @brief	Enable brown-out detection reset
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_EnableBODReset(void)
{
	LPC_SYSCTL->BODCTRL |= (1 << 4);
}

/**
 * @brief	Disable brown-out detection reset
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_DisableBODReset(void)
{
	LPC_SYSCTL->BODCTRL &= ~(1 << 4);
}

#endif

/**
 * @brief	Set System tick timer calibration value
 * @param	sysCalVal	: System tick timer calibration value
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_SetSYSTCKCAL(uint32_t sysCalVal)
{
	LPC_SYSCTL->SYSTCKCAL = sysCalVal;
}

#if defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC1125)
/**
 * @brief	Set System IRQ latency
 * @param	latency	: Latency in clock ticks
 * @return	Nothing
 * @note	Sets the IRQ latency, a value between 0 and 255 clocks. Lower
 * values allow better latency.
 */
STATIC INLINE void Chip_SYSCTL_SetIRQLatency(uint32_t latency)
{
	LPC_SYSCTL->IRQLATENCY = latency;
}

/**
 * @brief	Get System IRQ latency
 * @return	Latency in clock ticks
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetIRQLatency(void)
{
	return LPC_SYSCTL->IRQLATENCY;
}

#endif

#if defined(CHIP_LPC11AXX) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC1125)
/**
 * @brief	Set source for non-maskable interrupt (NMI)
 * @param	intsrc	: IRQ number to assign to the NMI
 * @return	Nothing
 * @note	The NMI source will be disabled upon exiting this function. use the
 * Chip_SYSCTL_EnableNMISource() function to enable the NMI source.
 */
STATIC INLINE void Chip_SYSCTL_SetNMISource(uint32_t intsrc)
{
	LPC_SYSCTL->NMISRC = intsrc;
}

/**
 * @brief	Enable interrupt used for NMI source
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_EnableNMISource(void)
{
	LPC_SYSCTL->NMISRC |= SYSCTL_NMISRC_ENABLE;
}

/**
 * @brief	Disable interrupt used for NMI source
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_DisableNMISource(void)
{
	LPC_SYSCTL->NMISRC &= ~(SYSCTL_NMISRC_ENABLE);
}

#endif

#if defined(CHIP_LPC11AXX)
/**
 * @brief	Setup a pin source for the pin interrupts (0-7)
 * @param	intno	: IRQ number
 * @param	port		: port number 0/1)
 * @param	pin		: pin number (0->31)
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_SetPinInterrupt(uint32_t intno, uint8_t port, uint8_t pin)
{
	LPC_SYSCTL->PINTSEL[intno] = (uint32_t) ((port << 5) | pin);
}

#elif defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX)
/**
 * @brief	Setup a pin source for the pin interrupts (0-7)
 * @param	intno	: IRQ number
 * @param	port		: port number 0/1)
 * @param	pin		: pin number (0->23 for GPIO Port 0 and 0->31 for GPIO Port 1)
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_SetPinInterrupt(uint32_t intno, uint8_t port, uint8_t pin)
{
	LPC_SYSCTL->PINTSEL[intno] = (uint32_t) (port * 24 + pin);
}

#endif

#if defined(CHIP_LPC11UXX)
/**
 * @brief	Setup USB clock control
 * @param	ap_clk	: USB need_clock signal control (0 or 1)
 * @param	pol_clk	: USB need_clock polarity for triggering the USB wake-up interrupt (0 or 1)
 * @return	Nothing
 * @note	See the USBCLKCTRL register in the user manual for these settings.
 */
STATIC INLINE void Chip_SYSCTL_SetUSBCLKCTRL(uint32_t ap_clk, uint32_t pol_clk)
{
	LPC_SYSCTL->USBCLKCTRL = ap_clk | (pol_clk << 1);
}

/**
 * @brief	Returns the status of the USB need_clock signal
 * @return	true if USB need_clock statis is high, otherwise false
 */
STATIC INLINE bool Chip_SYSCTL_GetUSBCLKStatus(void)
{
	return (bool) ((LPC_SYSCTL->USBCLKST & 0x1) != 0);
}

#endif

#if defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC1125)
/**
 * @brief	Set edge for PIO start logic
 * @param	pin		: PIO pin number
 * @param	edge	: 0 for falling edge, 1 for rising edge
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCTL_SetStartPin(uint32_t pin, uint32_t edge)
{
	if (edge) {
		LPC_SYSCTL->STARTAPRP0 |= (1 << pin);
	}
	else {
		LPC_SYSCTL->STARTAPRP0 &= ~(1 << pin);
	}
}

/**
 * @brief	Enable PIO start logic for a pin
 * @param	pin	: PIO pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCTL_EnableStartPin(uint32_t pin)
{
	LPC_SYSCTL->STARTERP0 |= (1 << pin);
}

/**
 * @brief	Disable PIO start logic for a pin
 * @param	pin	: PIO pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCTL_DisableStartPin(uint32_t pin)
{
	LPC_SYSCTL->STARTERP0 &= ~(1 << pin);
}

/**
 * @brief	Clear PIO start logic state
 * @param	pin	: PIO pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCTL_ResetStartPin(uint32_t pin)
{
	LPC_SYSCTL->STARTRSRP0CLR = (1 << pin);
}

/**
 * @brief	Returns status of pin wakeup
 * @param	pin	: PIO pin number
 * @return	true if a pin start signal is pending, otherwise false
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE bool Chip_SYSCTL_GetStartPinStatus(uint32_t pin)
{
	return (bool) ((LPC_SYSCTL->STARTSRP0 & (1 << pin)) != 0);
}

#endif

#if defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX)
/**
 * @brief	Enables a pin's (PINT) wakeup logic
 * @param	pin	: pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCTL_EnablePINTWakeup(uint32_t pin)
{
	LPC_SYSCTL->STARTERP0 |= (1 << pin);
}

/**
 * @brief	Disables a pin's (PINT) wakeup logic
 * @param	pin	: pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCTL_DisablePINTWakeup(uint32_t pin)
{
	LPC_SYSCTL->STARTERP0 &= ~(1 << pin);
}

/**
 * Peripheral interrupt wakeup events, LPC11E/Uxx only
 */
#define SYSCTL_WAKEUP_WWDTINT    (1 << 12)	/*!< WWDT interrupt wake-up */
#define SYSCTL_WAKEUP_BODINT     (1 << 13)	/*!< Brown Out Detect (BOD) interrupt wake-up */
#define SYSCTL_WAKEUP_USB_WAKEUP (1 << 19)	/*!< USB need_clock signal wake-up, LPC11Uxx only */
#define SYSCTL_WAKEUP_GPIOINT0   (1 << 20)	/*!< GPIO GROUP0 interrupt wake-up */
#define SYSCTL_WAKEUP_GPIOINT1   (1 << 21)	/*!< GPIO GROUP1 interrupt wake-up */

/**
 * @brief	Enables a peripheral's wakeup logic
 * @param	periphmask	: OR'ed values of SYSCTL_WAKEUP_* for wakeup
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_EnablePeriphWakeup(uint32_t periphmask)
{
	LPC_SYSCTL->STARTERP1 |= periphmask;
}

/**
 * @brief	Disables a peripheral's wakeup logic
 * @param	periphmask	: OR'ed values of SYSCTL_WAKEUP_* for wakeup
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCTL_DisablePeriphWakeup(uint32_t periphmask)
{
	LPC_SYSCTL->STARTERP1 &= ~periphmask;
}

#endif

#if !defined(LPC11AXX)
/**
 * Deep sleep setup values
 */
#define SYSCTL_DEEPSLP_BOD_PD    (1 << 3)	/*!< BOD power-down control in Deep-sleep mode, powered down */
#define SYSCTL_DEEPSLP_WDTOSC_PD (1 << 6)	/*!< Watchdog oscillator power control in Deep-sleep, powered down */
#if defined(CHIP_LPC11XXLV)
#define SYSCTL_DEEPSLP_IRCOUT_PD (1 << 0)	/*!< IRC oscillator output in Deep-sleep mode, powered down */
#define SYSCTL_DEEPSLP_IRC_PD    (1 << 1)	/*!< IRC oscillator in Deep-sleep, powered down */
#endif

/**
 * @brief	Setup deep sleep behaviour for power down
 * @param	sleepmask	: OR'ed values of SYSCTL_DEEPSLP_* values (high to powerdown on deepsleep)
 * @return	Nothing
 * @note	This must be setup prior to using deep sleep. See the user manual
 ***(PDSLEEPCFG register) for more info on setting this up. This function selects
 * which peripherals are powered down on deep sleep.
 * This function should only be called once with all options for power-down
 * in that call.
 */
void Chip_SYSCTL_SetDeepSleepPD(uint32_t sleepmask);

/**
 * @brief	Returns current deep sleep mask
 * @return	OR'ed values of SYSCTL_DEEPSLP_* values
 * @note	A high bit indicates the peripheral will power down on deep sleep.
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetDeepSleepPD(void)
{
	return LPC_SYSCTL->PDSLEEPCFG;
}

/**
 * Deep sleep to wakeup setup values
 */
#define SYSCTL_SLPWAKE_IRCOUT_PD (1 << 0)	/*!< IRC oscillator output wake-up configuration */
#define SYSCTL_SLPWAKE_IRC_PD    (1 << 1)	/*!< IRC oscillator power-down wake-up configuration */
#define SYSCTL_SLPWAKE_FLASH_PD  (1 << 2)	/*!< Flash wake-up configuration */
#define SYSCTL_SLPWAKE_BOD_PD    (1 << 3)	/*!< BOD wake-up configuration */
#define SYSCTL_SLPWAKE_ADC_PD    (1 << 4)	/*!< ADC wake-up configuration */
#define SYSCTL_SLPWAKE_SYSOSC_PD (1 << 5)	/*!< System oscillator wake-up configuration */
#define SYSCTL_SLPWAKE_WDTOSC_PD (1 << 6)	/*!< Watchdog oscillator wake-up configuration */
#define SYSCTL_SLPWAKE_SYSPLL_PD (1 << 7)	/*!< System PLL wake-up configuration */
#if defined(CHIP_LPC11UXX)
#define SYSCTL_SLPWAKE_USBPLL_PD (1 << 8)	/*!< USB PLL wake-up configuration */
#define SYSCTL_SLPWAKE_USBPAD_PD (1 << 10)	/*!< USB transceiver wake-up configuration */
#endif

/**
 * @brief	Setup wakeup behaviour from deep sleep
 * @param	wakeupmask	: OR'ed values of SYSCTL_SLPWAKE_* values (high is powered down)
 * @return	Nothing
 * @note	This must be setup prior to using deep sleep. See the user manual
 * (PDWAKECFG register) for more info on setting this up. This function selects
 * which peripherals are powered up on exit from deep sleep.
 * This function should only be called once with all options for wakeup
 * in that call.
 */
void Chip_SYSCTL_SetWakeup(uint32_t wakeupmask);

/**
 * @brief	Return current wakeup mask
 * @return	OR'ed values of SYSCTL_SLPWAKE_* values
 * @note	A high state indicates the peripehral will powerup on wakeup.
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetWakeup(void)
{
	return LPC_SYSCTL->PDWAKECFG;
}

#endif

/**
 * Power down configuration values
 */
#define SYSCTL_POWERDOWN_IRCOUT_PD (1 << 0)	/*!< IRC oscillator output power down */
#define SYSCTL_POWERDOWN_IRC_PD    (1 << 1)	/*!< IRC oscillator power-down */
#define SYSCTL_POWERDOWN_FLASH_PD  (1 << 2)	/*!< Flash power down */
#if !defined(CHIP_LPC11AXX)
#define SYSCTL_POWERDOWN_BOD_PD    (1 << 3)	/*!< BOD power down */
#endif
#define SYSCTL_POWERDOWN_ADC_PD    (1 << 4)	/*!< ADC power down */
#define SYSCTL_POWERDOWN_SYSOSC_PD (1 << 5)	/*!< System oscillator power down */
#define SYSCTL_POWERDOWN_WDTOSC_PD (1 << 6)	/*!< Watchdog oscillator power down */
#define SYSCTL_POWERDOWN_SYSPLL_PD (1 << 7)	/*!< System PLL power down */
#if defined(CHIP_LPC11UXX)
#define SYSCTL_POWERDOWN_USBPLL_PD (1 << 8)	/*!< USB PLL power-down */
#define SYSCTL_POWERDOWN_USBPAD_PD (1 << 10)/*!< USB transceiver power-down */
#endif
#if defined(CHIP_LPC11AXX)
#define SYSCTL_POWERDOWN_LFOSC_PD (1 << 13)	/*!< Low frequency oscillator power-down */
#define SYSCTL_POWERDOWN_DAC_PD   (1 << 14)	/*!< DAC power-down */
#define SYSCTL_POWERDOWN_TS_PD    (1 << 15)	/*!< Temperature Sensor power-down */
#define SYSCTL_POWERDOWN_ACOMP_PD (1 << 16)	/*!< Analog Comparator power-down */
#endif

/**
 * @brief	Power down one or more blocks or peripherals
 * @param	powerdownmask	: OR'ed values of SYSCTL_POWERDOWN_* values
 * @return	Nothing
 */
void Chip_SYSCTL_PowerDown(uint32_t powerdownmask);

/**
 * @brief	Power up one or more blocks or peripherals
 * @param	powerupmask	: OR'ed values of SYSCTL_POWERDOWN_* values
 * @return	Nothing
 */
void Chip_SYSCTL_PowerUp(uint32_t powerupmask);

/**
 * @brief	Get power status
 * @return	OR'ed values of SYSCTL_POWERDOWN_* values
 * @note	A high state indicates the peripheral is powered down.
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetPowerStates(void)
{
	return LPC_SYSCTL->PDRUNCFG;
}

/**
 * @brief	Return the device ID
 * @return	the device ID
 */
STATIC INLINE uint32_t Chip_SYSCTL_GetDeviceID(void)
{
	return LPC_SYSCTL->DEVICEID;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /*!< __SYSCTL_11XX_H_ */
