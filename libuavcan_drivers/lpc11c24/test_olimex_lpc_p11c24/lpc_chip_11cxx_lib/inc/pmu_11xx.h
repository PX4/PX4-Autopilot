/*
 * @brief LPC11xx PMU chip driver
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

#ifndef __PMU_11XX_H_
#define __PMU_11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup PMU_11XX CHIP: LPC11xx Power Management Unit block driver
 * @ingroup CHIP_11XX_Drivers
 * This driver only applies to devices in the CHIP_LPC11AXX, CHIP_LPC11CXX,
 * CHIP_LPC11EXX, CHIP_LPC11UXX, and CHIP_LPC1125 families. Note different
 * families may have slightly different PMU support.
 * @{
 */

#if defined(CHIP_LPC11AXX) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC1125)
#if defined(CHIP_LPC1125)
#error "LPC1125 support for the PMU driver is not ready"
#endif
 
/**
 * @brief LPC11xx Power Management Unit register block structure
 */
typedef struct {
	__IO uint32_t PCON;		/*!< Offset: 0x000 Power control Register (R/W) */
	__IO uint32_t GPREG[4];	/*!< Offset: 0x004 General purpose Registers 0..3 (R/W) */
} LPC_PMU_T;

/**
 * @brief LPC11xx low power mode type definitions
 */
typedef enum CHIP_PMU_MCUPOWER {
	PMU_MCU_SLEEP = 0,			/*!< Sleep mode */
#if defined(CHIP_LPC11AXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX)
	PMU_MCU_DEEP_SLEEP,			/*!< Deep Sleep mode */
	PMU_MCU_POWER_DOWN,			/*!< Power down mode */
	PMU_MCU_DEEP_PWRDOWN		/*!< Deep power down mode */
#elif defined(CHIP_LPC11CXX)
	PMU_MCU_DEEP_PWRDOWN = 3	/*!< Deep power down mode */
#endif
} CHIP_PMU_MCUPOWER_T;

/**
 * PMU PCON register bit fields & masks
 */
#define PMU_PCON_PM_SLEEP			(0x0)		/*!< ARM WFI enter sleep mode */
#if defined(CHIP_LPC11AXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX)
#define PMU_PCON_PM_DEEPSLEEP		(0x1)		/*!< ARM WFI enter Deep-sleep mode */
#define PMU_PCON_PM_POWERDOWN		(0x2)		/*!< ARM WFI enter Power-down mode */
#define PMU_PCON_PM_DEEPPOWERDOWN	(0x3)		/*!< ARM WFI enter Deep Power-down mode */
#elif defined(CHIP_LPC11CXX)
#define PMU_PCON_PM_DEEPPOWERDOWN   (0x2)
#endif
#define PMU_PCON_SLEEPFLAG			(1 << 8)	/*!< Sleep mode flag */
#define PMU_PCON_DPDFLAG			(1 << 11)	/*!< Deep power-down flag */

/**
 * @brief	Write a value to a GPREG register
 * @param	pPMU		: Pointer to PMU register block
 * @param	regIndex	: Register index to write to, must be 0..3
 * @param	value		: Value to write
 * @return	None
 */
STATIC INLINE void Chip_PMU_WriteGPREG(LPC_PMU_T *pPMU, uint8_t regIndex, uint32_t value)
{
	pPMU->GPREG[regIndex] = value;
}

/**
 * @brief	Read a value to a GPREG register
 * @param	pPMU		: Pointer to PMU register block
 * @param	regIndex	: Register index to read from, must be 0..3
 * @return	Value read from the GPREG register
 */
STATIC INLINE uint32_t Chip_PMU_ReadGPREG(LPC_PMU_T *pPMU, uint8_t regIndex)
{
	return pPMU->GPREG[regIndex];
}

/**
 * @brief	Enter MCU Sleep mode
 * @param	pPMU	: Pointer to PMU register block
 * @return	None
 * @note	The sleep mode affects the ARM Cortex-M0+ core only. Peripherals
 * and memories are active. 
 */
void Chip_PMU_SleepState(LPC_PMU_T *pPMU);

#if defined(CHIP_LPC11AXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11UXX)
/**
 * @brief	Enter MCU Deep Sleep mode
 * @param	pPMU	: Pointer to PMU register block
 * @return	None
 * @note	In Deep-sleep mode, the peripherals receive no internal clocks.
 * The flash is in stand-by mode. The SRAM memory and all peripheral registers
 * as well as the processor maintain their internal states. The WWDT, WKT,
 * and BOD can remain active to wake up the system on an interrupt.
 */
void Chip_PMU_DeepSleepState(LPC_PMU_T *pPMU);

/**
 * @brief	Enter MCU Power down mode
 * @param	pPMU	: Pointer to PMU register block
 * @return	None
 * @note	In Power-down mode, the peripherals receive no internal clocks.
 * The internal SRAM memory and all peripheral registers as well as the
 * processor maintain their internal states. The flash memory is powered
 * down. The WWDT, WKT, and BOD can remain active to wake up the system
 * on an interrupt.
 */
void Chip_PMU_PowerDownState(LPC_PMU_T *pPMU);
#endif

/**
 * @brief	Enter MCU Deep Power down mode
 * @param	pPMU	: Pointer to PMU register block
 * @return	None
 * @note	For maximal power savings, the entire system is shut down
 * except for the general purpose registers in the PMU and the self
 * wake-up timer. Only the general purpose registers in the PMU maintain
 * their internal states. The part can wake up on a pulse on the WAKEUP
 * pin or when the self wake-up timer times out. On wake-up, the part
 * reboots.
 */
void Chip_PMU_DeepPowerDownState(LPC_PMU_T *pPMU);

/**
 * @brief	Place the MCU in a low power state
 * @param	pPMU		: Pointer to PMU register block
 * @param	SleepMode	: Sleep mode
 * @return	None
 */
void Chip_PMU_Sleep(LPC_PMU_T *pPMU, CHIP_PMU_MCUPOWER_T SleepMode);

/**
 * @brief	Returns sleep/power-down flags
 * @param	pPMU	: Pointer to PMU register block
 * @return	Or'ed values of PMU_PCON_SLEEPFLAG and PMU_PCON_DPDFLAG
 * @note	These indicate that the PMU is setup for entry into a low
 * power state on the next WFI() instruction.
 */
STATIC INLINE uint32_t Chip_PMU_GetSleepFlags(LPC_PMU_T *pPMU)
{
	return (pPMU->PCON & (PMU_PCON_SLEEPFLAG | PMU_PCON_DPDFLAG));
}

/**
 * @brief	Clears sleep/power-down flags
 * @param	pPMU	: Pointer to PMU register block
 * @param	flags	: Or'ed value of PMU_PCON_SLEEPFLAG and PMU_PCON_DPDFLAG
 * @return	Nothing
 * @note	Use this function to clear a low power state prior to calling
 * WFI().
 */
STATIC INLINE void Chip_PMU_ClearSleepFlags(LPC_PMU_T *pPMU, uint32_t flags)
{
	pPMU->PCON &= ~flags;
}

#endif /* defined(CHIP_LPC11AXX) || defined(CHIP_LPC11CXX) || ... */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __PMU_11XX_H_ */
