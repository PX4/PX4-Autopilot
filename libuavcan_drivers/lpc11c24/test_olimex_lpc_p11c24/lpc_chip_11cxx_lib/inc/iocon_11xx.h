/*
 * @brief IOCON registers and control functions
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

#ifndef __IOCON_11XX_H_
#define __IOCON_11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IOCON_11XX CHIP: LPC11xx IO Control driver
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

/**
 * @brief IO Configuration Unit register block structure
 */
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
typedef struct {						/*!< LPC11AXX/LPC11UXX/LPC11EXX IOCON Structure */
	__IO uint32_t  PIO0[24];
	__IO uint32_t  PIO1[32];
} LPC_IOCON_T;

#else
/**
 * @brief LPC11XX I/O Configuration register offset
 */
typedef enum CHIP_IOCON_PIO {
	IOCON_PIO0_0 = (0x00C >> 2),
	IOCON_PIO0_1 = (0x010 >> 2),
	IOCON_PIO0_2 = (0x01C >> 2),
	IOCON_PIO0_3 = (0x02C >> 2),
	IOCON_PIO0_4 = (0x030 >> 2),
	IOCON_PIO0_5 = (0x034 >> 2),
	IOCON_PIO0_6 = (0x04C >> 2),
	IOCON_PIO0_7 = (0x050 >> 2),
	IOCON_PIO0_8 = (0x060 >> 2),
	IOCON_PIO0_9 = (0x064 >> 2),
	IOCON_PIO0_10 = (0x070 >> 2),
	IOCON_PIO0_11 = (0x074 >> 2),

	IOCON_PIO1_0 = (0x078 >> 2),
	IOCON_PIO1_1 = (0x07C >> 2),
	IOCON_PIO1_2 = (0x080 >> 2),
	IOCON_PIO1_3 = (0x090 >> 2),
	IOCON_PIO1_4 = (0x094 >> 2),
	IOCON_PIO1_5 = (0x0A0 >> 2),
	IOCON_PIO1_6 = (0x0A4 >> 2),
	IOCON_PIO1_7 = (0x0A8 >> 2),
	IOCON_PIO1_8 = (0x014 >> 2),
	IOCON_PIO1_9 = (0x038 >> 2),
	IOCON_PIO1_10 = (0x06C >> 2),
	IOCON_PIO1_11 = (0x098 >> 2),

	IOCON_PIO2_0 = (0x008 >> 2),
	IOCON_PIO2_1 = (0x028 >> 2),
	IOCON_PIO2_2 = (0x05C >> 2),
	IOCON_PIO2_3 = (0x08C >> 2),
	IOCON_PIO2_4 = (0x040 >> 2),
	IOCON_PIO2_5 = (0x044 >> 2),
	IOCON_PIO2_6 = (0x000 >> 2),
	IOCON_PIO2_7 = (0x020 >> 2),
	IOCON_PIO2_8 = (0x024 >> 2),
	IOCON_PIO2_9 = (0x054 >> 2),
	IOCON_PIO2_10 = (0x058 >> 2),
#if !defined(CHIP_LPC1125)
	IOCON_PIO2_11 = (0x070 >> 2),
#endif

	IOCON_PIO3_0 = (0x084 >> 2),
#if !defined(CHIP_LPC1125)
	IOCON_PIO3_1 = (0x088 >> 2),
#endif    
	IOCON_PIO3_2 = (0x09C >> 2),
	IOCON_PIO3_3 = (0x0AC >> 2),
	IOCON_PIO3_4 = (0x03C >> 2),
	IOCON_PIO3_5 = (0x048 >> 2),
} CHIP_IOCON_PIO_T;

/**
 * @brief LPC11XX Pin location select
 */
typedef enum CHIP_IOCON_PIN_LOC {
	IOCON_SCKLOC_PIO0_10        = (0xB0),		/*!< Selects SCK0 function in pin location PIO0_10 */
#if !defined(CHIP_LPC1125)
	IOCON_SCKLOC_PIO2_11        = (0xB0 | 1),	/*!< Selects SCK0 function in pin location PIO2_11 */
#endif    
	IOCON_SCKLOC_PIO0_6         = (0xB0 | 2),	/*!< Selects SCK0 function in pin location PIO0_6 */
    
	IOCON_DSRLOC_PIO2_1         = (0xB4),		/*!< Selects DSR function in pin location PIO2_1 */
#if !defined(CHIP_LPC1125)
	IOCON_DSRLOC_PIO3_1         = (0xB4 | 1),	/*!< Selects DSR function in pin location PIO3_1 */
#endif    
    
	IOCON_DCDLOC_PIO2_2         = (0xB8),		/*!< Selects DCD function in pin location PIO2_2 */
	IOCON_DCDLOC_PIO3_2         = (0xB8 | 1),	/*!< Selects DCD function in pin location PIO3_2 */
    
	IOCON_RILOC_PIO2_3          = (0xBC),		/*!< Selects RI function in pin location PIO2_3 */
	IOCON_RILOC_PIO3_3          = (0xBC | 1),	/*!< Selects Ri function in pin location PIO3_3 */

#if defined(CHIP_LPC1125)
	IOCON_SSEL1_LOC_PIO2_2      = (0x18),		/*!< Selects SSEL1 function in pin location PIO2_2 */
	IOCON_SSEL1_LOC_PIO2_4      = (0x18 | 1),	/*!< Selects SSEL1 function in pin location PIO2_4 */
	
    IOCON_CT16B0_CAP0_LOC_PIO0_2 = (0xC0),		/*!< Selects SSEL1 CTB16B0_CAP0 function in pin location PIO0_2 */
    IOCON_CT16B0_CAP0_LOC_PIO3_3 = (0xC0 | 1),	/*!< Selects SSEL1 CTB16B0_CAP0 function in pin location PIO3_3 */
    
    IOCON_SCK1_LOC_PIO2_1       = (0xC4),		/*!< Selects SCK1 function in pin location PIO2_1 */
    IOCON_SCK1_LOC_PIO3_2       = (0xC4 | 1),	/*!< Selects SCK1 function in pin location PIO3_2 */
    
    IOCON_MISO1_LOC_PIO2_2      = (0xC8),		/*!< Selects MISO1 function in pin location PIO2_2 */
    IOCON_MISO1_LOC_PIO1_10     = (0xC8 | 1),	/*!< Selects MISO1 function in pin location PIO1_10 */
    
    IOCON_MOSI1_LOC_PIO2_3      = (0xCC),		/*!< Selects MOSI1 function in pin location PIO2_3 */
    IOCON_MOSI1_LOC_PIO1_9      = (0xCC),		/*!< Selects MOSI1 function in pin location PIO1_9 */
    
    IOCON_CT326B0_CAP0_LOC_PIO1_5 = (0xD0),		/*!< Selects CT32B0_CAP0 function in pin location PIO1_5 */
    IOCON_CT326B0_CAP0_LOC_PIO2_9 = (0xD0 | 1),	/*!< Selects CT32B0_CAP0 function in pin location PIO2_9 */
    
    IOCON_U0_RXD_LOC_PIO1_6     = (0xD4),		/*!< Selects U0 RXD function in pin location PIO1_6 */
    IOCON_U0_RXD_LOC_PIO2_7     = (0xD4 | 1),   /*!< Selects U0 RXD function in pin location PIO2_7 */
    IOCON_U0_RXD_LOC_PIO3_4     = (0xD4 | 3),   /*!< Selects U0 RXD function in pin location PIO3_4 */
#endif

} CHIP_IOCON_PIN_LOC_T;

typedef struct {						/*!< LPC11XX/LPC11XXLV/LPC11UXX IOCON Structure */
	__IO uint32_t  REG[48];
} LPC_IOCON_T;
#endif

/**
 * IOCON function and mode selection definitions
 * See the User Manual for specific modes and functions supported by the
 * various LPC11xx devices. Functionality can vary per device.
 */
#define IOCON_FUNC0             0x0				/*!< Selects pin function 0 */
#define IOCON_FUNC1             0x1				/*!< Selects pin function 1 */
#define IOCON_FUNC2             0x2				/*!< Selects pin function 2 */
#define IOCON_FUNC3             0x3				/*!< Selects pin function 3 */
#define IOCON_FUNC4             0x4				/*!< Selects pin function 4 */
#define IOCON_FUNC5             0x5				/*!< Selects pin function 5 */
#define IOCON_FUNC6             0x6				/*!< Selects pin function 6 */
#define IOCON_FUNC7             0x7				/*!< Selects pin function 7 */
#define IOCON_MODE_INACT        (0x0 << 3)		/*!< No addition pin function */
#define IOCON_MODE_PULLDOWN     (0x1 << 3)		/*!< Selects pull-down function */
#define IOCON_MODE_PULLUP       (0x2 << 3)		/*!< Selects pull-up function */
#define IOCON_MODE_REPEATER     (0x3 << 3)		/*!< Selects pin repeater function */
#define IOCON_HYS_EN            (0x1 << 5)		/*!< Enables hysteresis */
#define IOCON_INV_EN            (0x1 << 6)		/*!< Enables invert function on input */
#define IOCON_ADMODE_EN         (0x0 << 7)		/*!< Enables analog input function (analog pins only) */
#define IOCON_DIGMODE_EN        (0x1 << 7)		/*!< Enables digital function (analog pins only) */
#define IOCON_SFI2C_EN          (0x0 << 8)		/*!< I2C standard mode/fast-mode */
#define IOCON_STDI2C_EN         (0x1 << 8)		/*!< I2C standard I/O functionality */
#define IOCON_FASTI2C_EN        (0x2 << 8)		/*!< I2C Fast-mode Plus */
#define IOCON_FILT_DIS          (0x1 << 8)		/*!< Disables noise pulses filtering (10nS glitch filter) */
#define IOCON_OPENDRAIN_EN      (0x1 << 10)		/*!< Enables open-drain function */

/**
 * IOCON function and mode selection definitions (old)
 * For backwards compatibility.
 */
#define MD_PLN					(0x0 << 3)		/*!< Disable pull-down and pull-up resistor at resistor at pad */
#define MD_PDN					(0x1 << 3)		/*!< Enable pull-down resistor at pad */
#define MD_PUP					(0x2 << 3)		/*!< Enable pull-up resistor at pad */
#define MD_BUK					(0x3 << 3)		/*!< Enable pull-down and pull-up resistor at resistor at pad (repeater mode) */
#define MD_HYS					(0x1 << 5)		/*!< Enable hysteresis */
#define MD_INV					(0x1 << 6)		/*!< Invert enable */
#define MD_ADMODE				(0x0 << 7)		/*!< Select analog mode */
#define MD_DIGMODE				(0x1 << 7)		/*!< Select digitial mode */
#define MD_DISFIL				(0x0 << 8)		/*!< Disable 10nS input glitch filter */
#define MD_ENFIL				(0x1 << 8)		/*!< Enable 10nS input glitch filter */
#define MD_SFI2C				(0x0 << 8)		/*!< I2C standard mode/fast-mode */
#define MD_STDI2C				(0x1 << 8)		/*!< I2C standard I/O functionality */
#define MD_FASTI2C				(0x2 << 8)		/*!< I2C Fast-mode Plus */
#define MD_OPENDRAIN			(0x1 << 10)		/*!< Open drain mode bit */
#define FUNC0 0x0
#define FUNC1 0x1
#define FUNC2 0x2
#define FUNC3 0x3
#define FUNC4 0x4
#define FUNC5 0x5
#define FUNC6 0x6
#define FUNC7 0x7

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
/**
 * @brief	Sets I/O Control pin mux
 * @param	pIOCON		: The base of IOCON peripheral on the chip
 * @param	port		: GPIO port to mux
 * @param	pin			: GPIO pin to mux
 * @param	modefunc	: OR'ed values or type IOCON_*
 * @return	Nothing
 */
void Chip_IOCON_PinMuxSet(LPC_IOCON_T *pIOCON, uint8_t port, uint8_t pin, uint32_t modefunc);

/**
 * @brief	I/O Control pin mux
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	port	: GPIO port to mux
 * @param	pin		: GPIO pin to mux
 * @param	mode	: OR'ed values or type IOCON_*
 * @param	func	: Pin function, value of type IOCON_FUNC?
 * @return	Nothing
 */
STATIC INLINE void Chip_IOCON_PinMux(LPC_IOCON_T *pIOCON, uint8_t port, uint8_t pin, uint16_t mode, uint8_t func)
{
	Chip_IOCON_PinMuxSet(pIOCON, port, pin, (uint32_t) (mode | func));
}

#else

/**
 * @brief	Sets I/O Control pin mux
 * @param	pIOCON		: The base of IOCON peripheral on the chip
 * @param	pin			: GPIO pin to mux
 * @param	modefunc	: OR'ed values or type IOCON_*
 * @return	Nothing
 */
STATIC INLINE void Chip_IOCON_PinMuxSet(LPC_IOCON_T *pIOCON, CHIP_IOCON_PIO_T pin, uint32_t modefunc)
{
	pIOCON->REG[pin] = modefunc;
}

/**
 * @brief	I/O Control pin mux
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: GPIO pin to mux
 * @param	mode	: OR'ed values or type IOCON_*
 * @param	func	: Pin function, value of type IOCON_FUNC?
 * @return	Nothing
 */
STATIC INLINE void Chip_IOCON_PinMux(LPC_IOCON_T *pIOCON, CHIP_IOCON_PIO_T pin, uint16_t mode, uint8_t func)
{
	Chip_IOCON_PinMuxSet(pIOCON, pin, (uint32_t) (mode | func));
}

/**
 * @brief	Select pin location
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	sel		: location selection
 * @return	Nothing
 */
STATIC INLINE void Chip_IOCON_PinLocSel(LPC_IOCON_T *pIOCON, CHIP_IOCON_PIN_LOC_T sel)
{
	pIOCON->REG[sel >> 2] = sel & 0x03;
}

#endif /* defined(CHIP_LPC11UXX) || defined (CHIP_LPC11EXX) || defined (CHIP_LPC11AXX) */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __IOCON_11XX_H_ */
