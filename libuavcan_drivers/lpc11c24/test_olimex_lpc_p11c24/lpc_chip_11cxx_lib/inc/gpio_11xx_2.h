/*
 * @brief LPC11xx GPIO driver for CHIP_LPC11CXX, CHIP_LPC110X, CHIP_LPC11XXLV,
 * and CHIP_LPC1125 families only.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
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

#ifndef __GPIO_11XX_2_H_
#define __GPIO_11XX_2_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup GPIO_11XX_2 CHIP: LPC11xx GPIO driver for CHIP_LPC11CXX, CHIP_LPC110X, and CHIP_LPC11XXLV families
 * @ingroup CHIP_11XX_Drivers
 * For device familes identified with CHIP definitions CHIP_LPC11CXX,
 * CHIP_LPC110X, and CHIP_LPC11XXLV only.
 * @{
 */

#if defined(CHIP_LPC11CXX) || defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC1125)

/**
 * @brief  GPIO port register block structure
 */
typedef struct {				/*!< GPIO_PORT Structure */
	__IO uint32_t DATA[4096];			/*!< Offset: 0x0000 to 0x3FFC Data address masking register (R/W) */
	uint32_t RESERVED1[4096];
	__IO uint32_t DIR;					/*!< Offset: 0x8000 Data direction register (R/W) */
	__IO uint32_t IS;					/*!< Offset: 0x8004 Interrupt sense register (R/W) */
	__IO uint32_t IBE;					/*!< Offset: 0x8008 Interrupt both edges register (R/W) */
	__IO uint32_t IEV;					/*!< Offset: 0x800C Interrupt event register  (R/W) */
	__IO uint32_t IE;					/*!< Offset: 0x8010 Interrupt mask register (R/W) */
	__I  uint32_t RIS;					/*!< Offset: 0x8014 Raw interrupt status register (R/ ) */
	__I  uint32_t MIS;					/*!< Offset: 0x8018 Masked interrupt status register (R/ ) */
	__O  uint32_t IC;					/*!< Offset: 0x801C Interrupt clear register (W) */
	uint32_t RESERVED2[8184];			/* Padding added for aligning contiguous GPIO blocks */
} LPC_GPIO_T;

/**
 * @brief	Initialize GPIO block
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @return	Nothing
 */
void Chip_GPIO_Init(LPC_GPIO_T *pGPIO);

/**
 * @brief	De-Initialize GPIO block
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @return	Nothing
 */
void Chip_GPIO_DeInit(LPC_GPIO_T *pGPIO);

/**
 * @brief	Set a GPIO port/bit state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to set
 * @param	bit		: GPIO bit to set
 * @param	setting	: true for high, false for low
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_WritePortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit, bool setting)
{
	pGPIO[port].DATA[1 << bit] = setting << bit;
}

/**
 * @brief	Set a GPIO pin state via the GPIO byte register
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: GPIO pin to set
 * @param	setting	: true for high, false for low
 * @return	Nothing
 * @note	This function replaces Chip_GPIO_WritePortBit()
 */
STATIC INLINE void Chip_GPIO_SetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting)
{
	pGPIO[port].DATA[1 << pin] = setting << pin;
}

/**
 * @brief	Read a GPIO state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to read
 * @param	bit		: GPIO bit to read
 * @return	true of the GPIO is high, false if low
 * @note	It is recommended to use the Chip_GPIO_GetPinState() function instead.
 */
STATIC INLINE bool Chip_GPIO_ReadPortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit)
{
	return (bool) ((pGPIO[port].DATA[1 << bit] >> bit) & 1);
}

/**
 * @brief	Get a GPIO pin state via the GPIO byte register
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: GPIO pin to get state for
 * @return	true if the GPIO is high, false if low
 * @note	This function replaces Chip_GPIO_ReadPortBit()
 */
STATIC INLINE bool Chip_GPIO_GetPinState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	return (pGPIO[port].DATA[1 << pin]) != 0;
}

/**
 * @brief	Seta GPIO direction
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to set
 * @param	bit		: GPIO bit to set
 * @param	setting	: true for output, false for input
 * @return	Nothing
 * @note	It is recommended to use the Chip_GPIO_SetPinDIROutput(), 
 * Chip_GPIO_SetPinDIRInput() or Chip_GPIO_SetPinDIR() functions instead
 * of this function.
 */
void Chip_GPIO_WriteDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit, bool setting);

/**
 * @brief	Set GPIO direction for a single GPIO pin to an output
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: GPIO pin to set direction on as output
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetPinDIROutput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO[port].DIR |= (1UL << pin);
}

/**
 * @brief	Set GPIO direction for a single GPIO pin to an input
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: GPIO pin to set direction on as input
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetPinDIRInput(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO[port].DIR &= ~(1UL << pin);
}

/**
 * @brief	Set GPIO direction for a single GPIO pin
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: GPIO pin to set direction for
 * @param	output	: true for output, false for input
 * @return	Nothing
 */
void Chip_GPIO_SetPinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool output);

/**
 * @brief	Read a GPIO direction (out or in)
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to read
 * @param	bit		: GPIO bit to read
 * @return	true of the GPIO is an output, false if input
 * @note	It is recommended to use the Chip_GPIO_GetPinDIR() function instead.
 */
STATIC INLINE bool Chip_GPIO_ReadDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit)
{
	return (bool) (((pGPIO[port].DIR) >> bit) & 1);
}

/**
 * @brief	Get GPIO direction for a single GPIO pin
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: GPIO pin to get direction for
 * @return	true if the GPIO is an output, false if input
 */
STATIC INLINE bool Chip_GPIO_GetPinDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	return (bool) (pGPIO[port].DIR >> pin) & 1;
}

/**
 * @brief	Set Direction for a GPIO port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port Number
 * @param	bit		: GPIO bit to set
 * @param	out		: Direction value, 0 = input, !0 = output
 * @return	None
 * @note	Bits set to '0' are not altered. It is recommended to use the
 * Chip_GPIO_SetPortDIR() function instead.
 */
void Chip_GPIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t bit, uint8_t out);

/**
 * @brief	Set GPIO direction for a all selected GPIO pins to an output
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinMask	: GPIO pin mask to set direction on as output (bits 0..n for pins 0..n)
 * @return	Nothing
 * @note	Sets multiple GPIO pins to the output direction, each bit's position that is
 * high sets the corresponding pin number for that bit to an output.
 */
STATIC INLINE void Chip_GPIO_SetPortDIROutput(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinMask)
{
	pGPIO[port].DIR |= pinMask;
}

/**
 * @brief	Set GPIO direction for a all selected GPIO pins to an input
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinMask	: GPIO pin mask to set direction on as input (bits 0..b for pins 0..n)
 * @return	Nothing
 * @note	Sets multiple GPIO pins to the input direction, each bit's position that is
 * high sets the corresponding pin number for that bit to an input.
 */
STATIC INLINE void Chip_GPIO_SetPortDIRInput(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinMask)
{
	pGPIO[port].DIR &= ~pinMask;
}

/**
 * @brief	Set GPIO direction for a all selected GPIO pins to an input or output
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinMask	: GPIO pin mask to set direction on (bits 0..b for pins 0..n)
 * @param	outSet	: Direction value, false = set as inputs, true = set as outputs
 * @return	Nothing
 * @note	Sets multiple GPIO pins to the input direction, each bit's position that is
 * high sets the corresponding pin number for that bit to an input.
 */
void Chip_GPIO_SetPortDIR(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinMask, bool outSet);

/**
 * @brief	Get GPIO direction for a all GPIO pins
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	a bitfield containing the input and output states for each pin
 * @note	For pins 0..n, a high state in a bit corresponds to an output state for the
 * same pin, while a low  state corresponds to an input state.
 */
STATIC INLINE uint32_t Chip_GPIO_GetPortDIR(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO[port].DIR;
}

/**
 * @brief	Set all GPIO raw pin states (regardless of masking)
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	value	: Value to set all GPIO pin states (0..n) to
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetPortValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t value)
{
	pGPIO[port].DATA[0xFFF] = value;
}

/**
 * @brief	Get all GPIO raw pin states (regardless of masking)
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	Current (raw) state of all GPIO pins
 */
STATIC INLINE uint32_t Chip_GPIO_GetPortValue(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO[port].DATA[0xFFF];
}

/**
 * @brief	Set a GPIO port/bit to the high state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	bit		: Bit(s) in the port to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output. It is recommended to use the
 * Chip_GPIO_SetPortOutHigh() function instead.
 */
STATIC INLINE void Chip_GPIO_SetValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t bit)
{
	pGPIO[port].DATA[bit] = bit;
}

/**
 * @brief	Set selected GPIO output pins to the high state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pins	: pins (0..n) to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetPortOutHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
	pGPIO[port].DATA[pins] = 0xFFF;
}

/**
 * @brief	Set an individual GPIO output pin to the high state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: pin number (0..n) to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetPinOutHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO[port].DATA[1 << pin] = (1 << pin);
}

/**
 * @brief	Set a GPIO port/bit to the low state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	bit		: Bit(s) in the port to set low
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_ClearValue(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t bit)
{
	pGPIO[port].DATA[bit] = ~bit;
}

/**
 * @brief	Set selected GPIO output pins to the low state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pins	: pins (0..n) to set low
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetPortOutLow(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
	pGPIO[port].DATA[pins] = 0;
}

/**
 * @brief	Set an individual GPIO output pin to the low state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: pin number (0..n) to set low
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetPinOutLow(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO[port].DATA[1 << pin] = 0;
}

/**
 * @brief	Toggle selected GPIO output pins to the opposite state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pins	: pins (0..n) to toggle
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetPortToggle(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pins)
{
	pGPIO[port].DATA[pins] ^= 0xFFF;
}

/**
 * @brief	Toggle an individual GPIO output pin to the opposite state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pin		: pin number (0..n) to toggle
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetPinToggle(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO[port].DATA[1 << pin] ^= (1 << pin);
}

/**
 * @brief	Read current bit states for the selected port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number to read
 * @return	Current value of GPIO port
 * @note	The current states of the bits for the port are read, regardless of
 * whether the GPIO port bits are input or output.
 */
STATIC INLINE uint32_t Chip_GPIO_ReadValue(LPC_GPIO_T *pGPIO, uint8_t port)
{
	return pGPIO[port].DATA[4095];
}

/**
 * @brief	Configure the pins as edge sensitive for interrupts 
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to set to edge mode (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetPinModeEdge(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IS &= ~pinmask;
}

/**
 * @brief	Configure the pins as level sensitive for interrupts 
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to set to level mode (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetPinModeLevel(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IS |= pinmask;
}

/**
 * @brief	Returns current GPIO edge or high level interrupt configuration for all pins for a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	A bifield containing the edge/level interrupt configuration for each
 * pin for the selected port. Bit 0 = pin 0, 1 = pin 1.
 * For each bit, a 0 means the edge interrupt is configured, while a 1 means a level
 * interrupt is configured. Mask with this return value to determine the
 * edge/level configuration for each pin in a port.
 */
STATIC INLINE uint32_t Chip_GPIO_IsLevelEnabled(LPC_GPIO_T *pGPIO, uint8_t port)
{
    return pGPIO[port].IS;
}

/**
 * @brief	Sets GPIO interrupt configuration for both edges for selected pins
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to set to dual edge mode (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetEdgeModeBoth(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IBE |= pinmask;
}

/**
 * @brief	Sets GPIO interrupt configuration for a single edge for selected pins
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to set to single edge mode (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_SetEdgeModeSingle(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IBE &= ~pinmask;
}

/**
 * @brief	Returns current GPIO interrupt dual or single edge configuration for all pins for a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	A bifield containing the single/dual interrupt configuration for each
 * pin for the selected port. Bit 0 = pin 0, 1 = pin 1.
 * For each bit, a 0 means the interrupt triggers on a single edge (use the
 * Chip_GPIO_SetEdgeModeHigh() and Chip_GPIO_SetEdgeModeLow() functions to configure
 * selected edge), while a 1 means the interrupt triggers on both edges. Mask
 * with this return value to determine the edge/level configuration for each pin in
 * a port.
 */
STATIC INLINE uint32_t Chip_GPIO_GetEdgeModeDir(LPC_GPIO_T *pGPIO, uint8_t port)
{
    return pGPIO[port].IBE;
}

/**
 * @brief	Sets GPIO interrupt configuration when in single edge or level mode to high edge trigger or high level
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to set to high mode (ORed value of bits 0..11)
 * @return	Nothing
 * @note	Use this function to select high level or high edge interrupt mode
 * for the selected pins on the selected port when not in dual edge mode.
 */
STATIC INLINE void Chip_GPIO_SetModeHigh(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IEV |= pinmask;
}

/**
 * @brief	Sets GPIO interrupt configuration when in single edge or level mode to low edge trigger or low level
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to set to low mode (ORed value of bits 0..11)
 * @return	Nothing
 * @note	Use this function to select low level or low edge interrupt mode
 * for the selected pins on the selected port when not in dual edge mode.
 */
STATIC INLINE void Chip_GPIO_SetModeLow(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IEV &= ~pinmask;
}

/**
 * @brief	Returns current GPIO interrupt edge direction or level mode
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	A bifield containing the low or high direction of the interrupt
 * configuration for each pin for the selected port. Bit 0 = pin 0, 1 = pin 1.
 * For each bit, a 0 means the interrupt triggers on a low level or edge, while a
 * 1 means the interrupt triggers on a high level or edge. Mask with this
 * return value to determine the high/low configuration for each pin in a port.
 */
STATIC INLINE uint32_t Chip_GPIO_GetModeHighLow(LPC_GPIO_T *pGPIO, uint8_t port)
{
    return pGPIO[port].IEV;
}

/**
 * @brief	Enables interrupts for selected pins on a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to enable interrupts for (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_EnableInt(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IE |= pinmask;
}

/**
 * @brief	Disables interrupts for selected pins on a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to disable interrupts for (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_DisableInt(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IE &= ~pinmask;
}

/**
 * @brief	Returns current enable pin interrupts for a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	A bifield containing the enabled pin interrupts (0..11)
 */
STATIC INLINE uint32_t Chip_GPIO_GetEnabledInts(LPC_GPIO_T *pGPIO, uint8_t port)
{
    return pGPIO[port].IE;
}

/**
 * @brief	Returns raw interrupt pending status for pin interrupts for a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	A bifield containing the raw pending interrupt states for each pin (0..11) on the port
 */
STATIC INLINE uint32_t Chip_GPIO_GetRawInts(LPC_GPIO_T *pGPIO, uint8_t port)
{
    return pGPIO[port].RIS;
}

/**
 * @brief	Returns masked interrupt pending status for pin interrupts for a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @return	A bifield containing the masked pending interrupt states for each pin (0..11) on the port
 */
STATIC INLINE uint32_t Chip_GPIO_GetMaskedInts(LPC_GPIO_T *pGPIO, uint8_t port)
{
    return pGPIO[port].MIS;
}

/**
 * @brief	Clears pending interrupts for selected pins for a port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: Port number
 * @param	pinmask	: Pins to clear interrupts for (ORed value of bits 0..11)
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_ClearInts(LPC_GPIO_T *pGPIO, uint8_t port, uint32_t pinmask)
{
    pGPIO[port].IC = pinmask;
}

/**
 * @brief  GPIO interrupt mode definitions
 */
typedef enum {
	GPIO_INT_ACTIVE_LOW_LEVEL  = 0x0,		/*!< Selects interrupt on pin to be triggered on LOW level */
	GPIO_INT_ACTIVE_HIGH_LEVEL = 0x1,		/*!< Selects interrupt on pin to be triggered on HIGH level */
	GPIO_INT_FALLING_EDGE      = 0x2,		/*!< Selects interrupt on pin to be triggered on FALLING level */
	GPIO_INT_RISING_EDGE       = 0x3,		/*!< Selects interrupt on pin to be triggered on RISING level */
	GPIO_INT_BOTH_EDGES        = 0x6		/*!< Selects interrupt on pin to be triggered on both edges */
} GPIO_INT_MODE_T;

/**
 * @brief	Composite function for setting up a full interrupt configuration for a single pin
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	port		: Port number
 * @param	pin			: Pin number (0..11)
 * @param	modeFlags	: GPIO interrupt mode selection
 * @return	Nothing
 */
void Chip_GPIO_SetupPinInt(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, GPIO_INT_MODE_T mode);

#endif /* defined(CHIP_LPC11CXX) || defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC1125) */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_11XX_2_H_ */
