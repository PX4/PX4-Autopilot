#pragma once

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#define	RP2040_GPIO_PAD_MASK	0xff		// GPIO PAD register mask
#define	RP2040_GPIO_PIN_MASK	0x1f00		// GPIO pin number mask
#define	RP2040_GPIO_FUN_MASK	0x3e000		// GPIO pin function mask
#define	RP2040_GPIO_OEN_MASK	0x40000		// GPIO output enable mask
#define	RP2040_GPIO_OUT_MASK	0x80000		// GPIO output value mask

#define	RP2040_PADS_BANK0_GPIO_RESET	0x56	// GPIO PAD register default value
#define	RP2040_IO_BANK0_GPIO_CTRL_RESET	0x1f	// GPIO_CTRL register default value

#include <rp2040_spi.h>
#include <rp2040_i2c.h>
#include <rp2040_gpio.h>

// RP2040 doesn't really have a cpu register with unique id.
// However, there is a function in pico-sdk which can provide
// a device unique id from its flash which is 64 bits in length.
// For now, a common device id will be used for all RP2040 based devices.
// This can be done by defining a macro in the board's board_config.h file as shown below
// #define BOARD_OVERRIDE_UUID "MYFC2040"	// must be of length 8

#define PX4_CPU_UUID_BYTE_LENGTH                8
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* By not defining PX4_CPU_UUID_CORRECT_CORRELATION the following maintains the legacy incorrect order
 * used for selection of significant digits of the UUID in the PX4 code base.
 * This is done to avoid the ripple effects changing the IDs used on STM32 base platforms
 */
#if defined(PX4_CPU_UUID_CORRECT_CORRELATION)
# define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            2 /* Most significant digits change the least */
#else
/* Legacy incorrect ordering */
# define PX4_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
#endif

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

// #define px4_savepanic(fileno, context, length)  stm32_bbsram_savepanic(fileno, context, length)
#define px4_savepanic(fileno, context, length)  (0)	// Turn off px4_savepanic for rp2040 as it is not implemented in nuttx

#define PX4_BUS_OFFSET       1                  /* RP2040 buses are 0 based and adjustment is needed */
#define px4_spibus_initialize(bus_num_0based)   rp2040_spibus_initialize(bus_num_0based)

#define px4_i2cbus_initialize(bus_num_0based)   rp2040_i2cbus_initialize(bus_num_0based)
#define px4_i2cbus_uninitialize(pdev)           rp2040_i2cbus_uninitialize(pdev)


// This part of the code is specific to rp2040.
// RP2040 does not have the gpio configuration process similar to stm or tiva devices.
// There are multiple different registers which are required to be configured based on the function selection.
// The pinset below can be defined using a 32-bit value where,
// bits		Function
// 0-7		PADS Bank 0 GPIO register. Take a look at rp2040 datasheet page 321.
// 8-15		GPIO number. 0-29 is valid.
// 16-23	GPIO function select
// 24		Output enable
// 25		Output value
// 26-31	Unused
// Take a look at k66 in nxp for referance.
void rp2040_pinconfig(uint32_t pinset){}
#define px4_arch_configgpio(pinset)             rp2040_pinconfig(pinset)		// Implemented in io_pins/pin_config.c
#define px4_arch_unconfiggpio(pinset)           0					// Needs to be implemented (can be done in io_pins)
#define px4_arch_gpioread(pinset)               rp2040_gpio_get(pinset)			// Use gpio_get
#define px4_arch_gpiowrite(pinset, value)       rp2040_gpio_put(pinset, value)		// Use gpio_put
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  0					// Needs to be implemented (can be done in io_pins)

// Following are quick defines to be used with the functions defined above
// These defines create a bit-mask which is supposed to be used in the
// functions defined above to set up gpios correctly.
#define PX4_MAKE_GPIO_INPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET))

#define PX4_GPIO_PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_2MHz))

__END_DECLS
