#pragma once

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

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
// However, only four value are required for the most part: Pin number, Pull up/down, direction, set/clear and function
// The pinset below can be defined using a 16-bit value where,
// bits		Function
// 0-4		GPIO number. 0-29 is valid.
// 5		Pull up
// 6		Pull down
// 7		Direction
// 8		Set/clear
// 9-13		GPIO function select
// 14-15	Unused
#define GPIO_PU		(1 << 5)	// Pull-up resistor
#define GPIO_PD		(1 << 6)	// Pull-down resistor
#define GPIO_OUT	(1 << 7)	// Output enable
#define GPIO_SET	(1 << 8)	// Output set
#define GPIO_FUN(func)	(func << 9)	// Function select

#define GPIO_NUM_MASK	0x1f
#define	GPIO_PU_MASK	0x20		// GPIO PAD register mask
#define	GPIO_PD_MASK	0x40		// GPIO pin number mask
#define	GPIO_OUT_MASK	0x80		// GPIO pin function mask
#define	GPIO_SET_MASK	0x100		// GPIO pin function mask
#define	GPIO_FUN_MASK	0x3E00		// GPIO output enable mask

#define px4_arch_configgpio(pinset) {									\
	rp2040_gpio_set_pulls(pinset & GPIO_NUM_MASK, pinset & GPIO_PU_MASK, pinset & GPIO_PD_MASK);	\
	if ((pinset & GPIO_FUN_MASK) >> 9 == RP2040_GPIO_FUNC_SIO)					\
	{												\
		rp2040_gpio_setdir(pinset & GPIO_NUM_MASK, pinset & GPIO_OUT_MASK);			\
		rp2040_gpio_put(pinset & GPIO_NUM_MASK, pinset & GPIO_SET_MASK);			\
	}												\
	rp2040_gpio_set_function(pinset & GPIO_NUM_MASK, (pinset & GPIO_FUN_MASK) >> 9);		\
}
#define px4_arch_unconfiggpio(pinset)           rp2040_gpio_init(pinset & GPIO_NUM_MASK)	// Reset the pin as input SIO
#define px4_arch_gpioread(pinset)               rp2040_gpio_get(pinset & GPIO_NUM_MASK)		// Use gpio_get
#define px4_arch_gpiowrite(pinset, value)       rp2040_gpio_put(pinset & GPIO_NUM_MASK, value)	// Use gpio_put
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a) {							\
	rp2040_gpio_disable_irq(pinset & GPIO_NUM_MASK);						\
	if (f & e & fp)											\
	{												\
		rp2040_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP2040_GPIO_INTR_EDGE_LOW, fp, a);	\
		rp2040_gpio_enable_irq(pinset & GPIO_NUM_MASK);						\
	}												\
	if (r & e & fp)											\
	{												\
		rp2040_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP2040_GPIO_INTR_EDGE_HIGH, fp, a);	\
		rp2040_gpio_enable_irq(pinset & GPIO_NUM_MASK);						\
	}												\
}

// Following are quick defines to be used with the functions defined above
// These defines create a bit-mask which is supposed to be used in the
// functions defined above to set up gpios correctly.
#define PX4_MAKE_GPIO_INPUT(gpio) (gpio | GPIO_PU | RP2040_GPIO_FUNC_SIO)
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (gpio | GPIO_OUT | RP2040_GPIO_FUNC_SIO)
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (gpio | GPIO_OUT | GPIO_SET | RP2040_GPIO_FUNC_SIO)

#define PX4_GPIO_PIN_OFF(pinset) ((pinset & GPIO_NUM_MASK) | RP2040_GPIO_FUNC_SIO | RP2040_GPIO_PD)

__END_DECLS
