#pragma once

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#if defined(CONFIG_ARCH_CHIP_RP23XX)

#include <rp23xx_spi.h>
#include <rp23xx_i2c.h>
#include <rp23xx_gpio.h>

#else

#include <rp2040_spi.h>
#include <rp2040_i2c.h>
#include <rp2040_gpio.h>

#endif
// RP2040 doesn't have a bbsram. Following two defines are copied from nxp/k66.
// This will remove the errors of undefined PX4_BBSRAM_SIZE when logger module is activated.
// Fixme: using ??
#define PX4_BBSRAM_SIZE             2048
#define PX4_HF_GETDESC_IOCTL        0

// RP2040 doesn't really have a cpu register with unique id.
// However, there is a function in pico-sdk which can provide
// a device unique id from its flash which is 64 bits in length.
// For now, a common device id will be used for all RP2040 based devices.
// Take a look at board_identity.c file in version folder.

#define PX4_CPU_UUID_BYTE_LENGTH                12
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

#define PX4_BUS_OFFSET       1                  /* RP2040 buses are 0 based and adjustment is needed */

#if defined(CONFIG_ARCH_CHIP_RP23XX)

	#define px4_spibus_initialize(bus_num_1based)   rp23xx_spibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

	#define px4_i2cbus_initialize(bus_num_1based)   rp23xx_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
	#define px4_i2cbus_uninitialize(pdev)           rp23xx_i2cbus_uninitialize(pdev)

#else
	#define px4_spibus_initialize(bus_num_1based)   rp2040_spibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

        #define px4_i2cbus_initialize(bus_num_1based)   rp2040_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
        #define px4_i2cbus_uninitialize(pdev)           rp2040_i2cbus_uninitialize(pdev)

#endif

// FIXME: ???!!!
// This part of the code is specific to rp2040.
// RP2040 does not have the gpio configuration process similar to stm or tiva devices.
// There are multiple different registers which are required to be configured based on the function selection.
// However, only five values are required for the most part: Pin number, Pull up/down, direction, set/clear and function

#if defined(CONFIG_ARCH_CHIP_RP23XX)
// The pinset below can be defined using a 16-bit value where,
// bits		Function
// 0-5		GPIO number. 0-29 is valid.

// +1
// 5		Pull up
// 6		Pull down
// 7		Direction
// 8		Set/clear
// 9-13		GPIO function select
// 14-15	Unused
#define GPIO_PU		(1 << 6)	// Pull-up resistor
#define GPIO_PD		(1 << 7)	// Pull-down resistor
#define GPIO_OUT	(1 << 8)	// Output enable
#define GPIO_SET	(1 << 9)	// Output set
#define GPIO_FUN(func)	(func << 10)	// Function select

// FIXME: need to support more pins on rp2350B !!! the rp2040 masks are incorrect!!!
#define GPIO_NUM_MASK	0x3f  		//                                000111111 = 63 FIXME
#define	GPIO_PU_MASK	0x40		// GPIO PAD register mask.        001000000 = 64
#define	GPIO_PD_MASK	0x80		// GPIO pin number mask.          010000000 = 128
#define	GPIO_OUT_MASK	0x100		// GPIO pin function mask         100000000 = 256
#define	GPIO_SET_MASK	0x200		// GPIO pin function mask        1000000000 = 512
#define	GPIO_FUN_MASK	0x7C00		// GPIO output enable mask 0111110000000000

#else

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

#define GPIO_NUM_MASK	0x1f  		//                                00011111 = 31 FIXME
#define	GPIO_PU_MASK	0x20		// GPIO PAD register mask.        00100000 = 32
#define	GPIO_PD_MASK	0x40		// GPIO pin number mask.          01000000 = 64
#define	GPIO_OUT_MASK	0x80		// GPIO pin function mask         10000000 = 128
#define	GPIO_SET_MASK	0x100		// GPIO pin function mask        100000000 =256
#define	GPIO_FUN_MASK	0x3E00		// GPIO output enable mask 011111000000000
#endif

#if defined(CONFIG_ARCH_CHIP_RP23XX)
	// shift10 instead !!!
	#define GPIO_FUNC_SIO_SHIFT_9 GPIO_FUN(RP23XX_GPIO_FUNC_SIO)
#else
	#define GPIO_FUNC_SIO_SHIFT_9 GPIO_FUN(RP2040_GPIO_FUNC_SIO)
#endif

#if defined(CONFIG_ARCH_CHIP_RP23XX)

int rp23xx_gpioconfig(uint32_t pinset);
int rp23xx_setgpioevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);

#define px4_arch_configgpio(pinset)		rp23xx_gpioconfig(pinset)			// Defined in io_pins/rp2040_pinset.c
#define px4_arch_unconfiggpio(pinset)           rp23xx_gpio_init(pinset & GPIO_NUM_MASK)	// Reset the pin as input SIO
#define px4_arch_gpioread(pinset)               rp23xx_gpio_get(pinset & GPIO_NUM_MASK)		// Use gpio_get
#define px4_arch_gpiowrite(pinset, value)       rp23xx_gpio_put(pinset & GPIO_NUM_MASK, value)	// Use gpio_put
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a) rp23xx_setgpioevent(pinset,r,f,e,fp,a)		// Defined in io_pins/rp2040_pinset.c


#else

int rp2040_gpioconfig(uint32_t pinset);
int rp2040_setgpioevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);

#define px4_arch_configgpio(pinset)		rp2040_gpioconfig(pinset)			// Defined in io_pins/rp2040_pinset.c
#define px4_arch_unconfiggpio(pinset)           rp2040_gpio_init(pinset & GPIO_NUM_MASK)	// Reset the pin as input SIO
#define px4_arch_gpioread(pinset)               rp2040_gpio_get(pinset & GPIO_NUM_MASK)		// Use gpio_get
#define px4_arch_gpiowrite(pinset, value)       rp2040_gpio_put(pinset & GPIO_NUM_MASK, value)	// Use gpio_put
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a) rp2040_setgpioevent(pinset,r,f,e,fp,a)		// Defined in io_pins/rp2040_pinset.c

#endif /* defined(CONFIG_ARCH_CHIP_RP23XX) */


// Following are quick defines to be used with the functions defined above
// These defines create a bit-mask which is supposed to be used in the
// functions defined above to set up gpios correctly.
#define PX4_MAKE_GPIO_INPUT(gpio) (gpio | GPIO_PU | GPIO_FUNC_SIO_SHIFT_9)
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (gpio | GPIO_OUT | GPIO_FUNC_SIO_SHIFT_9)
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (gpio | GPIO_OUT | GPIO_SET | GPIO_FUNC_SIO_SHIFT_9)

#define PX4_GPIO_PIN_OFF(pinset) ((pinset & GPIO_NUM_MASK) | GPIO_FUNC_SIO_SHIFT_9 | GPIO_PD)



#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

__END_DECLS
