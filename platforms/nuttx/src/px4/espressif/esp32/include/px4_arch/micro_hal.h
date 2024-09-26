#pragma once

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#include <esp32_tim.h>
#include <esp32_spi.h>
#include <esp32_i2c.h>
#include <esp32_gpio.h>

#  define INPUT             (1 << 0)
#  define OUTPUT            (1 << 1)
#  define FUNCTION          (1 << 2)

#define PULLUP              (1 << 3)
#define PULLDOWN            (1 << 4)
#define OPEN_DRAIN          (1 << 5)

// bits		Function
// 0-5		GPIO number. 0-40 is valid.
// 6		input
// 7		output
// 8		fun
// 9		pull up
// 10		pull down
// 11		open drain
// 12-14	GPIO function select
// 15-31	Unused

#define GPIO_SET_SHIFT  6
#define GPIO_INPUT	(1 << 6)
#define GPIO_OUTPUT	(1 << 7)
#define GPIO_FUNCTION	(1 << 8)

#define GPIO_PULLUP	(1 << 9)
#define GPIO_PULLDOWN	(1 << 10)
#define GPIO_OPEN_DRAIN	(1 << 11)

#define GPIO_FUN(func)	(func << 12)	// Function select

#define GPIO_NUM_MASK		0x3f
#define	GPIO_INPUT_MASK		0x40
#define	GPIO_OUTPUT_MASK	0x80
#define	GPIO_FUNCTION_MASK	0x100
#define	GPIO_PULLUP_MASK	0x200
#define	GPIO_PULLDOWN_MASK	0x400
#define	GPIO_OPEN_DRAIN_MASK	0x800
#define	GPIO_FUN_SELECT_MASK	0x7000

#define PX4_NUMBER_I2C_BUSES 2

#define PX4_SOC_ARCH_ID PX4_SOC_ARCH_ID_UNUSED

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
#define px4_spibus_initialize(bus_num_1based)   esp32_spibus_initialize(bus_num_1based)

#define px4_i2cbus_initialize(bus_num_1based)   esp32_i2cbus_initialize(bus_num_1based)
#define px4_i2cbus_uninitialize(pdev)           esp32_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)		esp32_configgpio(pinset, 0)			// Defined in io_pins/rp2040_pinset.c
#define px4_arch_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               esp32_gpioread(pinset)		// Use gpio_get
#define px4_arch_gpiowrite(pinset, value)       esp32_gpiowrite(pinset, value)	// Use gpio_put

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

__END_DECLS
