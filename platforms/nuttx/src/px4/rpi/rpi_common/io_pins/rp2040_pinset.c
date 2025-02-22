#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>

#include <px4_arch/micro_hal.h>
#include <errno.h>

#ifdef CONFIG_ARCH_CHIP_RP23XX
	#define RP_GPIO_NUM_TOTAL RP23XX_GPIO_NUM

int rp23xx_gpioconfig(uint32_t pinset)
{
	if ((pinset & GPIO_NUM_MASK) > RP_GPIO_NUM_TOTAL) {
		return -EINVAL;
	}

	rp23xx_gpio_set_pulls(pinset & GPIO_NUM_MASK, pinset & GPIO_PU_MASK, pinset & GPIO_PD_MASK);

	if ((pinset & GPIO_FUN_MASK) >> 9 == RP23XX_GPIO_FUNC_SIO) {
		rp23xx_gpio_setdir(pinset & GPIO_NUM_MASK, pinset & GPIO_OUT_MASK);
		rp23xx_gpio_put(pinset & GPIO_NUM_MASK, pinset & GPIO_SET_MASK);
	}

	rp23xx_gpio_set_function(pinset & GPIO_NUM_MASK, (pinset & GPIO_FUN_MASK) >> 9);

	return OK;
}

// Be careful when using this function. Current nuttx implementation allows for only one type of interrupt
// (out of four types rising, falling, level high, level low) to be active at a time.
int rp23xx_setgpioevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg)
{
	int ret = -ENOSYS;

	if (fallingedge & event & (func != NULL)) {
		ret = rp23xx_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP23XX_GPIO_INTR_EDGE_LOW, func, arg);
		rp23xx_gpio_enable_irq(pinset & GPIO_NUM_MASK);

	} else if (risingedge & event & (func != NULL)) {
		ret = rp23xx_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP23XX_GPIO_INTR_EDGE_HIGH, func, arg);
		rp23xx_gpio_enable_irq(pinset & GPIO_NUM_MASK);

	} else {
		rp23xx_gpio_disable_irq(pinset & GPIO_NUM_MASK);
		ret = rp23xx_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP23XX_GPIO_INTR_EDGE_LOW, NULL, NULL);
	}

	return ret;
}




#else
	#define RP_GPIO_NUM_TOTAL RP2040_GPIO_NUM

int rp2040_gpioconfig(uint32_t pinset)
{
	if ((pinset & GPIO_NUM_MASK) > RP_GPIO_NUM_TOTAL) {
		return -EINVAL;
	}

	rp2040_gpio_set_pulls(pinset & GPIO_NUM_MASK, pinset & GPIO_PU_MASK, pinset & GPIO_PD_MASK);

	if ((pinset & GPIO_FUN_MASK) >> 9 == RP2040_GPIO_FUNC_SIO) {
		rp2040_gpio_setdir(pinset & GPIO_NUM_MASK, pinset & GPIO_OUT_MASK);
		rp2040_gpio_put(pinset & GPIO_NUM_MASK, pinset & GPIO_SET_MASK);
	}

	rp2040_gpio_set_function(pinset & GPIO_NUM_MASK, (pinset & GPIO_FUN_MASK) >> 9);

	return OK;
}

// Be careful when using this function. Current nuttx implementation allows for only one type of interrupt
// (out of four types rising, falling, level high, level low) to be active at a time.
int rp2040_setgpioevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg)
{
	int ret = -ENOSYS;

	if (fallingedge & event & (func != NULL)) {
		ret = rp2040_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP2040_GPIO_INTR_EDGE_LOW, func, arg);
		rp2040_gpio_enable_irq(pinset & GPIO_NUM_MASK);

	} else if (risingedge & event & (func != NULL)) {
		ret = rp2040_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP2040_GPIO_INTR_EDGE_HIGH, func, arg);
		rp2040_gpio_enable_irq(pinset & GPIO_NUM_MASK);

	} else {
		rp2040_gpio_disable_irq(pinset & GPIO_NUM_MASK);
		ret = rp2040_gpio_irq_attach(pinset & GPIO_NUM_MASK, RP2040_GPIO_INTR_EDGE_LOW, NULL, NULL);
	}

	return ret;
}
#endif /* CONFIG_ARCH_CHIP_RP23XX */
