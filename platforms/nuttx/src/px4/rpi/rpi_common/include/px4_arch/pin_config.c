#include <rp2040_gpio.h>
#include <hardware/rp2040_io_bank0.h>
#include <hardware/rp2040_pads_bank0.h>
#include <hardware/rp2040_sio.h>
#include <hardware/rp2040_memorymap.h>
// #include <px4_arch/micro_hal.h>

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>

#include <errno.h>

void rp2040_pinconfig(uint32_t pinset)
{
	uint8_t pad = (oinset & RP2040_GPIO_PAD_MASK) >> 0;
	uint8_t pin = (pinset & RP2040_GPIO_PIN_MASK) >> 8;
	uint8_t fun = (pinset & RP2040_GPIO_FUN_MASK) >> 16;
	bool	oen = (pinset & RP2040_GPIO_OEN_MASK) >> 24;
	bool	out = (pinset & RP2040_GPIO_OUT_MASK) >> 25;
	if (pin < RP2040_GPIO_NUM)
	{
		putreg32(uint32_t(fun), RP2040_IO_BANK0_GPIO_CTRL(pin));
		putreg32(uint32_t(pad) | RP2040_PADS_BANK0_GPIO_RESET, RP2040_PADS_BANK0_GPIO(pin));

		if ((pinset & RP2040_GPIO_FUN_MASK) >> 16 == RP2040_GPIO_FUNC_SIO)
		{
			if (oen)
				setbits_reg32(oen << pin, RP2040_SIO_GPIO_OE_SET);
			else
				setbits_reg32(oen << pin, RP2040_SIO_GPIO_OE_CLR);
			if (out)
				setbits_reg32(out << pin, RP2040_SIO_GPIO_OUT_SET);
			else
				setbits_reg32(out << pin, RP2040_SIO_GPIO_OUT_CLR);
		}
		else
		{
			if (oen)
				setbits_reg32(3 << 12, RP2040_IO_BANK0_GPIO_CTRL(pin));
			else
				setbits_reg32(2 << 12, RP2040_IO_BANK0_GPIO_CTRL(pin));
			if (out)
				setbits_reg32(3 << 8, RP2040_IO_BANK0_GPIO_CTRL(pin));
			else
				setbits_reg32(2 << 8, RP2040_IO_BANK0_GPIO_CTRL(pin));
		}
	}
}
