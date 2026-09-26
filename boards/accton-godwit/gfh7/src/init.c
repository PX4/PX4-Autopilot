/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * Accton Godwit GFH7 board-specific early startup.
 *
 * Two hooks are provided here:
 *   - stm32_boardinitialize(): called very early by NuttX, before device
 *     drivers exist. Only GPIO / pin-mux work allowed.
 *   - board_app_initialize(): called once NuttX user space is up; starts
 *     platform services (HRT, DMA pool, LED device, hardfault logger,
 *     parameter flash store, HW manifest).
 */

#include "board_config.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>
#include <px4_arch/io_timer.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <stm32_uart.h>
#include <stm32_gpio.h>

#include <drivers/drv_board_led.h>
#include <drivers/drv_hrt.h>
#include <systemlib/px4_macros.h>
#include <parameters/flashparams/flashfs.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void board_clock_outputs_initialize(void);
__END_DECLS

__EXPORT int board_app_initialize(uintptr_t arg);

/****************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *   Called by PX4 system-reset to power-cycle external peripherals.
 *   GFH7 has no general peripheral power-cycle rail, so this is a no-op.
 ****************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);
}

/****************************************************************************
 * Name: board_on_reset
 *
 * Description:
 *   Put PWM outputs into a safe (input) state on reset.  When reset comes
 *   from the running system (status >= 0), hold low for 100 ms to ensure
 *   ESCs are disarmed before further init continues.
 ****************************************************************************/
__EXPORT void board_on_reset(int status)
{
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(io_timer_channel_get_as_pwm_input(i)));
	}

	if (status >= 0) {
		up_mdelay(100);
	}
}

/****************************************************************************
 * Name: stm32_boardinitialize
 ****************************************************************************/
__EXPORT void stm32_boardinitialize(void)
{
	/* Keep the debug port available across low-power states. */
	*(volatile uint32_t *)0x5C001004 |= (1u << 0) | (1u << 1) | (1u << 2);

	board_on_reset(-1);

	/* LEDs - NuttX arch-level hook (no-op unless CONFIG_ARCH_LEDS=y, which is
	 * off on this port; real LED setup happens later via drv_led_start() in
	 * board_app_initialize) */
	board_autoled_initialize();

	/* Configure ADC, CAN, board controls and revision straps. */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	board_clock_outputs_initialize();

	/* USB OTG_FS with physical VBUS sense on PA9. */
	stm32_usbinitialize();
}

/****************************************************************************
 * Name: board_app_initialize
 ****************************************************************************/
__EXPORT int board_app_initialize(uintptr_t arg)
{
	/* Need HRT running before ADC / workqueues can be used */
	px4_platform_init();

	/* SPI buses so onboard devices can probe. */
	stm32_spiinitialize();

	/* DMA allocator pool (required for SPI / UART DMA buffers on H7) */
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#if defined(SERIAL_HAVE_RXDMA)
	static struct hrt_call serial_dma_call;
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#endif

	/* Register LED device and set initial state (all off) */
	drv_led_start();
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	/* BBSRAM crash-dump logger */
	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_GREEN);
	}

	/* SDMMC2 / SD card bring-up. */
#ifdef CONFIG_MMCSD
	int sd_ret = stm32_sdio_initialize();

	if (sd_ret != OK) {
		syslog(LOG_ERR, "[boot] SDIO init FAILED: %d\n", sd_ret);
		led_on(LED_GREEN);
	}

#endif

	/* Parameters in internal flash: sector 15 at 0x081E0000 (128 KB) --
	 * matches bootloader's APP_RESERVATION_SIZE. */
#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{15, 128 * 1024, 0x081E0000},
		{0, 0, 0},
	};

	if (parameter_flashfs_init(params_sector_map, NULL, 0) != OK) {
		syslog(LOG_ERR, "[boot] parameter_flashfs_init FAILED\n");
		led_on(LED_GREEN);
	}

#endif

	/* Configure HW per manifest (must be last) */
	px4_platform_configure();

	return OK;
}
