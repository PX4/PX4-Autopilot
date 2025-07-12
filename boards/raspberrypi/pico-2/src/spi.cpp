/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
#ifdef CONFIG_RP23XX_BOARD_MADFLIGHT_FC1
// --- BBX --- Black Box Data Logger  (use spi -OR- mmc)
//   bbx_gizmo     SDSPI  // options: NONE, SDSPI, SDMMC
//   pin_bbx_cs    39  // spi
//   bbx_spi_bus   0  // spi
//   pin_mmc_dat   -1  // mmc
//   pin_mmc_clk   -1  // mmc
//   pin_mmc_cmd   -1  // mmc
// SDIO_CLK/SPI0_SLCK (bbx)

//   35		SDIO_CMD/SPI0_MOSI (bbx)
//   36		SDIO_D0/SPI0_MISO (bbx)
//   37		SDIO_D1 (bbx)
//   38		SDIO_D2 (bbx)
//   39		SDIO_D3/SPI0_CS (bbx)
initSPIBus(SPI::Bus::SPI0, {
		initSPIDevice(SPIDEV_MMCSD(0), SPI::CS{GPIO::Pin39}),  // FIXME: CS GPIO?
}),

// imu_gizmo     ICM42688
//   imu_bus_type  SPI
//   imu_align     CW0
//   imu_spi_bus   1 //spi
//   pin_imu_cs    29 //spi
//   pin_imu_int   27 //spi and i2c

initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::Pin29}, SPI::DRDY{GPIO::Pin27}),
	   }),

#else
	#if defined(CONFIG_RP23XX_SPI0)
		// initSPIBusExternal(SPI::Bus::SPI0, {
		// 	initSPIConfigExternal(SPI::CS{GPIO::Pin13}),
		// }),
		initSPIBus(SPI::Bus::SPI0, {
			#if defined(CONFIG_NSH_MMCSDSPIPORTNO)
			initSPIDevice(SPIDEV_MMCSD(0), SPI::CS{GPIO::Pin5}),  // FIXME: CS GPIO?
			#endif
			initSPIDevice(DRV_IMU_DEVTYPE_ICM45686, SPI::CS{GPIO::Pin5}),
		}),
	#endif
	#if defined(CONFIG_RP23XX_SPI1)
		initSPIBusExternal(SPI::Bus::SPI1, {
			initSPIConfigExternal(SPI::CS{GPIO::Pin13}),
		}),
	#endif
#endif
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
