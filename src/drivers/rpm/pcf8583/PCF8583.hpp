/****************************************************************************
 *
 *   Copyright (c) 2020, 2021 PX4 Development Team. All rights reserved.
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
 * @file PCF8583.hpp
 *
 * @author ThunderFly s.r.o., Vít Hanousek <hanousekvit@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace/TFRPM01
 *
 * Driver for Main Rotor frequency sensor using PCF8583 I2C counter.
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/rpm.h>
#include <drivers/drv_hrt.h>

/* Configuration Constants */
#define PCF8583_BASEADDR_DEFAULT             0x50

class PCF8583 : public device::I2C, public ModuleParams, public I2CSPIDriver<PCF8583>
{
public:
	PCF8583(const I2CSPIDriverConfig &config);
	~PCF8583() override = default;

	static void print_usage();

	void		RunImpl();

	int    init() override;
	void   print_status() override;

private:

	int  probe() override;

	void           initCounter();
	uint32_t       getCounter();
	void           resetCounter();

	uint8_t        readRegister(uint8_t reg);
	void           setRegister(uint8_t reg, uint8_t value);

	uint8_t        hiWord(uint8_t in) { return (in & 0x0fu); }
	uint8_t        loWord(uint8_t in) { return ((in & 0xf0u) >> 4); }

	uint32_t       _count{0};
	uint16_t       _reset_count{0};
	hrt_abstime    _last_measurement_time{0};
	hrt_abstime    _last_reset_time{0};
	int            _tranfer_fail_count{0};
	uint8_t        _last_config_register_content{0x00};

	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PCF8583_POOL>) _param_pcf8583_pool,
		(ParamInt<px4::params::PCF8583_RESET>) _param_pcf8583_reset,
		(ParamInt<px4::params::PCF8583_MAGNET>) _param_pcf8583_magnet
	)
};
