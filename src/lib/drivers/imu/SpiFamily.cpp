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
 * @file SpiFamily.cpp
 * @brief Non-templated SPI CLI parsing, instance probing, and status helpers.
 */

#include "SpiFamily.hpp"

#if defined(CONFIG_SPI)

#include <cinttypes>
#include <pthread.h>

namespace imu
{

namespace
{

pthread_mutex_t           g_spi_endpoint_mutex = PTHREAD_MUTEX_INITIALIZER;
SpiEndpointClaimTable<32> g_spi_endpoint_claims {};

SpiEndpointKey resolveSpiEndpointKey(const I2CSPIDriverConfig &config)
{
	const uint32_t selector = static_cast<uint32_t>(PX4_SPI_DEV_ID(config.spi_devid));

#if !defined(__PX4_LINUX) && !defined(__PX4_ROS2) && defined(SPI_BUS_MAX_BUS_ITEMS) && (SPI_BUS_MAX_BUS_ITEMS > 0)

	if (config.bus >= 0
	    && config.bus_device_index >= 0
	    && config.bus_device_index < SPI_BUS_MAX_DEVICES) {
# if defined(BOARD_NUM_SPI_CFG_HW_VERSIONS) && (BOARD_NUM_SPI_CFG_HW_VERSIONS > 1)
		const px4_spi_bus_t *const buses = px4_spi_buses;

		if (buses != nullptr) {
# else
		const px4_spi_bus_t *const buses = &px4_spi_buses[0];

		{
# endif

			for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
				if (buses[i].bus == -1) {
					break;
				}

				if (buses[i].bus == config.bus) {
					const uint32_t cs_gpio = buses[i].devices[config.bus_device_index].cs_gpio;

					if (cs_gpio != 0) {
						return SpiEndpointKey { config.bus, cs_gpio, true };
					}

					break;
				}
			}
		}
	}

#endif // !__PX4_LINUX && !__PX4_ROS2 && SPI_BUS_MAX_BUS_ITEMS > 0

	return SpiEndpointKey { config.bus, selector, false };
}

bool parseSpiOptions(
	int argc,
	char *argv[],
	BusCLIArguments &cli,
	SpiOptions &options,
	const SpiCliCapabilities &capabilities,
	const char *extra_optstring,
	bool (*on_extra)(int ch, const char *arg, void *context),
	void *context)
{
	char optspec[32] { "T:R:" };
	bool rotation_specified = false;
	bool clock_specified    = false;

	const auto append_opts = [&optspec](const char *suffix) {
		if (!suffix || !*suffix) {
			return true;
		}

		const size_t base_len   = strlen(optspec);
		const size_t suffix_len = strlen(suffix);

		if (base_len + suffix_len >= sizeof(optspec)) {
			return false;
		}

		memcpy(optspec + base_len, suffix, suffix_len + 1);
		return true;
	};

	if ((capabilities.supports_clock_input && !append_opts("C:"))
	    || (capabilities.supports_split_endpoints && !append_opts("AG"))
	    || !append_opts(extra_optstring)) {
		return false;
	}

	int ch = 0;

	while ((ch = cli.getOpt(argc, argv, optspec)) != EOF) {
		switch (ch) {
		case 'T': {
				options.type = cli.optArg();
				break;
			}

		case 'R': {
				int rotation = 0;

				if (!parseInteger(cli.optArg(), 0, ROTATION_MAX - 1, rotation)) {
					return false;
				}

				cli.rotation       = static_cast<Rotation>(rotation);
				rotation_specified = true;
				break;
			}

		case 'C': {
				if (!parseSpiClockHz(cli.optArg(), options.clock_hz)) {
					return false;
				}

				clock_specified = true;
				break;
			}

		case 'A':
		case 'G': {
				if (options.component && options.component != ch) {
					PX4_ERR("select one endpoint: -A or -G");
					return false;
				}

				options.component = static_cast<char>(ch);
				break;
			}

		default: {
				if (!on_extra || !on_extra(ch, cli.optArg(), context)) {
					return false;
				}

				break;
			}
		}
	}

	options.verb    = cli.optArg();
	options.command = parseSpiCommand(options.verb);

	if (options.command == SpiCommand::kInvalid || argc < 2 || options.verb != argv[argc - 1]) {
		return false;
	}

	if (options.command != SpiCommand::kStart
	    && (rotation_specified
		|| clock_specified
		|| cli.bus_frequency != 0
		|| cli.spi_mode != SPIDEV_MODE3
		|| cli.quiet_start)) {
		return false;
	}

	return true;
}

bool hasRunningInstances(const char *instance_key, const BusCLIArguments &cli, uint16_t device_type)
{
	BusInstanceIterator iterator(instance_key, cli, device_type);

	while (iterator.next()) {
		if (iterator.instance() != nullptr) {
			return true;
		}
	}

	return false;
}

} // namespace

SpiEndpointGuard::SpiEndpointGuard(const I2CSPIDriverConfig &config) :
	_key(resolveSpiEndpointKey(config)),
	_quiet_start(config.quiet_start)
{
}

SpiEndpointGuard::~SpiEndpointGuard()
{
	release();
}

bool SpiEndpointGuard::acquire()
{
	if (_acquired) {
		return true;
	}

	if (!_key.valid()) {
		if (!_quiet_start) {
			PX4_ERR("invalid SPI endpoint (bus %d)", _key.bus);
		}

		return false;
	}

	pthread_mutex_lock(&g_spi_endpoint_mutex);
	const bool acquired = g_spi_endpoint_claims.acquire(_key);
	pthread_mutex_unlock(&g_spi_endpoint_mutex);

	if (!acquired) {
		if (!_quiet_start) {
			PX4_WARN("SPI endpoint unavailable or already in use (bus %d, %s 0x%" PRIx32 ")",
				 _key.bus,
				 _key.uses_cs_gpio ? "cs" : "slot",
				 _key.selector);
		}

		return false;
	}

	_acquired = true;
	return true;
}

void SpiEndpointGuard::release()
{
	if (!_acquired) {
		return;
	}

	pthread_mutex_lock(&g_spi_endpoint_mutex);
	g_spi_endpoint_claims.release(_key);
	pthread_mutex_unlock(&g_spi_endpoint_mutex);

	_acquired = false;
}

void printSpiStatus(
	const SpiModel &model,
	spi_mode_e active_mode,
	int config_frequency,
	int data_frequency,
	unsigned current_frequency)
{
	PX4_INFO("SPI mode %u: config %d Hz, data %d Hz, current %u Hz",
		 unsigned(active_mode),
		 config_frequency,
		 data_frequency,
		 current_frequency);

	PX4_INFO("SPI dummy %u B, data prefix %u B, max %u B, data CS %s",
		 unsigned(model.register_dummy_bytes),
		 unsigned(model.data_prefix_bytes),
		 unsigned(model.max_transfer_bytes),
		 model.continuous_data_cs ? "continuous" : "split");
}

namespace detail
{

int spiFamilyRun(
	int argc,
	char *argv[],
	const char *module_name,
	const SpiCliCapabilities &capabilities,
	const char *extra_optstring,
	const SpiFamilyDispatch &dispatch)
{
	BusCLIArguments cli { false, true };
	SpiOptions options {};

	cli.default_spi_frequency = 0;
	cli.spi_mode              = SPIDEV_MODE3;

	if (!parseSpiOptions(
		    argc,
		    argv,
		    cli,
		    options,
		    capabilities,
		    extra_optstring,
		    dispatch.extra_option,
		    dispatch.context)) {
		dispatch.print_usage();
		return PX4_ERROR;
	}

	const bool start = options.command == SpiCommand::kStart;

	if ((start && (!options.type || !*options.type))
	    || (options.type && !*options.type)) {
		dispatch.print_usage();
		return PX4_ERROR;
	}

	if (start && cli.bus_option == I2CSPIBusOption::All) {
		PX4_ERR("need to specify a bus type");
		dispatch.print_usage();
		return PX4_ERROR;
	}

	const uint8_t *base      = static_cast<const uint8_t *>(dispatch.models);
	bool           matched   = false;
	bool           started   = false;
	bool           running   = false;
	bool           any_error = false;

	for (size_t i = 0; i < dispatch.model_count; ++i) {
		const void     *model_ptr = base + i * dispatch.model_stride;
		const SpiModel &device    = dispatch.model_device(model_ptr);

		if (!device.name || !*device.name) {
			continue;
		}

		if (options.type && strcmp(options.type, device.name) != 0) {
			continue;
		}

		if (options.component && options.component != device.component) {
			continue;
		}

		if (dispatch.filter_model) {
			const SpiFilterResult filter = dispatch.filter_model(model_ptr, options.command, dispatch.context);

			if (filter == SpiFilterResult::kSkip) {
				continue;
			}

			if (filter == SpiFilterResult::kInvalid) {
				return PX4_ERROR;
			}
		}

		matched = true;

		if (start && device.component && !options.component) {
			PX4_ERR("split sensor requires -A or -G");
			return PX4_ERROR;
		}

		BusCLIArguments device_cli = cli;
		device_cli.custom_data     = dispatch.prepare_custom_data(model_ptr, options, dispatch.context);

		if (start) {
			if (device.frequency <= 0 || device.data_frequency <= 0) {
				PX4_ERR("invalid model SPI frequency");
				return PX4_ERROR;
			}

			const int maximum_frequency = device.frequency > device.data_frequency ? device.frequency : device.data_frequency;
			const bool mode_valid       = (device_cli.spi_mode == device.mode)
						      || (device.allow_mode0 && device_cli.spi_mode == SPIDEV_MODE0);

			if (!validSpiClockHz(device, options.clock_hz)
			    || !mode_valid
			    || device_cli.bus_frequency < 0
			    || device_cli.bus_frequency > maximum_frequency) {
				PX4_ERR("unsupported SPI mode, frequency or clock input");
				return PX4_ERROR;
			}

			device_cli.custom1       = options.clock_hz;
			device_cli.custom2       = device_cli.bus_frequency;
			device_cli.bus_frequency = spiConfigFrequency(device, device_cli.custom2);
		}

		const char *instance_key = device.instance_key ? device.instance_key : module_name;

		if (!start && !hasRunningInstances(instance_key, device_cli, device.device_type)) {
			continue;
		}

		if (!start) {
			running = true;
		}

		BusInstanceIterator iterator(instance_key, device_cli, device.device_type);
		const int ret = dispatch.module_main(options.verb, device_cli, iterator);

		if (start) {
			if (ret == PX4_OK) {
				started = true;

			} else {
				any_error = true;
			}

		} else if (ret != PX4_OK) {
			any_error = true;
		}
	}

	if (!matched) {
		PX4_ERR("type or endpoint not compiled in this family");
		return PX4_ERROR;
	}

	if (start) {
		return (started && !any_error) ? PX4_OK : PX4_ERROR;
	}

	if (!running) {
		if (options.command == SpiCommand::kStop) {
			PX4_ERR("Not running");

		} else {
			PX4_INFO("Not running");
		}

		return PX4_ERROR;
	}

	return any_error ? PX4_ERROR : PX4_OK;
}

} // namespace detail

} // namespace imu

#endif // CONFIG_SPI
