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
 * @file
 * @brief SPI endpoint limits and typed dispatch through the native PX4 bus lifecycle.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace imu
{

/**
 * @brief Immutable limits for one existing board SPI registration, never a synthetic accel/gyro pair.
 * @note Phase frequencies are conservative driver limits for the implemented access paths, not necessarily
 * the chip's absolute maximum SPI clock. The family owns switching rates around each transaction.
 */
struct SpiModel {
	const char *name; ///< Exact CLI model name with static storage duration.
	uint16_t   device_type; ///< Existing DRV_ACC/GYR/IMU_DEVTYPE_* board registration ID.
	int        frequency; ///< Default and maximum configuration/register access clock, in Hz; positive.
	spi_mode_e mode; ///< Required SPI clock polarity and phase.
	char       component; ///< Zero for an integrated device, 'A' or 'G' for a split device.
	int        max_clock_hz; ///< Maximum external sensor reference-clock input in Hz; zero means unsupported, not unlimited.
	int        data_frequency; ///< Default and maximum FIFO/burst data access clock, in Hz; positive.
	uint16_t   max_transfer_bytes; ///< Largest data transaction in bytes, including command/prefix and payload.
	uint8_t    register_dummy_bytes; ///< Dummy bytes after the register address; excludes the address itself.
	uint8_t    data_prefix_bytes; ///< Command, dummy and status/count bytes before the data samples.
	bool       continuous_data_cs; ///< Whether the data transaction requires uninterrupted chip select.
	int        min_clock_hz { 0 }; ///< Minimum supported external reference-clock input; zero for an endpoint without CLKIN.
};

/**
 * @brief Select the configuration clock, capped by the endpoint's register-access limit.
 * @param[in] model Endpoint with a positive configuration frequency in Hz.
 * @param[in] requested Requested clock in Hz; zero or negative selects the profile default.
 * @return Configuration SPI clock in Hz.
 */
constexpr int spiConfigFrequency(const SpiModel &model, int requested)
{
	return requested > 0 && requested < model.frequency ? requested : model.frequency;
}

/**
 * @brief Select the data clock independently of the configuration clock.
 * @param[in] model Endpoint with a positive data frequency in Hz.
 * @param[in] requested Requested clock in Hz; zero or negative selects the profile default.
 * @return Data SPI clock in Hz, capped by the endpoint's data-access limit.
 */
constexpr int spiDataFrequency(const SpiModel &model, int requested)
{
	return requested > 0 && requested < model.data_frequency ? requested : model.data_frequency;
}

/**
 * @brief Print endpoint transfer constraints and actual selected clocks.
 * @param[in] model Endpoint limits and transfer layout.
 * @param[in] config_frequency Selected register clock in Hz.
 * @param[in] data_frequency Selected data clock in Hz.
 * @param[in] current_frequency Current device::SPI clock in Hz.
 */
inline void printSpiStatus(
	const SpiModel &model,
	int config_frequency,
	int data_frequency,
	unsigned current_frequency)
{
	PX4_INFO("SPI mode %u: config %d Hz, data %d Hz, current %u Hz",
		 unsigned(model.mode),
		 config_frequency,
		 data_frequency,
		 current_frequency);

	PX4_INFO("SPI dummy %u B, data prefix %u B, max %u B, data CS %s",
		 unsigned(model.register_dummy_bytes),
		 unsigned(model.data_prefix_bytes),
		 unsigned(model.max_transfer_bytes),
		 model.continuous_data_cs ? "continuous" : "split");
}

/**
 * @brief Parse a complete decimal integer argument within inclusive bounds.
 * @param[in] text Null-terminated input; null/empty input is invalid. Leading whitespace/sign follows strtol().
 * @param[in] minimum Inclusive lower bound.
 * @param[in] maximum Inclusive upper bound.
 * @param[out] value Parsed value; unchanged on failure.
 * @return True on success; false on range errors, overflow or trailing characters.
 */
inline bool parseInteger(const char *text, int minimum, int maximum, int &value)
{
	if (!text || !*text) {
		return false;
	}

	char *end = nullptr;

	errno = 0;

	const long parsed = strtol(text, &end, 10);

	if (errno != 0
	    || *end != '\0'
	    || parsed < minimum
	    || parsed > maximum) {
		return false;
	}

	value = static_cast<int>(parsed);

	return true;
}

/** Native lifecycle operation selected by the family CLI. */
enum class SpiCommand : uint8_t {
	kInvalid,
	kStart,
	kStop,
	kStatus,
};

/**
 * @brief Map an exact native lifecycle verb to a command.
 * @param[in] verb Null-terminated start, stop or status; null is allowed.
 * @return Invalid for null or an unrecognized verb, otherwise the matching command.
 */
inline SpiCommand parseSpiCommand(const char *verb)
{
	if (verb) {
		if (strcmp(verb, "start") == 0) {
			return SpiCommand::kStart;
		}

		if (strcmp(verb, "stop") == 0) {
			return SpiCommand::kStop;
		}

		if (strcmp(verb, "status") == 0) {
			return SpiCommand::kStatus;
		}
	}

	return SpiCommand::kInvalid;
}

/**
 * @brief Dispatch one selected endpoint through PX4's native instance lifecycle.
 * @tparam Driver I2CSPIDriver implementation providing module_start/stop/status.
 * @param[in] command Start, Stop or Status; Invalid returns PX4_ERROR.
 * @param[in,out] cli Parsed native SPI options. On start, custom1 is external reference-clock input in Hz;
 * custom2 receives the original SPI frequency request in Hz, bus_frequency is capped for configuration,
 * and an unspecified bus becomes SPIInternal. Stop/status do not apply these start-only constraints.
 * @param[in] instance_key Stable lifecycle key; models sharing a device ID need distinct keys for exact-model filtering.
 * @param[in] device Immutable endpoint limits and existing board registration ID.
 * @param[out] stop_failed Optional sticky flag: set when a failed stop leaves matching instances registered.
 * @return Native lifecycle result, or PX4_ERROR for unsupported mode, frequency or clock input.
 * @pre For a family driver, cli.custom_data contains its constructor arguments; temporary options must be copied.
 * A single-model driver needs no opaque model pointer.
 */
template<typename Driver>
int dispatchSpiCommand(
	SpiCommand command,
	BusCLIArguments &cli,
	const char *instance_key,
	const SpiModel &device,
	bool *stop_failed = nullptr)
{
	if (command == SpiCommand::kInvalid) {
		return PX4_ERROR;
	}

	if (command == SpiCommand::kStart) {
		const int maximum_frequency = device.frequency > device.data_frequency ? device.frequency : device.data_frequency;

		if (cli.custom1 < 0
		    || cli.custom1 > device.max_clock_hz
		    || (cli.custom1 != 0 && cli.custom1 < device.min_clock_hz)
		    || cli.spi_mode != device.mode
		    || cli.bus_frequency < 0
		    || cli.bus_frequency > maximum_frequency) {
			PX4_ERR("unsupported SPI mode, frequency or clock input");

			return PX4_ERROR;
		}

		if (cli.bus_option == I2CSPIBusOption::All) {
			cli.bus_option = I2CSPIBusOption::SPIInternal;
		}

		// Preserve the override independently of the capped configuration rate.
		cli.custom2       = cli.bus_frequency;
		cli.bus_frequency = spiConfigFrequency(device, cli.custom2);
	}

	int result;

	{
		BusInstanceIterator iterator(instance_key, cli, device.device_type);

		result = command == SpiCommand::kStart ? Driver::module_start(cli, iterator)
			 : command == SpiCommand::kStop ? Driver::module_stop(iterator)
			 : Driver::module_status(iterator);
	}

	if (command == SpiCommand::kStop && result != PX4_OK && stop_failed) {
		// Native stop uses the same error for no instance and a stop timeout. Recheck
		// exact selectors after releasing its iterator lock; never nest list locks.
		// A concurrent completed stop is harmless. Any remaining instance makes the
		// aggregate fail conservatively, including one started concurrently.
		BusInstanceIterator remaining(instance_key, cli, device.device_type);

		while (remaining.next()) {
			if (remaining.instance()) {
				*stop_failed = true;
				break;
			}
		}
	}

	return result;
}

/** Parsed SPI selectors, independent of whether the driver supports one or several models. */
struct SpiOptions {
	const char *type { nullptr };
	char component { 0 };
	SpiCommand command { SpiCommand::kInvalid };
};

/**
 * @brief Parse native bus options and common model, rotation, clock and component selectors.
 * @param[in] argc Native argument count.
 * @param[in] argv Native command-line arguments.
 * @param[in,out] cli Bus options; the caller sets the default SPI mode before parsing.
 * @param[out] options Parsed selectors and lifecycle command; discard on failure.
 * @return True for a valid command and selectors; model-specific validation is deferred to dispatch.
 */
inline bool parseSpiOptions(int argc, char *argv[], BusCLIArguments &cli, SpiOptions &options)
{
	using namespace frequency_literals;


	const char *&type = options.type;
	char &component = options.component;
	int ch;

	while ((ch = cli.getOpt(argc, argv, "T:R:C:AG")) != EOF) {
		switch (ch) {
		case 'T': {
				type = cli.optArg();
				break;
			}

		case 'R': {
				int rotation;

				if (!parseInteger(cli.optArg(), 0, ROTATION_MAX - 1, rotation)) {
					return false;
				}

				cli.rotation = static_cast<Rotation>(rotation);
				break;
			}

		case 'C': {
				if (!parseInteger(cli.optArg(), 1_Hz, 1_MHz, cli.custom1)) {
					return false;
				}

				break;
			}

		case 'A':
		case 'G': {
				if (component && component != ch) {
					PX4_ERR("select one endpoint: -A or -G");

					return false;
				}

				component = ch;
				break;
			}

		default: {
				return false;
			}
		}
	}

	options.command = parseSpiCommand(cli.optArg());

	return options.command != SpiCommand::kInvalid;
}

/**
 * @brief Family front end using PX4's existing bus iterator and instance lifecycle.
 * @tparam Driver Native driver with print_usage() and the lifecycle methods required by dispatchSpiCommand().
 * @tparam Model Type containing a SpiModel member named device.
 * @tparam model_count Number of compiled endpoint profiles.
 * @param[in] argc Native command-line argument count.
 * @param[in] argv Native command-line arguments; every command requires an exact -T model.
 * @param[in] module_name Stable instance key for this family.
 * @param[in] models Immutable endpoint profiles with static storage duration, passed through cli.custom_data.
 * @return PX4_OK if at least one selected endpoint succeeds and no failed stop leaves a matching instance.
 * PX4_ERROR otherwise; endpoints that were not running do not mask a successful stop of another endpoint.
 * @pre Distinct exact models must not share a device ID within this front end; split endpoints use their
 * existing separate accel/gyro IDs. Families sharing IDs must select unique instance keys themselves.
 * @note No lifecycle operation occurs without an explicit type. Split start requires exactly one of -A/-G;
 * integrated start accepts neither. Typed stop/status may select both split endpoints, optionally filtered
 * by component and native bus selectors. Success does not imply every matching endpoint started.
 */
template<typename Driver, typename Model, size_t model_count>
int spiFamilyMain(
	int argc,
	char *argv[],
	const char *module_name,
	const Model(&models)[model_count])
{
	BusCLIArguments cli { false, true };
	SpiOptions options {};

	cli.default_spi_frequency = 0; // The selected endpoint supplies the default below.

	if (!parseSpiOptions(argc, argv, cli, options)
	    || !options.type
	    || !*options.type) {
		Driver::print_usage();

		return PX4_ERROR;
	}

	const SpiCommand command = options.command;
	const bool start = command == SpiCommand::kStart;
	const char *type = options.type;
	const char component = options.component;

	int  result      = PX4_ERROR;
	bool matched     = false;
	bool stop_failed = false;

	for (const Model &model : models) {
		const SpiModel &device = model.device;

		if (strcmp(type, device.name) != 0 || (component && component != device.component)) {
			continue;
		}

		matched = true;

		if (start
		    && device.component
		    && !component) {
			PX4_ERR("split sensor requires -A or -G");

			return PX4_ERROR;
		}

		// Models are immutable; the PX4 opaque argument is not const-qualified.
		cli.custom_data = const_cast<Model *>(&model);

		const int ret = dispatchSpiCommand<Driver>(command, cli, module_name, device, &stop_failed);

		if (ret == PX4_OK) {
			result = PX4_OK;
		}
	}

	if (!matched) {
		PX4_ERR("type or endpoint not compiled in this family");
	}

	return stop_failed ? PX4_ERROR : result;
}

/**
 * @brief Front end for one immutable SPI endpoint without a single-element family table.
 * @tparam Driver Native SPI driver providing constexpr spiModel(), print_usage() and lifecycle methods.
 * @param[in] argc Native argument count.
 * @param[in] argv Native arguments; -T is optional and, if supplied, must match the sole model.
 * @param[in] module_name Stable native lifecycle key.
 * @return Native lifecycle result, or PX4_ERROR for invalid selectors.
 * @note Omitted -T selects only this model, never a scan of unrelated device types.
 * An optional -A/-G must match the sole endpoint; no component selector is required.
 * The existing multi-model front end keeps its explicit-type and split-selection rules.
 */
template<typename Driver>
int spiSingleMain(
	int argc,
	char *argv[],
	const char *module_name)
{
	constexpr SpiModel device { Driver::spiModel() };
	BusCLIArguments cli { false, true };
	SpiOptions options {};

	cli.default_spi_frequency = 0;
	cli.spi_mode = device.mode;

	if (!parseSpiOptions(argc, argv, cli, options)) {
		Driver::print_usage();

		return PX4_ERROR;
	}

	if ((options.type && strcmp(options.type, device.name) != 0)
	    || (options.component && options.component != device.component)) {
		PX4_ERR("type or endpoint does not match this driver");

		return PX4_ERROR;
	}

	// An exact endpoint needs no runtime model pointer or family constructor arguments.
	return dispatchSpiCommand<Driver>(options.command, cli, module_name, device);
}

} // namespace imu
