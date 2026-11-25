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
 * @file SpiFamily.hpp
 * @brief SPI endpoint limits, cache-aligned transfer buffer, and shared CLI dispatch.
 */

#pragma once

#include "ByteCursor.hpp"

#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>

#if defined(CONFIG_SPI)
# include <px4_platform_common/i2c_spi_buses.h>
# include <px4_platform_common/module.h>
#endif // CONFIG_SPI

#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace imu
{

/**
 * @brief Immutable limits for one existing board SPI registration, never a synthetic accel/gyro pair.
 */
struct SpiModel {
	const char *name                 { nullptr };     ///< Exact CLI -T model name with static storage duration.
	const char *instance_key         { nullptr };     ///< Optional distinct lifecycle key when models share device_type.
	uint16_t    device_type          { 0 };           ///< Existing DRV_ACC/GYR/IMU_DEVTYPE_* board registration ID.
	int         frequency            { 0 };           ///< Default and maximum configuration/register SPI clock [Hz].
	int         data_frequency       { 0 };           ///< Default and maximum FIFO/burst SPI clock [Hz].
#if defined(CONFIG_SPI)
	spi_mode_e  mode                 { SPIDEV_MODE3 }; ///< Default SPI clock polarity and phase.
#endif // CONFIG_SPI
	bool        allow_mode0          { false };       ///< Allow explicit -m 0 when the sensor supports both Mode 0 and Mode 3.
	char        component            { 0 };           ///< Zero for an integrated device, 'A' or 'G' for a split device.
	int         min_clock_hz         { 0 };           ///< Minimum supported external reference-clock input [Hz].
	int         max_clock_hz         { 0 };           ///< Maximum external reference-clock input [Hz]; zero means unsupported.
	uint16_t    max_transfer_bytes   { 0 };           ///< Largest data transaction, including prefix and payload [bytes].
	uint8_t     register_dummy_bytes { 0 };           ///< Dummy bytes inserted during single-register reads [bytes].
	uint8_t     data_prefix_bytes    { 0 };           ///< Command/dummy prefix bytes preceding burst or FIFO payload [bytes].
	bool        continuous_data_cs   { true };        ///< Keep chip select asserted across consecutive burst/FIFO words.
};

/**
 * @brief Cache-line-aligned and padded SPI DMA bounce/transfer buffer.
 */
class SpiDmaBuffer
{
public:
#if defined(PX4_ARCH_DCACHE_ALIGNMENT)
	static constexpr size_t kAlignment { PX4_ARCH_DCACHE_ALIGNMENT };
#elif defined(ARMV7M_DCACHE_LINESIZE)
	static constexpr size_t kAlignment { ARMV7M_DCACHE_LINESIZE };
#else
	static constexpr size_t kAlignment { sizeof(void *) };
#endif

	static constexpr size_t alignSize(size_t size, size_t alignment = kAlignment)
	{
		if (size == 0
		    || alignment == 0
		    || (alignment & (alignment - 1)) != 0
		    || size > SIZE_MAX - (alignment - 1)) {
			return 0;
		}

		return (size + (alignment - 1)) & ~(alignment - 1);
	}

	explicit SpiDmaBuffer(size_t minimum_bytes) :
		_capacity(alignSize(minimum_bytes)),
		_data(_capacity > 0 ? static_cast<uint8_t *>(px4_cache_aligned_alloc(_capacity)) : nullptr)
	{
		if (_data) {
			memset(_data, 0, _capacity);

		} else {
			_capacity = 0;
		}
	}

	~SpiDmaBuffer() { free(_data); }

	SpiDmaBuffer(const SpiDmaBuffer &) = delete;
	SpiDmaBuffer &operator=(const SpiDmaBuffer &) = delete;
	SpiDmaBuffer(SpiDmaBuffer &&) = delete;
	SpiDmaBuffer &operator=(SpiDmaBuffer &&) = delete;

	explicit operator bool() const { return _data != nullptr; }
	uint8_t *data() { return _data; }
	const uint8_t *data() const { return _data; }
	uint16_t *words() { return _capacity >= sizeof(uint16_t) ? static_cast<uint16_t *>(static_cast<void *>(_data)) : nullptr; }
	size_t capacity() const { return _capacity; }

private:
	size_t   _capacity { 0 };
	uint8_t *_data     { nullptr };
};

constexpr int spiConfigFrequency(const SpiModel &model, int requested)
{
	if (model.frequency <= 0 || model.data_frequency <= 0 || requested < 0) {
		return 0;
	}

	return requested > 0 && requested < model.frequency ? requested : model.frequency;
}

constexpr int spiDataFrequency(const SpiModel &model, int requested)
{
	if (model.frequency <= 0 || model.data_frequency <= 0 || requested < 0) {
		return 0;
	}

	return requested > 0 && requested < model.data_frequency ? requested : model.data_frequency;
}

inline bool parseInteger(const char *text, int minimum, int maximum, int &value)
{
	if (!text || !*text) {
		return false;
	}

	char *end = nullptr;
	errno = 0;

	const long parsed = strtol(text, &end, 10);

	if (errno != 0 || *end != '\0' || parsed < minimum || parsed > maximum) {
		return false;
	}

	value = static_cast<int>(parsed);

	return true;
}

inline bool parseSpiClockHz(const char *text, int &clock_hz)
{
	return parseInteger(text, 0, 1 * MHz, clock_hz);
}

constexpr bool validSpiClockHz(const SpiModel &model, int clock_hz)
{
	return clock_hz >= 0
	       && clock_hz <= model.max_clock_hz
	       && (clock_hz == 0 || clock_hz >= model.min_clock_hz);
}

/** @brief Physical SPI endpoint key identifying a bus and chip-select GPIO or fallback slot. */
struct SpiEndpointKey {
	int      bus          { -1 };
	uint32_t selector     { 0 };
	bool     uses_cs_gpio { false };

	constexpr bool valid() const { return bus >= 0; }

	constexpr bool operator==(const SpiEndpointKey &other) const
	{
		return bus == other.bus
		       && selector == other.selector
		       && uses_cs_gpio == other.uses_cs_gpio;
	}
};

/** @brief Fixed-capacity zero-heap registry of active physical SPI endpoint claims. */
template<size_t capacity = 32>
class SpiEndpointClaimTable
{
	static_assert(capacity > 0, "SPI endpoint claim table capacity must be nonzero");

public:
	bool acquire(const SpiEndpointKey &key)
	{
		if (!key.valid()) {
			return false;
		}

		size_t free_index = capacity;

		for (size_t i = 0; i < capacity; ++i) {
			if (!_slots[i].valid()) {
				if (free_index == capacity) {
					free_index = i;
				}

			} else if (_slots[i] == key) {
				return false;
			}
		}

		if (free_index == capacity) {
			return false;
		}

		_slots[free_index] = key;
		return true;
	}

	void release(const SpiEndpointKey &key)
	{
		if (!key.valid()) {
			return;
		}

		for (size_t i = 0; i < capacity; ++i) {
			if (_slots[i] == key) {
				_slots[i] = SpiEndpointKey {};
				break;
			}
		}
	}

private:
	SpiEndpointKey _slots[capacity] {};
};

enum class SpiCommand : uint8_t {
	kInvalid,
	kStart,
	kStop,
	kStatus,
};

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

struct SpiOptions {
	const char *type      { nullptr };
	const char *verb      { nullptr };
	char        component { 0 };
	int         clock_hz  { 0 }; ///< Requested external reference-clock frequency [Hz].
	SpiCommand  command   { SpiCommand::kInvalid };
};

struct SpiCliCapabilities {
	bool supports_clock_input     { false };
	bool supports_split_endpoints { false };
};

enum class SpiFilterResult : uint8_t {
	kMatch,
	kSkip,
	kInvalid,
};

#if defined(CONFIG_SPI)

/** @brief Non-copyable RAII guard enforcing mutual exclusion on one physical SPI endpoint. */
class SpiEndpointGuard
{
public:
	explicit SpiEndpointGuard(const I2CSPIDriverConfig &config);
	~SpiEndpointGuard();

	SpiEndpointGuard(const SpiEndpointGuard &) = delete;
	SpiEndpointGuard &operator=(const SpiEndpointGuard &) = delete;
	SpiEndpointGuard(SpiEndpointGuard &&) = delete;
	SpiEndpointGuard &operator=(SpiEndpointGuard &&) = delete;

	[[nodiscard]] bool acquire();
	void release();

private:
	SpiEndpointKey _key         {};
	bool           _quiet_start { false };
	bool           _acquired    { false };
};

void printSpiStatus(
	const SpiModel &model,
	spi_mode_e active_mode,
	int config_frequency,
	int data_frequency,
	unsigned current_frequency);

namespace detail
{

template<typename T>
T decayCopy(T);

template<typename Model>
const SpiModel &spiModelDevice(const void *model)
{
	return static_cast<const Model *>(model)->device;
}

inline void *spiIdentityCustomData(const void *model, const SpiOptions &, void *)
{
	return const_cast<void *>(model);
}

struct SpiFamilyDispatch {
	const void *models        { nullptr };
	size_t      model_count   { 0 };
	size_t      model_stride  { 0 };
	const SpiModel &(*model_device)(const void *model) { nullptr };
	bool (*extra_option)(int ch, const char *arg, void *context) { nullptr };
	SpiFilterResult(*filter_model)(const void *model, SpiCommand command, void *context) { nullptr };
	void *(*prepare_custom_data)(const void *model, const SpiOptions &options, void *context) { nullptr };
	int (*module_main)(const char *verb, const BusCLIArguments &cli, BusInstanceIterator &iterator) { nullptr };
	void (*print_usage)() { nullptr };
	void *context { nullptr };
};

int spiFamilyRun(
	int argc,
	char *argv[],
	const char *module_name,
	const SpiCliCapabilities &capabilities,
	const char *extra_optstring,
	const SpiFamilyDispatch &dispatch);

} // namespace detail

/**
 * @brief Family front end supporting mandatory -T on start, optional -T on stop/status,
 * and optional family-specific CLI options via custom_data.
 *
 * @note config.custom_data points to stack-local Context::custom_payload and is valid
 * only during synchronous Driver construction and init(); drivers must copy needed values
 * in their constructor and never retain the raw config.custom_data pointer.
 */
template<typename Driver,
	 typename Model,
	 size_t model_count,
	 typename ExtraHandler,
	 typename ModelFilter,
	 typename MakeCustomData>
int spiFamilyMainCustom(
	int argc,
	char *argv[],
	const char *module_name,
	const Model(&models)[model_count],
	const SpiCliCapabilities &capabilities,
	const char *extra_optstring,
	const ExtraHandler &on_extra,
	const ModelFilter &model_filter,
	const MakeCustomData &make_custom_data)
{
	using CustomData = decltype(detail::decayCopy(make_custom_data(models[0], SpiOptions {})));

	struct Context {
		const ExtraHandler   &on_extra;
		const ModelFilter    &model_filter;
		const MakeCustomData &make_custom_data;
		CustomData            custom_payload {};

		static bool extraOption(int ch, const char *arg, void *context)
		{
			return static_cast<Context *>(context)->on_extra(ch, arg);
		}

		static SpiFilterResult filterModel(const void *model, SpiCommand command, void *context)
		{
			return static_cast<Context *>(context)->model_filter(*static_cast<const Model *>(model), command);
		}

		static void *prepareCustomData(const void *model, const SpiOptions &options, void *context)
		{
			auto *context_ptr = static_cast<Context *>(context);
			context_ptr->custom_payload = context_ptr->make_custom_data(*static_cast<const Model *>(model), options);
			return &context_ptr->custom_payload;
		}
	} context { on_extra, model_filter, make_custom_data };

	const detail::SpiFamilyDispatch dispatch {
		models,
		model_count,
		sizeof(Model),
		&detail::spiModelDevice<Model>,
		&Context::extraOption,
		&Context::filterModel,
		&Context::prepareCustomData,
		&Driver::module_main,
		&Driver::print_usage,
		&context,
	};

	return detail::spiFamilyRun(argc, argv, module_name, capabilities, extra_optstring, dispatch);
}

template<typename Driver, typename Model, size_t model_count>
int spiFamilyMain(
	int argc,
	char *argv[],
	const char *module_name,
	const Model(&models)[model_count],
	const SpiCliCapabilities &capabilities = {})
{
	const detail::SpiFamilyDispatch dispatch {
		models,
		model_count,
		sizeof(Model),
		&detail::spiModelDevice<Model>,
		nullptr,
		nullptr,
		&detail::spiIdentityCustomData,
		&Driver::module_main,
		&Driver::print_usage,
		nullptr,
	};

	return detail::spiFamilyRun(argc, argv, module_name, capabilities, nullptr, dispatch);
}

#endif // CONFIG_SPI

} // namespace imu
