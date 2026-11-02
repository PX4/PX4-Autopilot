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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include "IIOIMU.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

using namespace time_literals;

ModuleBase::Descriptor IIOIMU::desc{task_spawn, custom_command, print_usage};

namespace
{
constexpr const char *IIO_SYSFS_ROOT = "/sys/bus/iio/devices";
constexpr const char *CHANNEL_NAMES[iio_imu::MAX_CHANNEL_COUNT] {
	"in_accel_x", "in_accel_y", "in_accel_z",
	"in_anglvel_x", "in_anglvel_y", "in_anglvel_z", "in_timestamp", "in_temp"
};
} // namespace

IIOIMU::IIOIMU(const char *device_path, const char *name, uint32_t device_id, enum Rotation rotation,
	       uint8_t samples_per_publish) :
	_samples_per_publish(samples_per_publish),
	_px4_accel(device_id, rotation),
	_px4_gyro(device_id, rotation)
{
	if (device_path != nullptr) {
		strncpy(_device_path, device_path, sizeof(_device_path) - 1);
	}

	if (name != nullptr) {
		strncpy(_name, name, sizeof(_name) - 1);
	}
}

IIOIMU::~IIOIMU()
{
	disable_iio();

	if (_fd >= 0) {
		::close(_fd);
	}

	perf_free(_read_perf);
	perf_free(_publish_interval_perf);
	perf_free(_bad_read_perf);
	perf_free(_bad_scan_perf);
	perf_free(_timestamp_gap_perf);
	perf_free(_stale_sample_perf);
	perf_free(_bad_temperature_perf);
}

bool IIOIMU::write_text_file(const char *path, const char *value)
{
	const int fd = ::open(path, O_WRONLY | O_CLOEXEC);

	if (fd < 0) {
		PX4_ERR("open %s failed: %s", path, strerror(errno));
		return false;
	}

	const size_t length = strlen(value);
	const ssize_t written = ::write(fd, value, length);
	const int saved_errno = errno;
	::close(fd);

	if (written != static_cast<ssize_t>(length)) {
		PX4_ERR("write %s failed: %s", path, written < 0 ? strerror(saved_errno) : "short write");
		return false;
	}

	return true;
}

IIOIMU::ReadResult IIOIMU::read_text_file(const char *path, char *value, size_t value_len, bool optional)
{
	if (value_len < 2) {
		return ReadResult::Error;
	}

	value[0] = '\0';
	const int fd = ::open(path, O_RDONLY | O_CLOEXEC);

	if (fd < 0) {
		if (optional && errno == ENOENT) {
			return ReadResult::Missing;
		}

		PX4_ERR("open %s failed: %s", path, strerror(errno));
		return ReadResult::Error;
	}

	const ssize_t length = ::read(fd, value, value_len - 1);
	const int saved_errno = errno;
	::close(fd);

	if (length <= 0 || static_cast<size_t>(length) == value_len - 1) {
		PX4_ERR("read %s failed: %s", path, length < 0 ? strerror(saved_errno) : "empty or truncated attribute");
		return ReadResult::Error;
	}

	if (!iio_imu::normalize_attribute(value, static_cast<size_t>(length), value_len)) {
		value[0] = '\0';
		PX4_ERR("invalid text attribute %s", path);
		return ReadResult::Error;
	}

	return ReadResult::Ok;
}

IIOIMU::ReadResult IIOIMU::read_attribute(const char *attribute, char *value, size_t value_len, bool optional) const
{
	char path[PATH_MAX];
	snprintf(path, sizeof(path), "%s/%s", _sysfs_path, attribute);
	return read_text_file(path, value, value_len, optional);
}

bool IIOIMU::write_attribute(const char *attribute, const char *value) const
{
	char path[PATH_MAX];
	snprintf(path, sizeof(path), "%s/%s", _sysfs_path, attribute);
	return write_text_file(path, value);
}

bool IIOIMU::is_device_name(const char *name)
{
	constexpr size_t prefix_length = sizeof("iio:device") - 1;
	uint32_t number = 0;
	return strncmp(name, "iio:device", prefix_length) == 0
	       && iio_imu::parse_unsigned(name + prefix_length, number);
}

bool IIOIMU::find_iio_device()
{
	DIR *directory = opendir(IIO_SYSFS_ROOT);

	if (directory == nullptr) {
		PX4_ERR("open %s failed: %s", IIO_SYSFS_ROOT, strerror(errno));
		return false;
	}

	bool found = false;
	bool ambiguous = false;
	struct dirent *entry = nullptr;

	while ((entry = readdir(directory)) != nullptr) {
		if (!is_device_name(entry->d_name)) {
			continue;
		}

		char path[PATH_MAX];
		char name[sizeof(_name)];
		snprintf(path, sizeof(path), "%s/%s/name", IIO_SYSFS_ROOT, entry->d_name);

		if (read_text_file(path, name, sizeof(name), true) != ReadResult::Ok
		    || (_name[0] != '\0' && strcmp(name, _name) != 0)) {
			continue;
		}

		bool has_channels = true;

		for (size_t i = 0; i < iio_imu::CHANNEL_COUNT; ++i) {
			snprintf(path, sizeof(path), "%s/%s/scan_elements/%s_type", IIO_SYSFS_ROOT, entry->d_name, CHANNEL_NAMES[i]);

			if (::access(path, F_OK) != 0) {
				has_channels = false;
				break;
			}
		}

		if (has_channels) {
			if (found) {
				ambiguous = true;
				break;
			}

			snprintf(_device_path, sizeof(_device_path), "/dev/%s", entry->d_name);
			found = true;
		}
	}

	closedir(directory);

	if (ambiguous) {
		PX4_ERR("multiple IIO IMUs match; select one with -d or -n");
		return false;
	}

	if (!found) {
		PX4_ERR("no matching six-axis IIO IMU");
	}

	return found;
}

bool IIOIMU::configure_channels()
{
	char path[PATH_MAX];
	snprintf(path, sizeof(path), "%s/scan_elements", _sysfs_path);
	DIR *directory = opendir(path);

	if (directory == nullptr) {
		PX4_ERR("open %s failed: %s", path, strerror(errno));
		return false;
	}

	bool success = true;
	struct dirent *entry = nullptr;

	// Remove extra channels left enabled by earlier users before computing our layout.
	while ((entry = readdir(directory)) != nullptr) {
		const size_t length = strlen(entry->d_name);

		if (length > 3 && strcmp(entry->d_name + length - 3, "_en") == 0) {
			char attribute[sizeof(entry->d_name) + sizeof("scan_elements/")];
			snprintf(attribute, sizeof(attribute), "scan_elements/%s", entry->d_name);

			if (!write_attribute(attribute, "0")) {
				success = false;
				break;
			}
		}
	}

	closedir(directory);

	if (!success) {
		return false;
	}

	char temperature_type[128];
	const ReadResult temperature = read_attribute("scan_elements/in_temp_type", temperature_type, sizeof(temperature_type), true);

	if (temperature == ReadResult::Error) {
		return false;
	}

	_temperature_enabled = temperature == ReadResult::Ok;
	const size_t count = _temperature_enabled ? iio_imu::MAX_CHANNEL_COUNT : iio_imu::CHANNEL_COUNT;

	for (size_t i = 0; i < count; ++i) {
		char attribute[128];
		char value[128];
		snprintf(attribute, sizeof(attribute), "scan_elements/%s_type", CHANNEL_NAMES[i]);

		if (read_attribute(attribute, value, sizeof(value)) != ReadResult::Ok || !iio_imu::parse_type(value, _channels[i])) {
			PX4_ERR("unsupported %s type '%s'", CHANNEL_NAMES[i], value);
			return false;
		}

		snprintf(attribute, sizeof(attribute), "scan_elements/%s_index", CHANNEL_NAMES[i]);

		if (read_attribute(attribute, value, sizeof(value)) != ReadResult::Ok
		    || !iio_imu::parse_unsigned(value, _channels[i].index)) {
			PX4_ERR("invalid %s index '%s'", CHANNEL_NAMES[i], value);
			return false;
		}

		snprintf(attribute, sizeof(attribute), "scan_elements/%s_en", CHANNEL_NAMES[i]);

		if (!write_attribute(attribute, "1")) {
			return false;
		}
	}

	if (!iio_imu::make_layout(_channels, _scan_size, _temperature_enabled)) {
		PX4_ERR("unsupported or inconsistent IIO scan layout; see iio_imu/README.md");
		return false;
	}

	return true;
}

bool IIOIMU::configure_temperature()
{
	if (!_temperature_enabled) {
		return true;
	}

	char scale[128];
	char offset[128];

	if (read_attribute("in_temp_scale", scale, sizeof(scale)) != ReadResult::Ok) {
		return false;
	}

	const ReadResult offset_result = read_attribute("in_temp_offset", offset, sizeof(offset), true);

	if (offset_result == ReadResult::Error
	    || !_temperature_scale.configure(scale, offset_result == ReadResult::Missing ? nullptr : offset)) {
		PX4_ERR("invalid temperature scale or offset");
		return false;
	}

	return true;
}

void IIOIMU::set_temperature(float temperature)
{
	if (!PX4_ISFINITE(temperature)) {
		perf_count(_bad_temperature_perf);
		temperature = NAN;
	}

	_temperature = temperature;
	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
}

bool IIOIMU::check_zero_offset(const char *attribute) const
{
	char value[128];
	const ReadResult result = read_attribute(attribute, value, sizeof(value), true);
	float offset = 0.f;

	if (result == ReadResult::Missing) {
		return true;
	}

	if (result != ReadResult::Ok || !iio_imu::parse_float(value, offset) || fabsf(offset) > 0.f) {
		PX4_ERR("%s must be zero or absent", attribute);
		return false;
	}

	return true;
}

bool IIOIMU::read_sensor_scale(size_t first_axis, iio_imu::SensorScale &sensor)
{
	const char *prefix = first_axis == 0 ? "in_accel" : "in_anglvel";
	char attribute[128];
	char scale[128];
	char range[128];
	snprintf(attribute, sizeof(attribute), "%s_scale", prefix);

	if (read_attribute(attribute, scale, sizeof(scale)) != ReadResult::Ok) {
		return false;
	}

	snprintf(attribute, sizeof(attribute), "%s_range_raw", prefix);
	const ReadResult range_result = read_attribute(attribute, range, sizeof(range), true);

	if (range_result == ReadResult::Error
	    || !sensor.configure(scale, range_result == ReadResult::Missing ? nullptr : range, _channels[first_axis].bits)) {
		PX4_ERR("invalid %s scale or range_raw", prefix);
		return false;
	}

	snprintf(attribute, sizeof(attribute), "%s_offset", prefix);

	if (!check_zero_offset(attribute)) {
		return false;
	}

	for (size_t axis = first_axis; axis < first_axis + 3; ++axis) {
		char value[128];
		snprintf(attribute, sizeof(attribute), "%s_scale", CHANNEL_NAMES[axis]);

		if (read_attribute(attribute, value, sizeof(value), true) != ReadResult::Missing) {
			PX4_ERR("per-axis scale is unsupported: %s", attribute);
			return false;
		}

		snprintf(attribute, sizeof(attribute), "%s_range_raw", CHANNEL_NAMES[axis]);

		if (read_attribute(attribute, value, sizeof(value), true) != ReadResult::Missing) {
			PX4_ERR("per-axis range_raw is unsupported: %s", attribute);
			return false;
		}

		snprintf(attribute, sizeof(attribute), "%s_offset", CHANNEL_NAMES[axis]);

		if (!check_zero_offset(attribute)) {
			return false;
		}
	}

	return true;
}

bool IIOIMU::read_sample_rate()
{
	char value[128];
	const ReadResult global_result = read_attribute("sampling_frequency", value, sizeof(value), true);
	float global_rate = 0.f;

	if (global_result == ReadResult::Error
	    || (global_result == ReadResult::Ok && !iio_imu::parse_float(value, global_rate))) {
		return false;
	}

	const char *attributes[] {"in_accel_sampling_frequency", "in_anglvel_sampling_frequency"};
	float rates[2] {};

	for (size_t i = 0; i < 2; ++i) {
		const ReadResult result = read_attribute(attributes[i], value, sizeof(value), true);

		if (result == ReadResult::Error || (result == ReadResult::Missing && global_result != ReadResult::Ok)) {
			PX4_ERR("missing or unreadable sampling frequency");
			return false;
		}

		if (result == ReadResult::Missing) {
			rates[i] = global_rate;

		} else if (!iio_imu::parse_float(value, rates[i])
			   || (global_result == ReadResult::Ok && fabsf(rates[i] - global_rate) > 0.f)) {
			PX4_ERR("invalid or inconsistent %s", attributes[i]);
			return false;
		}
	}

	if (rates[0] < 1.f || rates[0] > 32000.f || fabsf(rates[0] - rates[1]) > 0.f) {
		PX4_ERR("expected equal accel/gyro sample rates in [1, 32000] Hz");
		return false;
	}

	_sample_rate_hz = rates[0];
	_sample_interval_us = 1e6f / _sample_rate_hz;
	return true;
}

bool IIOIMU::configure_iio()
{
	if (_device_path[0] == '\0' && !find_iio_device()) {
		return false;
	}

	const char *device_name = strrchr(_device_path, '/');
	device_name = device_name != nullptr ? device_name + 1 : _device_path;

	if (!is_device_name(device_name)) {
		PX4_ERR("expected /dev/iio:deviceN: %s", _device_path);
		return false;
	}

	snprintf(_sysfs_path, sizeof(_sysfs_path), "%s/%s", IIO_SYSFS_ROOT, device_name);

	if (read_attribute("name", _name, sizeof(_name)) != ReadResult::Ok) {
		return false;
	}

	// Claim the character device before changing sysfs state. A live legacy
	// reader holds the kernel's exclusive open and causes EBUSY here.
	_fd = ::open(_device_path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);

	if (_fd < 0) {
		if (errno == EBUSY) {
			PX4_ERR("%s is in use; stop the other IIO reader first", _device_path);

		} else {
			PX4_ERR("open %s failed: %s", _device_path, strerror(errno));
		}

		return false;
	}

	char value[128];
	uint32_t enabled = 0;

	if (read_attribute("buffer/enable", value, sizeof(value)) != ReadResult::Ok
	    || !iio_imu::parse_unsigned(value, enabled) || enabled > 1) {
		PX4_ERR("invalid buffer/enable '%s' on %s", value, _device_path);
		return false;
	}

	if (enabled != 0) {
		PX4_WARN("resetting pre-enabled IIO buffer on %s", _device_path);
	}

	// Closing the character device does not necessarily disable its buffer.
	// Recover that state before changing channels, scale metadata or clock.
	if (!write_attribute("buffer/enable", "0")
	    || read_attribute("buffer/enable", value, sizeof(value)) != ReadResult::Ok
	    || !iio_imu::parse_unsigned(value, enabled) || enabled != 0) {
		PX4_ERR("failed to disable IIO buffer on %s", _device_path);
		return false;
	}

	if (read_attribute("current_timestamp_clock", value, sizeof(value)) != ReadResult::Ok) {
		return false;
	}

	if (strcmp(value, "monotonic") != 0) {
		PX4_INFO("changing IIO clock from '%s' to monotonic", value);

		if (!write_attribute("current_timestamp_clock", "monotonic")) {
			PX4_ERR("failed to set IIO clock to monotonic on %s", _device_path);
			return false;
		}

		if (read_attribute("current_timestamp_clock", value, sizeof(value)) != ReadResult::Ok) {
			return false;
		}

		if (strcmp(value, "monotonic") != 0) {
			PX4_ERR("IIO clock readback is '%s', expected 'monotonic' on %s", value, _device_path);
			return false;
		}
	}

	if (!configure_channels() || !configure_temperature()
	    || !read_sensor_scale(0, _accel_scale) || !read_sensor_scale(3, _gyro_scale)
	    || !read_sample_rate()) {
		return false;
	}

	_px4_accel.set_scale(_accel_scale.scale);
	_px4_accel.set_range(_accel_scale.range());
	_px4_gyro.set_scale(_gyro_scale.scale);
	_px4_gyro.set_range(_gyro_scale.range());

	if (!write_attribute("buffer/length", "256")) {
		return false;
	}

	const ReadResult watermark = read_attribute("buffer/watermark", value, sizeof(value), true);

	if (watermark == ReadResult::Error) {
		return false;
	}

	if (watermark == ReadResult::Ok) {
		snprintf(value, sizeof(value), "%u", _samples_per_publish);

		if (!write_attribute("buffer/watermark", value)) {
			return false;
		}
	}

	if (!write_attribute("buffer/enable", "1")) {
		return false;
	}

	_buffer_enabled = true;
	return true;
}

void IIOIMU::disable_iio()
{
	if (_buffer_enabled) {
		write_attribute("buffer/enable", "0");
		_buffer_enabled = false;
	}
}

int IIOIMU::init()
{
	if (!configure_iio()) {
		return PX4_ERROR;
	}

	PX4_INFO("using %s (%s), %zu-byte scan, %.3f Hz", _device_path, _name, _scan_size, (double)_sample_rate_hz);
	return PX4_OK;
}


void IIOIMU::reset_pending()
{
	_pending_samples = 0;
	_accel_fifo = {};
	_gyro_fifo = {};
}

void IIOIMU::update_error_count()
{
	const uint64_t errors = perf_event_count(_bad_read_perf) + perf_event_count(_bad_scan_perf)
				+ perf_event_count(_timestamp_gap_perf) + perf_event_count(_stale_sample_perf)
				+ perf_event_count(_bad_temperature_perf);
	_px4_accel.set_error_count(errors);
	_px4_gyro.set_error_count(errors);
}

void IIOIMU::publish_pending()
{
	_accel_fifo.timestamp_sample = _last_timestamp_us;
	_gyro_fifo.timestamp_sample = _last_timestamp_us;
	_accel_fifo.dt = _sample_interval_us;
	_gyro_fifo.dt = _sample_interval_us;
	_accel_fifo.samples = _pending_samples;
	_gyro_fifo.samples = _pending_samples;
	update_error_count();
	_px4_accel.updateFIFO(_accel_fifo);
	_px4_gyro.updateFIFO(_gyro_fifo);
	perf_count(_publish_interval_perf);
	reset_pending();
}

bool IIOIMU::process_scan(const uint8_t *scan)
{
	const int64_t timestamp_ns = iio_imu::decode(scan, _channels[iio_imu::TIMESTAMP_CHANNEL]);

	if (timestamp_ns <= 0) {
		perf_count(_bad_scan_perf);
		reset_pending();
		return false;
	}

	const uint64_t timestamp_us = static_cast<uint64_t>(timestamp_ns / 1000);
	const hrt_abstime now = hrt_absolute_time();

	/* Reject a mismatched clock domain and samples stalled for an excessive time. */
	if (timestamp_us > now + 5_ms || now > timestamp_us + 1_s) {
		perf_count(_stale_sample_perf);
		reset_pending();
		return false;
	}

	if (_last_timestamp_us != 0) {
		if (timestamp_us <= _last_timestamp_us) {
			perf_count(_timestamp_gap_perf);
			reset_pending();
			return false;
		}

		const float delta_us = static_cast<float>(timestamp_us - _last_timestamp_us);

		if (delta_us < _sample_interval_us * 0.5f || delta_us > _sample_interval_us * 1.5f) {
			perf_count(_timestamp_gap_perf);
			reset_pending();
		}
	}

	if (_pending_samples >= _samples_per_publish) {
		perf_count(_bad_scan_perf);
		reset_pending();
	}

	const uint8_t sample = _pending_samples;

	if (_temperature_enabled) {
		const float raw = static_cast<float>(iio_imu::decode(scan, _channels[iio_imu::TEMPERATURE_CHANNEL]));
		set_temperature(_temperature_scale.celsius(raw));
	}

	_accel_fifo.x[sample] = static_cast<int16_t>(iio_imu::decode(scan, _channels[0]));
	_accel_fifo.y[sample] = static_cast<int16_t>(iio_imu::decode(scan, _channels[1]));
	_accel_fifo.z[sample] = static_cast<int16_t>(iio_imu::decode(scan, _channels[2]));
	_gyro_fifo.x[sample] = static_cast<int16_t>(iio_imu::decode(scan, _channels[3]));
	_gyro_fifo.y[sample] = static_cast<int16_t>(iio_imu::decode(scan, _channels[4]));
	_gyro_fifo.z[sample] = static_cast<int16_t>(iio_imu::decode(scan, _channels[5]));
	_pending_samples++;
	_last_timestamp_us = timestamp_us;

	if (_pending_samples == _samples_per_publish) {
		publish_pending();
	}

	return true;
}

void IIOIMU::read_available()
{
	perf_begin(_read_perf);

	for (unsigned iteration = 0; iteration < 8; iteration++) {
		const size_t read_size = _scan_size * MAX_READ_SCANS;
		const ssize_t bytes = ::read(_fd, _read_buffer, read_size);

		if (bytes < 0) {
			if (errno != EAGAIN && errno != EINTR) {
				perf_count(_bad_read_perf);
			}

			break;
		}

		if (bytes == 0) {
			break;
		}

		if ((bytes % _scan_size) != 0) {
			perf_count(_bad_read_perf);
			reset_pending();
			break;
		}

		const size_t scans = static_cast<size_t>(bytes) / _scan_size;

		for (size_t i = 0; i < scans; i++) {
			process_scan(&_read_buffer[i * _scan_size]);
		}

		if (static_cast<size_t>(bytes) < read_size) {
			break;
		}
	}

	perf_end(_read_perf);
}

void IIOIMU::run()
{
	struct pollfd poll_fd {};
	poll_fd.fd = _fd;
	poll_fd.events = POLLIN;

	while (!should_exit()) {
		poll_fd.revents = 0;
		const int ret = ::poll(&poll_fd, 1, POLL_TIMEOUT_MS);

		if (ret < 0) {
			if (errno != EINTR) {
				perf_count(_bad_read_perf);
				px4_usleep(1000);
			}

			continue;
		}

		if (ret == 0) {
			/* Watchdog read in case an IIO notification was missed. */
			read_available();
			continue;
		}

		if (poll_fd.revents & POLLIN) {
			read_available();
		}

		if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
			perf_count(_bad_read_perf);

			PX4_ERR("IIO poll error: 0x%x", static_cast<unsigned>(poll_fd.revents));
			break;
		}
	}
}

int IIOIMU::print_status()
{
	PX4_INFO("device: %s (%s), PX4 ID: 0x%06" PRIx32, _device_path, _name, _px4_accel.get_device_id());
	PX4_INFO("scan: %zu bytes, rate: %.3f Hz, dt: %.3f us, samples/publish: %u",
		 _scan_size, (double)_sample_rate_hz, (double)_sample_interval_us, _samples_per_publish);
	PX4_INFO("accel: scale %.9g, range_raw %" PRIu32 " (%s), range %.6g m/s^2",
		 (double)_accel_scale.scale, _accel_scale.range_raw,
		 _accel_scale.range_from_bits ? "from bits" : "kernel", (double)_accel_scale.range());
	PX4_INFO("gyro: scale %.9g, range_raw %" PRIu32 " (%s), range %.6g rad/s",
		 (double)_gyro_scale.scale, _gyro_scale.range_raw,
		 _gyro_scale.range_from_bits ? "from bits" : "kernel", (double)_gyro_scale.range());
	PX4_INFO("temperature: %.3f C (%s)", (double)_temperature, _temperature_enabled ? "in_temp channel" : "unavailable");

	if (_temperature_enabled) {
		PX4_INFO("temp scale: %.9g mC/count, offset: %.9g counts", (double)_temperature_scale.scale,
			 (double)_temperature_scale.offset);
	}

	const size_t count = _temperature_enabled ? iio_imu::MAX_CHANNEL_COUNT : iio_imu::CHANNEL_COUNT;

	for (size_t i = 0; i < count; ++i) {
		PX4_INFO("%s: index %" PRIu32 ", offset %zu, %s:%c%u/%u>>%u", CHANNEL_NAMES[i],
			 _channels[i].index, _channels[i].offset, _channels[i].little_endian ? "le" : "be",
			 _channels[i].is_signed ? 's' : 'u', _channels[i].bits, _channels[i].storage_bits, _channels[i].shift);
	}

	PX4_INFO("sysfs: %s", _sysfs_path);
	PX4_INFO("buffer enabled: %s, pending scans: %u", _buffer_enabled ? "yes" : "no", _pending_samples);
	PX4_INFO("last sample timestamp: %" PRIu64 " us", _last_timestamp_us);
	perf_print_counter(_read_perf);
	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_bad_read_perf);
	perf_print_counter(_bad_scan_perf);
	perf_print_counter(_timestamp_gap_perf);
	perf_print_counter(_stale_sample_perf);
	perf_print_counter(_bad_temperature_perf);
	return 0;
}

int IIOIMU::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return IIOIMU::instantiate(ac, av);
	}, argc, argv);
}

uint32_t IIOIMU::make_device_id(uint8_t slot)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType_UNKNOWN;
	id.devid_s.bus = 0;
	id.devid_s.address = slot;
	id.devid_s.devtype = DRV_IMU_DEVTYPE_IIO;
	return id.devid;
}

int IIOIMU::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd(MODULE_NAME,
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_ACTUATOR_OUTPUTS,
					  20000,
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	if (wait_until_running(desc) != PX4_OK) {
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

IIOIMU *IIOIMU::instantiate(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *device_path = nullptr;
	const char *name = nullptr;
	uint32_t device_id = 0;
	uint32_t slot = 0;
	bool device_id_supplied = false;
	bool slot_supplied = false;
	uint32_t rotation = ROTATION_NONE;
	uint32_t samples_per_publish = 2;
	int ch;

	while ((ch = px4_getopt(argc, argv, "d:n:i:s:R:p:", &myoptind, &myoptarg)) != EOF) {
		bool valid = true;

		switch (ch) {
		case 'd':
			device_path = myoptarg;
			valid = strlen(device_path) < PATH_MAX && strncmp(device_path, "/dev/", 5) == 0
				&& is_device_name(device_path + 5);
			break;

		case 'n':
			name = myoptarg;
			valid = name[0] != '\0' && strlen(name) < sizeof(_name);
			break;

		case 'i':
			valid = iio_imu::parse_unsigned(myoptarg, device_id, 0);
			device_id_supplied = true;
			break;

		case 's':
			valid = iio_imu::parse_unsigned(myoptarg, slot) && slot <= 255;
			slot_supplied = true;
			break;

		case 'R':
			valid = iio_imu::parse_unsigned(myoptarg, rotation) && rotation <= 35;
			break;

		case 'p':
			valid = iio_imu::parse_unsigned(myoptarg, samples_per_publish)
				&& samples_per_publish >= 1 && samples_per_publish <= 32;
			break;

		default:
			valid = false;
			break;
		}

		if (!valid) {
			print_usage("invalid argument");
			return nullptr;
		}
	}

	if (device_id_supplied && slot_supplied) {
		print_usage("use either -s slot or -i device ID");
		return nullptr;
	}

	if (!device_id_supplied) {
		device_id = make_device_id(static_cast<uint8_t>(slot));
	}

	device::Device::DeviceId id{};
	id.devid = device_id;

	if (device_id > 0xffffff || id.devid_s.devtype == 0 || id.devid_s.devtype == DRV_DEVTYPE_UNUSED
	    || (device_path != nullptr && name != nullptr)) {
		print_usage("invalid device ID or conflicting -d/-n selectors");
		return nullptr;
	}

	IIOIMU *instance = new IIOIMU(device_path, name, device_id, static_cast<enum Rotation>(rotation),
				      static_cast<uint8_t>(samples_per_publish));

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

int IIOIMU::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IIOIMU::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Generic Linux buffered IIO IMU adapter. Reads six-axis scan layout, scale,
optional range_raw and temperature channel, sample rate and monotonic timestamps.
See src/drivers/imu/iio_imu/README.md for the required kernel ABI.
The kernel owns the sensor; configure its sample rate and range before starting.
Uses a generic IIO IMU device ID with a stable slot (default 0).
Use -i to override the full device ID when retaining an existing calibration identity.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("iio_imu", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "IIO device path (exclusive with -n)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, "<name>", "IIO name (otherwise discover by channels)", true);
	PRINT_MODULE_USAGE_PARAM_INT('s', 0, 0, 255, "Stable IIO IMU identity slot (exclusive with -i)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('i', nullptr, "<device_id>", "Override PX4 device ID, decimal or 0x hex", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 2, 1, 32, "Samples per FIFO publication", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int iio_imu_main(int argc, char *argv[])
{
	return ModuleBase::main(IIOIMU::desc, argc, argv);
}

