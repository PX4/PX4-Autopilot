/****************************************************************************
 *
 *	Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <containers/LockGuard.hpp>
#include <board_config.h>
extern "C" {
#include <lib/crc/crc.h>
}

namespace vtx
{

/**
 * Class storing the VTX frequencies and power levels and the map from RC channels to VTX settings.
 * Everything is 0-indexed.
 * @author Niklas Hauser <niklas@auterion.com>
 */
class Config
{
public:
	static constexpr size_t BANDS{24};
	static constexpr size_t CHANNELS{16};
	static constexpr size_t POWER_LEVELS{16};
	static constexpr size_t MAP_LENGTH{160};
	static constexpr size_t NAME_LENGTH{16};
	static constexpr size_t BAND_NAME_LENGTH{12};
	static constexpr size_t POWER_LABEL_LENGTH{4};
	enum class BandAttribute : uint8_t {
		FACTORY = 0,
		CUSTOM = 1,
	};
	// Allows to detect changes in the configuration
	using ChangeType = uint16_t;
	inline ChangeType get_change() { return _change_value; }

	constexpr Config() = default;
	inline ~Config()
	{
		pthread_mutex_destroy(&_mutex);
	}

#ifdef CONFIG_VTXTABLE_AUX_MAP
	// ================================== RC MAP ==================================
	inline int set_map_entry(uint8_t index, uint8_t rc_channel, uint8_t band, uint8_t channel,
				 int8_t power_level, uint16_t start_range, uint16_t end_range)
	{
		if (index >= MAP_LENGTH) {
			return -EINVAL;
		}

		LockGuard lg{_mutex};
		_data.rc_map[index].rc_channel = rc_channel;
		_data.rc_map[index].band = band;
		_data.rc_map[index].channel = channel;
		_data.rc_map[index].power_level = power_level;
		_data.rc_map[index].start_range = start_range;
		_data.rc_map[index].end_range = end_range;
		_change_value++;

		return 0;
	}

	inline int map_entry(uint8_t index, uint8_t *rc_channel, uint8_t *band, uint8_t *channel,
			     int8_t *power_level, uint16_t *start_range, uint16_t *end_range) const
	{
		if (index >= MAP_LENGTH) {
			return -EINVAL;
		}

		LockGuard lg{_mutex};

		if (rc_channel) { *rc_channel = _data.rc_map[index].rc_channel; }

		if (band) { *band = _data.rc_map[index].band; }

		if (channel) { *channel = _data.rc_map[index].channel; }

		if (power_level) { *power_level = _data.rc_map[index].power_level; }

		if (start_range) { *start_range = _data.rc_map[index].start_range; }

		if (end_range) { *end_range = _data.rc_map[index].end_range; }

		return 0;
	}

	inline int map_lookup(uint16_t *rc_values, size_t rc_count,
			      int8_t *band, int8_t *channel, int8_t *power_level) const
	{
		LockGuard lg{_mutex};

		for (uint8_t i = 0; i < MAP_LENGTH; i++) {
			if (_data.rc_map[i].rc_channel >= rc_count || _data.rc_map[i].is_empty()) {
				continue;
			}

			const uint16_t pwm_value = rc_values[_data.rc_map[i].rc_channel];

			if (pwm_value >= _data.rc_map[i].start_range &&
			    pwm_value < _data.rc_map[i].end_range) {
				if (_data.rc_map[i].band) { *band = _data.rc_map[i].band; }

				if (_data.rc_map[i].channel) { *channel = _data.rc_map[i].channel; }

				if (_data.rc_map[i].power_level) { *power_level = _data.rc_map[i].power_level; }
			}
		}

		return 0;
	}

	inline int map_clear()
	{
		LockGuard lg{_mutex};
		memset(&_data.rc_map, 0, sizeof(_data.rc_map));
		_change_value++;
		return 0;
	}

	inline size_t map_size() const
	{
		LockGuard lg{_mutex};
		size_t count{};

		for (uint_fast8_t i = 0; i < MAP_LENGTH; i++) {
			if (!_data.rc_map[i].is_empty()) {
				count++;
			}
		}

		return count;
	}
#endif

	// ================================ VTX TABLE =================================
	inline const char *name() const
	{
		LockGuard lg{_mutex};
		return _data.table.name;
	}
	inline bool set_name(const char *name)
	{
		if (!name) { return false; }

		LockGuard lg{_mutex};
		strncpy(_data.table.name, name, NAME_LENGTH);
		_data.table.name[NAME_LENGTH] = 0;
		_change_value++;
		return true;
	}
	inline uint16_t frequency(uint8_t band, uint8_t channel) const
	{
		LockGuard lg{_mutex};

		if (band >= _data.table.bands || channel >= _data.table.channels) {
			return 0;
		}

		return _data.table.frequency[band][channel];
	}
	inline bool set_frequency(uint8_t band, uint8_t channel, uint16_t frequency)
	{
		if (band >= BANDS || channel >= CHANNELS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.frequency[band][channel] = frequency;
		_change_value++;
		return true;
	}
	inline bool find_band_channel(uint16_t frequency, uint8_t *band, uint8_t *channel) const
	{
		LockGuard lg{_mutex};

		for (uint8_t bandi = 0; bandi < _data.table.bands; bandi++) {
			for (uint8_t channeli = 0; channeli < _data.table.channels; channeli++) {
				if (_data.table.frequency[bandi][channeli] == frequency) {
					*band = bandi;
					*channel = channeli;
					return true;
				}
			}
		}

		return false;
	}

	inline size_t channels() const
	{
		LockGuard lg{_mutex};
		return _data.table.channels;
	}
	inline bool set_channels(size_t channels)
	{
		if (channels > CHANNELS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.channels = channels;
		_change_value++;
		return true;
	}

	inline size_t bands() const
	{
		LockGuard lg{_mutex};
		return _data.table.bands;
	}
	inline bool set_bands(size_t bands)
	{
		if (bands > BANDS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.bands = bands;
		_change_value++;
		return true;
	}

	inline const char *band_name(uint8_t band) const
	{
		LockGuard lg{_mutex};

		if (band >= _data.table.bands) {
			return "?";
		}

		return const_cast<const char *>(_data.table.band[band]);
	}
	inline bool set_band_name(uint8_t band, const char *name)
	{
		if (band >= BANDS || !name) {
			return false;
		}

		LockGuard lg{_mutex};
		strncpy(_data.table.band[band], name, BAND_NAME_LENGTH);
		_data.table.band[band][BAND_NAME_LENGTH] = 0;
		_change_value++;
		return true;
	}

	inline char band_letter(uint8_t band) const
	{
		LockGuard lg{_mutex};

		if (band >= _data.table.bands) {
			return '?';
		}

		const char c = _data.table.letter[band];
		return c ? c : '?';
	}
	inline bool set_band_letter(uint8_t band, char c)
	{
		if (band >= BANDS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.letter[band] = c;
		_change_value++;
		return true;
	}

	inline BandAttribute band_attribute(uint8_t band) const
	{
		LockGuard lg{_mutex};

		if (band >= _data.table.bands) {
			return BandAttribute::FACTORY;
		}

		return _data.table.attribute[band];
	}
	inline bool set_band_attribute(uint8_t band, BandAttribute attribute)
	{
		if (band >= BANDS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.attribute[band] = attribute;
		_change_value++;
		return true;
	}

	inline size_t power_levels() const
	{
		LockGuard lg{_mutex};
		return _data.table.power_levels;
	}
	inline bool set_power_levels(size_t levels)
	{
		if (levels > POWER_LEVELS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.power_levels = levels;
		_change_value++;
		return true;
	}

	inline int16_t power_value(uint8_t level) const
	{
		LockGuard lg{_mutex};

		if (level >= _data.table.power_levels) {
			return 0;
		}

		return _data.table.power_value[level];
	}
	inline bool set_power_value(uint8_t level, int16_t value)
	{
		if (level >= POWER_LEVELS) {
			return false;
		}

		LockGuard lg{_mutex};
		_data.table.power_value[level] = value;
		_change_value++;
		return true;
	}
	inline const char *power_label(uint8_t level) const
	{
		LockGuard lg{_mutex};

		if (level >= _data.table.power_levels) {
			return "?";
		}

		return _data.table.power_label[level];
	}
	inline bool set_power_label(uint8_t level, const char *label)
	{
		if (level >= POWER_LEVELS || !label) {
			return false;
		}

		LockGuard lg{_mutex};
		strncpy(_data.table.power_label[level], label, POWER_LABEL_LENGTH);
		_data.table.power_label[level][POWER_LABEL_LENGTH] = 0;
		_change_value++;
		return true;
	}

	inline void table_clear()
	{
		LockGuard lg{_mutex};
		_data.table = {};
		_change_value++;
	}

	inline void clear()
	{
		LockGuard lg{_mutex};
		_data = {};
		_change_value++;
	}

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	inline int store(const char *filename) const
	{
		LockGuard lg{_mutex};
		int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC);

		if (fd < 0) {
			return errno;
		}

		// first write the magic/version
		const uint64_t magic = MAGIC;
		int written = write(fd, &magic, sizeof(MAGIC));

		// Then write the rest of the data
		written += write(fd, &_data, sizeof(Storage));

		// finally compute and write the crc16
		const uint16_t crc = crc16_signature(CRC16_INITIAL, sizeof(Storage), (const uint8_t *)&_data);
		written += write(fd, &crc, sizeof(crc));

		close(fd);
		return (written == FILE_SIZE) ? 0 : -EMSGSIZE;
	}

	inline int load(const char *filename)
	{
		if (access(filename, R_OK)) {
			return -ENOENT;
		}

		LockGuard lg{_mutex};
		int fd = open(filename, O_RDONLY);

		if (fd < 0) {
			return errno;
		}

		// Check file size first
		const off_t file_size = lseek(fd, 0, SEEK_END);

		if (file_size != FILE_SIZE) {
			close(fd);
			return -EMSGSIZE;
		}

		// Check the version next
		lseek(fd, 0, SEEK_SET);
		uint64_t magic{};
		const int read_size = read(fd, &magic, sizeof(magic));

		if (read_size != sizeof(magic) || magic != MAGIC) {
			close(fd);
			return -EPROTO;
		}

		// Compute and check crc16 of the data in chunks
		const size_t CHUNK{128};
		uint8_t buffer[CHUNK];
		size_t remaining{sizeof(Storage)};
		uint16_t computed_crc{0xffff};

		while (remaining > 0) {
			const int r = read(fd, buffer, (remaining < CHUNK) ? remaining : CHUNK);

			if (r <= 0) {
				close(fd);
				return -EIO;
			}

			computed_crc = crc16_signature(computed_crc, r, buffer) ^ CRC16_OUTPUT_XOR;
			remaining -= r;
		}

		// read the stored crc16
		uint16_t stored_crc{};
		const int r = read(fd, &stored_crc, sizeof(stored_crc));

		if (r != sizeof(stored_crc)) {
			close(fd);
			return -EIO;
		}

		if (computed_crc != stored_crc) {
			close(fd);
			return -EBADMSG;
		}

		// now read into the data structure directly
		lseek(fd, sizeof(magic), SEEK_SET);
		read(fd, &_data, sizeof(Storage));
		close(fd);

		_change_value++;
		return 0;
	}
#endif

private:
#ifdef CONFIG_VTXTABLE_AUX_MAP
	struct MapEntry {
		uint8_t rc_channel;
		uint8_t band;
		uint8_t channel;
		int8_t power_level;
		uint16_t start_range;
		uint16_t end_range;
		constexpr bool is_empty() const { return !start_range && !end_range; }
	} __attribute__((packed));
#endif

	struct Table {
		uint16_t frequency[BANDS][CHANNELS] {};
		int16_t power_value[POWER_LEVELS] {};
		char band[BANDS][BAND_NAME_LENGTH + 1] {};
		char power_label[POWER_LEVELS][POWER_LABEL_LENGTH + 1] {};
		char name[NAME_LENGTH + 1] {};
		char letter[BANDS] {};
		BandAttribute attribute[BANDS] {};
		uint8_t bands{};
		uint8_t channels{};
		uint8_t power_levels{};
	} __attribute__((packed));

	struct Storage {
		Table table{};
#ifdef CONFIG_VTXTABLE_AUX_MAP
		MapEntry rc_map[MAP_LENGTH];
#endif
	} __attribute__((packed));

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	static constexpr uint16_t VERSION {3};
	static constexpr uint64_t MAGIC{0x767478'636e66'0000ull | VERSION}; // "vtxcnf" + version magic
	static constexpr size_t FILE_SIZE{sizeof(Storage) + sizeof(MAGIC) + sizeof(uint16_t)};
#endif

	mutable pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	Storage _data{};
	ChangeType _change_value{};
};

}
