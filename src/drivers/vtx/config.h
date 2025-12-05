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
#ifdef CONFIG_VTX_UORB_CONFIG
#include <uORB/topics/vtx_aux_map.h>
#include <uORB/topics/vtx_table.h>
#endif

namespace vtx
{

/**
 * Class storing the VTX frequencies and power levels and the map from RC channels to VTX settings.
 * Everything is 0-indexed.
 * @author Niklas Hauser <niklas@auterion.com>
 */
class Config
{
	friend class VTX;
	static constexpr uint16_t VERSION{1};
	static constexpr uint64_t MAGIC{0x767478'636e66'0000ull | VERSION}; // "vtxcnf" + version magic
public:
	static constexpr size_t MAP_LENGTH{32};
	static constexpr size_t BANDS{8};
	static constexpr size_t CHANNELS{8};
	static constexpr size_t POWER_LEVELS{8};
	static constexpr size_t NAME_LENGTH{16};
#ifdef CONFIG_VTX_UORB_CONFIG
	static_assert(MAP_LENGTH == vtx_aux_map_s::MAX_LENGTH, "MAP_LENGTH mismatch");
	static_assert(BANDS == vtx_table_s::BANDS, "BANDS mismatch");
	static_assert(CHANNELS == vtx_table_s::CHANNELS, "CHANNELS mismatch");
	static_assert(POWER_LEVELS == vtx_table_s::LEVELS, "POWER_LEVELS mismatch");
	static_assert(NAME_LENGTH == vtx_table_s::NAME_LENGTH, "NAME_LENGTH mismatch");
#endif

	constexpr Config() = default;

	// ================================== RC MAP ==================================
	constexpr int set_map_entry(uint8_t index, uint8_t rc_channel, uint8_t band, uint8_t channel,
				    int8_t power_level, uint16_t start_range, uint16_t end_range)
	{
		if (index >= MAP_LENGTH) {
			return -EINVAL;
		}

		data.rc_map[index].rc_channel = rc_channel;
		data.rc_map[index].band = band;
		data.rc_map[index].channel = channel;
		data.rc_map[index].power_level = power_level;
		data.rc_map[index].start_range = start_range;
		data.rc_map[index].end_range = end_range;

		return 0;
	}

	constexpr int map_entry(uint8_t index, uint8_t *rc_channel, uint8_t *band, uint8_t *channel,
				int8_t *power_level, uint16_t *start_range, uint16_t *end_range)
	{
		if (index >= MAP_LENGTH) {
			return -EINVAL;
		}

		if (rc_channel) { *rc_channel = data.rc_map[index].rc_channel; }

		if (band) { *band = data.rc_map[index].band; }

		if (channel) { *channel = data.rc_map[index].channel; }

		if (power_level) { *power_level = data.rc_map[index].power_level; }

		if (start_range) { *start_range = data.rc_map[index].start_range; }

		if (end_range) { *end_range = data.rc_map[index].end_range; }

		return 0;
	}

	constexpr int map_lookup(uint16_t *rc_values, size_t rc_count,
				 int8_t *band, int8_t *channel, int8_t *power_level) const
	{
		for (uint8_t i = 0; i < MAP_LENGTH; i++) {
			if (data.rc_map[i].rc_channel >= rc_count || data.rc_map[i].is_empty()) {
				continue;
			}

			const uint16_t pwm_value = rc_values[data.rc_map[i].rc_channel];

			if (pwm_value >= data.rc_map[i].start_range &&
			    pwm_value < data.rc_map[i].end_range) {
				if (data.rc_map[i].band != 0) { *band = data.rc_map[i].band; }

				if (data.rc_map[i].channel != 0) { *channel = data.rc_map[i].channel; }

				if (data.rc_map[i].power_level != 0) { *power_level = data.rc_map[i].power_level; }
			}
		}

		return 0;
	}

	constexpr int map_clear()
	{
		memset(&data.rc_map, 0, sizeof(data.rc_map));
		return 0;
	}

	// ================================ VTX TABLE =================================
	constexpr const char *name() const
	{
		return data.table.name;
	}
	constexpr bool set_name(const char *name)
	{
		strncpy(data.table.name, name, NAME_LENGTH);
		data.table.name[NAME_LENGTH - 1] = 0;
		return true;
	}
	constexpr uint16_t frequency(uint8_t band, uint8_t channel) const
	{
		if (band >= data.table.bands || channel >= data.table.channels) {
			return 0;
		}

		return data.table.frequency[band][channel];
	}
	constexpr bool set_frequency(uint8_t band, uint8_t channel, uint16_t frequency)
	{
		if (band >= BANDS || channel >= CHANNELS) {
			return false;
		}

		data.table.frequency[band][channel] = frequency;
		return true;
	}
	constexpr bool find_band_channel(uint16_t frequency, uint8_t *band, uint8_t *channel) const
	{
		for (uint8_t bandi = 0; bandi < data.table.bands; bandi++) {
			for (uint8_t channeli = 0; channeli < data.table.channels; channeli++) {
				if (data.table.frequency[bandi][channeli] == frequency) {
					*band = bandi;
					*channel = channeli;
					return true;
				}
			}
		}

		return false;
	}

	constexpr size_t channels() const
	{
		return data.table.channels;
	}

	constexpr size_t bands() const
	{
		return data.table.bands;
	}
	constexpr bool set_bands(size_t bands)
	{
		if (bands > BANDS) {
			return false;
		}

		data.table.bands = bands;
		return true;
	}

	constexpr const char *band_name(uint8_t band) const
	{
		if (band >= data.table.bands) {
			return "?";
		}

		return const_cast<const char *>(data.table.band[band]);
	}
	constexpr bool set_band_name(uint8_t band, const char *name)
	{
		if (band >= BANDS) {
			return false;
		}

		strncpy(data.table.band[band], name, NAME_LENGTH);
		data.table.band[band][NAME_LENGTH - 1] = 0;
		return true;
	}

	constexpr char band_letter(uint8_t band) const
	{
		if (band >= data.table.bands) {
			return '?';
		}

		return data.table.letter[band];
	}
	constexpr bool set_band_letter(uint8_t band, char c)
	{
		if (band >= BANDS) {
			return false;
		}

		data.table.letter[band] = c;
		return true;
	}

	constexpr size_t power_levels() const
	{
		return data.table.levels;
	}
	constexpr bool set_power_levels(size_t levels)
	{
		if (levels > POWER_LEVELS) {
			return false;
		}

		data.table.levels = levels;
		return true;
	}

	constexpr int16_t power_value(uint8_t level) const
	{
		if (level >= data.table.levels) {
			return 0;
		}

		return data.table.power[level];
	}
	constexpr bool set_power_value(uint8_t level, int16_t value)
	{
		if (level >= POWER_LEVELS) {
			return false;
		}

		data.table.power[level] = value;
		return true;
	}
	constexpr uint16_t power_mW(uint8_t level) const
	{
		if (level >= data.table.levels) {
			return 0;
		}

		return data.table.power_mW[level];
	}
	constexpr bool set_power_mW(uint8_t level, uint16_t mW)
	{
		if (level >= POWER_LEVELS) {
			return false;
		}

		data.table.power_mW[level] = mW;
		return true;
	}

public:
#ifdef CONFIG_VTX_USE_STORAGE
	inline int store(const char *filename) const
	{
		int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC);

		if (fd < 0) {
			return errno;
		}

		const int size = write(fd, &data, sizeof(Storage));
		close(fd);
		return (size == sizeof(Storage)) ? 0 : -EMSGSIZE;
	}

	inline int load(const char *filename)
	{
		if (access(filename, R_OK)) {
			return -ENOENT;
		}

		int fd = open(filename, O_RDONLY);

		if (fd < 0) {
			return errno;
		}

		// Check the version before copying the data
		Storage load_data;
		const int size = read(fd, &load_data, sizeof(Storage));
		close(fd);

		if (size != sizeof(Storage)) {
			return -EMSGSIZE;
		}

		if (load_data.magic != MAGIC) {
			return -EPROTO;
		}

		data = load_data;
		return 0;
	}
#endif

#ifdef CONFIG_VTX_UORB_CONFIG
	inline void copy_to(vtx_table_s *table)
	{
		static_assert(BANDS == 8, "need to update copy_to()");
		memcpy(table->frequency0, data.table.frequency[0], sizeof(table->frequency0));
		memcpy(table->frequency1, data.table.frequency[1], sizeof(table->frequency1));
		memcpy(table->frequency2, data.table.frequency[2], sizeof(table->frequency2));
		memcpy(table->frequency3, data.table.frequency[3], sizeof(table->frequency3));
		memcpy(table->frequency4, data.table.frequency[4], sizeof(table->frequency4));
		memcpy(table->frequency5, data.table.frequency[5], sizeof(table->frequency5));
		memcpy(table->frequency6, data.table.frequency[6], sizeof(table->frequency6));
		memcpy(table->frequency7, data.table.frequency[7], sizeof(table->frequency7));
		memcpy(table->name, data.table.name, sizeof(table->name));
		memcpy(table->band0, data.table.band[0], sizeof(table->band0));
		memcpy(table->band1, data.table.band[1], sizeof(table->band1));
		memcpy(table->band2, data.table.band[2], sizeof(table->band2));
		memcpy(table->band3, data.table.band[3], sizeof(table->band3));
		memcpy(table->band4, data.table.band[4], sizeof(table->band4));
		memcpy(table->band5, data.table.band[5], sizeof(table->band5));
		memcpy(table->band6, data.table.band[6], sizeof(table->band6));
		memcpy(table->band7, data.table.band[7], sizeof(table->band7));
		memcpy(table->letter, data.table.letter, sizeof(table->letter));
		memcpy(table->power, data.table.power, sizeof(table->power));
		memcpy(table->power_mw, data.table.power_mW, sizeof(table->power_mw));

		table->bands = data.table.bands;
		table->channels = data.table.channels;
		table->levels = data.table.levels;
	}

	inline void copy_from(vtx_table_s *table)
	{
		static_assert(BANDS == 8, "need to update copy_from()");
		memcpy(data.table.frequency[0], table->frequency0, sizeof(data.table.frequency[0]));
		memcpy(data.table.frequency[1], table->frequency1, sizeof(data.table.frequency[1]));
		memcpy(data.table.frequency[2], table->frequency2, sizeof(data.table.frequency[2]));
		memcpy(data.table.frequency[3], table->frequency3, sizeof(data.table.frequency[3]));
		memcpy(data.table.frequency[4], table->frequency4, sizeof(data.table.frequency[4]));
		memcpy(data.table.frequency[5], table->frequency5, sizeof(data.table.frequency[5]));
		memcpy(data.table.frequency[6], table->frequency6, sizeof(data.table.frequency[6]));
		memcpy(data.table.frequency[7], table->frequency7, sizeof(data.table.frequency[7]));
		memcpy(data.table.name, table->name, sizeof(data.table.name));
		memcpy(data.table.band[0], table->band0, sizeof(data.table.band[0]));
		memcpy(data.table.band[1], table->band1, sizeof(data.table.band[1]));
		memcpy(data.table.band[2], table->band2, sizeof(data.table.band[2]));
		memcpy(data.table.band[3], table->band3, sizeof(data.table.band[3]));
		memcpy(data.table.band[4], table->band4, sizeof(data.table.band[4]));
		memcpy(data.table.band[5], table->band5, sizeof(data.table.band[5]));
		memcpy(data.table.band[6], table->band6, sizeof(data.table.band[6]));
		memcpy(data.table.band[7], table->band7, sizeof(data.table.band[7]));
		memcpy(data.table.letter, table->letter, sizeof(data.table.letter));
		memcpy(data.table.power, table->power, sizeof(data.table.power));
		memcpy(data.table.power_mW, table->power_mw, sizeof(data.table.power_mW));

		data.table.bands = table->bands > BANDS ? BANDS : table->bands;
		data.table.channels = table->channels > CHANNELS ? CHANNELS : table->channels;
		data.table.levels = table->levels > POWER_LEVELS ? POWER_LEVELS : table->levels;
	}

	inline void copy_to(vtx_aux_map_s *map)
	{
		map->length = MAP_LENGTH;

		for (uint8_t i = 0; i < MAP_LENGTH; i++) {
			map->aux_channel[i] = data.rc_map[i].rc_channel - 4 + 1;
			map->band[i] = data.rc_map[i].band;
			map->channel[i] = data.rc_map[i].channel;
			map->power_level[i] = data.rc_map[i].power_level;
			map->start_range[i] = data.rc_map[i].start_range;
			map->end_range[i] = data.rc_map[i].end_range;
		}
	}

	inline void copy_from(vtx_aux_map_s *map)
	{
		map_clear();

		for (uint8_t i = 0; i < MAP_LENGTH; i++) {
			set_map_entry(i, map->aux_channel[i] + 4 - 1,
				      map->band[i], map->channel[i], map->power_level[i],
				      map->start_range[i], map->end_range[i]);
		}
	}
#endif

protected:
	struct MapEntry {
		uint8_t rc_channel;
		uint8_t band;
		uint8_t channel;
		int8_t power_level;
		uint16_t start_range;
		uint16_t end_range;
		constexpr bool is_empty() const { return !start_range && !end_range; }
	} __attribute__((packed));

	struct Table {
		// The table content is empty by default to force initialization externally
		// Default content was confusing due to too many different VTX tables out there
		uint16_t frequency[BANDS][CHANNELS] {};
		char band[BANDS][NAME_LENGTH] {};
		// GCC9 does not support string initializers for arrays
		char name[NAME_LENGTH] {'U', 'N', 'I', 'N', 'I', 'T', 'I', 'A', 'L', 'I', 'Z', 'E', 'D', 0};
		char letter[BANDS] {};
		int16_t power[POWER_LEVELS] {};
		uint16_t power_mW[POWER_LEVELS] {};
		uint8_t levels{};
		uint8_t bands{};
		uint8_t channels{CHANNELS};
	} __attribute__((packed));

	struct Storage {
		uint64_t magic{MAGIC};
		MapEntry rc_map[MAP_LENGTH];
		Table table{};
	} __attribute__((packed));
	Storage data{};
};

}
