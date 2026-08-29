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

#include "mavlink_ftp_param.h"

#include <cstdlib>
#include <cstring>

static constexpr char kPath[] = "@PARAM/param.pck";
static constexpr size_t kPathLen = sizeof(kPath) - 1;

bool ParamPckFile::is_param_path(const char *path)
{
	return (strncmp(path, kPath, kPathLen) == 0) && ((path[kPathLen] == '\0') || (path[kPathLen] == '?'));
}

bool ParamPckFile::open(const char *path, uint16_t block_size)
{
	close();

	unsigned long start = 0;
	unsigned long count = 0;
	unsigned long with_defaults = 0;

	for (const char *c = strchr(path, '?'); c && *c; c = strchr(c, '&')) {
		c++;

		if (strncmp(c, "start=", 6) == 0) {
			start = strtoul(c + 6, nullptr, 10);

		} else if (strncmp(c, "count=", 6) == 0) {
			count = strtoul(c + 6, nullptr, 10);

		} else if (strncmp(c, "withdefaults=", 13) == 0) {
			with_defaults = strtoul(c + 13, nullptr, 10);
		}
	}

	const unsigned long used = param_count_used();

	if ((start >= used) || (count >= UINT16_MAX) || (with_defaults > 1) || (block_size == 0)) {
		return false;
	}

	_start = start;
	_total = used;
	_num = used - start;

	if ((count > 0) && (count < _num)) {
		_num = count;
	}

	_with_defaults = (with_defaults == 1);
	_block_size = block_size;

	// one pass: pack into a heap snapshot so a param_set during the download cannot
	// change later entry lengths or splice a retried block with a new value
	if (!grow(kHeaderLen + (uint32_t)_num * 16)) {
		close();
		return false;
	}

	rewind();
	write_header();

	param_t param;
	uint8_t entry[kMaxEntryLen];

	while (next_param(param)) {
		const size_t len = pack(entry, param, _cursor_ofs);

		if ((len == 0) || !grow(_cursor_ofs + len)) {
			close();
			return false;
		}

		memcpy(_data + _cursor_ofs, entry, len);
		advance(param, _cursor_ofs + len);
	}

	_size = _cursor_ofs;
	_open = true;
	return true;
}

void ParamPckFile::close()
{
	delete[] _data;
	_data = nullptr;
	_cap = 0;
	_size = 0;
	_open = false;
}

bool ParamPckFile::grow(uint32_t cap)
{
	if (cap <= _cap) {
		return true;
	}

	uint32_t ncap = (_cap > 0) ? _cap : 256;

	while (ncap < cap) {
		ncap *= 2;
	}

	uint8_t *data = new uint8_t[ncap];

	if (data == nullptr) {
		return false;
	}

	if ((_data != nullptr) && (_cursor_ofs > 0)) {
		memcpy(data, _data, _cursor_ofs);
	}

	delete[] _data;
	_data = data;
	_cap = ncap;
	return true;
}

void ParamPckFile::write_header()
{
	const uint16_t magic = _with_defaults ? kMagicWithDefaults : kMagic;
	memcpy(&_data[0], &magic, sizeof(magic));
	memcpy(&_data[2], &_num, sizeof(_num));
	memcpy(&_data[4], &_total, sizeof(_total));
}

void ParamPckFile::rewind()
{
	_cursor_ofs = kHeaderLen;
	_cursor_index = 0;
	_cursor_used = 0;
	memset(_prev_name, 0, sizeof(_prev_name));
}

bool ParamPckFile::next_param(param_t &param)
{
	const unsigned count = param_count();
	const uint16_t end_used = _start + _num;

	while ((_cursor_index < count) && (_cursor_used < end_used)) {
		param = param_for_index(_cursor_index);

		if (!param_used(param)) {
			_cursor_index++;

		} else if (_cursor_used < _start) {
			_cursor_index++;
			_cursor_used++;

		} else {
			return true;
		}
	}

	return false;
}

void ParamPckFile::advance(param_t param, uint32_t ofs)
{
	strncpy(_prev_name, param_name(param), sizeof(_prev_name) - 1);
	_prev_name[sizeof(_prev_name) - 1] = '\0';
	_cursor_ofs = ofs;
	_cursor_index++;
	_cursor_used++;
}

size_t ParamPckFile::pack(uint8_t *buf, param_t param, uint32_t ofs) const
{
	const char *name = param_name(param);
	size_t common_len = 0;

	while ((name[common_len] != '\0') && (name[common_len] == _prev_name[common_len])) {
		common_len++;
	}

	size_t name_len = strlen(name + common_len);

	if (name_len == 0) {
		if (common_len == 0) {
			return 0;
		}

		// the encoding cannot express an empty suffix
		name_len = 1;
		common_len--;
	}

	if ((common_len + name_len) > 16) {
		return 0;
	}

	const param_type_t ptype = param_type(param);

	if ((ptype != PARAM_TYPE_INT32) && (ptype != PARAM_TYPE_FLOAT)) {
		return 0;
	}

	const bool is_int32 = (ptype == PARAM_TYPE_INT32);
	union {
		int32_t i;
		float f;
		uint8_t b[4];
	} value;
	uint8_t default_value[4];
	bool add_default = false;

	if ((is_int32 ? param_get(param, &value.i) : param_get(param, &value.f)) != 0) {
		return 0;
	}

	if (_with_defaults && (param_get_default_value(param, default_value) == 0)) {
		add_default = (memcmp(value.b, default_value, sizeof(value.b)) != 0);
	}

	const uint8_t type = is_int32 ? kTypeInt32 : kTypeFloat;
	const size_t packed_len = 2 + name_len + sizeof(value.b) + (add_default ? sizeof(default_value) : 0);

	// pad so the trailing value, and default when present, do not straddle a block
	size_t pad = 0;
	const size_t data_len = sizeof(value.b) + (add_default ? sizeof(default_value) : 0);
	const uint32_t end_mod = (ofs + packed_len) % _block_size;

	if ((end_mod > 0) && (end_mod < data_len)) {
		pad = data_len - end_mod;
	}

	memset(buf, 0, pad);
	uint8_t *p = buf + pad;
	p[0] = type | (add_default ? 0x10 : 0x00);
	p[1] = common_len | ((name_len - 1) << 4);

	for (size_t i = 0; i < name_len; i++) {
		p[2 + i] = name[common_len + i];
	}

	memcpy(&p[2 + name_len], value.b, sizeof(value.b));

	if (add_default) {
		memcpy(&p[2 + name_len + sizeof(value.b)], default_value, sizeof(default_value));
	}

	return pad + packed_len;
}

int ParamPckFile::read(uint32_t offset, uint8_t *buf, uint16_t count)
{
	if (!_open || (_data == nullptr) || (count == 0) || (offset >= _size)) {
		return 0;
	}

	const uint32_t n = ((uint32_t)count < (_size - offset)) ? count : (_size - offset);
	memcpy(buf, _data + offset, n);
	return n;
}
