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

bool ParamPckFile::bit(const uint8_t *bits, param_t param) const
{
	return (param < kMaxParams) && ((bits[param / 8] & (1u << (param % 8))) != 0);
}

void ParamPckFile::set_bit(uint8_t *bits, param_t param)
{
	bits[param / 8] |= (uint8_t)(1u << (param % 8));
}

bool ParamPckFile::default_differs(param_t param) const
{
	union {
		int32_t i;
		float f;
		uint8_t b[4];
	} value, def {};

	const bool is_int32 = (param_type(param) == PARAM_TYPE_INT32);

	if ((is_int32 ? param_get(param, &value.i) : param_get(param, &value.f)) != 0) {
		return false;
	}

	if (param_get_default_value(param, def.b) != 0) {
		return false;
	}

	return memcmp(value.b, def.b, sizeof(value.b)) != 0;
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
	const unsigned nparams = param_count();

	if ((start >= used) || (count >= UINT16_MAX) || (with_defaults > 1) || (block_size == 0)
	    || (nparams > kMaxParams)) {
		return false;
	}

	_total = used;
	_num = used - start;

	if ((count > 0) && (count < _num)) {
		_num = count;
	}

	_with_defaults = (with_defaults == 1);
	_block_size = block_size;

	const uint16_t end_used = start + _num;
	uint16_t used_i = 0;

	for (unsigned i = 0; i < nparams; i++) {
		const param_t param = param_for_index(i);

		if (!param_used(param)) {
			continue;
		}

		if ((used_i >= start) && (used_i < end_used)) {
			set_bit(_included, param);

			if (_with_defaults && default_differs(param)) {
				set_bit(_has_default, param);
			}
		}

		used_i++;
	}

	rewind();
	param_t param;
	uint8_t entry[kMaxEntryLen];

	while (next_param(param)) {
		const size_t len = pack(entry, param, _cursor_ofs);

		if (len == 0) {
			close();
			return false;
		}

		advance(param, _cursor_ofs + len);
	}

	_size = _cursor_ofs;
	rewind();
	_open = true;
	return true;
}

bool ParamPckFile::open_write()
{
	close();
	_write = true;
	_write_failed = false;
	_header_done = false;
	_write_num = 0;
	_write_file_len = 0;
	_write_seen = 0;
	_applied = 0;
	_write_ofs = 0;
	_pending_len = 0;
	memset(_pending, 0, sizeof(_pending));
	memset(_prev_name, 0, sizeof(_prev_name));
	_open = true;
	return true;
}

void ParamPckFile::close()
{
	if (_write && (_applied > 0)) {
		param_notify_changes();
	}

	memset(_included, 0, sizeof(_included));
	memset(_has_default, 0, sizeof(_has_default));
	_size = 0;
	_open = false;
	_write = false;
	_write_failed = false;
	_header_done = false;
	_write_num = 0;
	_write_file_len = 0;
	_write_seen = 0;
	_applied = 0;
	_write_ofs = 0;
	_pending_len = 0;
}

void ParamPckFile::rewind()
{
	_cursor_ofs = kHeaderLen;
	_cursor_index = 0;
	memset(_prev_name, 0, sizeof(_prev_name));
}

bool ParamPckFile::next_param(param_t &param)
{
	const unsigned count = param_count();

	while (_cursor_index < count) {
		param = param_for_index(_cursor_index);

		if (bit(_included, param)) {
			return true;
		}

		_cursor_index++;
	}

	return false;
}

void ParamPckFile::advance(param_t param, uint32_t ofs)
{
	strncpy(_prev_name, param_name(param), sizeof(_prev_name) - 1);
	_prev_name[sizeof(_prev_name) - 1] = '\0';
	_cursor_ofs = ofs;
	_cursor_index++;
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
	uint8_t default_value[4] {};

	if ((is_int32 ? param_get(param, &value.i) : param_get(param, &value.f)) != 0) {
		return 0;
	}

	const bool add_default = bit(_has_default, param);

	if (add_default && (param_get_default_value(param, default_value) != 0)) {
		memset(default_value, 0, sizeof(default_value));
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
	if (!_open || _write || (count == 0) || (offset >= _size)) {
		return _write ? -1 : 0;
	}

	if (offset + count > _size) {
		count = _size - offset;
	}

	uint16_t copied = 0;

	if (offset < kHeaderLen) {
		const uint16_t magic = _with_defaults ? kMagicWithDefaults : kMagic;
		uint8_t hdr[kHeaderLen];
		memcpy(&hdr[0], &magic, sizeof(magic));
		memcpy(&hdr[2], &_num, sizeof(_num));
		memcpy(&hdr[4], &_total, sizeof(_total));

		const size_t n = ((kHeaderLen - offset) < count) ? (kHeaderLen - offset) : count;
		memcpy(buf, &hdr[offset], n);
		copied = n;
		offset += n;
	}

	if (offset < _cursor_ofs) {
		rewind();
	}

	param_t param;
	uint8_t entry[kMaxEntryLen];

	while ((copied < count) && next_param(param)) {
		const size_t len = pack(entry, param, _cursor_ofs);

		if (len == 0) {
			return -1;
		}

		const uint32_t entry_end = _cursor_ofs + len;

		if (offset < entry_end) {
			const size_t skip = offset - _cursor_ofs;
			const size_t avail = len - skip;
			const size_t n = (avail < static_cast<size_t>(count - copied)) ? avail : (count - copied);
			memcpy(buf + copied, entry + skip, n);
			copied += n;
			offset += n;
		}

		if (offset < entry_end) {
			break;
		}

		advance(param, entry_end);
	}

	return copied;
}

uint8_t ParamPckFile::packed_value_len(uint8_t ptype)
{
	switch (ptype) {
	case kTypeInt8:
		return 1;

	case kTypeInt16:
		return 2;

	case kTypeInt32:
	case kTypeFloat:
		return 4;

	default:
		return 0;
	}
}

bool ParamPckFile::apply_entry(const char *name, uint8_t ptype, const uint8_t *raw)
{
	const param_t param = param_find_no_notification(name);

	if ((param == PARAM_INVALID) || param_is_readonly(param)) {
		return true;
	}

	const param_type_t dest = param_type(param);
	union {
		int32_t i;
		float f;
		uint8_t b[4];
	} value {};

	int32_t as_int = 0;
	float as_float = 0.f;

	switch (ptype) {
	case kTypeInt8:
		as_int = static_cast<int8_t>(raw[0]);
		as_float = static_cast<float>(as_int);
		break;

	case kTypeInt16: {
			int16_t v;
			memcpy(&v, raw, sizeof(v));
			as_int = v;
			as_float = static_cast<float>(v);
			break;
		}

	case kTypeInt32:
		memcpy(&as_int, raw, sizeof(as_int));
		as_float = static_cast<float>(as_int);
		break;

	case kTypeFloat:
		memcpy(&as_float, raw, sizeof(as_float));
		as_int = static_cast<int32_t>(as_float);
		break;

	default:
		return false;
	}

	if (dest == PARAM_TYPE_INT32) {
		value.i = as_int;

	} else if (dest == PARAM_TYPE_FLOAT) {
		value.f = as_float;

	} else {
		return true;
	}

	if (param_set_no_notification(param, value.b) == 0) {
		_applied++;
	}

	return true;
}

bool ParamPckFile::parse_pending()
{
	if (!_header_done) {
		if (_pending_len < kHeaderLen) {
			return true;
		}

		uint16_t magic;
		memcpy(&magic, _pending, sizeof(magic));
		memcpy(&_write_num, _pending + 2, sizeof(_write_num));
		memcpy(&_write_file_len, _pending + 4, sizeof(_write_file_len));

		if ((magic != kMagic) || (_write_file_len < kHeaderLen)) {
			return false;
		}

		memmove(_pending, _pending + kHeaderLen, _pending_len - kHeaderLen);
		_pending_len -= kHeaderLen;
		_header_done = true;
	}

	while (_pending_len > 0) {
		if (_pending[0] == 0) {
			memmove(_pending, _pending + 1, _pending_len - 1);
			_pending_len--;
			continue;
		}

		if (_pending_len < 2) {
			return true;
		}

		const uint8_t ptype = _pending[0] & 0x0F;
		const uint8_t flags = _pending[0] >> 4;
		const uint8_t common_len = _pending[1] & 0x0F;
		const uint8_t name_len = (_pending[1] >> 4) + 1;
		const uint8_t vlen = packed_value_len(ptype);
		const size_t prev_len = strlen(_prev_name);

		if ((flags != 0) || (vlen == 0) || ((common_len + name_len) > 16) || (common_len > prev_len)) {
			return false;
		}

		const size_t need = 2 + name_len + vlen;

		if (need > sizeof(_pending)) {
			return false;
		}

		if (_pending_len < need) {
			return true;
		}

		char name[17];
		memcpy(name, _prev_name, common_len);
		memcpy(name + common_len, _pending + 2, name_len);
		name[common_len + name_len] = '\0';

		if (!apply_entry(name, ptype, _pending + 2 + name_len)) {
			return false;
		}

		strncpy(_prev_name, name, sizeof(_prev_name) - 1);
		_prev_name[sizeof(_prev_name) - 1] = '\0';
		memmove(_pending, _pending + need, _pending_len - need);
		_pending_len -= need;
		_write_seen++;
	}

	return true;
}

bool ParamPckFile::ingest(const uint8_t *data, uint16_t count)
{
	while (count > 0) {
		if (_pending_len == sizeof(_pending)) {
			if (!parse_pending() || (_pending_len == sizeof(_pending))) {
				return false;
			}
		}

		const uint16_t room = static_cast<uint16_t>(sizeof(_pending) - _pending_len);
		const uint16_t n = (count < room) ? count : room;
		memcpy(_pending + _pending_len, data, n);
		_pending_len += n;
		data += n;
		count -= n;
		_write_ofs += n;

		if (!parse_pending()) {
			return false;
		}

		if (_header_done && (_write_ofs > _write_file_len)) {
			return false;
		}
	}

	return true;
}

int ParamPckFile::write(uint32_t offset, const uint8_t *buf, uint16_t count)
{
	if (!_open || !_write || _write_failed) {
		return -1;
	}

	if (count == 0) {
		return 0;
	}

	const uint32_t end = offset + count;

	if (end <= _write_ofs) {
		return count;
	}

	if (offset > _write_ofs) {
		_write_failed = true;
		return -1;
	}

	const uint16_t skip = static_cast<uint16_t>(_write_ofs - offset);

	if (!ingest(buf + skip, count - skip)) {
		_write_failed = true;
		return -1;
	}

	return count;
}

bool ParamPckFile::finish_write()
{
	if (!_open || !_write || _write_failed) {
		return false;
	}

	if (!parse_pending()) {
		_write_failed = true;
		return false;
	}

	while ((_pending_len > 0) && (_pending[0] == 0)) {
		memmove(_pending, _pending + 1, _pending_len - 1);
		_pending_len--;
	}

	if (!_header_done || (_pending_len != 0) || (_write_ofs != _write_file_len)
	    || (_write_seen != _write_num)) {
		_write_failed = true;
		return false;
	}

	return true;
}
