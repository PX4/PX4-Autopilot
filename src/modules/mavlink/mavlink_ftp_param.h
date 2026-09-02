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

#pragma once

#include <cstddef>
#include <cstdint>

#include <parameters/param.h>

/**
 * The used parameter set as a virtual MAVLink FTP file, `@PARAM/param.pck`,
 * in ArduPilot's AP_Filesystem_Param wire format:
 *
 *   header: uint16 magic (0x671B, or 0x671C when defaults are included)
 *           uint16 num_params  (entries in this file)
 *           uint16 total_params  (download: used count; upload: byte length of the file)
 *   entry:  uint8  type:4 flags:4       1 int8, 2 int16, 3 int32, 4 float; flag bit 0 = default follows
 *           uint8  common_len:4 name_len-1:4   name bytes shared with the previous entry, remaining name length
 *           name[name_len]
 *           value[type size]
 *           default[type size]          only when flag bit 0 is set, i.e. value != default
 *
 * Zero bytes before an entry are padding so the trailing value (and default, when
 * present) never straddles a read block of block_size bytes. The path accepts a
 * query `?start=N&count=N&withdefaults=0|1`; count 0 means all parameters from start.
 *
 * open() freezes which used parameters appear and whether each includes a default
 * (~1 KB of bitsets). Values are read live. Padding keeps each value/default inside
 * one FTP block, so a param_set during the download cannot shift later entries or
 * splice two generations of a number across a retried block.
 *
 * Upload (CreateFile/WriteFile/Terminate) accepts magic 0x671B only, no defaults,
 * and treats total_params as the file length. Entries are applied as they complete;
 * unknown and read-only names are skipped. Packed int8/int16 values are widened to
 * the PX4 INT32/FLOAT type. Terminate ACKs only when the file is well-formed.
 */
class ParamPckFile
{
public:
	static constexpr size_t kHeaderLen = 6;
	static constexpr size_t kMaxEntryLen = 7 + 2 + 16 + 4 + 4; ///< pad + header + name + value + default
	static constexpr unsigned kMaxParams = 4096;

	ParamPckFile() = default;
	~ParamPckFile() { close(); }
	ParamPckFile(const ParamPckFile &) = delete;
	ParamPckFile &operator=(const ParamPckFile &) = delete;

	/// True if path names the packed parameter file, with or without a query
	static bool is_param_path(const char *path);

	/// Parse the query and freeze membership. False on an invalid query or empty range.
	bool open(const char *path, uint16_t block_size);

	/// Open for sequential WriteFile of a packed upload.
	bool open_write();

	void close();
	bool is_open() const { return _open; }
	bool is_writing() const { return _open && _write; }

	uint32_t size() const { return _size; }

	/**
	 * Copy up to count bytes starting at offset.
	 * @return bytes copied, 0 at EOF, -1 on a parameter that cannot be packed
	 */
	int read(uint32_t offset, uint8_t *buf, uint16_t count);

	/**
	 * Accept a WriteFile chunk. Sequential from 0, or a retry of already-consumed bytes.
	 * @return count on success, -1 on a hole or a malformed stream
	 */
	int write(uint32_t offset, const uint8_t *buf, uint16_t count);

	/// True when the uploaded file is complete and well-formed. Call before close() on Terminate.
	bool finish_write();

private:
	static constexpr uint16_t kMagic = 0x671B;
	static constexpr uint16_t kMagicWithDefaults = 0x671C;
	static constexpr uint8_t kTypeInt8 = 1;
	static constexpr uint8_t kTypeInt16 = 2;
	static constexpr uint8_t kTypeInt32 = 3;
	static constexpr uint8_t kTypeFloat = 4;
	static constexpr unsigned kBitBytes = kMaxParams / 8;

	/// Pack one entry starting at file offset ofs. Returns its length including padding, 0 on error.
	size_t pack(uint8_t *buf, param_t param, uint32_t ofs) const;

	/// Move the cursor to the next frozen parameter without consuming it. False at the end.
	bool next_param(param_t &param);

	/// Consume the parameter at the cursor, whose entry ends at ofs
	void advance(param_t param, uint32_t ofs);

	void rewind();
	bool bit(const uint8_t *bits, param_t param) const;
	void set_bit(uint8_t *bits, param_t param);
	bool default_differs(param_t param) const;

	bool ingest(const uint8_t *data, uint16_t count);
	bool parse_pending();
	bool apply_entry(const char *name, uint8_t ptype, const uint8_t *raw);
	static uint8_t packed_value_len(uint8_t ptype);

	bool _open{false};
	bool _write{false};
	bool _with_defaults{false};
	uint16_t _block_size{0};
	uint16_t _num{0};
	uint16_t _total{0};
	uint32_t _size{0};
	uint8_t _included[kBitBytes] {};
	uint8_t _has_default[kBitBytes] {};

	uint32_t _cursor_ofs{kHeaderLen};
	unsigned _cursor_index{0};    ///< param_for_index() index
	char _prev_name[17] {};

	bool _write_failed{false};
	bool _header_done{false};
	uint16_t _write_num{0};
	uint16_t _write_file_len{0};
	uint16_t _write_seen{0};
	uint16_t _applied{0};
	uint32_t _write_ofs{0};
	uint8_t _pending_len{0};
	uint8_t _pending[kMaxEntryLen] {};
};
