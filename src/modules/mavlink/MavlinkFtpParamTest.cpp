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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{

struct FakeParam {
	std::string name;
	param_type_t type;
	bool used;
	uint32_t value;
	uint32_t default_value;
	bool readonly{false};
};

std::vector<FakeParam> g_params;

uint32_t bits(float f)
{
	uint32_t u;
	memcpy(&u, &f, sizeof(u));
	return u;
}

FakeParam f32(const char *name, bool used, float value, float default_value)
{
	return {name, PARAM_TYPE_FLOAT, used, bits(value), bits(default_value)};
}

FakeParam i32(const char *name, bool used, int32_t value, int32_t default_value, bool readonly = false)
{
	return {name, PARAM_TYPE_INT32, used, static_cast<uint32_t>(value), static_cast<uint32_t>(default_value), readonly};
}

std::vector<FakeParam> used_params()
{
	std::vector<FakeParam> out;

	for (const auto &p : g_params) {
		if (p.used) {
			out.push_back(p);
		}
	}

	return out;
}

struct Entry {
	std::string name;
	uint8_t type;
	uint8_t common_len;
	uint32_t value;
	bool has_default;
	uint32_t default_value;
	size_t value_ofs;
};

struct Decoded {
	uint16_t magic;
	uint16_t num;
	uint16_t total;
	std::vector<Entry> entries;
};

bool decode(const std::vector<uint8_t> &f, Decoded &d)
{
	if (f.size() < ParamPckFile::kHeaderLen) {
		return false;
	}

	memcpy(&d.magic, f.data(), 2);
	memcpy(&d.num, f.data() + 2, 2);
	memcpy(&d.total, f.data() + 4, 2);
	d.entries.clear();

	std::string prev;
	size_t i = ParamPckFile::kHeaderLen;

	while (i < f.size()) {
		if (f[i] == 0) {
			i++;
			continue;
		}

		if ((i + 2) > f.size()) {
			return false;
		}

		Entry e{};
		e.type = f[i] & 0x0F;
		e.has_default = (f[i] >> 4) & 0x01;
		e.common_len = f[i + 1] & 0x0F;
		const uint8_t name_len = (f[i + 1] >> 4) + 1;
		i += 2;

		if ((e.common_len > prev.size()) || ((e.common_len + name_len) > 16) || ((i + name_len) > f.size())) {
			return false;
		}

		e.name = prev.substr(0, e.common_len) + std::string(reinterpret_cast<const char *>(f.data() + i), name_len);
		i += name_len;
		e.value_ofs = i;

		if ((i + 4) > f.size()) {
			return false;
		}

		memcpy(&e.value, f.data() + i, 4);
		i += 4;

		if (e.has_default) {
			if ((i + 4) > f.size()) {
				return false;
			}

			memcpy(&e.default_value, f.data() + i, 4);
			i += 4;
		}

		d.entries.push_back(e);
		prev = e.name;
	}

	return d.entries.size() == d.num;
}

/// Sequential reads of block bytes, the way a burst download proceeds
std::vector<uint8_t> download(ParamPckFile &file, uint16_t block)
{
	std::vector<uint8_t> out;
	uint8_t buf[512];

	for (uint32_t ofs = 0;;) {
		const int n = file.read(ofs, buf, block);
		EXPECT_GE(n, 0);

		if (n <= 0) {
			break;
		}

		out.insert(out.end(), buf, buf + n);
		ofs += n;
	}

	return out;
}

class MavlinkFtpParam : public ::testing::Test
{
protected:
	void SetUp() override
	{
		g_params = {
			f32("MPC_XY_P", true, 0.95f, 0.95f),
			f32("MPC_XY_VEL_P_ACC", true, 1.8f, 1.8f),
			f32("MPC_XY_VEL_I_ACC", false, 0.4f, 0.4f),
			f32("MPC_Z_P", true, 1.2f, 1.0f),
			i32("SYS_AUTOSTART", true, 4001, 0),
			i32("SYS_AUTOCONFIG", false, 0, 0),
			i32("SYS_HITL", true, 0, 0),
			i32("A", true, -1, -1),
			f32("ABCDEFGHIJKLMNOP", true, 2.f, 2.f),
			f32("ABCDEFGHIJKLMNOQ", true, 3.f, 2.f),
			i32("B", true, 7, 7),
			i32("CAL_GYRO0_ID", true, 99, 0, true),
		};

		for (int i = 0; i < 300; i++) {
			char name[17];
			snprintf(name, sizeof(name), "GEN_%03d_X", i);
			g_params.push_back(i32(name, (i % 5) != 0, i, (i % 3 == 0) ? 0 : i));
		}
	}
};

} // namespace

// parameter API backed by g_params; only the accessors ParamPckFile uses, plus the
// logger behind param.h's SITL type check
void px4_log_modulename(int, const char *, const char *, ...) {}
#undef param_get
unsigned param_count() { return g_params.size(); }
unsigned param_count_used() { return used_params().size(); }
param_t param_for_index(unsigned index) { return (index < g_params.size()) ? index : PARAM_INVALID; }
bool param_used(param_t param) { return (param < g_params.size()) && g_params[param].used; }
const char *param_name(param_t param) { return g_params[param].name.c_str(); }
param_type_t param_type(param_t param) { return g_params[param].type; }
int param_get(param_t param, void *val) { memcpy(val, &g_params[param].value, 4); return 0; }
int param_get_default_value(param_t param, void *val) { memcpy(val, &g_params[param].default_value, 4); return 0; }
param_t param_find_no_notification(const char *name)
{
	for (size_t i = 0; i < g_params.size(); i++) {
		if (g_params[i].name == name) {
			return static_cast<param_t>(i);
		}
	}

	return PARAM_INVALID;
}
bool param_is_readonly(param_t param) { return (param < g_params.size()) && g_params[param].readonly; }
int param_set_no_notification(param_t param, const void *val)
{
	if ((param >= g_params.size()) || g_params[param].readonly) {
		return -1;
	}

	memcpy(&g_params[param].value, val, 4);
	return 0;
}
void param_notify_changes() {}

TEST_F(MavlinkFtpParam, Path)
{
	EXPECT_TRUE(ParamPckFile::is_param_path("@PARAM/param.pck"));
	EXPECT_TRUE(ParamPckFile::is_param_path("@PARAM/param.pck?withdefaults=1"));
	EXPECT_FALSE(ParamPckFile::is_param_path("@PARAM/param.pckx"));
	EXPECT_FALSE(ParamPckFile::is_param_path("@PARAM/other"));
	EXPECT_FALSE(ParamPckFile::is_param_path("/fs/microsd/param.pck"));

	ParamPckFile file;
	EXPECT_TRUE(file.open("@PARAM/param.pck", 239));
	EXPECT_TRUE(file.open("@PARAM/param.pck?start=1&count=2&withdefaults=1", 239));
	EXPECT_TRUE(file.open("@PARAM/param.pck?unknown=1", 239));
	EXPECT_FALSE(file.open("@PARAM/param.pck?withdefaults=2", 239));
	EXPECT_FALSE(file.open("@PARAM/param.pck?count=65535", 239));
	EXPECT_FALSE(file.open("@PARAM/param.pck?start=100000", 239));
	EXPECT_FALSE(file.open("@PARAM/param.pck", 0));

	const std::string past_end = "@PARAM/param.pck?start=" + std::to_string(used_params().size());
	EXPECT_FALSE(file.open(past_end.c_str(), 239));
}

TEST_F(MavlinkFtpParam, Roundtrip)
{
	ParamPckFile file;
	ASSERT_TRUE(file.open("@PARAM/param.pck", 239));

	const std::vector<uint8_t> f = download(file, 239);
	EXPECT_EQ(f.size(), file.size());

	Decoded d;
	ASSERT_TRUE(decode(f, d));
	EXPECT_EQ(d.magic, 0x671B);

	const std::vector<FakeParam> used = used_params();
	EXPECT_EQ(d.num, used.size());
	EXPECT_EQ(d.total, used.size());
	ASSERT_EQ(d.entries.size(), used.size());

	for (size_t i = 0; i < used.size(); i++) {
		EXPECT_EQ(d.entries[i].name, used[i].name);
		EXPECT_EQ(d.entries[i].type, (used[i].type == PARAM_TYPE_INT32) ? 3 : 4);
		EXPECT_EQ(d.entries[i].value, used[i].value);
		EXPECT_FALSE(d.entries[i].has_default);
	}

	// "MPC_XY_VEL_P_ACC" after "MPC_XY_P", "ABCDEFGHIJKLMNOQ" after "ABCDEFGHIJKLMNOP"
	EXPECT_EQ(d.entries[0].common_len, 0);
	EXPECT_EQ(d.entries[1].common_len, 7);
	EXPECT_EQ(d.entries[7].common_len, 15);

	uint8_t buf[8];
	EXPECT_EQ(file.read(f.size(), buf, sizeof(buf)), 0);
	EXPECT_EQ(file.read(f.size() + 1000, buf, sizeof(buf)), 0);

	file.close();
	EXPECT_FALSE(file.is_open());
	EXPECT_EQ(file.read(0, buf, sizeof(buf)), 0);
}

TEST_F(MavlinkFtpParam, WithDefaults)
{
	ParamPckFile file;
	ASSERT_TRUE(file.open("@PARAM/param.pck?withdefaults=1", 239));

	const std::vector<uint8_t> f = download(file, 239);
	EXPECT_EQ(f.size(), file.size());

	Decoded d;
	ASSERT_TRUE(decode(f, d));
	EXPECT_EQ(d.magic, 0x671C);

	const std::vector<FakeParam> used = used_params();
	ASSERT_EQ(d.entries.size(), used.size());

	for (size_t i = 0; i < used.size(); i++) {
		EXPECT_EQ(d.entries[i].value, used[i].value);
		EXPECT_EQ(d.entries[i].has_default, used[i].value != used[i].default_value) << used[i].name;

		if (d.entries[i].has_default) {
			EXPECT_EQ(d.entries[i].default_value, used[i].default_value);
		}
	}
}

TEST_F(MavlinkFtpParam, StartCount)
{
	ParamPckFile file;
	ASSERT_TRUE(file.open("@PARAM/param.pck?start=2&count=3", 239));

	Decoded d;
	ASSERT_TRUE(decode(download(file, 239), d));

	const std::vector<FakeParam> used = used_params();
	EXPECT_EQ(d.num, 3);
	EXPECT_EQ(d.total, used.size());
	ASSERT_EQ(d.entries.size(), 3u);

	for (size_t i = 0; i < 3; i++) {
		EXPECT_EQ(d.entries[i].name, used[2 + i].name);
		EXPECT_EQ(d.entries[i].value, used[2 + i].value);
	}

	EXPECT_EQ(d.entries[0].common_len, 0);

	const std::string last = "@PARAM/param.pck?start=" + std::to_string(used.size() - 1);
	ASSERT_TRUE(file.open(last.c_str(), 239));
	ASSERT_TRUE(decode(download(file, 239), d));
	EXPECT_EQ(d.num, 1);
	ASSERT_EQ(d.entries.size(), 1u);
	EXPECT_EQ(d.entries[0].name, used.back().name);
}

TEST_F(MavlinkFtpParam, RandomAccessMatchesSequential)
{
	const uint16_t block = 32;
	ParamPckFile file;
	ASSERT_TRUE(file.open("@PARAM/param.pck?withdefaults=1", block));

	const std::vector<uint8_t> f = download(file, block);
	EXPECT_EQ(f.size(), file.size());
	ASSERT_GT(f.size(), 20u * block);

	std::vector<uint32_t> offsets;

	for (uint32_t ofs = 0; ofs < f.size(); ofs += block) {
		offsets.push_back(ofs);
	}

	// hole fill: same block size, arbitrary order
	std::reverse(offsets.begin(), offsets.end());
	std::swap(offsets[3], offsets[offsets.size() / 2]);

	for (const uint32_t ofs : offsets) {
		uint8_t buf[block];
		const int n = file.read(ofs, buf, block);
		ASSERT_GT(n, 0);
		ASSERT_EQ(static_cast<size_t>(n), std::min<size_t>(block, f.size() - ofs));
		EXPECT_EQ(memcmp(buf, f.data() + ofs, n), 0) << "offset " << ofs;
	}

	// reads smaller than the block size return the same bytes
	for (uint32_t ofs = 1; ofs < f.size(); ofs += 7) {
		uint8_t buf[5];
		const int n = file.read(ofs, buf, sizeof(buf));
		ASSERT_GT(n, 0);
		EXPECT_EQ(memcmp(buf, f.data() + ofs, n), 0) << "offset " << ofs;
	}
}

TEST_F(MavlinkFtpParam, FrozenMembershipLiveValues)
{
	ParamPckFile file;
	ASSERT_TRUE(file.open("@PARAM/param.pck?withdefaults=1", 239));

	Decoded d0;
	ASSERT_TRUE(decode(download(file, 239), d0));
	ASSERT_FALSE(d0.entries.empty());
	const uint32_t size0 = file.size();

	const auto find = [](const Decoded & d, const char *name) -> const Entry * {
		for (const Entry &e : d.entries)
		{
			if (e.name == name) {
				return &e;
			}
		}

		return nullptr;
	};

	ASSERT_NE(find(d0, "MPC_XY_P"), nullptr);
	ASSERT_NE(find(d0, "MPC_Z_P"), nullptr);
	ASSERT_TRUE(find(d0, "SYS_AUTOSTART")->has_default);

	// value crosses default, a used param drops, a new one appears
	g_params[0].value = bits(42.f);                 // MPC_XY_P, was at default
	g_params[4].value = g_params[4].default_value;  // SYS_AUTOSTART, had a default field
	g_params[3].used = false;                       // MPC_Z_P
	g_params.push_back(i32("NEW_PARAM", true, 1, 0));

	Decoded d1;
	ASSERT_TRUE(decode(download(file, 239), d1));
	EXPECT_EQ(file.size(), size0);
	ASSERT_EQ(d1.entries.size(), d0.entries.size());

	for (size_t i = 0; i < d0.entries.size(); i++) {
		EXPECT_EQ(d1.entries[i].name, d0.entries[i].name);
		EXPECT_EQ(d1.entries[i].has_default, d0.entries[i].has_default);
	}

	EXPECT_EQ(find(d1, "MPC_XY_P")->value, bits(42.f));
	EXPECT_FALSE(find(d1, "MPC_XY_P")->has_default);
	EXPECT_EQ(find(d1, "SYS_AUTOSTART")->value, g_params[4].default_value);
	EXPECT_TRUE(find(d1, "SYS_AUTOSTART")->has_default);
	EXPECT_NE(find(d1, "MPC_Z_P"), nullptr);
	EXPECT_EQ(find(d1, "NEW_PARAM"), nullptr);
}

TEST_F(MavlinkFtpParam, ValueNeverStraddlesBlock)
{
	for (const char *path : {"@PARAM/param.pck", "@PARAM/param.pck?withdefaults=1"}) {
		for (const uint16_t block : {16, 239}) {
			ParamPckFile file;
			ASSERT_TRUE(file.open(path, block));

			Decoded d;
			ASSERT_TRUE(decode(download(file, block), d));

			for (const Entry &e : d.entries) {
				const size_t data_len = 4 + (e.has_default ? 4 : 0);
				EXPECT_EQ(e.value_ofs / block, (e.value_ofs + data_len - 1) / block)
						<< e.name << " block " << block << " " << path;
			}
		}
	}
}

namespace
{

struct UploadEntry {
	std::string name;
	uint8_t ptype;
	std::vector<uint8_t> value;
};

UploadEntry up_i8(const char *name, int8_t v)
{
	return {name, 1, {static_cast<uint8_t>(v)}};
}

UploadEntry up_i32(const char *name, int32_t v)
{
	uint8_t b[4];
	memcpy(b, &v, 4);
	return {name, 3, {b, b + 4}};
}

UploadEntry up_f32(const char *name, float v)
{
	uint8_t b[4];
	memcpy(b, &v, 4);
	return {name, 4, {b, b + 4}};
}

std::vector<uint8_t> encode_upload(const std::vector<UploadEntry> &entries, uint16_t magic = 0x671B)
{
	std::vector<uint8_t> body;
	std::string prev;

	for (const auto &e : entries) {
		size_t common = 0;

		while ((common < e.name.size()) && (common < prev.size()) && (e.name[common] == prev[common])) {
			common++;
		}

		size_t name_len = e.name.size() - common;

		if (name_len == 0) {
			name_len = 1;
			common--;
		}

		body.push_back(e.ptype);
		body.push_back(static_cast<uint8_t>(common | ((name_len - 1) << 4)));
		body.insert(body.end(), e.name.begin() + common, e.name.end());
		body.insert(body.end(), e.value.begin(), e.value.end());
		prev = e.name;
	}

	std::vector<uint8_t> out(ParamPckFile::kHeaderLen + body.size());
	const uint16_t num = static_cast<uint16_t>(entries.size());
	const uint16_t flen = static_cast<uint16_t>(out.size());
	memcpy(out.data(), &magic, 2);
	memcpy(out.data() + 2, &num, 2);
	memcpy(out.data() + 4, &flen, 2);
	memcpy(out.data() + ParamPckFile::kHeaderLen, body.data(), body.size());
	return out;
}

bool upload_bytes(ParamPckFile &file, const std::vector<uint8_t> &f, uint16_t chunk)
{
	if (!file.open_write()) {
		return false;
	}

	for (size_t ofs = 0; ofs < f.size();) {
		const uint16_t n = static_cast<uint16_t>(std::min<size_t>(chunk, f.size() - ofs));

		if (file.write(static_cast<uint32_t>(ofs), f.data() + ofs, n) != n) {
			return false;
		}

		ofs += n;
	}

	return file.finish_write();
}

uint32_t param_bits(const char *name)
{
	const param_t p = param_find_no_notification(name);
	return g_params[p].value;
}

} // namespace

TEST_F(MavlinkFtpParam, UploadAppliesAndSkips)
{
	const std::vector<uint8_t> f = encode_upload({
		up_f32("MPC_XY_P", 1.5f),
		up_i8("SYS_HITL", 1),
		up_i32("SYS_AUTOSTART", 4002),
		up_i32("CAL_GYRO0_ID", 1234),
		up_f32("NO_SUCH_PARAM", 9.f),
	});

	ParamPckFile file;
	ASSERT_TRUE(upload_bytes(file, f, 8));
	file.close();

	EXPECT_EQ(param_bits("MPC_XY_P"), bits(1.5f));
	EXPECT_EQ(param_bits("SYS_HITL"), static_cast<uint32_t>(1));
	EXPECT_EQ(param_bits("SYS_AUTOSTART"), static_cast<uint32_t>(4002));
	EXPECT_EQ(param_bits("CAL_GYRO0_ID"), static_cast<uint32_t>(99));
}

TEST_F(MavlinkFtpParam, UploadPrefixAndRetry)
{
	const std::vector<uint8_t> f = encode_upload({
		up_f32("ABCDEFGHIJKLMNOP", 8.f),
		up_f32("ABCDEFGHIJKLMNOQ", 9.f),
	});

	ParamPckFile file;
	ASSERT_TRUE(file.open_write());
	ASSERT_EQ(file.write(0, f.data(), 6), 6);
	ASSERT_EQ(file.write(0, f.data(), 6), 6); // retry of the header
	ASSERT_EQ(file.write(6, f.data() + 6, static_cast<uint16_t>(f.size() - 6)),
		  static_cast<int>(f.size() - 6));
	ASSERT_TRUE(file.finish_write());
	file.close();

	EXPECT_EQ(param_bits("ABCDEFGHIJKLMNOP"), bits(8.f));
	EXPECT_EQ(param_bits("ABCDEFGHIJKLMNOQ"), bits(9.f));
}

TEST_F(MavlinkFtpParam, UploadRejectsDefaultsMagicAndHoles)
{
	ParamPckFile file;
	const std::vector<uint8_t> with_defaults = encode_upload({up_f32("MPC_XY_P", 1.f)}, 0x671C);
	EXPECT_FALSE(upload_bytes(file, with_defaults, 80));
	file.close();
	EXPECT_EQ(param_bits("MPC_XY_P"), bits(0.95f));

	const std::vector<uint8_t> ok = encode_upload({up_f32("MPC_Z_P", 4.f)});
	ASSERT_TRUE(file.open_write());
	EXPECT_EQ(file.write(3, ok.data(), static_cast<uint16_t>(ok.size())), -1);
	EXPECT_FALSE(file.finish_write());
	file.close();

	std::vector<uint8_t> truncated = encode_upload({up_i32("SYS_HITL", 2), up_i32("SYS_AUTOSTART", 1)});
	truncated.resize(truncated.size() - 3);
	EXPECT_FALSE(upload_bytes(file, truncated, 80));
}

TEST_F(MavlinkFtpParam, UploadRoundtripDownload)
{
	ParamPckFile src;
	ASSERT_TRUE(src.open("@PARAM/param.pck", 239));
	const std::vector<uint8_t> downloaded = download(src, 239);
	src.close();

	// download header total_params is the used count; rewrite as file length
	std::vector<uint8_t> f = downloaded;
	const uint16_t flen = static_cast<uint16_t>(f.size());
	memcpy(f.data() + 4, &flen, 2);

	g_params[0].value = bits(0.1f);
	g_params[4].value = 0;

	ParamPckFile dst;
	ASSERT_TRUE(upload_bytes(dst, f, 80));
	dst.close();

	EXPECT_EQ(param_bits("MPC_XY_P"), bits(0.95f));
	EXPECT_EQ(param_bits("SYS_AUTOSTART"), static_cast<uint32_t>(4001));
}
