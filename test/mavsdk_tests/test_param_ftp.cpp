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

#include <mavsdk/plugins/ftp/ftp.h>
#include <mavsdk/plugins/param/param.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <vector>

#include "autopilot_tester.h"

namespace
{

// ArduPilot packed parameter format, as served by PX4 at @PARAM/param.pck
struct PackedParam {
	uint8_t type; // 3 = int32, 4 = float
	uint32_t value_bits;
	bool has_default;
};

bool decode_param_pck(const std::vector<uint8_t> &f, uint16_t &num, uint16_t &total,
		      std::map<std::string, PackedParam> &out)
{
	if (f.size() < 6) {
		return false;
	}

	uint16_t magic;
	memcpy(&magic, f.data(), 2);
	memcpy(&num, f.data() + 2, 2);
	memcpy(&total, f.data() + 4, 2);

	if ((magic != 0x671B) && (magic != 0x671C)) {
		return false;
	}

	std::string prev;
	size_t i = 6;

	while (i < f.size()) {
		if (f[i] == 0) {
			i++; // pad
			continue;
		}

		if ((i + 2) > f.size()) {
			return false;
		}

		PackedParam p{};
		p.type = f[i] & 0x0F;
		p.has_default = (f[i] >> 4) & 0x01;
		const uint8_t common_len = f[i + 1] & 0x0F;
		const uint8_t name_len = (f[i + 1] >> 4) + 1;
		i += 2;

		if ((common_len > prev.size()) || ((common_len + name_len) > 16) || ((i + name_len + 4) > f.size())) {
			return false;
		}

		const std::string name = prev.substr(0, common_len) + std::string(reinterpret_cast<const char *>(f.data() + i),
					 name_len);
		i += name_len;
		memcpy(&p.value_bits, f.data() + i, 4);
		i += 4;

		if (p.has_default) {
			if ((i + 4) > f.size()) {
				return false;
			}

			i += 4;
		}

		out[name] = p;
		prev = name;
	}

	return out.size() == num;
}

template<typename T>
uint32_t bits_of(T v)
{
	uint32_t u;
	memcpy(&u, &v, sizeof(u));
	return u;
}

class AutopilotTesterParamFtp : public AutopilotTester
{
public:
	mavsdk::Param::AllParams all_params() { return getParams()->get_all_params(); }
	std::shared_ptr<mavsdk::System> system() { return get_system(); }
};

} // namespace

TEST_CASE("Parameters download via MAVLink FTP", "[multicopter]")
{
	using namespace std::chrono_literals;

	AutopilotTesterParamFtp tester;
	tester.connect(connection_url);

	// the reference: the conventional PARAM_VALUE stream
	const mavsdk::Param::AllParams all = tester.all_params();
	REQUIRE(all.int_params.size() + all.float_params.size() > 100);

	const std::filesystem::path local_dir = std::filesystem::temp_directory_path() / "px4_mavsdk_tests_param_ftp";
	std::filesystem::remove_all(local_dir);
	std::filesystem::create_directories(local_dir);

	mavsdk::Ftp ftp(tester.system());
	std::promise<mavsdk::Ftp::Result> done;
	auto result = done.get_future();
	bool finished = false;

	ftp.download_async("@PARAM/param.pck?withdefaults=1", local_dir.string(), true,
	[&](mavsdk::Ftp::Result r, mavsdk::Ftp::ProgressData) {
		if ((r != mavsdk::Ftp::Result::Next) && !finished) {
			finished = true;
			done.set_value(r);
		}
	});

	REQUIRE(result.wait_for(60s) == std::future_status::ready);
	REQUIRE(result.get() == mavsdk::Ftp::Result::Success);

	std::ifstream file(local_dir / "param.pck?withdefaults=1", std::ios::binary);
	REQUIRE(file.good());
	const std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

	uint16_t num = 0;
	uint16_t total = 0;
	std::map<std::string, PackedParam> packed;
	REQUIRE(decode_param_pck(data, num, total, packed));
	CHECK(num == total);

	size_t compared = 0;

	for (const auto &p : all.int_params) {
		if (p.name == "_HASH_CHECK") {
			continue;
		}

		CAPTURE(p.name);
		REQUIRE(packed.count(p.name) == 1);
		CHECK(packed[p.name].type == 3);
		CHECK(packed[p.name].value_bits == bits_of(p.value));
		compared++;
	}

	for (const auto &p : all.float_params) {
		CAPTURE(p.name);
		REQUIRE(packed.count(p.name) == 1);
		CHECK(packed[p.name].type == 4);
		CHECK(packed[p.name].value_bits == bits_of(p.value));
		compared++;
	}

	CHECK(compared == packed.size());
	std::cout << "Compared " << compared << " parameters, " << data.size() << " bytes packed" << std::endl;
}

std::vector<uint8_t> encode_one_float(const std::string &name, float value)
{
	std::vector<uint8_t> body;
	body.push_back(4);
	body.push_back(static_cast<uint8_t>((name.size() - 1) << 4));
	body.insert(body.end(), name.begin(), name.end());
	uint8_t bits[4];
	memcpy(bits, &value, 4);
	body.insert(body.end(), bits, bits + 4);

	std::vector<uint8_t> out(6 + body.size());
	const uint16_t magic = 0x671B;
	const uint16_t num = 1;
	const uint16_t flen = static_cast<uint16_t>(out.size());
	memcpy(out.data(), &magic, 2);
	memcpy(out.data() + 2, &num, 2);
	memcpy(out.data() + 4, &flen, 2);
	memcpy(out.data() + 6, body.data(), body.size());
	return out;
}

TEST_CASE("Parameters upload via MAVLink FTP", "[multicopter]")
{
	using namespace std::chrono_literals;

	AutopilotTesterParamFtp tester;
	tester.connect(connection_url);

	mavsdk::Param param(tester.system());
	const mavsdk::Param::AllParams all = param.get_all_params();
	REQUIRE_FALSE(all.float_params.empty());

	const std::string name = all.float_params.front().name;
	const float original = all.float_params.front().value;
	const float updated = original + 1.f;
	CAPTURE(name);

	const std::filesystem::path local_dir = std::filesystem::temp_directory_path() / "px4_mavsdk_tests_param_ftp_up";
	std::filesystem::remove_all(local_dir);
	std::filesystem::create_directories(local_dir);
	const std::filesystem::path local_file = local_dir / "param.pck";

	{
		const std::vector<uint8_t> packed = encode_one_float(name, updated);
		std::ofstream out(local_file, std::ios::binary);
		REQUIRE(out.good());
		out.write(reinterpret_cast<const char *>(packed.data()), static_cast<std::streamsize>(packed.size()));
	}

	mavsdk::Ftp ftp(tester.system());
	std::promise<mavsdk::Ftp::Result> done;
	auto result = done.get_future();
	bool finished = false;

	ftp.upload_async(local_file.string(), "@PARAM",
	[&](mavsdk::Ftp::Result r, mavsdk::Ftp::ProgressData) {
		if ((r != mavsdk::Ftp::Result::Next) && !finished) {
			finished = true;
			done.set_value(r);
		}
	});

	REQUIRE(result.wait_for(60s) == std::future_status::ready);
	REQUIRE(result.get() == mavsdk::Ftp::Result::Success);

	const auto got = param.get_param_float(name);
	REQUIRE(got.first == mavsdk::Param::Result::Success);
	CHECK(got.second == Approx(updated));

	REQUIRE(param.set_param_float(name, original) == mavsdk::Param::Result::Success);
}
