/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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


#include "septentrio.h"

#include <gtest/gtest.h>
#include <fuzztest/fuzztest.h>

#include "util.h"

TEST(SeptentrioTestDecoding, decodeMessage)
{
	// Check if a single message can be decoded
	septentrio::sbf::message_t m{};
	m.header.sync1 = '$';
	m.header.sync2 = '@';
	m.header.id_number = static_cast<uint16_t>(septentrio::sbf::BlockID::PVTGeodetic);
	m.header.length = sizeof(m.header) + sizeof(septentrio::sbf::PVTGeodetic);
	septentrio::sbf::PVTGeodetic pvt{};
	pvt.latitude = 123;
	pvt.longitude = 456;
	pvt.height = 789;
	memcpy(&m.payload, &pvt, sizeof(pvt));
	m.header.crc = septentrio::buffer_crc16((uint8_t *)&m + 4, m.header.length - 4);

	septentrio::sbf::Decoder decoder;
	const uint8_t *buffer = (const uint8_t *)&m;

	for (int i = 0; i < m.header.length; ++i) {
		decoder.add_byte(buffer[i]);
	}

	ASSERT_EQ(decoder.id(), septentrio::sbf::BlockID::PVTGeodetic);

	septentrio::sbf::Header header;
	ASSERT_EQ(decoder.parse(&header), 0);
}


class SeptentrioTestDecoder
{
public:
	SeptentrioTestDecoder()
	{
		// This object will be reused for each fuzzing iteration call
	}
	~SeptentrioTestDecoder() = default;

	void runTest(const std::vector<uint8_t> &data)
	{
		++_num_runs;

		for (int i = 0; i < data.size(); i++) {
			if (_decoder.add_byte(data[i]) == septentrio::sbf::Decoder::State::Done) {
				++_num_msgs;

				if (_num_runs % 100'000 == 0) {
					printf("Message: %i / %i / %i (%.6f)\n", _num_headers, _num_msgs, _num_runs, (double)_num_headers / _num_runs);
				}

				septentrio::sbf::Header message;

				if (_decoder.parse(&message) == 0) {
					++_num_headers;
				}

				_decoder.reset();
			}
		}

	}
private:
	septentrio::sbf::Decoder _decoder;
	int _num_runs{0};
	int _num_msgs{0};
	int _num_headers{0};
};

FUZZ_TEST_F(SeptentrioTestDecoder, runTest);

class SeptentrioTest
{
public:
	SeptentrioTest()
		: _driver("", septentrio::Instance::Main, 0)
	{
		// This object will be reused for each fuzzing iteration call
	}

	~SeptentrioTest() = default;

	void runTest(const std::vector<uint8_t> &data)
	{
		++_num_runs;

		for (int i = 0; i < data.size(); i++) {
			if (_driver.parse_char(data[i]) > 0) {
				++_num_msgs;
			}
		}

		if (_num_runs % 100'000 == 0) {
			// Print some stats (note that a high _num_msgs could also mean the fuzzer sends always the same message)
			printf("Parsed messages: %i / %i iterations (Rate: %.6f)\n", _num_msgs, _num_runs, (double)_num_msgs / _num_runs);
		}
	}

	std::vector<std::tuple<std::vector<uint8_t>>> seeds() { return _seeds; }

private:
	septentrio::SeptentrioDriver _driver;

	int _num_runs{0};
	int _num_msgs{0};

	// Provide a valid PVTGeodetic message as seed
	const std::vector<std::tuple<std::vector<uint8_t>>> _seeds = {{{0x24, 0x40, 0xe8, 0x88, 0xa7, 0x0f, 0x5e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}};
};

FUZZ_TEST_F(SeptentrioTest, runTest).WithSeeds(&SeptentrioTest::seeds);
