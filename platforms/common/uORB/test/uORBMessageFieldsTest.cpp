/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include <uORB/uORBMessageFields.hpp>

#include <gtest/gtest.h>
#include <containers/Bitset.hpp>
#include <px4_platform_common/param.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string>

// To run: make tests TESTFILTER=uORBMessageFields

class uORBMessageFieldsTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
	}
};

TEST_F(uORBMessageFieldsTest, decompressed_formats_match)
{
	char buffer[1600];
	static_assert(uORB::orb_untokenized_fields_max_length < sizeof(buffer) - HEATSHRINK_DECODER_INPUT_BUFFER_SIZE(_),
		      "msg definition too long / buffer too short");

	uORB::MessageFormatReader format_reader(buffer, sizeof(buffer));

	px4::Bitset<ORB_TOPICS_COUNT> formats_found;
	px4::Bitset<ORB_TOPICS_COUNT> dependencies;

	static const char *all_formats[] = ORB_DECOMPRESSED_MESSAGE_FIELDS;
	int format_idx = 0;

	bool done = false;

	while (!done) {
		switch (format_reader.readMore()) {
		case uORB::MessageFormatReader::State::FormatComplete: {
				const unsigned format_length = format_reader.formatLength();
				EXPECT_GT(format_length, 0);

				// Move the left-over (the part after the format if any) to the end of the buffer
				const unsigned leftover_length = format_reader.moveLeftoverToBufferEnd();

				for (const orb_id_size_t orb_id : format_reader.orbIDs()) {
					// Ensure each orb_id is set only once
					EXPECT_FALSE(formats_found[orb_id]);
					formats_found.set(orb_id);
					dependencies.set(orb_id, false); // Clear dependency
				}

				for (const orb_id_size_t orb_id : format_reader.orbIDsDependencies()) {
					dependencies.set(orb_id);
				}

				// Compare format
				ASSERT_LT(format_idx, sizeof(all_formats) / sizeof(all_formats[0]));
				const std::string format(buffer, format_length);
				const std::string format_expected(all_formats[format_idx]);
				EXPECT_EQ(format, format_expected);

				const int ret = uORB::MessageFormatReader::expandMessageFormat(buffer, format_length,
						sizeof(buffer) - leftover_length);
				EXPECT_GE(ret, 0);
				++format_idx;

				// Move left-over back
				format_reader.clearFormatAndRestoreLeftover();
				break;
			}
			break;

		case uORB::MessageFormatReader::State::Failure:
			PX4_ERR("Failed to read formats");
			done = true;
			ASSERT_FALSE(true);
			break;

		case uORB::MessageFormatReader::State::Complete:
			done = true;
			break;

		default:
			break;
		}
	}

	// Check that all formats are found
	for (size_t i = 0; i < formats_found.size(); ++i) {
		EXPECT_TRUE(formats_found[i]);
	}

	// Expect dependencies to be cleared. If this is not the case, the format ordering is incorrect.
	EXPECT_EQ(dependencies.count(), 0);
}

TEST_F(uORBMessageFieldsTest, decompress_formats_iterative)
{
	char buffer[64];
	uORB::MessageFormatReader format_reader(buffer, sizeof(buffer));

	px4::Bitset<ORB_TOPICS_COUNT> formats_found;

	static const char *all_formats[] = ORB_DECOMPRESSED_MESSAGE_FIELDS;
	int format_idx = 0;
	std::string current_format;

	bool done = false;

	while (!done) {
		switch (format_reader.readMore()) {
		case uORB::MessageFormatReader::State::ReadingFormat:
			current_format += std::string(buffer, format_reader.formatLength());
			format_reader.clearFormatFromBuffer();
			break;

		case uORB::MessageFormatReader::State::FormatComplete: {
				current_format += std::string(buffer, format_reader.formatLength());
				format_reader.clearFormatFromBuffer();

				EXPECT_FALSE(current_format.empty());

				for (const orb_id_size_t orb_id : format_reader.orbIDs()) {
					// Ensure each orb_id is set only once
					EXPECT_FALSE(formats_found[orb_id]);
					formats_found.set(orb_id);
				}

				// Compare format
				ASSERT_LT(format_idx, sizeof(all_formats) / sizeof(all_formats[0]));
				const std::string format_expected(all_formats[format_idx]);
				EXPECT_EQ(current_format, format_expected);

				++format_idx;
				current_format.clear();
				break;
			}
			break;

		case uORB::MessageFormatReader::State::Failure:
			PX4_ERR("Failed to read formats");
			done = true;
			ASSERT_FALSE(true);
			break;

		case uORB::MessageFormatReader::State::Complete:
			done = true;
			break;

		default:
			break;
		}
	}

	// Check that all formats are found
	for (size_t i = 0; i < formats_found.size(); ++i) {
		EXPECT_TRUE(formats_found[i]);
	}
}

TEST_F(uORBMessageFieldsTest, decompress_formats_buffer_too_short)
{
	char buffer[64];
	static_assert(uORB::orb_tokenized_fields_max_length > sizeof(buffer), "Test expects smaller buffer");
	uORB::MessageFormatReader format_reader(buffer, sizeof(buffer));

	bool done = false;

	while (!done) {
		switch (format_reader.readMore()) {
		case uORB::MessageFormatReader::State::Failure:
		case uORB::MessageFormatReader::State::Complete:
			done = true;
			break;

		default:
			break;
		}
	}

	EXPECT_EQ(format_reader.readMore(), uORB::MessageFormatReader::State::Failure);
}

TEST_F(uORBMessageFieldsTest, decompress_specific_format)
{
	char format[512];
	char buffer[128];
	uORB::MessageFormatReader format_reader(buffer, sizeof(buffer));

	ASSERT_TRUE(format_reader.readUntilFormat((orb_id_size_t)ORB_ID::orb_test));

	int field_length = 0;
	int format_length = 0;

	while (format_reader.readNextField(field_length)) {
		format_length += snprintf(format + format_length, sizeof(buffer) - format_length - 1, "%s;", buffer);
	}

	ASSERT_GT(uORB::MessageFormatReader::expandMessageFormat(format, format_length, sizeof(format)), 0);

	const std::string expected_format = "uint64_t timestamp;int32_t val;uint8_t[4] _padding0;";
	ASSERT_EQ(expected_format, format);
}
