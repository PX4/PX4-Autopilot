#include <unit_test.h>

#include <systemlib/err.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>

#define DSM_DEBUG
#include <lib/rc/sbus.h>
#include <lib/rc/dsm.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>
#include <lib/rc/crsf.h>
#include <lib/rc/ghst.hpp>
#include <lib/rc/common_rc.h>

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#define TEST_DATA_PATH "./test_data/"
#else
#define TEST_DATA_PATH CONFIG_BOARD_ROOT_PATH
#endif

extern "C" __EXPORT int rc_tests_main(int argc, char *argv[]);

class RCTest : public UnitTest
{
public:
	bool run_tests() override;

private:
	bool crsfTest();
	bool ghstTest();
	bool dsmTest(const char *filepath, unsigned expected_chancount, unsigned expected_dropcount, unsigned chan0);
	bool dsmTest10Ch();
	bool dsmTest16Ch();
	bool dsmTest22msDSMX16Ch();
	bool dsmTestOrangeDsmx();
	bool sbus2Test();
	bool st24MalformedLengthTest();
	bool st24Test();
	bool sumdTest();
	bool sumdMalformedLengthTest();
};

bool RCTest::run_tests()
{
	ut_run_test(crsfTest);
	ut_run_test(ghstTest);
	ut_run_test(dsmTest10Ch);
	ut_run_test(dsmTest16Ch);
	ut_run_test(dsmTest22msDSMX16Ch);
	ut_run_test(dsmTestOrangeDsmx);
	ut_run_test(sbus2Test);
	ut_run_test(st24MalformedLengthTest);
	ut_run_test(st24Test);
	ut_run_test(sumdTest);
	ut_run_test(sumdMalformedLengthTest);

	return (_tests_failed == 0);
}

bool RCTest::crsfTest()
{
	const char *filepath = TEST_DATA_PATH "crsf_rc_channels.txt";

	FILE *fp = fopen(filepath, "rt");

	ut_test(fp);
	//PX4_INFO("loading data from: %s", filepath);

	const int line_size = 500;
	char line[line_size];
	bool has_decoded_values = false;
	const int max_channels = 16;
	uint16_t rc_values[max_channels];
	uint16_t num_values = 0;
	int line_counter = 1;

	while (fgets(line, line_size, fp) != nullptr)  {

		if (strncmp(line, "INPUT ", 6) == 0) {

			if (has_decoded_values) {
				PX4_ERR("Parser decoded values that are not in the test file (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 6;
			int frame_len = 0;
			uint8_t frame[300];
			int offset;
			int number;

			while (sscanf(file_buffer, "%x, %n", &number, &offset) > 0) {
				frame[frame_len++] = number;
				file_buffer += offset;
			}

			// Pipe the data into the parser
			hrt_abstime now = hrt_absolute_time();

			bool result = crsf_parse(now, frame, frame_len, rc_values, &num_values, max_channels);

			if (result) {
				has_decoded_values = true;
			}

		} else if (strncmp(line, "DECODED ", 8) == 0) {

			if (!has_decoded_values) {
				PX4_ERR("Test file contains decoded values but the parser did not decode anything (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 8;
			int offset;
			int expected_rc_value;
			int expected_num_channels = 0;

			while (sscanf(file_buffer, "%x, %n", &expected_rc_value, &offset) > 0) {

				// allow a small difference
				if (abs(expected_rc_value - (int)rc_values[expected_num_channels]) > 10) {
					PX4_ERR("File line: %i, channel: %i", line_counter, expected_num_channels);
					ut_compare("Wrong decoded channel", expected_rc_value, rc_values[expected_num_channels]);
				}

				file_buffer += offset;
				++expected_num_channels;
			}

			if (expected_num_channels != num_values) {
				PX4_ERR("File line: %d", line_counter);
				ut_compare("Unexpected number of decoded channels", expected_num_channels, num_values);
			}

			has_decoded_values = false;
		}

		++line_counter;
	}

	return true;
}

bool RCTest::ghstTest()
{
	const char *filepath = TEST_DATA_PATH "ghst_rc_channels.txt";

	FILE *fp = fopen(filepath, "rt");

	ut_test(fp);

	int uart_fd = -1;
	const int line_size = 500;
	char line[line_size];
	bool has_decoded_values = false;
	const int max_channels = 16;
	uint16_t rc_values[max_channels];
	uint16_t num_values = 0;
	int line_counter = 1;
	ghstLinkStatistics_t link_stats;
	ghst_config(uart_fd);

	while (fgets(line, line_size, fp) != nullptr)  {

		if (strncmp(line, "INPUT ", 6) == 0) {

			if (has_decoded_values) {
				PX4_ERR("Parser decoded values that are not in the test file (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 6;
			int frame_len = 0;
			uint8_t frame[300];
			int offset;
			int number;

			while (sscanf(file_buffer, "%x, %n", &number, &offset) > 0) {
				frame[frame_len++] = number;
				file_buffer += offset;
			}

			// Pipe the data into the parser
			hrt_abstime now = hrt_absolute_time();

			bool result = ghst_parse(now, frame, frame_len, rc_values, &link_stats, &num_values, max_channels);

			if (result) {
				has_decoded_values = true;
			}

		} else if (strncmp(line, "DECODED ", 8) == 0) {

			if (!has_decoded_values) {
				PX4_ERR("Test file contains decoded values but the parser did not decode anything (line=%i)", line_counter);
				return false;
			}

			// read the values
			const char *file_buffer = line + 8;
			int offset;
			int expected_rc_value;
			int expected_num_channels = 0;

			while (sscanf(file_buffer, "%x, %n", &expected_rc_value, &offset) > 0) {

				// allow a small difference
				if (abs(expected_rc_value - (int)rc_values[expected_num_channels]) > 10) {
					PX4_ERR("File line: %i, channel: %i", line_counter, expected_num_channels);
					ut_compare("Wrong decoded channel", expected_rc_value, rc_values[expected_num_channels]);
				}

				file_buffer += offset;
				++expected_num_channels;
			}

			if (expected_num_channels != num_values) {
				PX4_ERR("File line: %d", line_counter);
				ut_compare("Unexpected number of decoded channels", expected_num_channels, num_values);
			}

			has_decoded_values = false;
		}

		++line_counter;
	}

	return true;
}

bool RCTest::dsmTest10Ch()
{
	return dsmTest(TEST_DATA_PATH "dsm_x_data.txt", 10, 2, 1500);
}

bool RCTest::dsmTest16Ch()
{
	return dsmTest(TEST_DATA_PATH "dsm_x_dx9_data.txt", 16, 1, 1500);
}

bool RCTest::dsmTest22msDSMX16Ch()
{
	return dsmTest(TEST_DATA_PATH "dsm_x_dx9_px4_binding_data.txt", 16, 1, 1499);
}

bool RCTest::dsmTestOrangeDsmx()
{
	return dsmTest(TEST_DATA_PATH "orangerx_dsmx_12.txt", 12, 1, 1499);
}

bool RCTest::dsmTest(const char *filepath, unsigned expected_chancount, unsigned expected_dropcount, unsigned chan0)
{
	FILE *fp;
	fp = fopen(filepath, "rt");

	ut_test(fp);
	//PX4_INFO("loading data from: %s", filepath);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	// Init the parser
	uint8_t frame[30];
	uint16_t rc_values[18];
	uint16_t num_values;
	bool dsm_11_bit;
	unsigned dsm_frame_drops = 0;
	uint16_t max_channels = sizeof(rc_values) / sizeof(rc_values[0]);

	int count = 0;
	unsigned last_drop = 0;

	dsm_proto_init();

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		frame[0] = x;
		unsigned len = 1;

		// Pipe the data into the parser
		bool result = dsm_parse(f * 1e6f, &frame[0], len, rc_values, &num_values,
					&dsm_11_bit, &dsm_frame_drops, nullptr, max_channels);

		if (result) {
			if (count > (16 * 20)) { // need to process enough data to have full channel count
				ut_compare("num_values == expected_chancount", num_values, expected_chancount);
			}

			ut_test(abs((int)chan0 - (int)rc_values[0]) < 30);

			//PX4_INFO("decoded packet with %d channels and %s encoding:", num_values, (dsm_11_bit) ? "11 bit" : "10 bit");

			for (unsigned i = 0; i < num_values; i++) {
				//PX4_INFO("chan #%u:\t%d", i, (int)rc_values[i]);
			}
		}

		if (last_drop != (dsm_frame_drops)) {
			//PX4_INFO("frame dropped, now #%d", (dsm_frame_drops));
			last_drop = dsm_frame_drops;
		}

		count++;
	}

	fclose(fp);

	ut_compare("num_values == expected_chancount", num_values, expected_chancount);

	ut_test(ret == EOF);
	//PX4_INFO("drop: %d", (int)last_drop);
	ut_compare("last_drop == expected_dropcount", last_drop, expected_dropcount);

	return true;
}

bool RCTest::sbus2Test()
{
	const char *filepath = TEST_DATA_PATH "sbus2_r7008SB.txt";

	FILE *fp;
	fp = fopen(filepath, "rt");

	ut_test(fp);
	//PX4_INFO("loading data from: %s", filepath);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	// Init the parser
	uint8_t frame[SBUS_BUFFER_SIZE];
	uint16_t rc_values[18];
	uint16_t num_values;
	unsigned sbus_frame_drops = 0;
	unsigned sbus_frame_resets = 0;
	bool sbus_failsafe;
	bool sbus_frame_drop;
	uint16_t max_channels = sizeof(rc_values) / sizeof(rc_values[0]);

	unsigned last_drop = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		frame[0] = x;
		unsigned len = 1;

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		// if (rate_limiter % byte_offset == 0) {
		bool result = sbus_parse(now, &frame[0], len, rc_values, &num_values,
					 &sbus_failsafe, &sbus_frame_drop, &sbus_frame_drops, max_channels);

		if (result) {
			//PX4_INFO("decoded packet");
		}

		// }

		if (last_drop != (sbus_frame_drops + sbus_frame_resets)) {
			PX4_WARN("frame dropped, now #%d", (sbus_frame_drops + sbus_frame_resets));
			last_drop = sbus_frame_drops + sbus_frame_resets;
		}

	}

	ut_test(ret == EOF);

	return true;
}

bool RCTest::st24MalformedLengthTest()
{
	uint8_t rssi{};
	uint8_t lost_count{};
	uint16_t channel_count{};
	uint16_t channels[24] {};
	const uint8_t invalid_lengths[] {0, 1};

	for (const uint8_t invalid_length : invalid_lengths) {
		const uint8_t prefix[] {ST24_STX1, ST24_STX2, invalid_length, 0};

		for (const uint8_t byte : prefix) {
			(void)st24_decode(byte, &rssi, &lost_count, &channel_count, channels, 24);
		}

		for (unsigned i = 0; i <= ST24_DATA_LEN_MAX; ++i) {
			(void)st24_decode(0xaa, &rssi, &lost_count, &channel_count, channels, 24);
		}
	}

	uint8_t minimal_packet[] {ST24_STX1, ST24_STX2, 2, 0xff, 0};
	minimal_packet[4] = st24_common_crc8(&minimal_packet[2], 2);
	int result = 1;

	for (const uint8_t byte : minimal_packet) {
		result = st24_decode(byte, &rssi, &lost_count, &channel_count, channels, 24);
	}

	ut_test(result == 2);
	return true;
}

bool RCTest::st24Test()
{
	const char *filepath = TEST_DATA_PATH "st24_data.txt";

	//PX4_INFO("loading data from: %s", filepath);

	FILE *fp;

	fp = fopen(filepath, "rt");
	ut_test(fp);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	float last_time = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		if (((f - last_time) * 1000 * 1000) > 3000) {
			// PX4_INFO("FRAME RESET\n\n");
		}

		uint8_t b = static_cast<uint8_t>(x);

		last_time = f;

		// Pipe the data into the parser
		//hrt_abstime now = hrt_absolute_time();

		uint8_t rssi;
		uint8_t rx_count;
		uint16_t channel_count;
		uint16_t channels[20];

		if (!st24_decode(b, &rssi, &rx_count, &channel_count, channels, sizeof(channels) / sizeof(channels[0]))) {
			//PX4_INFO("decoded: %u channels (converted to PPM range)", (unsigned)channel_count);

			for (unsigned i = 0; i < channel_count; i++) {
				//int16_t val = channels[i];
				//PX4_INFO("channel %u: %d 0x%03X", i, static_cast<int>(val), static_cast<int>(val));
			}
		}
	}

	ut_test(ret == EOF);

	return true;
}

bool RCTest::sumdTest()
{
	const char *filepath = TEST_DATA_PATH "sumd_data.txt";

	//PX4_INFO("loading data from: %s", filepath);

	FILE *fp;

	fp = fopen(filepath, "rt");
	ut_test(fp);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		char buf[200];
		(void)fgets(buf, sizeof(buf), fp);
	}

	float last_time = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		if (ret <= 0) {
			fclose(fp);
			ut_test(ret > 0);
		}

		if (((f - last_time) * 1000 * 1000) > 3000) {
			// PX4_INFO("FRAME RESET\n\n");
		}

		uint8_t b = static_cast<uint8_t>(x);

		last_time = f;

		// Pipe the data into the parser
		//hrt_abstime now = hrt_absolute_time();

		uint8_t rssi;
		uint8_t rx_count;
		uint16_t channel_count;
		uint16_t channels[32];
		bool sumd_failsafe;


		if (!sumd_decode(b, &rssi, &rx_count, &channel_count, channels, 32, &sumd_failsafe)) {
			//PX4_INFO("decoded: %u channels (converted to PPM range)", (unsigned)channel_count);

			for (unsigned i = 0; i < channel_count; i++) {
				//int16_t val = channels[i];
				//PX4_INFO("channel %u: %d 0x%03X", i, static_cast<int>(val), static_cast<int>(val));
			}
		}
	}

	ut_test(ret == EOF);

	return true;
}

bool RCTest::sumdMalformedLengthTest()
{
	// Regression test for the payload bound in sumd_decode().
	//
	// The decoder accepts length values from 2 up to SUMD_MAX_CHANNELS and writes
	// sumd_data[_rxlen] on entry to SUMD_DECODE_STATE_GOT_LEN, i.e. before the
	// length-derived bound is evaluated. With length == SUMD_MAX_CHANNELS the loop
	// reaches _rxlen == 64 while sumd_data has 64 elements (valid indices 0..63),
	// so the hidden write lands on the next member of the same packed struct.
	//
	// That byte is not observable through the decoder's outputs: GOT_DATA rewrites
	// the member it lands on, and the decoder never reads it back. It is observable
	// in the shared decode buffer, which is what this test uses: the byte written at
	// index 64 must not be one of our payload bytes.

	// Use a distinctive payload byte so we can tell an overflow from a legitimate
	// write to the following member.
	const uint8_t marker = 0x5A;

	uint8_t rssi = 0, rx_count = 0;
	uint16_t channel_count = 0;
	uint16_t channels[32] {};
	bool failsafe = false;

	const uint8_t len = SUMD_MAX_CHANNELS;

	// header, status, length
	sumd_decode(0xA8, &rssi, &rx_count, &channel_count, channels, 32, &failsafe);
	sumd_decode(0x01, &rssi, &rx_count, &channel_count, channels, 32, &failsafe);
	sumd_decode(len, &rssi, &rx_count, &channel_count, channels, 32, &failsafe);

	// len*2 payload bytes, all equal to the marker
	for (int i = 0; i < len * 2; ++i) {
		sumd_decode(marker, &rssi, &rx_count, &channel_count, channels, 32, &failsafe);
	}

	// At this point the decoder has written every payload byte. With the bound in
	// place the byte just past sumd_data[] keeps whatever the decoder last put there
	// (the length/status path), and in particular is not forced to the marker.
	//
	// That byte is struct offset 67, which is the named member crc16_high, so read it
	// by name. Indexing sumd_data[SUMD_MAX_CHANNELS * 2] would name sumd_data[64], one
	// past the 64-element array: undefined behaviour by the standard, even though the
	// byte is present in the enclosing struct.
	uint8_t after[1];
	memcpy(after, &rc_decode_buf._hottrxpacket.crc16_high, 1);


	ut_test(after[0] != marker);

	// The frame is not complete yet (CRC bytes pending), and the channel count must
	// never exceed what we allowed.
	ut_test(channel_count <= 32);

	return true;
}

ut_declare_test_c(rc_tests_main, RCTest)
