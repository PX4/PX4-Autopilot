#include <unit_test.h>

#include <systemlib/err.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>

#include "../ghst_parser.hpp"

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#define TEST_DATA_PATH "./test_data/"
#else
#define TEST_DATA_PATH "/fs/microsd"
#endif

class RcGhstTest : public UnitTest
{
public:
	bool run_tests() override;

private:
	bool ghstTest();
};

bool RcGhstTest::run_tests()
{
	ut_run_test(ghstTest);

	return (_tests_failed == 0);
}

bool RcGhstTest::ghstTest()
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
	int8_t ghst_rssi = -1;
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

			bool result = ghst_parse(now, frame, frame_len, rc_values, &ghst_rssi, &num_values, max_channels);

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

ut_declare_test_c(rc_ghst_test_main, RcGhstTest)
