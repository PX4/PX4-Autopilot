#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "../../src/systemcmds/tests/tests.h"
#include <drivers/drv_hrt.h>
#include <px4iofirmware/px4io.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>

#include "gtest/gtest.h"

TEST(SBUS2Test, SBUS2)
{
	const char *filepath = "testdata/sbus2_r7008SB.txt";

	FILE *fp;
	fp = fopen(filepath, "rt");

	ASSERT_TRUE(fp);
	warnx("loading data from: %s", filepath);

	// if (argc < 2)
	// 	errx(1, "Need a filename for the input file");

	int byte_offset = 7;

	// if (argc > 2) {
	// 	char* end;
	// 	byte_offset = strtol(argv[2],&end,10);
	// }

	warnx("RUNNING TEST WITH BYTE OFFSET OF: %d", byte_offset);

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		(void)fscanf(fp, "%f,%x,,", &f, &x);
	}

	// Init the parser
	uint8_t frame[SBUS_BUFFER_SIZE];
	unsigned partial_frame_count = 0;
	uint16_t rc_values[18];
	uint16_t num_values;
	uint16_t sbus_frame_drops = 0;
	unsigned sbus_frame_resets = 0;
	bool sbus_failsafe;
	bool sbus_frame_drop;
	uint16_t max_channels = sizeof(rc_values) / sizeof(rc_values[0]);

	float last_time = 0;

	int rate_limiter = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		unsigned last_drop = sbus_frame_drops + sbus_frame_resets;

		unsigned interval_us = ((f - last_time) * 1000 * 1000);

		if (interval_us > SBUS_INTER_FRAME_TIMEOUT) {
			if (partial_frame_count != 0) {
				warnx("[ %08.4fs ] INTERVAL: %u - FRAME RESET, DROPPED %d bytes",
					interval_us, f, partial_frame_count);

				printf("\t\tdropped: ");
				for (int i = 0; i < partial_frame_count; i++) {
					printf("%02X ", frame[i]);
				}
				printf("\n");

				sbus_frame_resets++;
			}

			partial_frame_count = 0;
		}

		last_time = f;

		frame[partial_frame_count] = x;
		partial_frame_count++;

		//warnx("%f: 0x%02x, first: 0x%02x, last: 0x%02x, pcount: %u", (double)f, x, frame[0], frame[24], partial_frame_count);

		if (partial_frame_count == sizeof(frame)) {
			partial_frame_count = 0;
			warnx("FRAME SIZE OVERFLOW!\n\n\n");
		}

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		if (rate_limiter % byte_offset == 0) {
			bool result = sbus_parse(now, frame, &partial_frame_count, rc_values, &num_values,
				&sbus_failsafe, &sbus_frame_drop, max_channels);
			sbus_frame_drops = sbus_dropped_frames();

			if (result)
				warnx("decoded packet");
		}

		if (last_drop != (sbus_frame_drops + sbus_frame_resets))
			warnx("frame dropped, now #%d", (sbus_frame_drops + sbus_frame_resets));

		rate_limiter++;
	}

	ASSERT_EQ(ret, EOF);
}
