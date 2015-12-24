#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "../../src/systemcmds/tests/tests.h"
#include <drivers/drv_hrt.h>
// Enable S.BUS parser output
#define SBUS_DEBUG
#include <rc/sbus.h>
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

	int rate_limiter = 0;
	unsigned last_drop = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {

		ASSERT_GT(ret, 0);

		frame[0] = x;
		unsigned len = 1;

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		// if (rate_limiter % byte_offset == 0) {
		bool result = sbus_parse(now, &frame[0], len, rc_values, &num_values,
					 &sbus_failsafe, &sbus_frame_drop, &sbus_frame_drops, max_channels);

		if (result) {
			//warnx("decoded packet");
		}

		// }

		if (last_drop != (sbus_frame_drops + sbus_frame_resets)) {
			warnx("frame dropped, now #%d", (sbus_frame_drops + sbus_frame_resets));
			last_drop = sbus_frame_drops + sbus_frame_resets;
		}

		rate_limiter++;
	}

	ASSERT_EQ(ret, EOF);
}
