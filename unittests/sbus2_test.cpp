
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4iofirmware/px4io.h>
#include "../../src/systemcmds/tests/tests.h"

int main(int argc, char *argv[]) {
	warnx("SBUS2 test started");

	char *filepath = 0;

	if (argc < 2) {
		warnx("Using default input file");
		filepath = "testdata/sbus2_r7008SB.txt";
	} else {
		filepath = argv[1];
	}

	warnx("loading data from: %s", filepath);

	FILE *fp;
	
	fp = fopen(filepath,"rt");

	if (!fp)
		errx(1, "failed opening file");

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
	unsigned partial_frame_count = 0;
	uint16_t rc_values[18];
	uint16_t num_values;
	bool sbus_failsafe;
	bool sbus_frame_drop;
	uint16_t max_channels = sizeof(rc_values) / sizeof(rc_values[0]);

	float last_time = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {
		if (((f - last_time) * 1000 * 1000) > 3000) {
			partial_frame_count = 0;
			//warnx("FRAME RESET\n\n");
		}

		frame[partial_frame_count] = x;
		partial_frame_count++;

		//warnx("%f: 0x%02x, first: 0x%02x, last: 0x%02x, pcount: %u", (double)f, x, frame[0], frame[24], partial_frame_count);

		if (partial_frame_count == sizeof(frame))
			partial_frame_count = 0;

		last_time = f;

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		//if (partial_frame_count % 25 == 0)
			//sbus_parse(now, frame, &partial_frame_count, rc_values, &num_values, &sbus_failsafe, &sbus_frame_drop, max_channels);
	}

	if (ret == EOF) {
		warnx("Test finished, reached end of file");
		ret = 0;
	} else {
		warnx("Test aborted, errno: %d", ret);
	}

	return ret;
}
