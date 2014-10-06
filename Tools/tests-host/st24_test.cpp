
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4iofirmware/px4io.h>
#include "../../src/systemcmds/tests/tests.h"

int main(int argc, char *argv[]) {
	warnx("ST24 test started");

	if (argc < 2)
		errx(1, "Need a filename for the input file");

	warnx("loading data from: %s", argv[1]);

	FILE *fp;
	
	fp = fopen(argv[1],"rt");

	if (!fp)
		errx(1, "failed opening file");

	float f;
	unsigned x;
	int ret;

	// Trash the first 20 lines
	for (unsigned i = 0; i < 20; i++) {
		(void)fscanf(fp, "%f,%x,,", &f, &x);
	}

	float last_time = 0;

	while (EOF != (ret = fscanf(fp, "%f,%x,,", &f, &x))) {
		if (((f - last_time) * 1000 * 1000) > 3000) {
			warnx("FRAME RESET\n\n");
		}

		warnx("%f: 0x%02x", (double)f, x);

		last_time = f;

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		//if (partial_frame_count % 25 == 0)
			//sbus_parse(now, frame, &partial_frame_count, rc_values, &num_values, &sbus_failsafe, &sbus_frame_drop, max_channels);
	}

	if (ret == EOF) {
		warnx("Test finished, reached end of file");
	} else {
		warnx("Test aborted, errno: %d", ret);
	}

}
