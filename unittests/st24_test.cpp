
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <rc/st24.h>
#include "../../src/systemcmds/tests/tests.h"

int main(int argc, char *argv[])
{
	warnx("ST24 test started");

	char* defaultfile = "testdata/st24_data.txt";

	char* filepath = 0;

	if (argc < 2) {
		warnx("Too few arguments. Using default file: %s", defaultfile);
		filepath = defaultfile;
	} else {
		filepath = argv[1];
	}

	warnx("loading data from: %s", filepath);

	FILE *fp;

	fp = fopen(filepath, "rt");

	if (!fp) {
		errx(1, "failed opening file");
	}

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
		if (((f - last_time) * 1000 * 1000) > 3000) {
			// warnx("FRAME RESET\n\n");
		}

		uint8_t b = static_cast<uint8_t>(x);

		last_time = f;

		// Pipe the data into the parser
		hrt_abstime now = hrt_absolute_time();

		uint8_t rssi;
		uint8_t rx_count;
		uint16_t channel_count;
		uint16_t channels[20];


		if (!st24_decode(b, &rssi, &rx_count, &channel_count, channels, sizeof(channels) / sizeof(channels[0]))) {
			//warnx("decoded: %u channels (converted to PPM range)", (unsigned)channel_count);

			for (unsigned i = 0; i < channel_count; i++) {

				int16_t val = channels[i];
				//warnx("channel %u: %d 0x%03X", i, static_cast<int>(val), static_cast<int>(val));
			}
		}
	}

	if (ret == EOF) {
		warnx("Test finished, reached end of file");
		ret = 0;
	} else {
		warnx("Test aborted, errno: %d", ret);
	}

	return ret;
}
