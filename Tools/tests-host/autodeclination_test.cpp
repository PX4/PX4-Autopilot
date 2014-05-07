
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>
#include <string.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4iofirmware/px4io.h>
#include "../../src/systemcmds/tests/tests.h"
#include <geo/geo.h>

int main(int argc, char *argv[]) {
	warnx("autodeclination test started");

	if (argc < 3)
		errx(1, "Need lat/lon!");

	char* p_end;

	float lat = strtod(argv[1], &p_end);
	float lon = strtod(argv[2], &p_end);

	float declination = get_mag_declination(lat, lon);

	printf("lat: %f lon: %f, dec: %f\n", lat, lon, declination);

}
