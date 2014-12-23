
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4iofirmware/px4io.h>
// #include "../../src/systemcmds/tests/tests.h"
#include <geo/geo.h>

int main(int argc, char *argv[]) {
	warnx("autodeclination test started");

	char* latstr = 0;
	char* lonstr = 0;
	char* declstr = 0;

	if (argc < 4) {
		warnx("Too few arguments. Using default lat / lon and declination");
		latstr = "47.0";
		lonstr = "8.0";
		declstr = "0.6";
	} else {
		latstr = argv[1];
		lonstr = argv[2];
		declstr = argv[3];
	}

	char* p_end;

	float lat = strtod(latstr, &p_end);
	float lon = strtod(lonstr, &p_end);
	float decl_truth = strtod(declstr, &p_end);

	float declination = get_mag_declination(lat, lon);

	printf("lat: %f lon: %f, expected dec: %f, estimated dec: %f\n", lat, lon, declination, decl_truth);

	int ret = 0;

	// Fail if the declination differs by more than one degree
	float decldiff = fabs(decl_truth - declination);
	if (decldiff > 0.5f) {
		warnx("declination differs more than 1 degree: difference: %12.8f", decldiff);
		ret = 1;
	}

	return ret;
}
