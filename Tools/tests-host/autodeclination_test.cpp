
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4iofirmware/px4io.h>
#include "../../src/systemcmds/tests/tests.h"

int main(int argc, char *argv[]) {
	warnx("autodeclination test started");

	if (argc < 3)
		errx(1, "Need lat/lon!");

	

}