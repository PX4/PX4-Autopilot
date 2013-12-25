#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include "../../src/systemcmds/tests/tests.h"

int main(int argc, char *argv[]) {
	warnx("Host execution started");

	char* args[] = {argv[0], "../../ROMFS/px4fmu_common/mixers/IO_pass.mix",
				 "../../ROMFS/px4fmu_common/mixers/FMU_quad_w.mix"};

	test_mixer(3, args);
}