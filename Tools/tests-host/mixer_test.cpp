#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>

extern int test_mixer(int argc, char *argv[]);

int main(int argc, char *argv[]) {
	warnx("Host execution started");

	char* args[] = {argv[0], "../../ROMFS/px4fmu_common/mixers/IO_pass.mix"};

	test_mixer(2, args);
}