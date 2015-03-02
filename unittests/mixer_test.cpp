#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include "../../src/systemcmds/tests/tests.h"

#include "gtest/gtest.h"


TEST(MixerTest, Mixer)
{
	char *args[] = {"empty", "../ROMFS/px4fmu_common/mixers/IO_pass.mix", "../ROMFS/px4fmu_common/mixers/quad_w.main.mix"};
	ASSERT_EQ(test_mixer(3, args), 0) << "IO_pass.mix failed";
}
