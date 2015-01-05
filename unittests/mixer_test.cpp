#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include "../../src/systemcmds/tests/tests.h"

#include "gtest/gtest.h"


TEST(MixerTest, IO_pass) {
	char* args[] = {"mixer", "../../ROMFS/px4fmu_common/mixers/IO_pass.mix"};
	ASSERT_EQ(test_mixer(2, args), 0) << "IO_pass.mix failed";
}

TEST(MixerTest, FMU_quad_w) {
        char* args[] = {"mixer", "../../ROMFS/px4fmu_common/mixers/FMU_quad_w.mix"};
        ASSERT_EQ(test_mixer(2, args), 0) << "FMU_quad_w.mix failed";
} 
