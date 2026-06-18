struct TestTag{};

#include <rl_tools/operations/cpu.h>

namespace rlt = rl_tools;

#include <gtest/gtest.h>


using DEVICE = rlt::devices::DefaultCPU;
using TEST_USE_CASE = rlt::numeric_types::UseCase<TestTag, float>;

using TYPE_POLICY = rlt::numeric_types::Policy<float, TEST_USE_CASE>;
static_assert(rlt::utils::typing::is_same_v<TYPE_POLICY::GET<TestTag>, float>);

TEST(RL_TOOLS_NUMERIC_TYPES_TYPE_POLICY, MAIN){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
}
