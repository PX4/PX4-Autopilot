#include <rl_tools/operations/cpu.h>

#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/rl/components/replay_buffer/operations_cpu.h>
#include <rl_tools/rl/components/replay_buffer/persist.h>

#include "replay_buffer.h"


#include <gtest/gtest.h>
#include <highfive/H5File.hpp>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;


TEST(RL_TOOLS_RL_COMPONENTS_REPLAY_BUFFER, PERSISTENCE) {
    std::string replay_buffer_path = "test_cuda_replay_buffer.h5";
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<DTYPE>;
    constexpr DEVICE::index_t OBSERVATION_DIM = 2;
    constexpr DEVICE::index_t ACTION_DIM = 3;
    constexpr DEVICE::index_t CAPACITY = 20;
    using REPLAY_BUFFER_SPEC = rlt::rl::components::replay_buffer::Specification<TYPE_POLICY, DEVICE::index_t, OBSERVATION_DIM, OBSERVATION_DIM, false, ACTION_DIM, CAPACITY>;
    using REPLAY_BUFFER = rlt::rl::components::ReplayBuffer<REPLAY_BUFFER_SPEC>;
    DEVICE device;
    REPLAY_BUFFER replay_buffer_1;
    REPLAY_BUFFER replay_buffer_2;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        rlt::malloc(device, replay_buffer_1);
        rlt::test::rl::components::replay_buffer::sample(device, replay_buffer_1, rng);
        set(replay_buffer_1.next_observations, 7, 0, 1337);
        auto data_file = HighFive::File(replay_buffer_path, HighFive::File::Overwrite);
        rlt::persist::backends::hdf5::Group<> replay_buffer_group = {data_file.createGroup("replay_buffer")};
        rlt::save(device, replay_buffer_1, replay_buffer_group);
    }
    {
        rlt::malloc(device, replay_buffer_2);
        auto data_file = HighFive::File(replay_buffer_path, HighFive::File::ReadOnly);
        rlt::persist::backends::hdf5::Group<> replay_buffer_group = {data_file.getGroup("replay_buffer")};
        rlt::load(device, replay_buffer_2, replay_buffer_group);
    }
    {
        auto abs_diff = rlt::abs_diff(device, replay_buffer_1, replay_buffer_2);
        ASSERT_FLOAT_EQ(abs_diff, 0);
        auto v = rlt::view<DEVICE, typename decltype(replay_buffer_2.next_observations)::SPEC, 3, 2>(device, replay_buffer_2.next_observations, 6, 0);
        rlt::print(device, v);
    }
    {
        rlt::free(device, replay_buffer_1);
        rlt::free(device, replay_buffer_2);
    }
}
