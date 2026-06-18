#ifndef RL_TOOLS_TESTS_SRC_RL_COMPONENTS_REPLAY_BUFFER_H
#define RL_TOOLS_TESTS_SRC_RL_COMPONENTS_REPLAY_BUFFER_H

#include <rl_tools/rl/components/replay_buffer/replay_buffer.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::test::rl::components::replay_buffer{
    template <typename DEVICE, typename SPEC, typename RNG>
    void sample(DEVICE& device, rl_tools::rl::components::ReplayBuffer<SPEC>& rb, RNG& rng){
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        randn(device, rb.observations, rng);
        randn(device, rb.actions, rng);
        randn(device, rb.next_observations, rng);
        randn(device, rb.rewards, rng);

        for (typename DEVICE::index_t row_i = 0; row_i < SPEC::CAPACITY; row_i++) {
            bool terminated = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), (T) 0, (T) 1, rng) > 0.5;
            bool truncated = terminated || random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), (T) 0, (T) 1, rng) > 0.5;
            set(rb.terminated, row_i, 0, terminated);
            set(rb.truncated, row_i, 0, truncated);
        }
        rb.position = 0;
        rb.full = true;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
