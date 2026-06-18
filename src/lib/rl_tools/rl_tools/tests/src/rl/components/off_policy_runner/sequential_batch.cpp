#include <rl_tools/operations/cpu.h>
#include <rl_tools/numeric_types/categories.h>
#include "dummy_environment.h"
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/rl/components/off_policy_runner/operations_generic.h>
namespace rlt = rl_tools;

#include <gtest/gtest.h>

using DEVICE = rlt::devices::DefaultCPU;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = DEVICE::index_t;

using ENVIRONMENT_SPEC = rlt::rl::environments::dummy::Specification<T, TI>;
using ENVIRONMENT = rlt::rl::environments::Dummy<ENVIRONMENT_SPEC>;
using EXPLORATION_POLICY_SPEC = rlt::nn_models::random_uniform::Specification<TYPE_POLICY, TI, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, rlt::nn_models::random_uniform::Range::MINUS_ONE_TO_ONE>;
using EXPLORATION_POLICY = rlt::nn_models::RandomUniform<EXPLORATION_POLICY_SPEC>;
using POLICIES = rl_tools::utils::Tuple<TI, EXPLORATION_POLICY>;
struct OFF_POLICY_RUNNER_PARAMETERS{
    static constexpr TI N_ENVIRONMENTS = 2;
    static constexpr bool ASYMMETRIC_OBSERVATIONS = !rl_tools::utils::typing::is_same_v<typename ENVIRONMENT::Observation, typename ENVIRONMENT::ObservationPrivileged>;
    static constexpr TI REPLAY_BUFFER_CAPACITY = 1000;
    static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
    static constexpr bool COLLECT_EPISODE_STATS = false;
    static constexpr TI EPISODE_STATS_BUFFER_SIZE = 1000;
    static constexpr bool SAMPLE_PARAMETERS = true;
};
using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, TI, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS>;
using OFF_POLICY_RUNNER = rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC>;

constexpr TI SEQUENCE_LENGTH = 10;
constexpr TI BATCH_SIZE = 1;

template <bool T_INCLUDE_FIRST_STEP_IN_TARGETS, bool T_ALWAYS_SAMPLE_FROM_INITIAL_STATE, bool T_RANDOM_SEQ_LENGTH, bool T_ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, TI T_NOMINAL_SEQUENCE_LENGTH_PROBABILITY>
struct SequentialBatchParameters{
    static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = T_INCLUDE_FIRST_STEP_IN_TARGETS;
    static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = T_ALWAYS_SAMPLE_FROM_INITIAL_STATE;
    static constexpr bool RANDOM_SEQ_LENGTH = T_RANDOM_SEQ_LENGTH;
    static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = T_ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY;
    static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = T_NOMINAL_SEQUENCE_LENGTH_PROBABILITY / 100.0;
};

template <bool T_INCLUDE_FIRST_STEP_IN_TARGETS, bool T_ALWAYS_SAMPLE_FROM_INITIAL_STATE, bool T_RANDOM_SEQ_LENGTH, bool T_ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, TI T_NOMINAL_SEQUENCE_LENGTH_PROBABILITY>
using SEQUENTIAL_BATCH = rlt::rl::components::off_policy_runner::SequentialBatch<rlt::rl::components::off_policy_runner::SequentialBatchSpecification<OFF_POLICY_RUNNER_SPEC, 10, BATCH_SIZE, SequentialBatchParameters<T_INCLUDE_FIRST_STEP_IN_TARGETS, T_ALWAYS_SAMPLE_FROM_INITIAL_STATE, T_RANDOM_SEQ_LENGTH, T_ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, T_NOMINAL_SEQUENCE_LENGTH_PROBABILITY>>>;

enum class SequenceState{
    FIRST_STEP,
    IN_SEQUENCE,
    PADDING,
    DEAD
};

template <typename DEVICE, typename BATCH>
void check_one_batch(DEVICE& device, BATCH& batch){
    constexpr bool EXCLUSIVE = !BATCH::SPEC::PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS;
    for (TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++){
        SequenceState state = SequenceState::FIRST_STEP;
        for (TI seq_step_i = 0; seq_step_i < SEQUENCE_LENGTH + 1; seq_step_i++){
            bool reset = seq_step_i < SEQUENCE_LENGTH ? rlt::get(device, batch.reset, seq_step_i, batch_step_i, 0) : true;
            bool next_reset = rlt::get(device, batch.next_reset_base, seq_step_i, batch_step_i, 0);
            bool final_step_mask = seq_step_i < SEQUENCE_LENGTH ? rlt::get(device, batch.final_step_mask, seq_step_i, batch_step_i, 0) : false;
            bool next_final_step_mask = rlt::get(device, batch.next_final_step_mask_base, seq_step_i, batch_step_i, 0);
            switch(state){
                case SequenceState::FIRST_STEP:
                    rlt::utils::assert_exit(device, reset, "reset");
                    rlt::utils::assert_exit(device, next_reset, "next_reset");
                    rlt::utils::assert_exit(device, !next_final_step_mask, "!next_final_step_mask");
                    break;
                case SequenceState::IN_SEQUENCE:
                    rlt::utils::assert_exit(device, !reset, "!reset");
                    rlt::utils::assert_exit(device, (EXCLUSIVE && (seq_step_i == 1 && next_reset)) || !next_reset, "!next_reset");
                    rlt::utils::assert_exit(device, !next_final_step_mask, "!next_final_step_mask");
                    break;
                case SequenceState::PADDING:
                    rlt::utils::assert_exit(device, reset, "reset");
                    rlt::utils::assert_exit(device, (EXCLUSIVE && (seq_step_i == 1 && next_reset)) || !next_reset, "!next_reset");
                    rlt::utils::assert_exit(device, next_final_step_mask, "next_final_step_mask");
                    break;
                case SequenceState::DEAD:
                    std::cout << "Reached DEAD state" << std::endl;
                    rlt::utils::assert_exit(device, reset, "reset");
                    rlt::utils::assert_exit(device, next_reset, "next_reset");
                    rlt::utils::assert_exit(device, !final_step_mask, "!final_step_mask");
                    rlt::utils::assert_exit(device, !next_final_step_mask, "!next_final_step_mask");
                    break;
            }
            switch(state){
                case SequenceState::FIRST_STEP:{
                        if constexpr(BATCH::SPEC::PARAMETERS::ALWAYS_SAMPLE_FROM_INITIAL_STATE){
                            T observation = rlt::get(device, batch.observations_current, seq_step_i, batch_step_i, 0);
                            rlt::utils::assert_exit(device, observation == 0, "observation == 0");
                        }
                    }
                    break;
                case SequenceState::IN_SEQUENCE:{
                        T observation = rlt::get(device, batch.observations_current, seq_step_i, batch_step_i, 0);
                        T previous_observation = rlt::get(device, batch.observations_current, seq_step_i-1, batch_step_i, 0);
                        rlt::utils::assert_exit(device, observation == previous_observation + 1, "observation == previous_observation + 1");
                    }
                    break;
                case SequenceState::PADDING:{
                        T observation = rlt::get(device, batch.observations_current, seq_step_i, batch_step_i, 0);
                        T previous_observation = rlt::get(device, batch.observations_current, seq_step_i-1, batch_step_i, 0);
                        rlt::utils::assert_exit(device, observation == previous_observation + 1, "observation == previous_observation + 1");
                    }
                    break;
                case SequenceState::DEAD:
                    break;
            }
            SequenceState next_state;
            switch(state){
                case SequenceState::FIRST_STEP:
                case SequenceState::IN_SEQUENCE:
                    if(seq_step_i == SEQUENCE_LENGTH-1){
                        next_state = SequenceState::PADDING;
                    }
                    else{
                        next_state = final_step_mask ? SequenceState::PADDING : SequenceState::IN_SEQUENCE;
                    }
                    break;
            case SequenceState::PADDING:
                    if(seq_step_i == SEQUENCE_LENGTH - 1){
                        next_state = SequenceState::DEAD;
                    }
                    else{
                        next_state = SequenceState::FIRST_STEP;
                    }
                    break;
                case SequenceState::DEAD:
                    next_state = SequenceState::DEAD;
                    break;
            }
            state = next_state;
        }
    }

}
template <typename DEVICE, typename BATCH, typename RNG>
void check_batch(DEVICE& device, BATCH& batch, RNG& rng){
    OFF_POLICY_RUNNER off_policy_runner;
    EXPLORATION_POLICY policy;
    EXPLORATION_POLICY::Buffer<> policy_buffer;
    rlt::malloc(device, off_policy_runner);
    rlt::malloc(device, batch);
    rlt::malloc(device, policy_buffer);
    rlt::init(device, off_policy_runner);
    for(TI step_i = 0; step_i < 1000; step_i++){
        rlt::step<0>(device, off_policy_runner, policy, policy_buffer, rng);
    }
    for(TI batch_i = 0; batch_i < 100000; batch_i++){
        rlt::gather_batch(device, off_policy_runner, batch, rng);
        check_one_batch(device, batch);
    }
    rlt::free(device, off_policy_runner);
    rlt::free(device, batch);
    rlt::free(device, policy_buffer);
}

TEST(RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_SQUENTIAL_BATCH, TEST){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    {
        constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = true;
        constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = true;
        constexpr bool RANDOM_SEQ_LENGTH = true;
        constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        constexpr TI NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 50;
        SEQUENTIAL_BATCH<INCLUDE_FIRST_STEP_IN_TARGETS, ALWAYS_SAMPLE_FROM_INITIAL_STATE, RANDOM_SEQ_LENGTH, ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, NOMINAL_SEQUENCE_LENGTH_PROBABILITY> batch;
        check_batch(device, batch, rng);
    }
    {
        constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = false;
        constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = true;
        constexpr bool RANDOM_SEQ_LENGTH = true;
        constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        constexpr TI NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 50;
        SEQUENTIAL_BATCH<INCLUDE_FIRST_STEP_IN_TARGETS, ALWAYS_SAMPLE_FROM_INITIAL_STATE, RANDOM_SEQ_LENGTH, ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, NOMINAL_SEQUENCE_LENGTH_PROBABILITY> batch;
        check_batch(device, batch, rng);
    }
    {
        constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = true;
        constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
        constexpr bool RANDOM_SEQ_LENGTH = true;
        constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        constexpr TI NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 50;
        SEQUENTIAL_BATCH<INCLUDE_FIRST_STEP_IN_TARGETS, ALWAYS_SAMPLE_FROM_INITIAL_STATE, RANDOM_SEQ_LENGTH, ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, NOMINAL_SEQUENCE_LENGTH_PROBABILITY> batch;
        check_batch(device, batch, rng);
    }
    {
        constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = true;
        constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
        constexpr bool RANDOM_SEQ_LENGTH = false;
        constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        constexpr TI NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 50;
        SEQUENTIAL_BATCH<INCLUDE_FIRST_STEP_IN_TARGETS, ALWAYS_SAMPLE_FROM_INITIAL_STATE, RANDOM_SEQ_LENGTH, ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY, NOMINAL_SEQUENCE_LENGTH_PROBABILITY> batch;
        check_batch(device, batch, rng);
    }
}
