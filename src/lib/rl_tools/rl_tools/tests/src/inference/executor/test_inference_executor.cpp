// #include "rl_tools_adapter_new.h"
#include <iostream>
#include <random>
#include <rl_tools/operations/cpu.h>

#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/inference/executor/executor.h>
#include <rl_tools/inference/executor/operations_generic.h>
#include <rl_tools/inference/applications/l2f/l2f.h>
#include <rl_tools/inference/applications/l2f/operations_generic.h>

#include "../../../../tests/data/test_inference_executor_policy.h"

namespace rlt = rl_tools;


using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using TI = typename DEVICE::index_t;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TIMESTAMP = uint64_t;

using POLICY = rlt::checkpoint::actor::TYPE;

// static constexpr uint OUTPUT_DIM = 4;
// int main(){
// }

#include <gtest/gtest.h>

TEST(RL_TOOLS_INFERENCE_EXECUTOR, MAIN){
    static constexpr TI ACTION_HISTORY_LENGTH = 1;
    static constexpr TI OUTPUT_DIM = POLICY::OUTPUT_SHAPE::LAST;
    static constexpr TIMESTAMP CONTROL_INTERVAL_INTERMEDIATE_NS = 2500 * 1000;
    static constexpr TIMESTAMP CONTROL_INTERVAL_NATIVE_NS = 10000 * 1000;
    static constexpr bool FORCE_SYNC_INTERMEDIATE = false;
    static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = false;
    static constexpr TI FORCE_SYNC_NATIVE = 0;
#if _MSC_VER
    static constexpr bool DYNAMIC_ALLOCATION = true; // MSVC doesn't like these large stack objects
    auto policy = rl_tools::checkpoint::actor::factory_function();
#else
    static constexpr bool DYNAMIC_ALLOCATION = false;
    auto& policy = rl_tools::checkpoint::actor::module;
#endif
    using SPEC = rlt::inference::applications::l2f::Specification<TYPE_POLICY, TI, TIMESTAMP, ACTION_HISTORY_LENGTH, OUTPUT_DIM, POLICY, CONTROL_INTERVAL_INTERMEDIATE_NS, CONTROL_INTERVAL_NATIVE_NS, FORCE_SYNC_INTERMEDIATE, FORCE_SYNC_NATIVE, FORCE_SYNC_NATIVE_RUNTIME, rlt::inference::executor::WarningLevelsDefault<TYPE_POLICY>, DYNAMIC_ALLOCATION>;

    DEVICE device;
    RNG rng;
    rlt::init(device);
    rlt::malloc(device, rng);
    TI seed = 0;
    rlt::init(device, rng, seed);
    // rlt::print(device, rl_tools::checkpoint::actor::layer_0::weights::parameters.parameters);
    rlt::print(device, rl_tools::checkpoint::actor::layer_1::initial_hidden_state::parameters.parameters);

    rlt::inference::applications::L2F<SPEC> executor;
    rlt::malloc(device, executor);
    rlt::print(device, executor.executor.policy_state.content_state.next_content_state.state.step);
    rlt::reset(device, executor, policy, rng);

    rlt::inference::applications::l2f::Action<SPEC> action;
    // float test_result = rl_tools_test(&action);
    // std::cout << "test: " << test_result << std::endl;
    // for(uint i = 0; i < OUTPUT_DIM; i++){
    //     std::cout << "action[" << i << "] = " << action.action[i] << std::endl;
    // }
    rlt::inference::applications::l2f::Observation<SPEC> observation;
    observation.position[0] = 0.0f;
    observation.position[1] = 0.0f;
    observation.position[2] = 0.0f;
    observation.orientation[0] = 1.0f;
    observation.orientation[1] = 0.0f;
    observation.orientation[2] = 0.0f;
    observation.orientation[3] = 0.0f;
    observation.linear_velocity[0] = 0.0f;
    observation.linear_velocity[1] = 0.0f;
    observation.linear_velocity[2] = 0.0f;
    observation.angular_velocity[0] = 0.0f;
    observation.angular_velocity[1] = 0.0f;
    observation.angular_velocity[2] = 0.0f;
    for(TI j = 0; j < OUTPUT_DIM; j++){
        observation.previous_action[j] = 0.0f;
    }

    TIMESTAMP timestamp = 0;
    auto status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
    ASSERT_EQ(status.step_type, decltype(status.step_type)::NATIVE);
    timestamp += 1000 * 1000; // 1
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 2
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 3
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
    ASSERT_EQ(status.step_type, decltype(status.step_type)::INTERMEDIATE);
    timestamp += 1000 * 1000; // 4
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 5
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 6
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
    ASSERT_EQ(status.step_type, decltype(status.step_type)::INTERMEDIATE);
    timestamp += 1000 * 1000; // 7
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 8
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 9
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
    ASSERT_EQ(status.step_type, decltype(status.step_type)::INTERMEDIATE);
    timestamp += 1000 * 1000; // 10
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 11
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::OBSERVATION);
    timestamp += 1000 * 1000; // 12
    status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
    ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
    ASSERT_EQ(status.step_type, decltype(status.step_type)::NATIVE);
}

TEST(RL_TOOLS_INFERENCE_EXECUTOR, SYNC_INTERMEDIATE){
    static constexpr TI ACTION_HISTORY_LENGTH = 1;
    static constexpr TI OUTPUT_DIM = POLICY::OUTPUT_SHAPE::LAST;
    static constexpr TIMESTAMP CONTROL_INTERVAL_INTERMEDIATE_NS = 2500 * 1000;
    static constexpr TIMESTAMP CONTROL_INTERVAL_NATIVE_NS = 10000 * 1000;
    static constexpr bool FORCE_SYNC_INTERMEDIATE = true;
    static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = false;
    static constexpr TI FORCE_SYNC_NATIVE = 0;
#if _MSC_VER
    static constexpr bool DYNAMIC_ALLOCATION = true; // MSVC doesn't like these large stack objects
    auto policy = rl_tools::checkpoint::actor::factory_function();
#else
    static constexpr bool DYNAMIC_ALLOCATION = false;
    auto& policy = rl_tools::checkpoint::actor::module;
#endif
    using SPEC = rlt::inference::applications::l2f::Specification<TYPE_POLICY, TI, TIMESTAMP, ACTION_HISTORY_LENGTH, OUTPUT_DIM, POLICY, CONTROL_INTERVAL_INTERMEDIATE_NS, CONTROL_INTERVAL_NATIVE_NS, FORCE_SYNC_INTERMEDIATE, FORCE_SYNC_NATIVE, FORCE_SYNC_NATIVE_RUNTIME, rlt::inference::executor::WarningLevelsDefault<TYPE_POLICY>, DYNAMIC_ALLOCATION>;
    DEVICE device;
    RNG rng;
    rlt::init(device);
    rlt::malloc(device, rng);
    TI seed = 0;
    rlt::init(device, rng, seed);

    rlt::inference::applications::L2F<SPEC> executor;
    rlt::malloc(device, executor);
    rlt::reset(device, executor, policy, rng);

    rlt::inference::applications::l2f::Action<SPEC> action;
    rlt::inference::applications::l2f::Observation<SPEC> observation;
    observation.position[0] = 0.0f;
    observation.position[1] = 0.0f;
    observation.position[2] = 0.0f;
    observation.orientation[0] = 1.0f;
    observation.orientation[1] = 0.0f;
    observation.orientation[2] = 0.0f;
    observation.orientation[3] = 0.0f;
    observation.linear_velocity[0] = 0.0f;
    observation.linear_velocity[1] = 0.0f;
    observation.linear_velocity[2] = 0.0f;
    observation.angular_velocity[0] = 0.0f;
    observation.angular_velocity[1] = 0.0f;
    observation.angular_velocity[2] = 0.0f;
    for(TI j = 0; j < OUTPUT_DIM; j++){
        observation.previous_action[j] = 0.0f;
    }

    rlt::inference::executor::Status<SPEC::EXECUTOR_SPEC> status;
    for (TI step=0; step <= 1000; step++){
        TIMESTAMP timestamp = step * 1000 * 1000;
        std::cout << "timestamp: " << timestamp << std::endl;
        status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
        ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
        if(timestamp % CONTROL_INTERVAL_NATIVE_NS == 0){
            ASSERT_EQ(status.step_type, decltype(status.step_type)::NATIVE);
            ASSERT_TRUE(status.timing_bias.OK);
            ASSERT_TRUE(status.timing_jitter.OK);
        }
        else{
            ASSERT_EQ(status.step_type, decltype(status.step_type)::INTERMEDIATE);
            if (step < 100){
                ASSERT_TRUE(status.timing_bias.OK);
                ASSERT_TRUE(status.timing_jitter.OK);
            }
            else{
                ASSERT_FALSE(status.timing_bias.OK);
                ASSERT_FALSE(status.timing_jitter.OK);
            }
        }
    }
}


TEST(RL_TOOLS_INFERENCE_EXECUTOR, SYNC_INTERMEDIATE_JITTER){
    static constexpr TI ACTION_HISTORY_LENGTH = 1;
    static constexpr TI OUTPUT_DIM = POLICY::OUTPUT_SHAPE::LAST;
    static constexpr TIMESTAMP CONTROL_INTERVAL_INTERMEDIATE_NS = 2500 * 1000;
    static constexpr TIMESTAMP CONTROL_INTERVAL_NATIVE_NS = 10000 * 1000;
    static constexpr bool FORCE_SYNC_INTERMEDIATE = true;
    static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = false;
    static constexpr TI FORCE_SYNC_NATIVE = 0;
#if _MSC_VER
    static constexpr bool DYNAMIC_ALLOCATION = true; // MSVC doesn't like these large stack objects
    auto policy = rl_tools::checkpoint::actor::factory_function();
#else
    static constexpr bool DYNAMIC_ALLOCATION = false;
    auto& policy = rl_tools::checkpoint::actor::module;
#endif
    using SPEC = rlt::inference::applications::l2f::Specification<TYPE_POLICY, TI, TIMESTAMP, ACTION_HISTORY_LENGTH, OUTPUT_DIM, POLICY, CONTROL_INTERVAL_INTERMEDIATE_NS, CONTROL_INTERVAL_NATIVE_NS, FORCE_SYNC_INTERMEDIATE, FORCE_SYNC_NATIVE, FORCE_SYNC_NATIVE_RUNTIME, rlt::inference::executor::WarningLevelsDefault<TYPE_POLICY>, DYNAMIC_ALLOCATION>;
    DEVICE device;
    RNG rng;
    rlt::init(device);
    rlt::malloc(device, rng);
    TI seed = 0;
    rlt::init(device, rng, seed);

    rlt::inference::applications::L2F<SPEC> executor;
    rlt::malloc(device, executor);
    rlt::reset(device, executor, policy, rng);

    rlt::inference::applications::l2f::Action<SPEC> action;
    rlt::inference::applications::l2f::Observation<SPEC> observation;
    observation.position[0] = 0.0f;
    observation.position[1] = 0.0f;
    observation.position[2] = 0.0f;
    observation.orientation[0] = 1.0f;
    observation.orientation[1] = 0.0f;
    observation.orientation[2] = 0.0f;
    observation.orientation[3] = 0.0f;
    observation.linear_velocity[0] = 0.0f;
    observation.linear_velocity[1] = 0.0f;
    observation.linear_velocity[2] = 0.0f;
    observation.angular_velocity[0] = 0.0f;
    observation.angular_velocity[1] = 0.0f;
    observation.angular_velocity[2] = 0.0f;
    for(TI j = 0; j < OUTPUT_DIM; j++){
        observation.previous_action[j] = 0.0f;
    }

    TIMESTAMP timestamp = 0;
    rlt::inference::executor::Status<SPEC::EXECUTOR_SPEC> status;
    for (TI step=0; step <= 10000; step++){
        std::cout << "timestamp: " << timestamp << std::endl;
        status = rlt::control(device, executor, timestamp, policy, observation, action, rng);
        ASSERT_EQ(status.source, decltype(status.source)::CONTROL);
        if((timestamp - (step > 1500 ? (TIMESTAMP)(1.5 * CONTROL_INTERVAL_NATIVE_NS) : 0)) % CONTROL_INTERVAL_NATIVE_NS == 0 || step == 1501){
            ASSERT_EQ(status.step_type, decltype(status.step_type)::NATIVE);
            ASSERT_TRUE(status.timing_bias.OK);
            if (step > 1500 && step <= 2500){
                ASSERT_FALSE(status.timing_jitter.OK);
            }
            else{
                ASSERT_TRUE(status.timing_jitter.OK);
            }
        }
        else{
            ASSERT_EQ(status.step_type, decltype(status.step_type)::INTERMEDIATE);
            if (step < 100){
                ASSERT_TRUE(status.timing_bias.OK);
                ASSERT_TRUE(status.timing_jitter.OK);
            }
            else{
                ASSERT_FALSE(status.timing_bias.OK);
                ASSERT_FALSE(status.timing_jitter.OK);
            }
        }
        timestamp += step == 1500 ? 1.5 * CONTROL_INTERVAL_NATIVE_NS : 1000 * 1000;
    }
}
