#ifndef RL_TOOLS_WASM
#include <rl_tools/operations/arm.h>
#else
#include <rl_tools/operations/wasm32.h>
#endif

#include <rl_tools/nn/layers/standardize/operations_generic.h>
#ifndef RL_TOOLS_WASM
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
// #include <rl_tools/nn/layers/dense/operations_generic.h>
#else
#include <rl_tools/nn/layers/dense/operations_generic.h>
#endif
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/inference/executor/executor.h>

#include "../../../../tests/data/test_inference_executor_policy.h"

#ifdef RL_TOOLS_ENABLE_INFORMATIVE_STATUS_MESSAGES
#include <cstdio>
#endif

namespace rlt = rl_tools;

namespace other{
#ifndef RL_TOOLS_WASM
    using DEV_SPEC = rlt::devices::DefaultARMSpecification;
    using DEVICE = rlt::devices::arm::OPT<DEV_SPEC>;
#else
    using DEVICE = rlt::devices::DefaultWASM32;
#endif
}


struct RL_TOOLS_INFERENCE_APPLICATIONS_L2F_CONFIG{
    using DEVICE = other::DEVICE;
    using TI = typename other::DEVICE::index_t;
    using RNG = other::DEVICE::SPEC::RANDOM::ENGINE<>;
    static constexpr TI TEST_SEQUENCE_LENGTH_ACTUAL = 5;
    static constexpr TI TEST_BATCH_SIZE_ACTUAL = 2;
    using ACTOR_TYPE_ORIGINAL = rlt::checkpoint::actor::TYPE;
    using POLICY_TEST = rlt::checkpoint::actor::TYPE::template CHANGE_BATCH_SIZE<TI, 1>::template CHANGE_SEQUENCE_LENGTH<TI, 1>;
    using POLICY = ACTOR_TYPE_ORIGINAL::template CHANGE_BATCH_SIZE<TI, 1>::template CHANGE_SEQUENCE_LENGTH<TI, 1>;
    using TYPE_POLICY = typename POLICY::SPEC::TYPE_POLICY;
    static auto& policy() {
#ifdef _MSC_VER
        static auto _policy = rl_tools::checkpoint::actor::factory_function();
        return _policy;
#else
        return rlt::checkpoint::actor::module;
#endif
    }
    static constexpr TI ACTION_HISTORY_LENGTH = 1; //rl_tools::checkpoint::environment::ACTION_HISTORY_LENGTH
    static constexpr TI CONTROL_INTERVAL_INTERMEDIATE_NS = 1 * 1000 * 1000; // Inference is at 500hz
    static constexpr TI CONTROL_INTERVAL_NATIVE_NS = 10 * 1000 * 1000; // Training is 100hz
    static constexpr TI TIMING_STATS_NUM_STEPS = 100;
    static constexpr bool FORCE_SYNC_INTERMEDIATE = true;
    static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = false;
    static constexpr TI FORCE_SYNC_NATIVE = 0;
    static constexpr bool DYNAMIC_ALLOCATION = false;
    using WARNING_LEVELS = rlt::inference::executor::WarningLevelsDefault<TYPE_POLICY>;
};

#include <rl_tools/inference/applications/l2f/c_backend.h>

