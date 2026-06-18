#include "../../executor/operations_generic.h"
#include "c_interface.h"
#include "../../executor/c_backend.h"
#include "operations_generic.h"

namespace rl_tools::inference::applications::l2f{
    using CONFIG = RL_TOOLS_INFERENCE_APPLICATIONS_L2F_CONFIG;
    using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
    using TI = typename CONFIG::TI;

    static constexpr TI TEST_SEQUENCE_LENGTH = rlt::checkpoint::example::input::SHAPE::template GET<0>;
    static constexpr TI TEST_BATCH_SIZE = rlt::checkpoint::example::input::SHAPE::template GET<1>;
    static_assert(CONFIG::TEST_BATCH_SIZE_ACTUAL <= TEST_BATCH_SIZE);
    static_assert(CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL <= TEST_SEQUENCE_LENGTH);

    static constexpr TI INPUT_DIM = CONFIG::POLICY::INPUT_SHAPE::LAST;
    static constexpr TI OUTPUT_DIM = CONFIG::POLICY::OUTPUT_SHAPE::LAST;
    static_assert(OUTPUT_DIM == 4);
    static_assert(INPUT_DIM == (18 + CONFIG::ACTION_HISTORY_LENGTH * OUTPUT_DIM));


    // state
    using SPEC = rl_tools::inference::applications::l2f::Specification<TYPE_POLICY, TI, RLtoolsInferenceTimestamp, CONFIG::ACTION_HISTORY_LENGTH, OUTPUT_DIM, typename CONFIG::POLICY, CONFIG::CONTROL_INTERVAL_INTERMEDIATE_NS, CONFIG::CONTROL_INTERVAL_NATIVE_NS, CONFIG::FORCE_SYNC_INTERMEDIATE, CONFIG::FORCE_SYNC_NATIVE, CONFIG::FORCE_SYNC_NATIVE_RUNTIME, CONFIG::WARNING_LEVELS, CONFIG::DYNAMIC_ALLOCATION>;
    typename CONFIG::DEVICE device;
    typename CONFIG::RNG rng;
    static rl_tools::inference::applications::L2F<SPEC> executor;
    // Test Buffers
    #ifndef RL_TOOLS_DISABLE_TEST
    static CONFIG::POLICY_TEST::template Buffer<false> buffers_test;
    static CONFIG::POLICY_TEST::State<false> policy_state_test;
    static rl_tools::Tensor<rl_tools::tensor::Specification<TYPE_POLICY::DEFAULT, TI, rl_tools::tensor::Shape<TI, 1, OUTPUT_DIM>, false>> output;
    #endif
}



// Main functions (possibly with side effects)
void rl_tools_inference_applications_l2f_reset(){
    using namespace rl_tools::inference::applications::l2f;
    rl_tools::reset(device, executor, CONFIG::policy(), rng);
}
void rl_tools_inference_applications_l2f_init(){
    using namespace rl_tools::inference::applications::l2f;
    TI seed = 0;
    rl_tools::malloc(device, executor);
    rl_tools::init(device, rng, seed);
    rl_tools_inference_applications_l2f_reset();
}

const char* rl_tools_inference_applications_l2f_checkpoint_name(){
    return rl_tools::checkpoint::meta::name;
}

float rl_tools_inference_applications_l2f_test(RLtoolsInferenceApplicationsL2FAction* p_output){
    using namespace rl_tools::inference::applications::l2f;
#ifndef RL_TOOLS_DISABLE_TEST
    rl_tools::Mode<rl_tools::mode::Evaluation<>> mode;
    float acc = 0;
    uint64_t num_values = 0;
    for(TI batch_i = 0; batch_i < CONFIG::TEST_BATCH_SIZE_ACTUAL; batch_i++){
        rl_tools::reset(device, CONFIG::policy(), policy_state_test, rng);
        for(TI step_i = 0; step_i < CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL; step_i++){
            const auto step_input = rl_tools::view(device, rl_tools::checkpoint::example::input::container, step_i);
            const auto batch_input = rl_tools::view_range(device, step_input, batch_i, rl_tools::tensor::ViewSpec<0, 1>{});
            rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, batch_input), "input is nan");
            // rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, policy_state_test.content_state.next_content_state.state.state), "state is nan");
            rl_tools::evaluate_step(device, CONFIG::policy(), batch_input, policy_state_test, output, buffers_test, rng, mode);
            rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, output), "output is nan");
            for(TI action_i = 0; action_i < OUTPUT_DIM; action_i++){
                acc += rl_tools::math::abs(device.math, rl_tools::get(device, output, 0, action_i) - rl_tools::get(device, rl_tools::checkpoint::example::output::container, step_i, batch_i, action_i));
                num_values += 1;
                rl_tools::utils::assert_exit(device, !rl_tools::math::is_nan(device.math, acc), "output is nan");
                if(batch_i == 0 && step_i == CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL-1){
                    p_output->action[action_i] = rl_tools::get(device, output, 0, action_i);
                }
            }
        }
    }
    return acc / num_values;
#else
    return 0;
#endif
}

void rl_tools_inference_applications_l2f_set_force_sync_native(uint32_t force_sync_native){
    using namespace rl_tools::inference::applications::l2f;
    executor.executor.force_sync_native = force_sync_native;
    executor.executor.force_sync_native_initialized = true;
}


RLtoolsInferenceExecutorStatus rl_tools_inference_applications_l2f_control(RLtoolsInferenceTimestamp nanoseconds, RLtoolsInferenceApplicationsL2FObservation* c_observation, RLtoolsInferenceApplicationsL2FAction* c_action){
    using namespace rl_tools::inference::applications::l2f;
    static_assert(RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM == OUTPUT_DIM);
    rl_tools::inference::applications::l2f::Observation<SPEC> observation;
    for (TI dim_i = 0; dim_i < 3; dim_i++){
        observation.position[dim_i] = c_observation->position[dim_i];
        observation.orientation[dim_i] = c_observation->orientation[dim_i];
        observation.linear_velocity[dim_i] = c_observation->linear_velocity[dim_i];
        observation.angular_velocity[dim_i] = c_observation->angular_velocity[dim_i];
    }
    observation.orientation[3] = c_observation->orientation[3];
    for (TI action_i=0; action_i < OUTPUT_DIM; action_i++){
        observation.previous_action[action_i] = c_observation->previous_action[action_i];
    }
    rl_tools::inference::applications::l2f::Action<SPEC> action;
    auto status = rl_tools::control(device, executor, nanoseconds, CONFIG::policy(), observation, action, rng);
    for (TI action_i=0; action_i < OUTPUT_DIM; action_i++){
        c_action->action[action_i] = action.action[action_i];
    }
    return rl_tools::convert(status);
}
