// #define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#ifdef RL_TOOLS_ENABLE_TRACY
#include "Tracy.hpp"
#endif

#define MUX
#ifdef MUX
#include <rl_tools/operations/cpu_mux.h>
#else
#include <rl_tools/operations/cpu.h>
#endif
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#ifdef MUX
#include <rl_tools/nn/operations_cpu_mux.h>
#else
#include <rl_tools/nn/operations_cpu.h>
#endif
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/memory/operations_cpu.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/persist/backends/tar/operations_generic.h>

#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/checkpoint/operations_cpu.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

#ifdef MUX
using DEVICE = rlt::devices::DEVICE_FACTORY<>;
#else
using DEVICE = rlt::devices::DefaultCPU;
#endif
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

#include "parameters.h"

#include <numeric>


using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

//static_assert(!MEMORY || ENVIRONMENT::EPISODE_STEP_LIMIT >= SEQUENCE_LENGTH, "Environment episode step limit should be greater than or equal to sequence length");

int main(){
    TI seed = 1;
    DEVICE device;
    LOOP_STATE ts;
    ts.extrack_config.name = "sequential";
    ts.extrack_config.population_variates = "algorithm_environment";
    ts.extrack_config.population_values = "sac_memory";
    rlt::malloc(device);
    rlt::init(device);
    rlt::malloc(device, ts);
    rlt::init(device, ts, seed);
    DEVICE::SPEC::RANDOM::ENGINE<> myrng;
    rlt::init(device, myrng, seed);
#ifdef RL_TOOLS_ENABLE_TENSORBOARD
    rlt::init(device, device.logger, ts.extrack_paths.seed);
#endif
    bool done = false;
    while(!done){
#ifdef RL_TOOLS_ENABLE_TRACY
        FrameMark;
#endif
//        rlt::set_all(device, ts.actor_critic.critic_1.content.b_iz, -100);
//        rlt::set_all(device, ts.actor_critic.critic_target_1.content.b_iz, -100);
//        rlt::set_all(device, ts.actor_critic.critic_2.content.b_iz, -100);
//        rlt::set_all(device, ts.actor_critic.critic_target_2.content.b_iz, -100);
//        rlt::set_all(device, ts.actor_critic.actor.content.b_iz, -100);
        if(ts.step % 1000 == 0){
//            rlt::logging::tensorboard::print_topic_frequencies(device, device.logger);

            constexpr TI TEST_SEQUENCE_LENGTH = SEQUENCE_LENGTH;
            rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, TEST_SEQUENCE_LENGTH, 1, 2>>> test_critic_input;
            rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, TEST_SEQUENCE_LENGTH, 1, 1>>> test_critic_output;
            using EVALUATION_ACTOR = decltype(ts.actor_critic.actor)::CHANGE_BATCH_SIZE<TI, 1>;
            using EVALUATION_CRITIC = rlt::utils::typing::remove_reference_t<decltype(ts.actor_critic.critics[0])>::CHANGE_BATCH_SIZE<TI, 1>;
            EVALUATION_ACTOR::Buffer<1> actor_buffer;
            EVALUATION_CRITIC::Buffer<1> critic_buffer;
            rlt::malloc(device, test_critic_input);
            rlt::malloc(device, test_critic_output);
            rlt::malloc(device, actor_buffer);
            rlt::malloc(device, critic_buffer);
            auto test_actor_input = rlt::view_range(device, test_critic_input, 0, rlt::tensor::ViewSpec<2, 1>{});
            auto test_actor_output = rlt::view_range(device, test_critic_input, 1, rlt::tensor::ViewSpec<2, 1>{});
            constexpr TI N_EXAMPLES = 10;
            TI critic_correct_examples = 0;
            TI actor_correct_examples = 0;
            for(TI example_i = 0; example_i < N_EXAMPLES; example_i++){
                rlt::Mode<rlt::mode::Evaluation<>> mode;
                std::vector<TI> values;
                if(TEST_SEQUENCE_LENGTH >= 2){
                    for(TI seq_i = 0; seq_i < TEST_SEQUENCE_LENGTH-1; seq_i++){
                        TI value = rlt::random::uniform_real_distribution(device.random, (T)0, (T)1, myrng) < ENVIRONMENT_PARAMETERS::INPUT_PROBABILITY ? 1 : 0;
                        values.push_back(value);
                        while(values.size() > ENVIRONMENT_PARAMETERS::HORIZON){
                            values.erase(values.begin());
                        }
                        rlt::set(device, test_critic_input, (T)value, seq_i, 0, 0);
                    }
                }

//            rlt::Mode<rlt::nn::layers::gru::StepByStepMode<TI, rlt::mode::Evaluation>> mode;
//            mode.reset = true;
                while(values.size() > ENVIRONMENT_PARAMETERS::HORIZON-1){
                    values.erase(values.begin());
                }
                TI pre_count = std::accumulate(values.begin(), values.end(), 0);

                for(TI input_i = 0; input_i < 2; input_i++){
//                    TI input_i = real_input_i - 1;
                    rlt::set(device, test_critic_input, (T)input_i, TEST_SEQUENCE_LENGTH-1, 0, 0);
                    TI count = pre_count + input_i;
                    // line search
                    T max_value = 0;
                    bool max_value_set = false;
                    TI max_action = 0;
                    for(TI action_i = 0; action_i < 5; action_i++){
                        T action = ((T)action_i)/10;
                        rlt::set(device, test_critic_input, action, TEST_SEQUENCE_LENGTH-1, 0, 1);
//                    rlt::utils::assert_exit(device, rlt::get(device, test_critic_input, TEST_SEQUENCE_LENGTH-2, 0, 0) + rlt::get(device, test_critic_input, TEST_SEQUENCE_LENGTH-1, 0, 0) == count, "Count mismatch");
//                    rlt::print(device, test_critic_input);
                        rlt::evaluate(device, ts.actor_critic.actor, test_actor_input, test_actor_output, actor_buffer, myrng, mode); // to calculate the missing action
                        rlt::evaluate(device, ts.actor_critic.critics[0], test_critic_input, test_critic_output, critic_buffer, myrng, mode);
                        T value = rlt::get(device, test_critic_output, TEST_SEQUENCE_LENGTH-1, 0, 0);
                        if(!max_value_set || value > max_value){
                            max_value = value;
                            max_value_set = true;
                            max_action = action_i;
                        }
//                        std::cout << "Count " << count << " action " << action << " value: " << rlt::get(device, test_critic_output, TEST_SEQUENCE_LENGTH-1, 0, 0) << std::endl;
                    }
                    critic_correct_examples += max_action == count;
//                    std::cout << "Input " << input_i << " max_action " << max_action << (max_action == count ? " correct" : " incorrect") << std::endl;
                    rlt::evaluate(device, ts.actor_critic.actor, test_actor_input, test_actor_output, actor_buffer, myrng, mode);
                    bool actor_correct = round(rlt::get(device, test_actor_output, TEST_SEQUENCE_LENGTH-1, 0, 0) * 10) == count;
                    std::cout << "Count " << count << " actor_action " << rlt::get(device, test_actor_output, TEST_SEQUENCE_LENGTH-1, 0, 0) << (actor_correct ? " ✅" : " ❌") << std::endl;
                    actor_correct_examples += actor_correct;
                }
            }
            rlt::add_scalar(device, device.logger, "critic_evaluation_accuracy", critic_correct_examples / ((T)2*N_EXAMPLES));
            rlt::add_scalar(device, device.logger, "actor_evaluation_accuracy", actor_correct_examples / ((T)2*N_EXAMPLES));
            rlt::free(device, test_critic_input);
            rlt::free(device, test_critic_output);
            rlt::free(device, actor_buffer);
            rlt::free(device, critic_buffer);
        }
        done = rlt::step(device, ts);
    }
    return 0;
}
