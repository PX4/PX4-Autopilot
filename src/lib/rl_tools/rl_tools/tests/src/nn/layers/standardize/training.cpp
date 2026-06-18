#define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>

#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/rl/environment_wrappers/scale_observations/operations_generic.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>

#include <gtest/gtest.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
using TI = typename DEVICE::index_t;
constexpr bool DYNAMIC_ALLOCATION = true;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;

namespace config{
    using namespace rlt;
    template <typename T_ENVIRONMENT>
    struct _LoopConfig{
        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, T_ENVIRONMENT>{
            static constexpr TI STEP_LIMIT = 20000;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 64;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 64;
        };
        template<typename T, typename TI, typename ENVIRONMENT, typename PARAMETERS, bool DYNAMIC_ALLOCATION>
        struct ConfigApproximatorsSequential{
            template <typename CAPABILITY>
            struct ACTOR{
                static constexpr TI HIDDEN_DIM = PARAMETERS::ACTOR_HIDDEN_DIM;
                static constexpr auto ACTIVATION_FUNCTION = PARAMETERS::ACTOR_ACTIVATION_FUNCTION;
                using INPUT_SHAPE = tensor::Shape<TI, 1, PARAMETERS::SAC_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
                using LAYER_0_SPEC = nn::layers::standardize::Configuration<T, TI>;
                using LAYER_0 = nn::layers::standardize::BindConfiguration<LAYER_0_SPEC>;
                using LAYER_1_SPEC = nn::layers::dense::Configuration<T, TI, HIDDEN_DIM, ACTIVATION_FUNCTION, typename PARAMETERS::INITIALIZER, nn::parameters::groups::Normal>;
                using LAYER_1 = nn::layers::dense::BindConfiguration<LAYER_1_SPEC>;
                using LAYER_2_SPEC = nn::layers::dense::Configuration<T, TI, HIDDEN_DIM, ACTIVATION_FUNCTION, typename PARAMETERS::INITIALIZER, nn::parameters::groups::Normal>;
                using LAYER_2 = nn::layers::dense::BindConfiguration<LAYER_2_SPEC>;
                static constexpr TI ACTOR_OUTPUT_DIM = ENVIRONMENT::ACTION_DIM * 2; // to express mean and log_std for each action
                using LAYER_3_SPEC = nn::layers::dense::Configuration<T, TI, ACTOR_OUTPUT_DIM, nn::activation_functions::ActivationFunction::IDENTITY, typename PARAMETERS::INITIALIZER, nn::parameters::groups::Output>; // note the output activation should be identity because we want to sample from a gaussian and then squash afterwards (taking into account the squashing in the distribution)
                using LAYER_3 = nn::layers::dense::BindConfiguration<LAYER_3_SPEC>;
                using SAC_PARAMETERS = typename PARAMETERS::SAC_PARAMETERS;
//                struct SAMPLE_AND_SQUASH_LAYER_PARAMETERS{
//                    static constexpr T LOG_STD_LOWER_BOUND = SAC_PARAMETERS::LOG_STD_LOWER_BOUND;
//                    static constexpr T LOG_STD_UPPER_BOUND = SAC_PARAMETERS::LOG_STD_UPPER_BOUND;
//                    static constexpr T LOG_PROBABILITY_EPSILON = SAC_PARAMETERS::LOG_PROBABILITY_EPSILON;
//                    static constexpr bool ADAPTIVE_ALPHA = SAC_PARAMETERS::ADAPTIVE_ALPHA;
//                    static constexpr T ALPHA = SAC_PARAMETERS::ALPHA;
//                    static constexpr T TARGET_ENTROPY = SAC_PARAMETERS::TARGET_ENTROPY;
//                };
                using SAMPLE_AND_SQUASH_LAYER_SPEC = nn::layers::sample_and_squash::Configuration<T, TI, nn::layers::sample_and_squash::DefaultParameters<T>>;
                using SAMPLE_AND_SQUASH_LAYER = nn::layers::sample_and_squash::BindConfiguration<SAMPLE_AND_SQUASH_LAYER_SPEC>;

                template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
                using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
                using MODULE_CHAIN = Module<LAYER_0, Module<LAYER_1, Module<LAYER_2, Module<LAYER_3, Module<SAMPLE_AND_SQUASH_LAYER>>>>>;

                using MODEL = nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
            };

            template <typename CAPABILITY>
            struct CRITIC{
                static constexpr TI HIDDEN_DIM = PARAMETERS::CRITIC_HIDDEN_DIM;
                static constexpr auto ACTIVATION_FUNCTION = PARAMETERS::CRITIC_ACTIVATION_FUNCTION;

                using INPUT_SHAPE = tensor::Shape<TI, 1, PARAMETERS::SAC_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
                using LAYER_1_SPEC = nn::layers::dense::Configuration<T, TI, HIDDEN_DIM, ACTIVATION_FUNCTION, typename PARAMETERS::INITIALIZER, nn::parameters::groups::Input>;
                using LAYER_1 = nn::layers::dense::BindConfiguration<LAYER_1_SPEC>;
                using LAYER_2_SPEC = nn::layers::dense::Configuration<T, TI, HIDDEN_DIM, ACTIVATION_FUNCTION, typename PARAMETERS::INITIALIZER, nn::parameters::groups::Normal>;
                using LAYER_2 = nn::layers::dense::BindConfiguration<LAYER_2_SPEC>;
                using LAYER_3_SPEC = nn::layers::dense::Configuration<T, TI, 1, nn::activation_functions::ActivationFunction::IDENTITY, typename PARAMETERS::INITIALIZER, nn::parameters::groups::Output>;
                using LAYER_3 = nn::layers::dense::BindConfiguration<LAYER_3_SPEC>;

                template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
                using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
                using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;

                using MODEL = nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
            };

            using ACTOR_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::ACTOR_OPTIMIZER_PARAMETERS>;
            using CRITIC_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::CRITIC_OPTIMIZER_PARAMETERS>;
            using ALPHA_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::ALPHA_OPTIMIZER_PARAMETERS>;

            using ACTOR_OPTIMIZER = nn::optimizers::Adam<ACTOR_OPTIMIZER_SPEC>;
            using CRITIC_OPTIMIZER = nn::optimizers::Adam<CRITIC_OPTIMIZER_SPEC>;
            using ALPHA_OPTIMIZER = nn::optimizers::Adam<ALPHA_OPTIMIZER_SPEC>;

            using ACTOR_TYPE = typename ACTOR<nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>>::MODEL;
            using CRITIC_TYPE = typename CRITIC<nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>>::MODEL;
            using CRITIC_TARGET_TYPE = typename CRITIC<nn::capability::Forward<DYNAMIC_ALLOCATION>>::MODEL;
        };
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, T_ENVIRONMENT, LOOP_CORE_PARAMETERS, ConfigApproximatorsSequential, DYNAMIC_ALLOCATION>;
        struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_CORE_CONFIG>{
            static constexpr TI NUM_EVALUATION_EPISODES = 100;
        };
        using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, LOOP_EVAL_PARAMETERS>;
        using LOOP_CONFIG = LOOP_EVAL_CONFIG;
    };
}

template <TI T_SCALE>
struct SCALE_OBSERVATIONS_WRAPPER_SPEC: rlt::rl::environment_wrappers::scale_observations::Specification<TYPE_POLICY, TI>{
    static constexpr T SCALE = T_SCALE;
};

template <TI SCALE, TI N_SEEDS=10>
T lifetime_return(){
    using WRAPPED_ENVIRONMENT = rlt::rl::environment_wrappers::ScaleObservations<SCALE_OBSERVATIONS_WRAPPER_SPEC<SCALE>, ENVIRONMENT>;
    using LOOP_CONFIG = typename config::_LoopConfig<WRAPPED_ENVIRONMENT>::LOOP_CONFIG;
    DEVICE device;
    typename LOOP_CONFIG::template State<LOOP_CONFIG> ts;

    T acc = 0;
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
    static constexpr TI N_SEEDS_ACTUAL = 1;
#else
    static constexpr TI N_SEEDS_ACTUAL = N_SEEDS;
#endif
    for(TI seed = 0; seed < N_SEEDS_ACTUAL; seed++){
        rlt::malloc(device);
        rlt::init(device);
        rlt::malloc(device, ts);
        rlt::init(device, ts, seed);
        while(!rlt::step(device, ts)){
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if(ts.step > 2000){
                break;
            }
#endif
            if(ts.step == 5000){
                std::cout << "steppin yourself > callbacks 'n' hooks: " << ts.step << std::endl;
            }
        }
        for(TI i = 0; i < LOOP_CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS; i++){
            acc += rlt::get(ts.evaluation_results, 0, i).returns_mean;
        }
        rlt::free(device);
        rlt::free(device, ts);
    }
    return acc / N_SEEDS;
}

TEST(RL_TOOLS_NN_LAYERS_STANDARDIZE, DETRIMENT_TRAINING) {
//    T return_1x = lifetime_return<1>();
    T return_10x = lifetime_return<2>();
//    T return_100x = lifetime_return<100>();
//    T return_1000x = lifetime_return<1000>();
    T return_10000x = lifetime_return<10000>();

//    std::cout << "return_1x: " << return_1x << std::endl;
    std::cout << "return_10x: " << return_10x << std::endl;
//    std::cout << "return_100x: " << return_100x << std::endl;
//    std::cout << "return_1000x: " << return_1000x << std::endl;
    std::cout << "return_10000x: " << return_10000x << std::endl;
#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    ASSERT_GT(return_10000x / return_10x, 3);
#endif
}

//TEST(RL_TOOLS_NN_LAYERS_STANDARDIZE, DETRIMENT_TRAINING_DEBUG) {
//    using LOOP_CONFIG = typename config::_LoopConfig<ENVIRONMENT>::LOOP_CONFIG;
//    DEVICE device;
//    auto rng = rlt::random::default_engine(DEVICE{}, 10);
//    typename LOOP_CONFIG::template State<LOOP_CONFIG> ts;
//    rlt::print(device, ts.actor_critic.actor);
//
//    constexpr TI INPUT_DIM = decltype(ts.actor_critic.actor)::INPUT_DIM;
//    constexpr TI OUTPUT_DIM = decltype(ts.actor_critic.actor)::OUTPUT_DIM;
//    auto& actor_buffer = ts.actor_buffers[0];
//    constexpr TI BATCH_SIZE = rlt::utils::typing::remove_reference<decltype(actor_buffer)>::type::BATCH_SIZE;
//    rlt::MatrixStatic<rlt::matrix::Specification<T, TI, BATCH_SIZE, INPUT_DIM>> input;
//    rlt::MatrixStatic<rlt::matrix::Specification<T, TI, BATCH_SIZE, OUTPUT_DIM>> output, d_output;
//
//    rlt::malloc(device, ts);
//    rlt::init(device, ts, 0);
//
//    rlt::forward(device, ts.actor_critic.actor, input, output, actor_buffer, rng);
//    rlt::backward(device, ts.actor_critic.actor, input, d_output, actor_buffer);
//
//}
