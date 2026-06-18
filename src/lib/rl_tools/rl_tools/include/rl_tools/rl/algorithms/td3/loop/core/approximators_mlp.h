#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_APPROXIMATORS_MLP_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_APPROXIMATORS_MLP_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms::td3::loop::core{
    // The approximator config sets up any types that support the usual rl_tools::forward and rl_tools::backward operations (can be custom as well)
    // We provide approximators based on the sequential and mlp models. The latter (mlp) allows for a variable number of layers, but is restricted to a uniform hidden layer size while the former allows for arbitrary layers to be combined in a sequential manner. Both support compile-time autodiff
    template<typename TYPE_POLICY, typename TI, typename ENVIRONMENT, typename PARAMETERS, bool DYNAMIC_ALLOCATION>
    struct ConfigApproximatorsMLP{
        using TD3_PARAMETERS = typename PARAMETERS::TD3_PARAMETERS;
        template <typename CAPABILITY>
        struct ACTOR{
            using INPUT_SHAPE = tensor::Shape<TI, TD3_PARAMETERS::SEQUENCE_LENGTH, TD3_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
            using MLP_CONFIG = nn_models::mlp::Configuration<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM, PARAMETERS::ACTOR_NUM_LAYERS, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::ACTOR_ACTIVATION_FUNCTION, nn::activation_functions::ActivationFunction::TANH>;
            using MLP = nn_models::mlp::BindConfiguration<MLP_CONFIG>;
            struct SAMPLING_PARAMETERS: nn::layers::td3_sampling::DefaultParameters<TYPE_POLICY>{
                static constexpr typename TYPE_POLICY::DEFAULT STD = PARAMETERS::EXPLORATION_NOISE;
            };
            using SAMPLING_CONFIG = nn::layers::td3_sampling::Configuration<TYPE_POLICY, TI, SAMPLING_PARAMETERS>;
            using SAMPLING = nn::layers::td3_sampling::BindConfiguration<SAMPLING_CONFIG>;

            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

            using MODULE_CHAIN = Module<MLP, Module<SAMPLING>>;
            using MODEL = nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
        };

        template <typename CAPABILITY>
        struct CRITIC{
            static constexpr TI HIDDEN_DIM = PARAMETERS::CRITIC_HIDDEN_DIM;
            static constexpr auto ACTIVATION_FUNCTION = PARAMETERS::CRITIC_ACTIVATION_FUNCTION;

            using INPUT_SHAPE = tensor::Shape<TI, TD3_PARAMETERS::SEQUENCE_LENGTH, TD3_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::ObservationPrivileged::DIM + ENVIRONMENT::ACTION_DIM>;
            using MLP_CONFIG = nn_models::mlp::Configuration<TYPE_POLICY, TI, 1, PARAMETERS::ACTOR_NUM_LAYERS, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::ACTOR_ACTIVATION_FUNCTION, nn::activation_functions::ActivationFunction::IDENTITY>;
            using MLP = nn_models::mlp::BindConfiguration<MLP_CONFIG>;

            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

            using MODULE_CHAIN = Module<MLP>;
            using MODEL = nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
        };

        using OPTIMIZER_SPEC = nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION>;

        using OPTIMIZER = nn::optimizers::Adam<OPTIMIZER_SPEC>;

        using ACTOR_TYPE = typename ACTOR<nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>>::MODEL;
        using ACTOR_TARGET_TYPE = typename ACTOR<nn::capability::Forward<DYNAMIC_ALLOCATION>>::MODEL;
        using CRITIC_TYPE = typename CRITIC<nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>>::MODEL;
        using CRITIC_TARGET_TYPE = typename CRITIC<nn::capability::Forward<DYNAMIC_ALLOCATION>>::MODEL;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif