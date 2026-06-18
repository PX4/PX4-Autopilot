#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_APPROXIMATORS_GRU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_APPROXIMATORS_GRU_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms::td3::loop::core{
    template<typename T, typename TI, typename ENVIRONMENT, typename PARAMETERS, bool DYNAMIC_ALLOCATION>
    struct ConfigApproximatorsGRU{
    //    static constexpr bool USE_GRU = true;
        using TD3_PARAMETERS = typename PARAMETERS::TD3_PARAMETERS;
        template <typename CAPABILITY>
        struct Actor{
            using INPUT_SHAPE = tensor::Shape<TI, TD3_PARAMETERS::SEQUENCE_LENGTH, TD3_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
            using INPUT_LAYER_CONFIG = nn::layers::dense::Configuration<T, TI, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<T, TI>, nn::parameters::groups::Input>;
            using INPUT_LAYER = nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
            using GRU_CONFIG = nn::layers::gru::Configuration<T, TI, PARAMETERS::ACTOR_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU = nn::layers::gru::BindConfiguration<GRU_CONFIG>;
            using GRU2_CONFIG = nn::layers::gru::Configuration<T, TI, PARAMETERS::ACTOR_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU2 = nn::layers::gru::BindConfiguration<GRU2_CONFIG>;
            using DENSE_LAYER_CONFIG = nn::layers::dense::Configuration<T, TI, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::ACTOR_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<T, TI>, nn::parameters::groups::Normal>;
            using DENSE_LAYER = nn::layers::dense::BindConfiguration<DENSE_LAYER_CONFIG>;
            using OUTPUT_CONFIG = nn::layers::dense::Configuration<T, TI, ENVIRONMENT::ACTION_DIM, nn::activation_functions::ActivationFunction::IDENTITY, nn::layers::dense::DefaultInitializer<T, TI>, nn::parameters::groups::Output>;
            using OUTPUT = nn::layers::dense::BindConfiguration<OUTPUT_CONFIG>;

            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

            using MODULE_GRU = Module<GRU, Module<OUTPUT>>;
            using MODULE_GRU_TWO_LAYER = Module<GRU, Module<GRU2, Module<OUTPUT>>>;
            using MODULE_GRU_THREE_LAYER = Module<GRU, Module<GRU2, Module<DENSE_LAYER, Module<OUTPUT>>>>;

            using SELECTED_MODULE = rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 3, MODULE_GRU, rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 4, MODULE_GRU_TWO_LAYER, MODULE_GRU_THREE_LAYER>>;
            static_assert(PARAMETERS::CRITIC_NUM_LAYERS == 3 || PARAMETERS::CRITIC_NUM_LAYERS == 4 || PARAMETERS::CRITIC_NUM_LAYERS == 5, "Only 3/4/5 layers (1 input + 1/2 GRU + 1/2 Output) are supported right now");
            using MODEL = nn_models::sequential::Build<CAPABILITY, SELECTED_MODULE, INPUT_SHAPE>;
        };
        template <typename CAPABILITY>
        struct Critic{
            using INPUT_SHAPE = tensor::Shape<TI, TD3_PARAMETERS::SEQUENCE_LENGTH, TD3_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::ObservationPrivileged::DIM + ENVIRONMENT::ACTION_DIM>;
            using INPUT_LAYER_CONFIG = nn::layers::dense::Configuration<T, TI, PARAMETERS::CRITIC_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<T, TI>, nn::parameters::groups::Input>;
            using INPUT_LAYER = nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
            using GRU_CONFIG = nn::layers::gru::Configuration<T, TI, PARAMETERS::CRITIC_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU = nn::layers::gru::BindConfiguration<GRU_CONFIG>;
            using GRU2_CONFIG = nn::layers::gru::Configuration<T, TI, PARAMETERS::CRITIC_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU2 = nn::layers::gru::BindConfiguration<GRU2_CONFIG>;
            using DENSE_LAYER_CONFIG = nn::layers::dense::Configuration<T, TI, PARAMETERS::CRITIC_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<T, TI>, nn::parameters::groups::Normal>;
            using DENSE_LAYER = nn::layers::dense::BindConfiguration<DENSE_LAYER_CONFIG>;
            using OUTPUT_CONFIG = nn::layers::dense::Configuration<T, TI, 1, nn::activation_functions::ActivationFunction::IDENTITY, nn::layers::dense::DefaultInitializer<T, TI>, nn::parameters::groups::Output>;
            using OUTPUT = nn::layers::dense::BindConfiguration<OUTPUT_CONFIG>;
            static constexpr TI INPUT_DIM = ENVIRONMENT::ObservationPrivileged::DIM+ENVIRONMENT::ACTION_DIM;

            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

            using MODULE_GRU = Module<GRU, Module<OUTPUT>>;
            using MODULE_GRU_TWO_LAYER = Module<GRU, Module<GRU2, Module<OUTPUT>>>;
            using MODULE_GRU_THREE_LAYER = Module<GRU, Module<GRU2, Module<DENSE_LAYER, Module<OUTPUT>>>>;

            using SELECTED_MODULE = rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 3, MODULE_GRU, rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 4, MODULE_GRU_TWO_LAYER, MODULE_GRU_THREE_LAYER>>;
            static_assert(PARAMETERS::CRITIC_NUM_LAYERS == 3 || PARAMETERS::CRITIC_NUM_LAYERS == 4 || PARAMETERS::CRITIC_NUM_LAYERS == 5, "Only 3/4/5 layers (1 input + 1/2 GRU + 1/2 Output) are supported right now");
            using MODEL = nn_models::sequential::Build<CAPABILITY, SELECTED_MODULE, INPUT_SHAPE>;
        };

        using CAPABILITY_ACTOR = nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>;
        using CAPABILITY_CRITIC = nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>;
        using ACTOR_TYPE = typename Actor<CAPABILITY_ACTOR>::MODEL;
        using CRITIC_TYPE = typename Critic<CAPABILITY_CRITIC>::MODEL;
        using CRITIC_TARGET_TYPE = typename Critic<nn::capability::Forward<>>::MODEL;
        using ACTOR_TARGET_TYPE = typename Actor<nn::capability::Forward<>>::MODEL;
        using OPTIMIZER_SPEC = nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION>;
        using OPTIMIZER = nn::optimizers::Adam<OPTIMIZER_SPEC>;

    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif


