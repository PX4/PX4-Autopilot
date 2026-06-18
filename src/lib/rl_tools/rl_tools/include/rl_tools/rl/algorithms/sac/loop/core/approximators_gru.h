#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_SAC_LOOP_CORE_APPROXIMATORS_GRU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_SAC_LOOP_CORE_APPROXIMATORS_GRU_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms::sac::loop::core{
    template<typename TYPE_POLICY, typename TI, typename ENVIRONMENT, typename PARAMETERS, bool DYNAMIC_ALLOCATION=true>
    struct ConfigApproximatorsGRU{
        static constexpr bool USE_GRU = true;
        using SAC_PARAMETERS = typename PARAMETERS::SAC_PARAMETERS;
        template <typename CAPABILITY>
        struct Actor{
            using INPUT_SHAPE = tensor::Shape<TI, SAC_PARAMETERS::SEQUENCE_LENGTH, PARAMETERS::SAC_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
            using INPUT_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, nn::parameters::groups::Input>;
            using INPUT_LAYER = nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
            using GRU_SPEC = nn::layers::gru::Configuration<TYPE_POLICY, TI, PARAMETERS::ACTOR_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU = nn::layers::gru::BindConfiguration<GRU_SPEC>;
            using GRU2_SPEC = nn::layers::gru::Configuration<TYPE_POLICY, TI, PARAMETERS::ACTOR_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU2 = nn::layers::gru::BindConfiguration<GRU2_SPEC>;
            using DENSE_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::ACTOR_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, nn::parameters::groups::Normal>;
            using DENSE_LAYER = nn::layers::dense::BindConfiguration<DENSE_LAYER_CONFIG>;
            using OUTPUT_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, 2*ENVIRONMENT::ACTION_DIM, nn::activation_functions::ActivationFunction::IDENTITY, nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, nn::parameters::groups::Output>;
            using OUTPUT_LAYER = nn::layers::dense::BindConfiguration<OUTPUT_LAYER_CONFIG>;
            struct SAMPLE_AND_SQUASH_LAYER_PARAMETERS{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr T LOG_STD_LOWER_BOUND = SAC_PARAMETERS::LOG_STD_LOWER_BOUND;
                static constexpr T LOG_STD_UPPER_BOUND = SAC_PARAMETERS::LOG_STD_UPPER_BOUND;
                static constexpr T LOG_PROBABILITY_EPSILON = SAC_PARAMETERS::LOG_PROBABILITY_EPSILON;
                static constexpr bool ADAPTIVE_ALPHA = SAC_PARAMETERS::ADAPTIVE_ALPHA;
                static constexpr bool UPDATE_ALPHA_WITH_ACTOR = false;
                static constexpr T ALPHA = SAC_PARAMETERS::ALPHA;
                static constexpr T TARGET_ENTROPY = SAC_PARAMETERS::TARGET_ENTROPY;
            };
            using SAMPLE_AND_SQUASH_LAYER_SPEC = nn::layers::sample_and_squash::Configuration<TYPE_POLICY, TI, SAMPLE_AND_SQUASH_LAYER_PARAMETERS>;
            using SAMPLE_AND_SQUASH_LAYER = nn::layers::sample_and_squash::BindConfiguration<SAMPLE_AND_SQUASH_LAYER_SPEC>;
            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
            using SAMPLE_AND_SQUASH_MODULE = Module<SAMPLE_AND_SQUASH_LAYER>;
            using MODULE_GRU = Module<INPUT_LAYER, Module<GRU, Module<OUTPUT_LAYER, SAMPLE_AND_SQUASH_MODULE>>>;
            using MODULE_GRU_TWO_LAYER = Module<INPUT_LAYER, Module<GRU, Module<GRU2, Module<OUTPUT_LAYER, SAMPLE_AND_SQUASH_MODULE>>>>;
            using MODULE_GRU_THREE_LAYER = Module<INPUT_LAYER, Module<GRU, Module<GRU2, Module<DENSE_LAYER, Module<OUTPUT_LAYER, SAMPLE_AND_SQUASH_MODULE>>>>>;
            using SELECTED_MODULE = rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 3, MODULE_GRU, rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 4, MODULE_GRU_TWO_LAYER, MODULE_GRU_THREE_LAYER>>;
            static_assert(PARAMETERS::CRITIC_NUM_LAYERS == 3 || PARAMETERS::CRITIC_NUM_LAYERS == 4 || PARAMETERS::CRITIC_NUM_LAYERS == 5, "Only 3/4/5 layers (1 input + 1/2 GRU + 1/2 Output) are supported right now");
            using MODEL = nn_models::sequential::Build<CAPABILITY, SELECTED_MODULE, INPUT_SHAPE>;
        };
        template <typename CAPABILITY>
        struct Critic{
            static constexpr TI INPUT_DIM = ENVIRONMENT::ObservationPrivileged::DIM+ENVIRONMENT::ACTION_DIM;
            using INPUT_SHAPE = tensor::Shape<TI, SAC_PARAMETERS::SEQUENCE_LENGTH, PARAMETERS::SAC_PARAMETERS::CRITIC_BATCH_SIZE, INPUT_DIM>;
            using INPUT_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMETERS::CRITIC_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, nn::parameters::groups::Input>;
            using INPUT_LAYER = nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
            using GRU_SPEC = nn::layers::gru::Configuration<TYPE_POLICY, TI, PARAMETERS::CRITIC_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU = nn::layers::gru::BindConfiguration<GRU_SPEC>;
            using GRU2_SPEC = nn::layers::gru::Configuration<TYPE_POLICY, TI, PARAMETERS::CRITIC_HIDDEN_DIM, nn::parameters::groups::Normal, true>;
            using GRU2 = nn::layers::gru::BindConfiguration<GRU2_SPEC>;
            using DENSE_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMETERS::CRITIC_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, nn::parameters::groups::Normal>;
            using DENSE_LAYER = nn::layers::dense::BindConfiguration<DENSE_LAYER_CONFIG>;
            using OUTPUT_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, 1, nn::activation_functions::ActivationFunction::IDENTITY, nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, nn::parameters::groups::Output>;
            using OUTPUT_LAYER = nn::layers::dense::BindConfiguration<OUTPUT_LAYER_CONFIG>;
            template <typename T_CONTENT, typename T_NEXT_MODULE = nn_models::sequential::OutputModule>
            using Module = typename nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
            using MODULE_GRU = Module<INPUT_LAYER, Module<GRU, Module<OUTPUT_LAYER>>>;
            using MODULE_GRU_TWO_LAYER = Module<INPUT_LAYER, Module<GRU, Module<GRU2, Module<OUTPUT_LAYER>>>>;
            using MODULE_GRU_THREE_LAYER = Module<INPUT_LAYER, Module<GRU, Module<GRU2, Module<DENSE_LAYER, Module<OUTPUT_LAYER>>>>>;
            using SELECTED_MODULE = rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 3, MODULE_GRU, rl_tools::utils::typing::conditional_t<PARAMETERS::CRITIC_NUM_LAYERS == 4, MODULE_GRU_TWO_LAYER, MODULE_GRU_THREE_LAYER>>;
            static_assert(PARAMETERS::CRITIC_NUM_LAYERS == 3 || PARAMETERS::CRITIC_NUM_LAYERS == 4 || PARAMETERS::CRITIC_NUM_LAYERS == 5, "Only 3/4/5 layers (1 input + 1/2 GRU + 1/2 Output) are supported right now");
            using MODEL = nn_models::sequential::Build<CAPABILITY, SELECTED_MODULE, INPUT_SHAPE>;
        };

        using CAPABILITY_ACTOR = nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>;
        using CAPABILITY_CRITIC = nn::capability::Gradient<nn::parameters::Adam, DYNAMIC_ALLOCATION>;
        using ACTOR_TYPE = typename Actor<CAPABILITY_ACTOR>::MODEL;
        using CRITIC_TYPE = typename Critic<CAPABILITY_CRITIC>::MODEL;
        using CRITIC_TARGET_TYPE = typename Critic<nn::capability::Forward<DYNAMIC_ALLOCATION>>::MODEL;
        using ACTOR_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::ACTOR_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION>;
        using CRITIC_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::CRITIC_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION>;
        using ALPHA_OPTIMIZER_SPEC = nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::ALPHA_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION>;
        using ACTOR_OPTIMIZER = nn::optimizers::Adam<ACTOR_OPTIMIZER_SPEC>;
        using CRITIC_OPTIMIZER = nn::optimizers::Adam<CRITIC_OPTIMIZER_SPEC>;
        using ALPHA_OPTIMIZER = nn::optimizers::Adam<ALPHA_OPTIMIZER_SPEC>;

    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

