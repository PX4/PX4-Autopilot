#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MLP_UNCONDITIONAL_STDDEV_NETWORK_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MLP_UNCONDITIONAL_STDDEV_NETWORK_H

#include "../../nn_models/mlp/network.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::mlp_unconditional_stddev {

    template <typename T_SPEC, template <typename> typename T_BASE = nn_models::mlp::NeuralNetworkForward>
    struct NeuralNetworkForward: T_BASE<T_SPEC>{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        // using LOG_STD_CONTAINER_SPEC = matrix::Specification<typename TYPE_POLICY::DEFAULT, TI, 1, SPEC::OUTPUT_DIM, SPEC::DYNAMIC_ALLOCATION>;
        // using LOG_STD_CONTAINER_TYPE = Matrix<LOG_STD_CONTAINER_SPEC>;
        using LOG_STD_CONTAINER_SHAPE = tensor::Shape<TI, SPEC::OUTPUT_DIM>;
        using LOG_STD_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, LOG_STD_CONTAINER_SHAPE, nn::parameters::groups::Output, nn::parameters::categories::Weights, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<LOG_STD_PARAMETER_SPEC> log_std;
        template <typename TT_SPEC>
        using BASE = T_BASE<TT_SPEC>;
    };
    template <typename SPEC, template <typename> typename BASE = nn_models::mlp::NeuralNetworkBackward>
    struct NeuralNetworkBackward: NeuralNetworkForward<SPEC, BASE>{};
    template <typename SPEC, template <typename> typename BASE = nn_models::mlp::NeuralNetworkGradient>
    struct NeuralNetworkGradient: NeuralNetworkBackward<SPEC, BASE>{};

    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using NeuralNetwork =
    typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward, NeuralNetworkForward<nn_models::mlp::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
    typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward, NeuralNetworkBackward<nn_models::mlp::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
    typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient, NeuralNetworkGradient<nn_models::mlp::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

    template <typename CONFIG>
    struct BindConfiguration{
        template <typename CAPABILITY, typename INPUT_SHAPE>
        using Layer = nn_models::mlp_unconditional_stddev::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    };


}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
