#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_RANDOM_UNIFORM_MODEL_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_RANDOM_UNIFORM_MODEL_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::random_uniform{
    enum class Range{
        MINUS_ONE_TO_ONE,
        ZERO_TO_ONE
    };
    template <typename T_TYPE_POLICY, typename T_TI, T_TI T_INPUT_DIM, T_TI T_OUTPUT_DIM, Range T_RANGE>
    struct Specification{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr T_TI INPUT_DIM = T_INPUT_DIM;
        static constexpr T_TI OUTPUT_DIM = T_OUTPUT_DIM;
        static constexpr Range RANGE = T_RANGE;
    };
    struct State{};
    struct Buffer{};
}
RL_TOOLS_NAMESPACE_WRAPPER_END
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models{
    template <typename T_SPEC>
    struct RandomUniform{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr TI INPUT_DIM = SPEC::INPUT_DIM;
        static constexpr TI OUTPUT_DIM = SPEC::OUTPUT_DIM;
        using INPUT_SHAPE = tensor::Shape<TI, 0, 0, INPUT_DIM>;
        using OUTPUT_SHAPE = tensor::Shape<TI, 0, 0, OUTPUT_DIM>;

        template <bool DYNAMIC_ALLOCATION=true>
        using State = random_uniform::State;
        template <TI BATCH_SIZE=1>
        using Buffer = typename random_uniform::Buffer;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
