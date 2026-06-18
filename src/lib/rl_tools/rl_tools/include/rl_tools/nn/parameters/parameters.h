#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_PARAMETERS_PARAMETERS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_PARAMETERS_PARAMETERS_H

#include "../../numeric_types/categories.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::parameters{
    namespace mode{
        template <typename T_BASE, typename T_SPEC = bool>
        struct ParametersOnly: T_BASE{
            using BASE = T_BASE;
            using SPEC = T_SPEC;
        };
        template <typename T_BASE, typename T_SPEC = bool>
        struct GradientOnly: T_BASE{
            using BASE = T_BASE;
            using SPEC = T_SPEC;
        };
    }
    namespace groups{
        struct Normal{};
        struct Input{};
        struct Output{};
    }

    namespace categories{
        struct Weights{};
        struct Biases{};
        struct Constant{};
    }

    struct Plain{
        // todo: evaluate replacing the instance mechanism with a tag similar to the container type tags
        template <typename T_TYPE_POLICY, typename T_TI, typename T_SHAPE, typename T_GROUP_TAG, typename T_CATEGORY_TAG, bool T_DYNAMIC_ALLOCATION, bool T_CONST=false>
        struct Specification{
            using TYPE_POLICY = T_TYPE_POLICY;
            using TI = T_TI;
            using SHAPE = T_SHAPE;
            using GROUP_TAG = T_GROUP_TAG;
            using CATEGORY_TAG = T_CATEGORY_TAG;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            static constexpr bool CONST = T_CONST;
        };
        template <typename T_SPEC>
        struct Instance{
            using SPEC = T_SPEC;
            using T_PARAMETER = typename T_SPEC::TYPE_POLICY::template GET<numeric_types::categories::Parameter>;
            using TENSOR_SPEC = tensor::Specification<T_PARAMETER, typename SPEC::TI, typename SPEC::SHAPE, SPEC::DYNAMIC_ALLOCATION, tensor::RowMajorStride<typename SPEC::SHAPE>, SPEC::CONST>;
            Tensor<TENSOR_SPEC> parameters;
        };
    };
    struct Gradient{
        template <typename T_TYPE_POLICY, typename T_TI, typename T_SHAPE, typename T_GROUP_TAG, typename T_CATEGORY_TAG, bool T_DYNAMIC_ALLOCATION, bool T_CONST=false>
        struct Specification{
            using TYPE_POLICY = T_TYPE_POLICY;
            using TI = T_TI;
            using SHAPE = T_SHAPE;
            using GROUP_TAG = T_GROUP_TAG;
            using CATEGORY_TAG = T_CATEGORY_TAG;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            static constexpr bool CONST = T_CONST;
        };
        template <typename T_SPEC>
        struct Instance: Plain::Instance<T_SPEC>{
            using PARENT = Plain::Instance<T_SPEC>;
            using SPEC = T_SPEC;
            using T_GRADIENT = typename T_SPEC::TYPE_POLICY::template GET<numeric_types::categories::Gradient>;
            using TENSOR_SPEC = tensor::Specification<T_GRADIENT, typename SPEC::TI, typename SPEC::SHAPE, SPEC::DYNAMIC_ALLOCATION, tensor::RowMajorStride<typename SPEC::SHAPE>, SPEC::CONST>;
            Tensor<TENSOR_SPEC> gradient;
        };
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
