#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_MODEL_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_MODEL_H

#include "../../nn/nn.h"
#include "../../nn_models/sequential/model.h"
#include "../../nn/parameters/parameters.h"
#include "../../nn/optimizers/sgd/sgd.h"
#include "../../nn/optimizers/adam/adam.h"
#include "../../utils/generic/typing.h"
#include "../../containers/matrix/matrix.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::multi_agent_wrapper {
    template <typename T_TYPE_POLICY, typename T_TI, T_TI T_N_AGENTS, typename T_MODULE>
    struct Configuration{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr TI N_AGENTS = T_N_AGENTS;
        using MODULE = T_MODULE;
    };

//    template <typename T_T, typename T_TI, T_TI T_N_AGENTS, typename T_MODULE>
//    using SpecificationFixedModule = Specification<T_T, T_TI, T_N_AGENTS, BindModule<T_MODULE>::template Module>;

    template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CAPABILITY, T_CONFIG{
        using TYPE_POLICY = typename T_CONFIG::TYPE_POLICY;
        using TI = typename T_CONFIG::TI;
        using CONFIG = T_CONFIG;
        using CAPABILITY = T_CAPABILITY;

        using PARAMETER_TYPE = typename CAPABILITY::PARAMETER_TYPE;

        static constexpr TI INPUT_DIM = get_last(T_INPUT_SHAPE{});
        static constexpr TI BATCH_AXIS = length(T_INPUT_SHAPE{})-2;
        static constexpr TI BATCH_SIZE = get<BATCH_AXIS>(T_INPUT_SHAPE{});
        static_assert(INPUT_DIM % CONFIG::N_AGENTS == 0, "The input dimension must be divisible by the number of agents");
        static constexpr TI PER_AGENT_INPUT_DIM = INPUT_DIM / CONFIG::N_AGENTS;
        static constexpr TI INTERNAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<T_INPUT_SHAPE>>{}) * CONFIG::N_AGENTS; // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
        using INPUT_SHAPE = T_INPUT_SHAPE;
        using INTERNAL_INPUT_SHAPE_TEMP = tensor::Replace<T_INPUT_SHAPE, PER_AGENT_INPUT_DIM, length(T_INPUT_SHAPE{})-1>;
        using INTERNAL_INPUT_SHAPE = tensor::Replace<INTERNAL_INPUT_SHAPE_TEMP, INTERNAL_BATCH_SIZE, BATCH_AXIS>;
        using MODEL = nn_models::sequential::Build<CAPABILITY, typename CONFIG::MODULE, INTERNAL_INPUT_SHAPE>;
        static constexpr TI PER_AGENT_OUTPUT_DIM = get_last(typename MODEL::OUTPUT_SHAPE{});
        static constexpr TI OUTPUT_DIM = PER_AGENT_OUTPUT_DIM * CONFIG::N_AGENTS;
        using OUTPUT_SHAPE = tensor::Replace<INPUT_SHAPE, OUTPUT_DIM, length(typename MODEL::INPUT_SHAPE{})-1>;
    };

    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION=true>
    struct ModuleStateSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };

    template<typename T_SPEC>
    struct ModuleState{
        using STATE_SPEC = T_SPEC;
        using SPEC = typename STATE_SPEC::SPEC;

        using INNER_STATE = typename SPEC::MODEL::template State<SPEC::DYNAMIC_ALLOCATION>;
        INNER_STATE inner_state;
    };

    template<typename T_SPEC, typename T_MODEL, bool T_DYNAMIC_ALLOCATION=true>
    struct ModuleBuffersSpecification{
        using SPEC = T_SPEC;
        using TI = typename SPEC::TI;
        static constexpr TI BATCH_SIZE = SPEC::BATCH_SIZE;
        using MODEL = T_MODEL;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };

    template<typename T_BUFFER_SPEC>
    struct ModuleBuffer{
        using BUFFER_SPEC = T_BUFFER_SPEC;
        using SPEC = typename BUFFER_SPEC::SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr TI BATCH_SIZE = T_BUFFER_SPEC::BATCH_SIZE;
        static constexpr TI INNER_BATCH_SIZE = BATCH_SIZE * SPEC::N_AGENTS;

        using T_INPUT = typename TYPE_POLICY::template GET<numeric_types::categories::Input>;
        using T_OUTPUT = typename TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;

        using INPUT_BUFFER_SPEC = tensor::Specification<T_INPUT, TI, tensor::Shape<TI, 1, BATCH_SIZE, SPEC::INPUT_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
        using INPUT_BUFFER_TYPE = Tensor<INPUT_BUFFER_SPEC>;
        using OUTPUT_BUFFER_SPEC = tensor::Specification<T_OUTPUT, TI, tensor::Shape<TI, 1, BATCH_SIZE, SPEC::OUTPUT_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
        using OUTPUT_BUFFER_TYPE = Tensor<OUTPUT_BUFFER_SPEC>;
        INPUT_BUFFER_TYPE input, d_input;
        OUTPUT_BUFFER_TYPE output;

        typename BUFFER_SPEC::MODEL::template Buffer<BUFFER_SPEC::DYNAMIC_ALLOCATION> buffer;
    };

    template<typename T_SPEC>
    struct ModuleForward{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        using MODEL = typename SPEC::MODEL;
        static constexpr TI N_AGENTS = SPEC::N_AGENTS;
        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        MODEL content;
        template <bool DYNAMIC_ALLOCATION=true>
        using State = ModuleState<ModuleStateSpecification<SPEC,  DYNAMIC_ALLOCATION>>;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = ModuleBuffer<ModuleBuffersSpecification<SPEC, MODEL, DYNAMIC_ALLOCATION>>;
    };

    template<typename SPEC>
    struct ModuleBackward: public ModuleForward<SPEC>{
//        static constexpr typename SPEC::TI BATCH_SIZE = SPEC::BATCH_SIZE;
    };
    template<typename T_SPEC>
    struct ModuleGradient: public ModuleBackward<T_SPEC>{
        using TI = typename T_SPEC::TI;
    };

    template <typename T_CAPABILITY, typename T_SPEC>
    using UpgradeCapabilityBatchSize = typename T_CAPABILITY::template CHANGE_BATCH_SIZE<T_CAPABILITY::BATCH_SIZE*T_SPEC::N_AGENTS>;

    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using Module =
            typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward, ModuleForward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
            typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward, ModuleBackward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
            typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient, ModuleGradient<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;


    template <typename CAPABILITY, typename CONFIG, typename INPUT_SHAPE>
    struct Build: Module<CONFIG, CAPABILITY, INPUT_SHAPE>{
        template <typename TI, TI BATCH_SIZE>
        struct CHANGE_BATCH_SIZE_IMPL{
            using NEW_INPUT_SHAPE = tensor::Replace<INPUT_SHAPE, BATCH_SIZE, 1>;
            using CHANGE_BATCH_SIZE = Build<CAPABILITY, CONFIG, NEW_INPUT_SHAPE>;
        };
        template <typename TI, TI BATCH_SIZE>
        using CHANGE_BATCH_SIZE = typename CHANGE_BATCH_SIZE_IMPL<TI, BATCH_SIZE>::CHANGE_BATCH_SIZE;
        template <typename NEW_CAPABILITY>
        using CHANGE_CAPABILITY = Build<NEW_CAPABILITY, CONFIG, INPUT_SHAPE>;
    };


}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
