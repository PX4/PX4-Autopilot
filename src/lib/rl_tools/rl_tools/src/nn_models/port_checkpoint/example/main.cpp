using T = float;
constexpr unsigned SEQUENCE_LENGTH = 5;
constexpr unsigned BATCH_SIZE = 6;
constexpr unsigned INPUT_DIM = 22;
constexpr unsigned HIDDEN_DIM = 16;
constexpr unsigned ACTION_DIM = 4;
constexpr unsigned SEED = 0;
constexpr bool DYNAMIC_ALLOCATION = true;

#define RL_TOOLS_NAMESPACE_WRAPPER base
#include "base/include/rl_tools/operations/cpu.h"
#include "base/include/rl_tools/nn/layers/gru/operations_generic.h"
#include "base/include/rl_tools/nn/layers/dense/operations_generic.h"
#include "base/include/rl_tools/nn_models/sequential/operations_generic.h"

#include "base/include/rl_tools/nn/layers/gru/persist.h"
#include "base/include/rl_tools/nn/layers/dense/persist.h"
#include "base/include/rl_tools/nn_models/sequential/persist.h"

namespace base{
    namespace rlt = rl_tools;
    using DEVICE = rl_tools::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
    using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

    using INPUT_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<T, TI, HIDDEN_DIM, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::layers::dense::DefaultInitializer<T, TI>, rlt::nn::parameters::groups::Input>;
    using INPUT_LAYER = rlt::nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<T, TI, HIDDEN_DIM, rlt::nn::parameters::groups::Normal>;
    using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
    using OUTPUT_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<T, TI, ACTION_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<T, TI>, rlt::nn::parameters::groups::Output>;
    using OUTPUT_LAYER = rlt::nn::layers::dense::BindConfiguration<OUTPUT_LAYER_CONFIG>;
    using MODULE_CHAIN = Module<INPUT_LAYER, Module<GRU, Module<OUTPUT_LAYER>>>;
    using CAPABILITY = rlt::nn::capability::Forward<DYNAMIC_ALLOCATION>;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    using POLICY = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
}

#undef RL_TOOLS_NAMESPACE_WRAPPER
#define RL_TOOLS_NAMESPACE_WRAPPER target
#define RL_TOOLS_NAMESPACE RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools
#define RL_TOOLS_NAMESPACE_WRAPPER_START namespace RL_TOOLS_NAMESPACE_WRAPPER {
#define RL_TOOLS_NAMESPACE_WRAPPER_END }
#define RL_TOOLS_DISABLE_INCLUDE_GUARDS
#define RL_TOOLS_CONTAINERS_TENSOR_COPY_FROM_GENERIC_DISABLE_SHAPE_CHECK

#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

namespace target{
    namespace rlt = rl_tools;
    using DEVICE = rl_tools::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
    using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

    using INPUT_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Input>;
    using INPUT_LAYER = rlt::nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::parameters::groups::Normal>;
    using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
    using OUTPUT_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, ACTION_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Output>;
    using OUTPUT_LAYER = rlt::nn::layers::dense::BindConfiguration<OUTPUT_LAYER_CONFIG>;
    using MODULE_CHAIN = Module<INPUT_LAYER, Module<GRU, Module<OUTPUT_LAYER>>>;
    using CAPABILITY = rlt::nn::capability::Forward<DYNAMIC_ALLOCATION>;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    using POLICY = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
}





int main(){
    base::DEVICE base_device;
    base::DEVICE::SPEC::RANDOM::ENGINE<> base_rng;
    base::POLICY base_policy;
    base::POLICY::Buffer<DYNAMIC_ALLOCATION> base_policy_buffer;
    base::rlt::Tensor<base::rlt::tensor::Specification<T, base::TI, typename base::POLICY::INPUT_SHAPE>> base_input;
    base::rlt::Tensor<base::rlt::tensor::Specification<T, base::TI, typename base::POLICY::OUTPUT_SHAPE>> base_output;
    base::rlt::malloc(base_device);
    base::rlt::malloc(base_device, base_rng);
    base::rlt::malloc(base_device, base_policy);
    base::rlt::malloc(base_device, base_policy_buffer);
    base::rlt::malloc(base_device, base_input);
    base::rlt::malloc(base_device, base_output);
    base::rlt::init(base_device);
    base::rlt::init(base_device, base_rng, SEED + 1);
    base::rlt::init_weights(base_device, base_policy, base_rng);
    base::rlt::randn(base_device, base_input, base_rng);
    base::rlt::evaluate(base_device, base_policy, base_input, base_output, base_policy_buffer, base_rng);
    base::rlt::print(base_device, base_output);

    target::DEVICE target_device;
    target::DEVICE::SPEC::RANDOM::ENGINE<> target_rng;
    target::POLICY target_policy;
    target::POLICY::Buffer<DYNAMIC_ALLOCATION> target_policy_buffer;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::INPUT_SHAPE>> target_input;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::OUTPUT_SHAPE>> target_output;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::OUTPUT_SHAPE>> target_base_output;
    target::rlt::malloc(target_device);
    target::rlt::malloc(target_device, target_rng);
    target::rlt::malloc(target_device, target_policy);
    target::rlt::malloc(target_device, target_policy_buffer);
    target::rlt::malloc(target_device, target_input);
    target::rlt::malloc(target_device, target_output);
    target::rlt::malloc(target_device, target_base_output);
    target::rlt::init(target_device);
    target::rlt::init(target_device, target_rng, SEED);
    target::rlt::init_weights(target_device, target_policy, target_rng);
    target::rlt::randn(target_device, target_input, target_rng);

    {
        target::rlt::evaluate(target_device, target_policy, target_input, target_output, target_policy_buffer, target_rng);
        // target::rlt::print(target_device, target_output);
        target::rlt::copy_from_generic(target_device, base_device, base_output, target_base_output);

        T abs_diff = target::rlt::abs_diff(target_device, target_base_output, target_output);

        std::cout << "Difference pre copy: " << abs_diff << std::endl;
    }

    target::rlt::copy_from_generic(base_device, target_device, base_input, target_input);
    target::rlt::copy_from_generic(base_device, target_device, base_policy, target_policy);

    {
        target::rlt::evaluate(target_device, target_policy, target_input, target_output, target_policy_buffer, target_rng);
        // target::rlt::print(target_device, target_output);
        target::rlt::copy_from_generic(target_device, base_device, base_output, target_base_output);

        T abs_diff = target::rlt::abs_diff(target_device, target_base_output, target_output);

        std::cout << "Difference post copy: " << abs_diff << std::endl;
    }



    return 0;

}