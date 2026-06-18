
#define RL_TOOLS_NAMESPACE_WRAPPER base
#include <unistd.h>

#include "base/include/rl_tools/operations/cpu.h"
#include "base/include/rl_tools/nn/layers/gru/operations_generic.h"
#include "base/include/rl_tools/nn/layers/dense/operations_generic.h"
#include "base/include/rl_tools/nn_models/sequential/operations_generic.h"

#include "raptor_policy/policy.h"

using T = float;
constexpr unsigned SEQUENCE_LENGTH = rl_tools::checkpoint::example::input::SHAPE::FIRST;
constexpr unsigned BATCH_SIZE = rl_tools::checkpoint::example::input::SHAPE::template GET<1>;
constexpr unsigned INPUT_DIM = rl_tools::checkpoint::example::input::SHAPE::LAST;
constexpr unsigned HIDDEN_DIM = rl_tools::checkpoint::actor::layer_1::TYPE::SPEC::HIDDEN_DIM;
constexpr unsigned ACTION_DIM = rl_tools::checkpoint::actor::TYPE::OUTPUT_SHAPE::LAST;
constexpr unsigned SEED = 0;
constexpr bool DYNAMIC_ALLOCATION = true;

namespace base{
    namespace rlt = rl_tools;
    using DEVICE = rl_tools::devices::DefaultCPU;
    using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
    using TI = typename DEVICE::index_t;
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

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/gru/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn_models/sequential/persist_code.h>

#include <rl_tools/persist/backends/tar/operations_cpu.h>
#ifdef RL_TOOLS_ENABLE_HDF5
#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#endif
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>



#include <fstream>
#include <filesystem>

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
    base::RNG base_rng;
    rl_tools::checkpoint::actor::TYPE::Buffer<DYNAMIC_ALLOCATION> base_policy_buffer;
    base::rlt::Tensor<base::rlt::tensor::Specification<T, base::TI, rl_tools::checkpoint::actor::TYPE::OUTPUT_SHAPE>> base_output;
    base::rlt::malloc(base_device);
    base::rlt::malloc(base_device, base_rng);
    base::rlt::malloc(base_device, base_policy_buffer);
    base::rlt::malloc(base_device, base_output);
    base::rlt::init(base_device);
    base::rlt::init(base_device, base_rng, SEED);
    base::rlt::evaluate(base_device, rl_tools::checkpoint::actor::module, rl_tools::checkpoint::example::input::container, base_output, base_policy_buffer, base_rng);
    {
        T abs_diff = base::rlt::abs_diff(base_device, rl_tools::checkpoint::example::output::container, base_output) / decltype(base_output)::SPEC::SIZE;
        std::cout << "Difference base <-> orig: " << abs_diff << std::endl;
    }


    target::DEVICE target_device;
    target::DEVICE::SPEC::RANDOM::ENGINE<> target_rng;
    target::POLICY target_policy;
    target::POLICY::Buffer<DYNAMIC_ALLOCATION> target_policy_buffer;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::INPUT_SHAPE>> target_input;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::OUTPUT_SHAPE>> target_output;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::OUTPUT_SHAPE>> target_original_output;
    target::rlt::Tensor<target::rlt::tensor::Specification<T, target::TI, typename target::POLICY::OUTPUT_SHAPE>> target_base_output;
    target::rlt::malloc(target_device);
    target::rlt::malloc(target_device, target_rng);
    target::rlt::malloc(target_device, target_policy);
    target::rlt::malloc(target_device, target_policy_buffer);
    target::rlt::malloc(target_device, target_input);
    target::rlt::malloc(target_device, target_output);
    target::rlt::malloc(target_device, target_original_output);
    target::rlt::malloc(target_device, target_base_output);
    target::rlt::init(target_device);
    target::rlt::init(target_device, target_rng, SEED);
    target::rlt::init_weights(target_device, target_policy, target_rng);
    target::rlt::randn(target_device, target_input, target_rng);
    target::rlt::copy_from_generic(base_device, target_device, rl_tools::checkpoint::example::input::container, target_input);
    target::rlt::copy_from_generic(base_device, target_device, rl_tools::checkpoint::actor::module, target_policy);

    {
        target::rlt::evaluate(target_device, target_policy, target_input, target_output, target_policy_buffer, target_rng);
        target::rlt::copy_from_generic(target_device, base_device, rl_tools::checkpoint::example::output::container, target_original_output);
        target::rlt::copy_from_generic(target_device, base_device, base_output, target_base_output);

        {
            T abs_diff = target::rlt::abs_diff(target_device, target_base_output, target_output) / decltype(target_output)::SPEC::SIZE;
            std::cout << "Difference base <-> target: " << abs_diff << std::endl;
        }
        {
            T abs_diff = target::rlt::abs_diff(target_device, target_original_output, target_output) / decltype(target_output)::SPEC::SIZE;
            std::cout << "Difference target <-> orig: " << abs_diff << std::endl;
        }
    }
    const std::filesystem::path this_file = __FILE__;
    const std::filesystem::path this_dir  = this_file.parent_path();
    const std::filesystem::path converted_checkpoint_path = this_dir / "policy.h";
    std::string code = target::rlt::save_code(target_device, target_policy, "rl_tools::checkpoint::actor");

    std::ofstream ofs(converted_checkpoint_path);
    ofs << code;
    ofs << "\n" << target::rlt::save_code(target_device, target_input, std::string("rl_tools::checkpoint::example::input"), true);
    ofs << "\n" << target::rlt::save_code(target_device, target_original_output, std::string("rl_tools::checkpoint::example::output"), true);
    ofs << "\n" << "namespace rl_tools::checkpoint::meta{";
    ofs << "\n" << "   " << "char name[] = \"" << rl_tools::checkpoint::meta::name << "\";";
    ofs << "\n" << "   " << "char commit_hash[] = \"" << rl_tools::checkpoint::meta::commit_hash << "\";";
    ofs << "\n" << "}";
    ofs.close();


    auto write_checkpoint = [&target_device, &target_policy, &target_input, &target_original_output](auto& actor_file, bool bare = false){
        auto actor_group = target::rlt::create_group(target_device, actor_file, "actor");
        target::rlt::set_attribute(target_device, actor_group, "checkpoint_name", std::string(rl_tools::checkpoint::meta::name).c_str());
        target::rlt::save(target_device, target_policy, actor_group);
        if (!bare){
            target::rlt::set_attribute(target_device, actor_group, "meta", std::string("{\"environment\": {\"name\": \"l2f\",\"observation\": \"Position.OrientationRotationMatrix.LinearVelocity.AngularVelocityDelayed(0).ActionHistory(1)\"}}").c_str());
            auto example_group = target::rlt::create_group(target_device, actor_file, "example");
            target::rlt::save(target_device, target_input, example_group, "input");
            target::rlt::save(target_device, target_original_output, example_group, "output");
        }
    };

    for (bool bare : {true, false}){
        target::rlt::persist::backends::tar::Writer tar_writer;
        target::rlt::persist::backends::tar::WriterGroup<target::rlt::persist::backends::tar::WriterGroupSpecification<target::TI, decltype(tar_writer)>> tar_group;
        tar_group.writer = &tar_writer;
        write_checkpoint(tar_group, bare);

        std::ofstream tar_ofs(this_dir / (bare ? "policy_bare.tar" : "policy.tar"), std::ios::binary);
        tar_ofs.write(tar_writer.buffer.data(), tar_writer.buffer.size());
        tar_ofs.close();
    }

#ifdef RL_TOOLS_ENABLE_HDF5
    auto actor_file = HighFive::File(this_dir / "policy.h5", HighFive::File::Overwrite);
    write_checkpoint(actor_file);
#endif

    return 0;
}