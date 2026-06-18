#include <rl_tools/operations/cpu.h>

#include "policy.h"

#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#include <filesystem>
#include <fstream>


namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
constexpr bool DYNAMIC_ALLOCATION = true;
using T = rl_tools::checkpoint::actor::TYPE::TYPE_POLICY::DEFAULT;
using TI = typename DEVICE::index_t;
constexpr TI SEED = 0;

int main(){
    DEVICE device;
    RNG rng;
    rl_tools::checkpoint::actor::TYPE::Buffer<DYNAMIC_ALLOCATION> policy_buffer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rl_tools::checkpoint::actor::TYPE::OUTPUT_SHAPE>> output;
    rlt::malloc(device);
    rlt::malloc(device, rng);
    rlt::malloc(device, policy_buffer);
    rlt::malloc(device, output);
    rlt::init(device);
    rlt::init(device, rng, SEED);
    rlt::evaluate(device, rl_tools::checkpoint::actor::module, rl_tools::checkpoint::example::input::container, output, policy_buffer, rng);
    T abs_diff = rlt::abs_diff(device, rl_tools::checkpoint::example::output::container, output) / decltype(output)::SPEC::SIZE;
    std::cout << "Difference base <-> orig: " << abs_diff << std::endl;

    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> tar_reader_group;
    const std::filesystem::path this_file = __FILE__;
    const std::filesystem::path this_dir  = this_file.parent_path();
    const std::filesystem::path converted_checkpoint_path = this_dir / "policy.tar";
    std::ifstream checkpoint_file(converted_checkpoint_path, std::ios::binary);
    std::vector<char> file_buffer((std::istreambuf_iterator<char>(checkpoint_file)), std::istreambuf_iterator<char>());
    checkpoint_file.close();
    tar_reader_group.data.data = file_buffer.data();
    tar_reader_group.data.size = file_buffer.size();

    rlt::checkpoint::actor::TYPE::CHANGE_CAPABILITY <rlt::nn::capability::Forward<true, false>> tar_policy;
    decltype(tar_policy)::Buffer<true> tar_policy_buffer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, decltype(tar_policy)::OUTPUT_SHAPE>> tar_output;


    rlt::malloc(device, tar_policy);
    rlt::malloc(device, tar_policy_buffer);
    rlt::malloc(device, tar_output);
    auto actor_group = rlt::get_group(device, tar_reader_group, "actor");
    rlt::load(device, tar_policy, actor_group);
    rlt::evaluate(device, tar_policy, rl_tools::checkpoint::example::input::container, tar_output, tar_policy_buffer, rng);
    {
        T abs_diff = rlt::abs_diff(device, rl_tools::checkpoint::example::output::container, tar_output) / decltype(tar_output)::SPEC::SIZE;
        std::cout << "Difference tar <-> orig: " << abs_diff << std::endl;
    }



    return 0;
}