#include <gtest/gtest.h>
#include <rl_tools/operations/cpu.h>
#include <rl_tools/containers/tensor/operations_generic.h>
#include <rl_tools/containers/tensor/operations_cpu.h>
#include <rl_tools/containers/tensor/persist_code.h>

namespace rlt = rl_tools;


#include "../utils/utils.h"


constexpr double EPSILON = 1e-8;


#include <filesystem>
#include <fstream>
#include <optional>


#include "./tensor_persist_code_fill.h"

std::optional<std::string> get_env_var(const std::string& var) {
    const char* value = std::getenv(var.c_str());
    if (value) {
        return std::string(value);
    } else {
        return std::nullopt;
    }
}






template <typename DEVICE, typename T, typename TI, typename SHAPE>
std::string persist(DEVICE& device, std::string name){
    using SPEC = rlt::tensor::Specification<T, TI, SHAPE>;
    using TENSOR = rlt::Tensor<SPEC>;
    TENSOR tensor;
    rlt::malloc(device, tensor);
    fill(device, tensor);

    auto output = rlt::save_code(device, tensor, name, true);;
    rlt::free(device, tensor);
    return output;
}

TEST(RL_TOOLS_CONTAINERS_TENSOR_PERSIST_CODE, SAVE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;

    using SHAPE_0 = rlt::tensor::Shape<TI, 1>;
    using SHAPE_1 = rlt::tensor::Shape<TI, 10>;
    using SHAPE_2 = rlt::tensor::Shape<TI, 10, 10>;
    using SHAPE_3 = rlt::tensor::Shape<TI, 10, 1, 20>;
    using SHAPE_4 = rlt::tensor::Shape<TI, 10, 10, 20>;

    std::string output;

    output += persist<DEVICE, T, TI, SHAPE_1>(device, "shape_0") + "\n";
    output += persist<DEVICE, T, TI, SHAPE_1>(device, "shape_1") + "\n";
    output += persist<DEVICE, T, TI, SHAPE_2>(device, "shape_2") + "\n";
    output += persist<DEVICE, T, TI, SHAPE_3>(device, "shape_3") + "\n";
    output += persist<DEVICE, T, TI, SHAPE_4>(device, "shape_4") + "\n";


    std::ofstream file;
    std::string output_path = "tests/data/test_containers_tensor_persist_code.h" + std::string((get_env_var("GITHUB_ACTIONS") ? ".disabled" : ""));
    file.open(output_path, std::ios::out | std::ios::trunc);
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;
    std::cout << "Full file path: " << std::filesystem::absolute(output_path) << std::endl;
    file << output;
    file.close();
}
