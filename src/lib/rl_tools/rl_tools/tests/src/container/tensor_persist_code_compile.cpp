#include <gtest/gtest.h>
#include <rl_tools/operations/cpu.h>
#include <rl_tools/containers/tensor/operations_generic.h>
#include <rl_tools/containers/tensor/operations_cpu.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include "../../../tests/data/test_containers_tensor_persist_code.h"

namespace rlt = rl_tools;


#include "../utils/utils.h"


constexpr double EPSILON = 1e-8;

#include "./tensor_persist_code_fill.h"

#include <filesystem>
#include <fstream>

TEST(RL_TOOLS_CONTAINERS_TENSOR_PERSIST_CODE, COMPILE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;

    rlt::Tensor<rlt::tensor::Specification<T, TI, shape_0::SHAPE>> tensor_0;
    rlt::Tensor<rlt::tensor::Specification<T, TI, shape_1::SHAPE>> tensor_1;
    rlt::Tensor<rlt::tensor::Specification<T, TI, shape_2::SHAPE>> tensor_2;
    rlt::Tensor<rlt::tensor::Specification<T, TI, shape_3::SHAPE>> tensor_3;
    rlt::Tensor<rlt::tensor::Specification<T, TI, shape_4::SHAPE>> tensor_4;
    rlt::malloc(device, tensor_0);
    rlt::malloc(device, tensor_1);
    rlt::malloc(device, tensor_2);
    rlt::malloc(device, tensor_3);
    rlt::malloc(device, tensor_4);
    fill(device, tensor_0);
    fill(device, tensor_1);
    fill(device, tensor_2);
    fill(device, tensor_3);
    fill(device, tensor_4);

    T abs_diff = rlt::abs_diff(device, shape_0::container, tensor_0);
    print(device, shape_0::container);
    ASSERT_EQ(abs_diff, 0);
    abs_diff = rlt::abs_diff(device, shape_1::container, tensor_1);
    print(device, shape_1::container);
    ASSERT_EQ(abs_diff, 0);
    abs_diff = rlt::abs_diff(device, shape_2::container, tensor_2);
    print(device, shape_2::container);
    ASSERT_EQ(abs_diff, 0);
    abs_diff = rlt::abs_diff(device, shape_3::container, tensor_3);
    print(device, shape_3::container);
    ASSERT_EQ(abs_diff, 0);
    abs_diff = rlt::abs_diff(device, shape_4::container, tensor_4);
    print(device, shape_4::container);
    ASSERT_EQ(abs_diff, 0);
}
