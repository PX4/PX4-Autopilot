#include <rl_tools/operations/cpu.h>
#include <rl_tools/containers/tensor/tensor.h>
#include <rl_tools/containers/tensor/operations_generic.h>
#include <rl_tools/containers/tensor/operations_cpu.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using T = float;
using TI = typename DEVICE::index_t;
DEVICE device;

int main(){
    using SHAPE = rlt::tensor::Shape<TI, 10, 10, 10, 10>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
    rlt::malloc(device, tensor);
    rlt::set_all(device, tensor, 1.0);
    T sum = rlt::sum(device, tensor);
    return sum / 10;
}
