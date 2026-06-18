#include <rl_tools/operations/wasm32.h>

#include <rl_tools/containers/tensor/operations_generic.h>
//#include <rl_tools/containers/tensor/operations_cpu.h>

#include <rl_tools/nn/layers/gru/operations_generic.h>

#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>

namespace rlt = rl_tools;

int main(){
    using DEVICE = rlt::devices::DefaultWASM32;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    using TYPE_POLICY = rlt::numeric_types::Policy<double>;
    typename DEVICE::SPEC::RANDOM::ENGINE<> rng;
    constexpr T EPSILON = 1e-6;
    constexpr TI SEQUENCE_LENGTH = 50;
    constexpr TI BATCH_SIZE = 128;
    constexpr TI INPUT_DIM = 1;
    constexpr TI OUTPUT_DIM = 1;
    constexpr TI HIDDEN_DIM = 16;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, INPUT_SHAPE, false, rlt::tensor::RowMajorStride<INPUT_SHAPE>>> input, dinput;
    using GRU_OUTPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, HIDDEN_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_OUTPUT_SHAPE, false, rlt::tensor::RowMajorStride<GRU_OUTPUT_SHAPE>>> dloss_dgru_output;

    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::parameters::groups::Normal, true>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    rlt::nn::layers::gru::Layer<GRU_CONFIG, CAPABILITY, INPUT_SHAPE> gru;
    decltype(gru)::Buffer<false> buffers;
    rlt::malloc(device, gru);
    rlt::init(device, rng);
    rlt::init(device, buffers);
    rlt::randn(device, input, rng);

    rlt::forward(device, gru, input, buffers, rng);
//    rlt::print(device, gru.output);
    rlt::zero_gradient(device, gru);

    for(TI step=SEQUENCE_LENGTH-1; true; step--){
        rlt::backward_full(device, gru, input, dloss_dgru_output, dinput, buffers, step);
        if(step == 0){
            break;
        }
    }
    return 0;
}


