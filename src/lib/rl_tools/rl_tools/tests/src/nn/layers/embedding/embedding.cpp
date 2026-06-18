#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/embedding/operations_generic.h>
namespace rlt = rl_tools;

#include <gtest/gtest.h>


using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

constexpr TI NUM_CLASSES = 2<<8;
constexpr TI EMBEDDING_DIM = 5;
constexpr TI BATCH_SIZE = 8;
constexpr TI SEQ_LEN = 6;
using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQ_LEN, BATCH_SIZE, 1>;
using EMBEDDING_LAYER_CONFIG = rlt::nn::layers::embedding::Configuration<TYPE_POLICY, TI, NUM_CLASSES, EMBEDDING_DIM>;
using CAPABILITY = rlt::nn::capability::Backward<>;
using EMBEDDING_LAYER = rlt::nn::layers::embedding::Layer<EMBEDDING_LAYER_CONFIG, CAPABILITY, INPUT_SHAPE>;

TEST(RL_TOOLS_NN_LAYERS_EMBEDDING, MAIN){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    EMBEDDING_LAYER layer;
    EMBEDDING_LAYER::Buffer<> buffer;

    rlt::malloc(device, rng);
    rlt::init(device, rng);
    rlt::malloc(device, layer);
//    rlt::init_weights(device, layer, rng);
    for(TI class_i = 0; class_i < NUM_CLASSES; class_i++){
        for(TI dim_i = 0; dim_i < EMBEDDING_DIM; dim_i++){
            rlt::set(device, layer.weights.parameters, class_i * EMBEDDING_DIM + dim_i, class_i, dim_i);
        }
    }
    using INPUT_SPEC = rlt::tensor::Specification<TI, TI, INPUT_SHAPE>;
    rlt::Tensor<INPUT_SPEC> input;
    using OUTPUT_SHAPE = rlt::tensor::Shape<TI, SEQ_LEN, BATCH_SIZE, EMBEDDING_LAYER::OUTPUT_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, OUTPUT_SHAPE>> output;
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    for(TI sequence_i = 0; sequence_i < SEQ_LEN; sequence_i++){
        for(TI i = 0; i < BATCH_SIZE; i++){
            rlt::set(device, input, (sequence_i + i) % NUM_CLASSES, sequence_i, i, 0);
        }
    }
    rlt::evaluate(device, layer, input, output, buffer, rng);
    for(TI sequence_i = 0; sequence_i < SEQ_LEN; sequence_i++) {
        for (TI i = 0; i < BATCH_SIZE; i++) {
            TI expected_class = (sequence_i + i) % NUM_CLASSES;
            for (TI j = 0; j < EMBEDDING_DIM; j++) {
                T expected_value = expected_class * EMBEDDING_DIM + j;
                EXPECT_FLOAT_EQ(rlt::get(device, output, sequence_i, i, j), expected_value); //rlt::get(device, layer.weights.parameters, rlt::get(device, input, i), j));
            }
        }
    }

}
