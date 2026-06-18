#include <rl_tools/nn/nn.h>
#include "../utils/utils.h"


namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using DTYPE = double;
using TYPE_POLICY = rlt::numeric_types::Policy<DTYPE>;



using NN_DEVICE = rlt::devices::DefaultCPU;
using TI = typename NN_DEVICE::index_t;
constexpr TI BATCH_SIZE = 1;
using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, 1, 17>;
using NETWORK_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, NN_DEVICE::index_t, 13, 3, 50, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;

using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename NN_DEVICE::index_t>;
using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
constexpr bool DYNAMIC_ALLOCATION = true;
using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, DYNAMIC_ALLOCATION>;
using NetworkType = rlt::nn_models::mlp::NeuralNetwork<NETWORK_CONFIG, CAPABILITY_ADAM, INPUT_SHAPE>;

using NetworkTypeBackwardOnly = rlt::nn_models::mlp::NeuralNetwork<NETWORK_CONFIG, rlt::nn::capability::Backward<>, INPUT_SHAPE>;

constexpr typename NN_DEVICE::index_t INPUT_DIM = rlt::get_last(NetworkType::INPUT_SHAPE{});
constexpr typename NN_DEVICE::index_t LAYER_1_DIM = NetworkType::SPEC::HIDDEN_DIM;
constexpr typename NN_DEVICE::index_t LAYER_2_DIM = NetworkType::SPEC::HIDDEN_DIM;
constexpr typename NN_DEVICE::index_t OUTPUT_DIM = rlt::get_last(NetworkType::OUTPUT_SHAPE{});

class NeuralNetworkTest : public ::testing::Test {
protected:
    std::string DATA_FILE_PATH;;
    std::string model_name = "model_1";
    NeuralNetworkTest(){
        std::string DATA_FILE_NAME = "mlp_data.hdf5";
        const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
        this->DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;

        auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
        data_file.getDataSet("data/X_train").read(X_train);
        data_file.getDataSet("data/Y_train").read(Y_train);
        data_file.getDataSet("data/X_val").read(X_val);
        data_file.getDataSet("data/Y_val").read(Y_val);
        data_file.getDataSet("data/X_mean").read(X_mean);
        data_file.getDataSet("data/X_std").read(X_std);
        data_file.getDataSet("data/Y_mean").read(Y_mean);
        data_file.getDataSet("data/Y_std").read(Y_std);
    }

    std::vector<std::vector<DTYPE>> X_train;
    std::vector<std::vector<DTYPE>> Y_train;
    std::vector<std::vector<DTYPE>> X_val;
    std::vector<std::vector<DTYPE>> Y_val;
    std::vector<DTYPE> X_mean;
    std::vector<DTYPE> X_std;
    std::vector<DTYPE> Y_mean;
    std::vector<DTYPE> Y_std;
};

using Specification_3 = rlt::nn_models::mlp::Configuration<TYPE_POLICY, NN_DEVICE::index_t, 13, 3, 50, rlt::nn::activation_functions::GELU, rlt::nn::activation_functions::IDENTITY>;

using NetworkType_3 = rlt::nn_models::mlp::NeuralNetwork<Specification_3, CAPABILITY_ADAM, INPUT_SHAPE>;
