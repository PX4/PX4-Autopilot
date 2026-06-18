#include <rl_tools/operations/cpu.h>
#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/persist/backends/tar/operations_generic.h>

#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/nn/parameters/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn_models/mlp/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#define RL_TOOLS_STRINGIZE(x) #x
#define RL_TOOLS_MACRO_TO_STR(macro) RL_TOOLS_STRINGIZE(macro)


namespace rlt = rl_tools;

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>

using DEVICE = rl_tools::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using TI = typename DEVICE::index_t;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

#include <gtest/gtest.h>


TEST(TESTS_PERSIST_BACKENDS_TAR_TAR, test) {
    std::string content1 = "This is the first buffer's content.";
    std::vector<char> buffer1(content1.begin(), content1.end());

    std::string content2 = "This data is for the second entry in our archive!";
    std::vector<char> buffer2(content2.begin(), content2.end());

    const std::filesystem::path archive_path = "test_persist_backends_tar_test.tar";
    std::ofstream archive(archive_path, std::ios::binary);
    std::cout << "Creating archive: " << archive_path << std::endl;

    std::cout << "Writing 'buffer1.txt'..." << std::endl;
    DEVICE device;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::write_entry(device, writer, "buffer1.txt", buffer1.data(), buffer1.size());

    std::cout << "Writing 'entry2.log'..." << std::endl;
    rlt::persist::backends::tar::write_entry(device, writer, "entry2.log",  buffer2.data(), buffer2.size());

    rlt::persist::backends::tar::finalize(device, writer);

    archive.write(writer.buffer.data(), writer.buffer.size());

    archive.close();

    std::ifstream archive_file("test_persist_backends_tar_test.tar", std::ios::binary);
    std::vector<char> tar_data((std::istreambuf_iterator<char>(archive_file)), std::istreambuf_iterator<char>());
    archive_file.close();

    rlt::persist::backends::tar::BufferData<TI> tar_data_backend;
    tar_data_backend.data = tar_data.data();
    tar_data_backend.size = tar_data.size();

    char buffer[500];
    TI read_size;
    rlt::persist::backends::tar::get(device, tar_data_backend, "buffer1.txt", buffer, sizeof(buffer), read_size);
    ASSERT_TRUE(rlt::utils::string::compare("abcdefg", "abcdefg", 7));
    ASSERT_FALSE(rlt::utils::string::compare("abcdefg", "abbdefg", 7));
    ASSERT_TRUE(rlt::utils::string::compare("abcdefg", "abcdefg ", 7));
    ASSERT_FALSE(rlt::utils::string::compare("abcdefg", "abcdefg ", 8));
    ASSERT_FALSE(rlt::utils::string::compare("abcdefg", "", 7));
    ASSERT_TRUE(rlt::utils::string::compare(buffer, content1.c_str(), content1.size()));
    rlt::persist::backends::tar::get(device, tar_data_backend, "entry2.log", buffer, sizeof(buffer), read_size);
    ASSERT_TRUE(rlt::utils::string::compare(buffer, content2.c_str(), content2.size()));
}

TEST(TEST_PERSIST_BACKENDS_TAR_TAR, tensor) {
    DEVICE device;
    RNG rng;
    constexpr TI seed = 0;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 5, 3, 2, 10>>> A, A_read_back;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 5, 4, 2, 10>>> A_read_back_fail;
    rlt::malloc(device, A);
    rlt::malloc(device, A_read_back);
    rlt::malloc(device, A_read_back_fail);
    rlt::randn(device, A, rng);
    rlt::print(device, A);
    std::string data_file_name = "test_persist_backends_tar_tensor.tar";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string data_file_path = std::string(data_path_stub) + "/" + data_file_name;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> writer_group{"", &writer};
    auto container_group = rlt::create_group(device, writer_group, "container");
    rlt::save(device, A, container_group, "A");
    rlt::persist::backends::tar::finalize(device, writer);
    std::ofstream archive(data_file_path, std::ios::binary);
    std::cout << "Creating archive: " << data_file_path << std::endl;
    archive.write(writer.buffer.data(), writer.buffer.size());
    archive.close();
    std::ifstream archive_file(data_file_path, std::ios::binary);
    std::vector<char> tar_data((std::istreambuf_iterator<char>(archive_file)), std::istreambuf_iterator<char>());
    archive_file.close();
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader_group;
    reader_group.data.data = tar_data.data();
    reader_group.data.size = tar_data.size();
    auto reader_container_group = rlt::create_group(device, reader_group, "container");
    rlt::load(device, A_read_back, reader_container_group, "A");

    rlt::print(device, A_read_back);

    T abs_diff = rlt::abs_diff(device, A, A_read_back);
    ASSERT_NEAR(abs_diff, 0, 1e-6);
}

TEST(TEST_PERSIST_BACKENDS_TAR_TAR, tensor_attribute) {
    DEVICE device;
    RNG rng;
    constexpr TI seed = 0;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 5, 3, 2, 10>>> A, A_read_back;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 5, 4, 2, 10>>> A_read_back_fail;
    rlt::malloc(device, A);
    rlt::malloc(device, A_read_back);
    rlt::malloc(device, A_read_back_fail);
    rlt::randn(device, A, rng);
    rlt::print(device, A);
    std::string data_file_name = "test_persist_backends_tar_tensor_attribute.tar";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string data_file_path = std::string(data_path_stub) + "/" + data_file_name;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> writer_group{"", &writer};
    auto container_group = rlt::create_group(device, writer_group, "container");
    rlt::save(device, A, container_group, "A");
    rlt::set_attribute(device, writer_group, "id", 1337);
    rlt::write_attributes(device, writer_group);
    rlt::persist::backends::tar::finalize(device, writer);
    std::ofstream archive(data_file_path, std::ios::binary);
    std::cout << "Creating archive: " << data_file_path << std::endl;
    archive.write(writer.buffer.data(), writer.buffer.size());
    archive.close();
    std::ifstream archive_file(data_file_path, std::ios::binary);
    std::vector<char> tar_data((std::istreambuf_iterator<char>(archive_file)), std::istreambuf_iterator<char>());
    archive_file.close();
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader_group;
    reader_group.data.data = tar_data.data();
    reader_group.data.size = tar_data.size();
    auto reader_container_group = rlt::create_group(device, reader_group, "container");
    rlt::load(device, A_read_back, reader_container_group, "A");

    TI id = rlt::get_attribute_int<TI>(device, reader_group, "id");
    ASSERT_EQ(id, 1337);

    rlt::print(device, A_read_back);

    T abs_diff = rlt::abs_diff(device, A, A_read_back);
    ASSERT_NEAR(abs_diff, 0, 1e-6);
}

TEST(TEST_PERSIST_BACKENDS_TAR_TAR, matrix) {
    DEVICE device;
    RNG rng;
    constexpr TI seed = 0;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 5, 3>> A, A_read_back;
    rlt::malloc(device, A);
    rlt::malloc(device, A_read_back);
    rlt::randn(device, A, rng);
    rlt::print(device, A);
    std::string data_file_name = "test_persist_backends_tar_matrix.tar";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string data_file_path = std::string(data_path_stub) + "/" + data_file_name;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> writer_group{"", &writer};
    rlt::save(device, A, writer_group, "A");
    rlt::persist::backends::tar::finalize(device, writer);
    std::ofstream archive(data_file_path, std::ios::binary);
    std::cout << "Creating archive: " << data_file_path << std::endl;
    archive.write(writer.buffer.data(), writer.buffer.size());
    archive.close();
    std::ifstream archive_file(data_file_path, std::ios::binary);
    std::vector<char> tar_data((std::istreambuf_iterator<char>(archive_file)), std::istreambuf_iterator<char>());
    archive_file.close();
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader_group;
    reader_group.data.data = tar_data.data();
    reader_group.data.size = tar_data.size();
    rlt::load(device, A_read_back, reader_group, "A");

    rlt::print(device, A_read_back);

    T abs_diff = rlt::abs_diff(device, A, A_read_back);
    ASSERT_NEAR(abs_diff, 0, 1e-6);
}

TEST(TEST_PERSIST_BACKENDS_TAR_TAR, dense_layer){
    DEVICE device;
    RNG rng;
    constexpr TI seed = 0;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);

    using CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    using SPEC = rlt::nn::layers::dense::Specification<CONFIG, CAPABILITY, rlt::tensor::Shape<TI, 1, 10, 15>>;
    rlt::nn::layers::dense::LayerForward<SPEC> layer, layer_read_back;
    rlt::malloc(device, layer);
    rlt::malloc(device, layer_read_back);
    rlt::init_weights(device, layer, rng);

    std::string data_file_name = "test_persist_backends_tar_dense_layer.tar";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string data_file_path = std::string(data_path_stub) + "/" + data_file_name;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> writer_group{"", &writer};
    auto layer_group = rlt::create_group(device, writer_group, "layer");
    rlt::save(device, layer, layer_group);
    rlt::persist::backends::tar::finalize(device, writer);
    std::ofstream archive(data_file_path, std::ios::binary);
    std::cout << "Creating archive: " << data_file_path << std::endl;
    archive.write(writer.buffer.data(), writer.buffer.size());
    archive.close();

    std::ifstream archive_data(data_file_path, std::ios::binary);
    std::vector<char> tar_data((std::istreambuf_iterator<char>(archive_data)), std::istreambuf_iterator<char>());
    archive_data.close();
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader_group{"", tar_data.data(), tar_data.size()};
    auto layer_group_readback = rlt::get_group(device, reader_group, "layer");
    rlt::load(device, layer_read_back, layer_group_readback);

    T abs_diff = rlt::abs_diff(device, layer, layer_read_back);
    std::cout << "Layer abs diff: " << abs_diff << std::endl;
    ASSERT_NEAR(abs_diff, 0, 1e-6);
}

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

TEST(TEST_PERSIST_BACKENDS_TAR_TAR, sequential_model){
    DEVICE device;
    RNG rng;
    constexpr TI seed = 0;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);

    using INPUT_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using INPUT_LAYER = rlt::nn::layers::dense::BindConfiguration<INPUT_LAYER_CONFIG>;
    using GRU_LAYER_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, 20>;
    using GRU_LAYER = rlt::nn::layers::gru::BindConfiguration<GRU_LAYER_CONFIG>;
    using MLP_LAYER_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, 15, 3, 7, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using MLP_LAYER = rlt::nn_models::mlp::BindConfiguration<MLP_LAYER_CONFIG>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    using MODULE_CHAIN = Module<INPUT_LAYER, Module<GRU_LAYER, Module<MLP_LAYER>>>;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, 5, 3, 10>;
    using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename MODEL::OUTPUT_SHAPE>> output, output_read_back;
    MODEL model, model_read_back;
    MODEL::Buffer<> buffer;
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, output_read_back);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);
    rlt::malloc(device, model_read_back);
    rlt::init_weights(device, model, rng);
    rlt::randn(device, input, rng);
    rlt::evaluate(device, model, input, output, buffer, rng);

    std::string data_file_name = "test_persist_backends_tar_sequential_model.tar";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string data_file_path = std::string(data_path_stub) + "/" + data_file_name;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> writer_group{"", &writer};
    auto layer_group = rlt::create_group(device, writer_group, "layer");
    rlt::save(device, model, layer_group);
    rlt::persist::backends::tar::finalize(device, writer);
    std::ofstream archive(data_file_path, std::ios::binary);
    std::cout << "Creating archive: " << data_file_path << std::endl;
    archive.write(writer.buffer.data(), writer.buffer.size());
    archive.close();

    std::ifstream archive_data(data_file_path, std::ios::binary);
    std::vector<char> tar_data((std::istreambuf_iterator<char>(archive_data)), std::istreambuf_iterator<char>());
    archive_data.close();
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader_group{"", tar_data.data(), tar_data.size()};
    auto layer_group_readback = rlt::get_group(device, reader_group, "layer");
    rlt::load(device, model_read_back, layer_group_readback);
    rlt::evaluate(device, model_read_back, input, output_read_back, buffer, rng);

    T abs_diff_model = rlt::abs_diff(device, model, model_read_back);
    std::cout << "Layer abs diff: " << abs_diff_model << std::endl;
    ASSERT_NEAR(abs_diff_model, 0, 1e-6);
    T abs_diff_output = rlt::abs_diff(device, output, output_read_back);
    std::cout << "Output abs diff: " << abs_diff_output << std::endl;
    ASSERT_NEAR(abs_diff_output, 0, 1e-6);
}
