#include <rl_tools/operations/cpu.h>
#define RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_CPU_NOT_INCLUDE_GENERIC
#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/persist/backends/tar/operations_posix.h>

#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>

namespace rlt = rl_tools;
using DEVICE = rl_tools::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using TI = typename DEVICE::index_t;
using T = float;

TEST(TESTS_PERSIST_BACKENDS_TAR_POSIX, test) {
    std::string content1 = "This is the first buffer's content.";
    std::vector<char> buffer1(content1.begin(), content1.end());

    std::string content2 = "This data is for the second entry in our archive!";
    std::vector<char> buffer2(content2.begin(), content2.end());

    const std::filesystem::path archive_path = "test_persist_backends_tar_posix.tar";
    std::ofstream archive(archive_path, std::ios::binary);
    
    DEVICE device;
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::write_entry(device, writer, "buffer1.txt", buffer1.data(), buffer1.size());
    rlt::persist::backends::tar::write_entry(device, writer, "entry2.log",  buffer2.data(), buffer2.size());
    rlt::persist::backends::tar::finalize(device, writer);

    archive.write(writer.buffer.data(), writer.buffer.size());
    archive.close();

    FILE* f = fopen(archive_path.c_str(), "rb");
    ASSERT_NE(f, nullptr);

    rlt::persist::backends::tar::PosixFileData<TI> tar_data_backend;
    tar_data_backend.f = f;
    fseek(f, 0, SEEK_END);
    tar_data_backend.size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char buffer[500];
    TI read_size;
    
    // Test 6-arg get
    bool found = rlt::persist::backends::tar::get(device, tar_data_backend, "buffer1.txt", buffer, sizeof(buffer), read_size);
    ASSERT_TRUE(found);
    ASSERT_EQ(read_size, content1.size());
    ASSERT_TRUE(rlt::utils::string::compare(buffer, content1.c_str(), content1.size()));

    found = rlt::persist::backends::tar::get(device, tar_data_backend, "entry2.log", buffer, sizeof(buffer), read_size);
    ASSERT_TRUE(found);
    ASSERT_EQ(read_size, content2.size());
    ASSERT_TRUE(rlt::utils::string::compare(buffer, content2.c_str(), content2.size()));

    fclose(f);
}

TEST(TESTS_PERSIST_BACKENDS_TAR_POSIX, tensor) {
    DEVICE device;
    rlt::init(device);
    RNG rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 5, 3, 2, 10>>> A, A_read_back;
    rlt::malloc(device, A);
    rlt::malloc(device, A_read_back);
    rlt::randn(device, A, rng);

    const std::filesystem::path archive_path = "test_persist_backends_tar_posix_tensor.tar";
    
    // Save Tensor
    {
        rlt::persist::backends::tar::Writer writer;
        rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> writer_group{"", &writer};
        auto container_group = rlt::create_group(device, writer_group, "container");
        rlt::save(device, A, container_group, "A");
        rlt::persist::backends::tar::finalize(device, writer);

        std::ofstream archive(archive_path, std::ios::binary);
        archive.write(writer.buffer.data(), writer.buffer.size());
        archive.close();
    }

    // Load Tensor
    FILE* f = fopen(archive_path.c_str(), "rb");
    ASSERT_NE(f, nullptr);
    rlt::persist::backends::tar::PosixFileData<TI> tar_data_backend;
    tar_data_backend.f = f;
    fseek(f, 0, SEEK_END);
    tar_data_backend.size = ftell(f);
    fseek(f, 0, SEEK_SET);

    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI, rlt::persist::backends::tar::PosixFileData<TI>>> reader_group;
    reader_group.data = tar_data_backend;
    
    auto reader_container_group = rlt::create_group(device, reader_group, "container");
    rlt::load(device, A_read_back, reader_container_group, "A");

    T abs_diff = rlt::abs_diff(device, A, A_read_back);
    ASSERT_NEAR(abs_diff, 0, 1e-6);

    fclose(f);
    rlt::free(device, A);
    rlt::free(device, A_read_back);
}
