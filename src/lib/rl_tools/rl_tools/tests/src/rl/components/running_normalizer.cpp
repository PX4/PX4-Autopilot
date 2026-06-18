#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/rl/components/running_normalizer/operations_generic.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include <gtest/gtest.h>
#include <highfive/H5File.hpp>


using DEVICE = rlt::devices::DEVICE_FACTORY<rlt::devices::DefaultCPUSpecification>;
using TI = typename DEVICE::index_t;

template <typename T, TI ROWS, TI COLS, TI BATCH_SIZE>
void test(){
    T threshold = 0.01;
    T std_threshold = 0.2;
    T mean_threshold_normalization = 0.01;
    T std_threshold_normalization = 3.0/BATCH_SIZE;
    static_assert((ROWS % BATCH_SIZE) == 0);
    DEVICE device;
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS>> data;
    rlt::rl::components::RunningNormalizer<rlt::rl::components::running_normalizer::Specification<T, TI, COLS>> running_normalizer;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::malloc(device, data);
    rlt::malloc(device, running_normalizer);
    rlt::init(device, running_normalizer);
    rlt::randn(device, data, rng);
//    auto last_part = rlt::view(device, data, rlt::matrix::ViewSpec<ROWS / 2, COLS>{}, ROWS/2, 0);
//    rlt::increment_all(device, last_part, 10);
    {
        auto file = HighFive::File("running_normalizer.h5", HighFive::File::Overwrite);
        rlt::save(device, data, file.createGroup("data"), "data");
    }
    for(TI batch_start = 0; batch_start < ROWS; batch_start += BATCH_SIZE){
        auto batch = rlt::view(device, data, rlt::matrix::ViewSpec<BATCH_SIZE, COLS>{}, batch_start, 0);
        rlt::update(device, running_normalizer, batch);
    }
    for(TI col_i = 0; col_i < COLS; col_i++){
        auto col = rlt::col(device, data, col_i);
        auto mean = rlt::mean(device, col);
        auto std = rlt::std(device, col);
        ASSERT_GE(std, 0);
        // output with high precision
        std::cout << "real mean[" << col_i << "]: " << std::setprecision(20) << std::fixed << mean << ", std[" << col_i << "]: " << std::setprecision(20) << std::fixed << std << std::endl;
        auto mean_diff = rlt::math::abs(DEVICE::SPEC::MATH(), mean - get(running_normalizer.mean, 0, col_i));
        ASSERT_GE(get(running_normalizer.std, 0, col_i), 0);
        auto std_diff = rlt::math::abs(DEVICE::SPEC::MATH(), std - get(running_normalizer.std, 0, col_i)); // there should be a deviation here because we are using incremental means instead of the mean over the whole dataset
        if(mean_diff >= threshold || (std_diff != 0 && std_diff/std >= std_threshold) || rlt::math::is_nan(DEVICE::SPEC::MATH(), std_diff)){
            std::cout << "mean_diff: " << mean_diff << std::endl;
            std::cout << "std_diff: " << std_diff << std::endl;
        }
        ASSERT_LT(mean_diff, threshold);
        ASSERT_LT(std_diff/std, std_threshold);
    }
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS>> data_normalized;
    rlt::malloc(device, data_normalized);
    rlt::normalize(device, running_normalizer, data, data_normalized);
    for(TI col_i = 0; col_i < COLS; col_i++){
        auto col = rlt::col(device, data_normalized, col_i);
        auto abs_mean = rlt::math::abs(DEVICE::SPEC::MATH(), rlt::mean(device, col));
        auto abs_std_diff = rlt::math::abs(DEVICE::SPEC::MATH(), rlt::std(device, col) - 1);

        if(abs_mean >= mean_threshold_normalization || abs_std_diff >= std_threshold_normalization){
            std::cout << "abs_mean: " << abs_mean << std::endl;
            std::cout << "abs_std_diff: " << abs_std_diff << std::endl;
        }
        ASSERT_LT(abs_mean, mean_threshold_normalization);
        ASSERT_LT(abs_std_diff, std_threshold_normalization);
    }
}

TEST(RL_TOOLS_RL_COMPONENTS_RUNNING_NORMALIZER, TEST){
    test<double, 100, 10, 10>();
    test<double, 20, 20, 20>();
    test<double, 20, 1, 20>();
    test<double, 20, 20, 20>();
    test<double, 30, 210, 30>();
    test<double, 32, 10, 32>();
    test<double, 32, 1, 32>();
    test<double, 100, 1, 50>();
    test<double, 100, 10, 50>();
    test<double, 10000, 10, 25>();
    test<double, 10000, 10, 10000>();
}
