#include <rl_tools/operations/arm.h>
#include <gtest/gtest.h>
#include <iostream>

template <auto MIN, auto MAX, int NUM_RUNS = 10000>
void test_int_uniform_limits(){
    namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
    using DEVICE = rlt::devices::DefaultARM;
    using T = float;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    for(unsigned i = 0; i < NUM_RUNS; ++i){
        auto number = rlt::random::uniform_int_distribution(DEVICE::SPEC::RANDOM(), MIN, MAX, rng);
        ASSERT_TRUE(number >= MIN);
        ASSERT_TRUE(number <= MAX);
    }
}
TEST(RL_TOOLS_RANDOM_ARM, TEST_INT_UNIFORM_LIMITS) {
    test_int_uniform_limits<0, 10>();
    test_int_uniform_limits<-10, 10>();
    test_int_uniform_limits<-10, 0>();
    test_int_uniform_limits<-1000, 1000000>();
}

template <auto MIN, auto MAX, int NUM_RUNS = 10000>
void test_int_uniform_distribution(){
    namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
    using DEVICE = rlt::devices::DefaultARM;
    using T = float;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    TI smaller_than_half = 0;
    TI bigger_than_half = 0;
    long int threshold = ((long int)MAX - (long int)MIN) / 2;
    for(unsigned i = 0; i < NUM_RUNS; ++i){
        auto number = rlt::random::uniform_int_distribution(DEVICE::SPEC::RANDOM(), MIN, MAX, rng);
        if(number - MIN <= threshold) {
            smaller_than_half++;
        }
        else{
            bigger_than_half++;
        }
        ASSERT_TRUE(number >= MIN);
        ASSERT_TRUE(number <= MAX);
    }
    std::cout << "smaller_than_half: " << (float)smaller_than_half/NUM_RUNS << " bigger_than_half: " << (float)bigger_than_half/NUM_RUNS << std::endl;
}
TEST(RL_TOOLS_RANDOM_ARM, TEST_INT_UNIFORM_DISTRIBUTION) {
    test_int_uniform_distribution<0, 10>();
    test_int_uniform_distribution<-10, 10>();
    test_int_uniform_distribution<-10, 0>();
    test_int_uniform_distribution<-1000, 1000000>();
    test_int_uniform_distribution<-1, 1>();
    test_int_uniform_distribution<0, 1>();
}

template <typename T, auto MIN, auto MAX, auto DENOMINATOR, int NUM_RUNS = 10000>
void test_real_uniform_limits(){
    namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
    using DEVICE = rlt::devices::DefaultARM;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    T min = (T)MIN / (T)DENOMINATOR;
    T max = (T)MAX / (T)DENOMINATOR;
    for(unsigned i = 0; i < NUM_RUNS; ++i){
        auto number = rlt::random::uniform_real_distribution(DEVICE::SPEC::RANDOM(), min, max, rng);
        if(number < min || number > max){
            std::cout << "number: " << number << " min: " << min << " max: " << max << std::endl;
        }
        ASSERT_TRUE(number >= min);
        ASSERT_TRUE(number <= max);
    }
}
TEST(RL_TOOLS_RANDOM_ARM, TEST_REAL_UNIFORM_LIMITS) {
    test_real_uniform_limits<float, 0, 10, 1000>();
    test_real_uniform_limits<float, -10, 10, 1000>();
    test_real_uniform_limits<float, -10, 0, 1000>();
    test_real_uniform_limits<float, -1000, 1000000, 1000>();
}

template <typename T, auto MIN, auto MAX, auto DENOMINATOR, int NUM_RUNS = 10000>
void test_real_uniform_distribution(){
    namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
    using DEVICE = rlt::devices::DefaultARM;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    T min = (T)MIN / (T)DENOMINATOR;
    T max = (T)MAX / (T)DENOMINATOR;
    TI smaller_than_half = 0;
    TI bigger_than_half = 0;

    T threshold = (max - min) / 2;
    for(unsigned i = 0; i < NUM_RUNS; ++i){
        auto number = rlt::random::uniform_real_distribution(DEVICE::SPEC::RANDOM(), min, max, rng);
        if(number - min <= threshold) {
            smaller_than_half++;
        }
        else{
            bigger_than_half++;
        }
        ASSERT_TRUE(number >= min);
        ASSERT_TRUE(number <= max);
    }
    std::cout << "smaller_than_half: " << (float)smaller_than_half/NUM_RUNS << " bigger_than_half: " << (float)bigger_than_half/NUM_RUNS << std::endl;
}
TEST(RL_TOOLS_RANDOM_ARM, TEST_REAL_UNIFORM_DISTRIBUTION) {
    test_real_uniform_distribution<float, 0, 10, 1000>();
    test_real_uniform_distribution<float, -10, 10, 1000>();
    test_real_uniform_distribution<float, -10, 0, 1000>();
    test_real_uniform_distribution<float, -1000, 1000000, 1000>();
    test_real_uniform_distribution<float, -1, 1, 1000>();
    test_real_uniform_distribution<float, 0, 1, 1000>();
}

template <typename T, auto MEAN, auto STD, auto DENOMINATOR, int NUM_RUNS = 10000000>
void test_normal_distribution(){
    namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
    using DEVICE = rlt::devices::DefaultARM;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    T mean = (T)MEAN / (T)DENOMINATOR;
    T std = (T)STD / (T)DENOMINATOR;
    TI smaller_than_mean = 0;
    TI bigger_than_mean = 0;
    TI smaller_than_one_sigma = 0;
    TI bigger_than_one_sigma = 0;
    TI smaller_than_two_sigma = 0;
    TI bigger_than_two_sigma = 0;
    TI smaller_than_three_sigma = 0;
    TI bigger_than_three_sigma = 0;
    TI smaller_than_four_sigma = 0;
    TI bigger_than_four_sigma = 0;


    for(unsigned i = 0; i < NUM_RUNS; ++i){
        auto number = rlt::random::normal_distribution::sample(DEVICE::SPEC::RANDOM(), mean, std, rng);
        if(number <= mean) {
            smaller_than_mean++;
        }
        else{
            bigger_than_mean++;
        }
        if(number <= mean - std) {
            smaller_than_one_sigma++;
        }
        else{
            if(number >= mean + std){
                bigger_than_one_sigma++;
            }
        }
        if(number <= mean - 2*std) {
            smaller_than_two_sigma++;
        }
        else{
            if(number >= mean + 2*std){
                bigger_than_two_sigma++;
            }
        }
        if(number <= mean - 3*std) {
            smaller_than_three_sigma++;
        }
        else{
            if(number >= mean + 3*std){
                bigger_than_three_sigma++;
            }
        }
        if(number <= mean - 4*std) {
            smaller_than_four_sigma++;
        }
        else{
            if(number >= mean + 4*std){
                bigger_than_four_sigma++;
            }
        }
    }
    std::cout << "smaller_than_mean: " << (float)smaller_than_mean/NUM_RUNS << " bigger_than_half: " << (float)bigger_than_mean/NUM_RUNS << std::endl;
    std::cout << "smaller_than_one_sigma: " << (float)smaller_than_one_sigma/NUM_RUNS << " bigger_than_one_sigma: " << (float)bigger_than_one_sigma/NUM_RUNS << std::endl;
    std::cout << "smaller_than_two_sigma: " << (float)smaller_than_two_sigma/NUM_RUNS << " bigger_than_two_sigma: " << (float)bigger_than_two_sigma/NUM_RUNS << std::endl;
    std::cout << "smaller_than_three_sigma: " << (float)smaller_than_three_sigma/NUM_RUNS << " bigger_than_three_sigma: " << (float)bigger_than_three_sigma/NUM_RUNS << std::endl;
    std::cout << "smaller_than_four_sigma: " << (float)smaller_than_four_sigma/NUM_RUNS << " bigger_than_four_sigma: " << (float)bigger_than_four_sigma/NUM_RUNS << std::endl;
    ASSERT_LT(std::abs((float)smaller_than_mean/NUM_RUNS - 0.5), 0.01);
    T thres = (T)10000 / (T)NUM_RUNS;
    T mass_bigger_than_one_sigma = (float)smaller_than_one_sigma/NUM_RUNS + (float)bigger_than_one_sigma/NUM_RUNS;
    T mass_bigger_than_one_sigma_diff = std::abs(mass_bigger_than_one_sigma - 0.31731050786291404);
    T mass_bigger_than_two_sigma = (float)smaller_than_two_sigma/NUM_RUNS + (float)bigger_than_two_sigma/NUM_RUNS;
    T mass_bigger_than_two_sigma_diff = std::abs(mass_bigger_than_two_sigma - 0.04550026389635797);
    T mass_bigger_than_three_sigma = (float)smaller_than_three_sigma/NUM_RUNS + (float)bigger_than_three_sigma/NUM_RUNS;
    T mass_bigger_than_three_sigma_diff = std::abs(mass_bigger_than_three_sigma - 0.002699796063259985);
    T mass_bigger_than_four_sigma = (float)smaller_than_four_sigma/NUM_RUNS + (float)bigger_than_four_sigma/NUM_RUNS;
    T mass_bigger_than_four_sigma_diff = std::abs(mass_bigger_than_four_sigma - 6.334248366601791e-5);
    if(
            mass_bigger_than_one_sigma_diff > thres ||
            mass_bigger_than_two_sigma_diff > thres ||
            mass_bigger_than_three_sigma_diff > thres ||
            mass_bigger_than_four_sigma_diff > thres
    ){
        std::cout << "mass_bigger_than_one_sigma_diff: " << mass_bigger_than_one_sigma_diff << std::endl;
        std::cout << "mass_bigger_than_two_sigma_diff: " << mass_bigger_than_two_sigma_diff << std::endl;
        std::cout << "mass_bigger_than_three_sigma_diff: " << mass_bigger_than_three_sigma_diff << std::endl;
        std::cout << "mass_bigger_than_four_sigma_diff: " << mass_bigger_than_four_sigma_diff << std::endl;
    }
    ASSERT_LT(mass_bigger_than_one_sigma_diff, thres);
    ASSERT_LT(mass_bigger_than_two_sigma_diff, thres);
    ASSERT_LT(mass_bigger_than_three_sigma_diff, thres);
    ASSERT_LT(mass_bigger_than_four_sigma_diff, thres);

}
TEST(RL_TOOLS_RANDOM_ARM, TEST_NORMAL_DISTRIBUTION) {
    test_normal_distribution<float, 0, 10, 1>();
    test_normal_distribution<float, -10, 10, 10>();
    test_normal_distribution<float, -10, 1, 1>();
    test_normal_distribution<float, -1000, 1000000, 100>();
    test_normal_distribution<float, -1, 1, 10>();
    test_normal_distribution<float, 0, 1, 1, 1000000>();
}
