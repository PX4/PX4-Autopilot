//#include <fstream>
//#include <iostream>
//#include <vector>
//
//uint16_t read_u16(std::ifstream& stream) {
//    uint16_t result;
//    stream.read(reinterpret_cast<char*>(&result), sizeof(result));
//    return result;
//}
//
//uint32_t read_u32(std::ifstream& stream) {
//    uint32_t result;
//    stream.read(reinterpret_cast<char*>(&result), sizeof(result));
//    return result;
//}
//
//int32_t read_i32(std::ifstream& stream) {
//    int32_t result;
//    stream.read(reinterpret_cast<char*>(&result), sizeof(result));
//    return result;
//}
//
//int main() {
//    std::ifstream file("data_test/track.bmp", std::ios::binary);
//
//    if (!file) {
//        std::cerr << "Could not open file\n";
//        return 1;
//    }
//
//    uint16_t fileType = read_u16(file);
//    if (fileType != 0x4D42) {
//        std::cerr << "File is not a bmp\n";
//        return 1;
//    }
//
//    uint32_t fileSize = read_u32(file);
//    /* uint16_t reserved1 = */ read_u16(file);
//    /* uint16_t reserved2 = */ read_u16(file);
//    uint32_t dataOffset = read_u32(file);
//
//    // Read BMP info header
//    /* uint32_t size = */ read_u32(file);
//    int32_t width = read_i32(file);
//    int32_t height = read_i32(file);
//    /* uint16_t planes = */ read_u16(file);
//    uint16_t bitCount = read_u16(file);
//    /* uint32_t compression = */ read_u32(file);
//    /* uint32_t sizeImage = */ read_u32(file);
//    /* int32_t xPelsPerMeter = */ read_i32(file);
//    /* int32_t yPelsPerMeter = */ read_i32(file);
//    /* uint32_t clrUsed = */ read_u32(file);
//    /* uint32_t clrImportant = */ read_u32(file);
//
//    std::cout << "BMP file size: " << fileSize << "\n";
//    std::cout << "Data offset: " << dataOffset << "\n";
//    std::cout << "Image width: " << width << "\n";
//    std::cout << "Image height: " << height << "\n";
//    std::cout << "Bit count: " << bitCount << "\n";  // This value determines the number of bits that define each pixel and the maximum number of colors in the bitmap.
//
//    file.seekg(dataOffset, file.beg); // Move read position to start of pixel data
//
//    int row_padded = (width*bitCount/8 + 3) & (~3);
//    int total_size = row_padded * height;
//    std::vector<uint8_t> allPixels(total_size);
//
//    file.read(reinterpret_cast<char*>(allPixels.data()), total_size);
//
//    for (int i = height-1; i >= 0; i--) {
//        uint8_t* pixelRow = allPixels.data() + (i * row_padded);
//
//        for (int j = 0; j < width; j++) {
//            // In a BMP file, each pixel is represented by 3 bytes (BGR format)
//            uint8_t blue = pixelRow[j*3];
//            uint8_t green = pixelRow[j*3 + 1];
//            uint8_t red = pixelRow[j*3 + 2];
//
//            // Print the color of each pixel
//            std::cout << "Pixel (" << j << "," << height-1 - i << ") : R=" << (int)red << " G=" << (int)green << " B=" << (int)blue << "\n";
//        }
//    }
//
//    return 0;
//}


#include <rl_tools/operations/cpu.h>
#include <rl_tools/rl/environments/car/operations_cpu.h>
#include <rl_tools/rl/environments/car/ui.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;


int main(){
    using DEV_SPEC = rlt::devices::DefaultCPUSpecification;
    using DEVICE = rlt::devices::CPU<DEV_SPEC>;
//    using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;
    using T = float;
    using TI = typename DEVICE::index_t;

    using ENV_SPEC = rlt::rl::environments::car::SpecificationTrack<T, DEVICE::index_t, 100, 100, 20>;
    using ENVIRONMENT = rlt::rl::environments::CarTrack<ENV_SPEC>;
#if RL_TOOLS_ENABLE_GTK
    using UI = rlt::rl::environments::car::UI<rlt::rl::environments::car::ui::Specification<T, TI, ENVIRONMENT, 1000, 60>>;
#else
    using UI = bool;
#endif
    DEVICE device;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters env_parameters;
    UI ui;
    ENVIRONMENT::State state;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM>> observation;
    rlt::malloc(device, action);
    rlt::malloc(device, observation);
    rlt::set_all(device, action, 0);
    rlt::set(action, 0, 1, 20.0/180.0*rlt::math::PI<T>);
    rlt::init(device, env, env_parameters);
    rlt::init(device, env, env_parameters, ui);
    rlt::initial_parameters(device, env, env_parameters);
    rlt::initial_state(device, env, env_parameters, state);
    std::cout << "BOUND_X_LOWER: " << ENVIRONMENT::SPEC::BOUND_X_LOWER << std::endl;
    TI counter = 0;
    while(true){
        counter++;
        if(counter % 1 == 0){
            rlt::sample_initial_state(device, env, env_parameters, state, rng);
        }
        rlt::set_state(device, env, env_parameters, ui, state);
        rlt::set_action(device, env, env_parameters, ui, action);
        rlt::render(device, env, env_parameters, ui);
        rlt::observe(device, env, env_parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);
        std::cout << "lidar: " << get(observation, 0, 6) << ", " << get(observation, 0, 7) << ", " << get(observation, 0, 8) << std::endl;
    }
    return 0;
}