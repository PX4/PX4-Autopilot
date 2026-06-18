#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_CAR_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_CAR_OPERATIONS_CPU_H

#include <stdint.h>
#include <fstream>
#include "operations_generic.h"
#include <cassert>
#include <filesystem>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::car{
    uint16_t read_u16(std::ifstream& stream) {
        uint16_t result;
        stream.read(reinterpret_cast<char*>(&result), sizeof(result));
        return result;
    }

    uint32_t read_u32(std::ifstream& stream) {
        uint32_t result;
        stream.read(reinterpret_cast<char*>(&result), sizeof(result));
        return result;
    }

    int32_t read_i32(std::ifstream& stream) {
        int32_t result;
        stream.read(reinterpret_cast<char*>(&result), sizeof(result));
        return result;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC, typename SPEC>
    void init(devices::CPU<DEV_SPEC>& device, rl::environments::CarTrack<SPEC>& env){
        using DEVICE = devices::CPU<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::T;
        using namespace rl::environments::car;
        std::filesystem::path file_path = "src/rl/environments/car/track.bmp";
        std::ifstream file(file_path, std::ios::binary);

        if(!file){
            std::cout << "Error opening file" << file_path << " (the working directory for this executable is assumed to be the root of the repo)" << std::endl;
            std::abort();
        }

        uint16_t fileType = read_u16(file);
        assert(fileType == 0x4D42);  //File is not a bmp

        uint32_t fileSize = read_u32(file);
        /* uint16_t reserved1 = */ read_u16(file);
        /* uint16_t reserved2 = */ read_u16(file);
        uint32_t dataOffset = read_u32(file);

        // Read BMP info header
        /* uint32_t size = */ read_u32(file);
        int32_t width = read_i32(file);
        int32_t height = read_i32(file);
        /* uint16_t planes = */ read_u16(file);
        uint16_t bitCount = read_u16(file);
        /* uint32_t compression = */ read_u32(file);
        /* uint32_t sizeImage = */ read_u32(file);
        /* int32_t xPelsPerMeter = */ read_i32(file);
        /* int32_t yPelsPerMeter = */ read_i32(file);
        /* uint32_t clrUsed = */ read_u32(file);
        /* uint32_t clrImportant = */ read_u32(file);
        assert(bitCount == 24);
        assert(height == SPEC::HEIGHT);
        assert(width == SPEC::WIDTH);

        std::cout << "BMP file size: " << fileSize << "\n";
        std::cout << "Data offset: " << dataOffset << "\n";
        std::cout << "Image width: " << width << "\n";
        std::cout << "Image height: " << height << "\n";
        std::cout << "Bit count: " << bitCount
                  << "\n";  // This value determines the number of bits that define each pixel and the maximum number of colors in the bitmap.

        file.seekg(dataOffset, file.beg); // Move read position to start of pixel data

        constexpr TI ROW_PADDED = (SPEC::WIDTH * 3 + 3) & (~3);
        constexpr TI TOTAL_SIZE = ROW_PADDED * SPEC::HEIGHT;
        uint8_t allPixels[TOTAL_SIZE];

        file.read(reinterpret_cast<char *>(allPixels), TOTAL_SIZE);

        std::cout << "Track: ";
        for(int i = height - 1; i >= 0; i--){
            uint8_t *pixelRow = allPixels + (i * ROW_PADDED);

            for (int j = 0; j < width; j++) {
                uint8_t blue = pixelRow[j * 3];
                uint8_t green = pixelRow[j * 3 + 1];
                uint8_t red = pixelRow[j * 3 + 2];

//                std::cout << "Pixel (" << j << "," << height - 1 - i << ") : R=" << (int) red << " G=" << (int) green << " B=" << (int) blue << "\n";
                bool drivable = (red > 0 || green > 0 || blue > 0) ? 1 : 0;
                env.parameters.track[height - 1 - i][j] = drivable;
                std::cout << (drivable ? "  " : "xx");
            }
            std::cout << std::endl;
        }
        env.initialized = true;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif