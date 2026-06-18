#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_LOGGING_OPERATIONS_CPU_TENSORBOARD_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_LOGGING_OPERATIONS_CPU_TENSORBOARD_H

#include "../containers/matrix/matrix.h"

#include <filesystem>
#include <cassert>
#include "operations_cpu.h"

#ifdef RL_TOOLS_ENABLE_LIBATTOPNG
#include <libattopng/libattopng.h>
#endif
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace logging::tensorboard{
        template <typename DEVICE, typename KEY_TYPE>
        void count_topic(DEVICE& device, devices::logging::CPU_TENSORBOARD<>& logger, KEY_TYPE key){ }
        template <typename DEVICE, typename KEY_TYPE>
        void count_topic(DEVICE& device, devices::logging::CPU_TENSORBOARD_FREQUENCY_EXTENSION& logger, KEY_TYPE key){
            if(!logger.topic_frequency_dict.count(key)){
                logger.topic_frequency_dict.insert({key, 0});
            }
            logger.topic_frequency_dict[key] += 1;
        }
        template <typename DEVICE>
        void print_topic_frequencies(DEVICE& device, devices::logging::CPU_TENSORBOARD_FREQUENCY_EXTENSION& logger){
            std::cout << "Tensorboard Topic Frequencies:" << std::endl;
            std::vector<std::pair<std::string, int>> vec(logger.topic_frequency_dict.begin(), logger.topic_frequency_dict.end());
            std::sort(vec.begin(), vec.end(),
                      [](const std::pair<std::string, int>& a, const std::pair<std::string, int>& b) {
                          return a.second > b.second; // For descending order
                      }
            );
            for (const auto& [key, val] : vec) {
                std::cout << key << " => " << val << std::endl;
            }
        }
    }
    template <typename DEVICE, typename SPEC>
    void init(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger, std::filesystem::path run_dir){
        if (!std::filesystem::is_directory(run_dir) || !std::filesystem::exists(run_dir)) {
            std::filesystem::create_directories(run_dir);
        }

        auto log_file = run_dir / std::string("logs.tfevents");
        std::cerr << "Tensorboard Logger logging to: " << log_file.string() << std::endl;
        TensorBoardLoggerOptions opts;
        opts.flush_period_s(1);
        logger.tb = new TensorBoardLogger(log_file.string(), opts);
    }
    template <typename DEVICE, typename SPEC>
    void free(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger){
        delete logger.tb;
    }
    template <typename DEVICE, typename SPEC>
    void set_step(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger, typename DEVICE::index_t step){
        logger.step = step;
    }
    template <typename DEVICE, typename SPEC>
    typename DEVICE::index_t get_step(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger){
        return logger.step;
    }
    template <typename DEVICE, typename KEY_TYPE, typename VALUE_TYPE, typename CADANCE_TYPE, typename SPEC>
    void add_scalar(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger, const KEY_TYPE key, const VALUE_TYPE value, const CADANCE_TYPE cadence){
        std::lock_guard<std::mutex> lock(logger.mutex);
        if(logger.tb){
            if(logger.step % cadence == 0){
                logger.tb->add_scalar(key, logger.step, (float)value);
                logging::tensorboard::count_topic(device, logger, key);
            }
        }
    }
    template <typename DEVICE, typename KEY_TYPE, typename VALUE_TYPE, typename SPEC>
    void add_scalar(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger, const KEY_TYPE key, const VALUE_TYPE value){
        add_scalar(device, logger, key, value, (typename DEVICE::index_t)1);
    }
    template <typename DEVICE, typename KEY_TYPE, typename T, typename TI, typename CADANCE_TYPE, typename SPEC>
    void add_histogram(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger, const KEY_TYPE key, const T* values, const TI n_values, const CADANCE_TYPE cadence = (typename DEVICE::index_t)1){
        std::lock_guard<std::mutex> lock(logger.mutex);
        if(logger.tb){
            if(logger.step % cadence == 0){
                logger.tb->add_histogram(key, logger.step, values, n_values);
                logging::tensorboard::count_topic(device, logger, key);
            }
        }
    }
    template <typename DEVICE, typename KEY_TYPE, typename T, typename TI, typename SPEC>
    void add_histogram(DEVICE& device, devices::logging::CPU_TENSORBOARD<SPEC>& logger, const KEY_TYPE key, const T* values, const TI n_values){
        add_histogram(device, logger, key, values, n_values, (typename DEVICE::index_t)1);
    }
#ifdef RL_TOOLS_ENABLE_LIBATTOPNG
    template <typename DEVICE, typename KEY_TYPE, typename LOGGING_SPEC, typename SPEC>
    void add_image(DEVICE& device, devices::logging::CPU_TENSORBOARD<LOGGING_SPEC>& logger, const KEY_TYPE key, rl_tools::Matrix<SPEC> values){
        std::lock_guard<std::mutex> lock(logger.mutex);
        if(logger.tb){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            libattopng_t* png = libattopng_new(SPEC::COLS, SPEC::ROWS, PNG_RGBA);

            for (TI y = 0; y < SPEC::ROWS; y++) {
                for (TI x = 0; x < SPEC::COLS; x++) {
                    uint8_t r = math::clamp<T>(device.math, get(values, x, y) * 255.0, 0, 255);
                    uint8_t g = r;
                    uint8_t b = r;
                    uint8_t a = 255;
                    uint32_t color = r | ((g) << 8) | ((b) << 16) | ((a) << 24);
                    libattopng_set_pixel(png, x, y, color);
                }
            }
            size_t len;
            char *data = libattopng_get_data(png, &len);
            std::string image_data = std::string(data, len);
            std::cout << "adding image at step: " << logger.step << std::endl;
            logger.tb->add_image(key, logger.step, image_data, SPEC::ROWS, SPEC::COLS, 4, "Test", "Image");
            libattopng_destroy(png);
            logging::tensorboard::count_topic(device, logger, key);
        }
    }
#else
    template <typename DEVICE, typename LOGGER_SPEC, typename SPEC>
    void add_image(DEVICE& device, devices::logging::CPU_TENSORBOARD<LOGGER_SPEC>& logger, rl_tools::Matrix<SPEC> values){ }
#endif
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
