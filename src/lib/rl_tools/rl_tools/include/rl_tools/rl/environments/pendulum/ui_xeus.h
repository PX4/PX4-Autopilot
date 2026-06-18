#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_UI_XEUS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_UI_XEUS_H

#include "xcanvas/xcanvas.hpp"
#include <thread>
#include <chrono>
// #include <string>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::pendulum::ui::xeus {
    template<typename T_T, typename T_TI, T_TI T_SIZE, T_TI T_PLAYBACK_SPEED>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
        static constexpr TI SIZE = T_SIZE;
        static constexpr T CENTER_X = SIZE/2.0;
        static constexpr T CENTER_Y = SIZE/2.0;
        static constexpr T PENDULUM_LENGTH = 0.75 * SIZE/2.0;
        static constexpr T PAYLOAD_DIAMETER = SIZE / 30.0;
        static constexpr T JOINT_DIAMETER = SIZE / 100.0;
        static constexpr T BEAM_WIDTH = 5;
        static constexpr T PLAYBACK_SPEED = T_PLAYBACK_SPEED/100.0;
        static constexpr T ACTION_INDICATOR_SIZE = SIZE * 1.0/10.0;
        static constexpr T ACTION_INDICATOR_WIDTH = ACTION_INDICATOR_SIZE / 4;
        static constexpr T ACTION_INDICATOR_TOP_OFFSET = SIZE * 0.9;
    };

    template<typename T_SPEC>
    struct UI {
        using SPEC = T_SPEC;
        xc::canvas canvas;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_render_time;
        rl::environments::pendulum::State<pendulum::StateSpecification<typename SPEC::T, typename SPEC::TI>> state;
        typename SPEC::T action;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename ENV_SPEC, typename PARAMETERS, typename SPEC>
    void render(DEVICE& device, const rl::environments::Pendulum<ENV_SPEC>& env, const PARAMETERS& parameters, rl::environments::pendulum::ui::xeus::UI<SPEC>& ui){
        auto now = std::chrono::high_resolution_clock::now();
        auto interval = (typename DEVICE::index_t)(1000.0 * ENV_SPEC::PARAMETERS::dt / SPEC::PLAYBACK_SPEED);
        auto next_render_time = ui.last_render_time + std::chrono::milliseconds(interval);
        if(now < next_render_time){
            auto diff = next_render_time - now;
            std::this_thread::sleep_for(diff);
        }

        using T = typename SPEC::T;
        T x, y;
        x = std::sin(ui.state.theta + math::PI<T>)*SPEC::PENDULUM_LENGTH + SPEC::CENTER_X;
        y = std::cos(ui.state.theta + math::PI<T>)*SPEC::PENDULUM_LENGTH + SPEC::CENTER_Y;

        ui.canvas.cache();
        ui.canvas.clear();

        {
            std::string color_green = "rgb(0, 200, 0)";
            std::string color_red = "rgb(200, 0, 0)";
            if(ui.action > 0){
                T a = ui.action;
                ui.canvas.fill_style = color_green;
                ui.canvas.fill_rect(SPEC::CENTER_X, SPEC::ACTION_INDICATOR_TOP_OFFSET, SPEC::ACTION_INDICATOR_SIZE/2 * a, SPEC::ACTION_INDICATOR_WIDTH);
            }
            else{
                T a = -ui.action;
                ui.canvas.fill_style = color_red;
                ui.canvas.fill_rect(SPEC::CENTER_X - SPEC::ACTION_INDICATOR_SIZE/2 + SPEC::ACTION_INDICATOR_SIZE/2 * (1-a), SPEC::ACTION_INDICATOR_TOP_OFFSET, SPEC::ACTION_INDICATOR_SIZE/2 * a, SPEC::ACTION_INDICATOR_WIDTH);
            }
            ui.canvas.fill_style = "black";
            ui.canvas.fill_rect(SPEC::CENTER_X - SPEC::ACTION_INDICATOR_WIDTH/2/4, SPEC::ACTION_INDICATOR_TOP_OFFSET, SPEC::ACTION_INDICATOR_WIDTH/4, SPEC::ACTION_INDICATOR_WIDTH);
        }
        {
            ui.canvas.fill_circle(x, y, SPEC::PAYLOAD_DIAMETER);
            ui.canvas.stroke_line(SPEC::CENTER_X, SPEC::CENTER_Y, x, y);
            ui.canvas.fill_circle(SPEC::CENTER_X, SPEC::CENTER_Y, SPEC::JOINT_DIAMETER);
        }
        // {
        //     ui.canvas.fill_rect(SPEC::CENTER_X - SPEC::ACTION_INDICATOR_SIZE/2, SPEC::ACTION_INDICATOR_TOP_OFFSET + SPEC::ACTION_INDICATOR_SIZE/2 - SPEC::ACTION_INDICATOR_WIDTH/2, SPEC::ACTION_INDICATOR_SIZE, SPEC::ACTION_INDICATOR_WIDTH);
        //     if(ui.action > 0){
        //         ui.canvas.fill_rect(SPEC::CENTER_X - SPEC::ACTION_INDICATOR_WIDTH / 2, SPEC::ACTION_INDICATOR_TOP_OFFSET, SPEC::ACTION_INDICATOR_WIDTH, SPEC::ACTION_INDICATOR_SIZE);
        //     }
        // }
        ui.canvas.flush();

        now = std::chrono::high_resolution_clock::now();
        ui.last_render_time = now;
    }
    template <typename DEVICE, typename ENV_SPEC, typename PARAMETERS, typename SPEC>
    void init(DEVICE& device, const rl::environments::Pendulum<ENV_SPEC>& env, const PARAMETERS& parameters, rl::environments::pendulum::ui::xeus::UI<SPEC>& ui){
        ui.canvas = xc::canvas().initialize()
            .width(300)
            .height(300)
            .finalize();        
        ui.canvas.line_width = SPEC::BEAM_WIDTH;
        ui.state.theta = 0;
        ui.state.theta_dot = 0;
        ui.action = 0;
        render(device, ui);
        ui.last_render_time = std::chrono::high_resolution_clock::now();
    }
    template <typename DEVICE, typename ENV_SPEC, typename PARAMETERS, typename SPEC, typename STATE>
    void set_truncated(DEVICE& device, const rl::environments::Pendulum<ENV_SPEC>& env, const PARAMETERS& parameters, rl::environments::pendulum::ui::xeus::UI<SPEC>& ui, const STATE& state){
    }
    template <typename DEVICE, typename ENV_SPEC, typename SPEC, typename PARAMETERS, typename STATE, typename ACTION_SPEC>
    void set_state(DEVICE& device, const rl::environments::Pendulum<ENV_SPEC>& env, const PARAMETERS& parameters, rl::environments::pendulum::ui::xeus::UI<SPEC>& ui, const STATE& state, const Matrix<ACTION_SPEC>& action){
        static_assert(ACTION_SPEC::ROWS == 1 && ACTION_SPEC::COLS == 1);
        ui.state = state;
        ui.action = get(action, 0, 0);
    }
    template <typename DEVICE, typename ENV_SPEC, typename SPEC, typename PARAMETERS, typename ACTION_SPEC>
    void set_action(DEVICE& device, const rl::environments::Pendulum<ENV_SPEC>& env, const PARAMETERS& parameters, rl::environments::pendulum::ui::xeus::UI<SPEC>& ui, const Matrix<ACTION_SPEC>& action){
        static_assert(ACTION_SPEC::ROWS == 1 && ACTION_SPEC::COLS == 1);
        ui.action = get(action, 0, 0);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif