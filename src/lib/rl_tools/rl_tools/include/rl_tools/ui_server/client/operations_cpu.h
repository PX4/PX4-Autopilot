#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UI_SERVER_CLIENT_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UI_SERVER_CLIENT_OPERATIONS_CPU_H

#include "client.h"

#include <thread>
#include <chrono>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace ui_server::client{
        std::string escape_json_string(const std::string& input) {
            std::string output;
            output.reserve(input.size());

            for (char c : input) {
                switch (c) {
                    case '"':  output += "\\\""; break;
                    case '\\': output += "\\\\"; break;
                    case '\b': output += "\\b"; break;
                    case '\f': output += "\\f"; break;
                    case '\n': output += "\\n"; break;
                    case '\r': output += "\\r"; break;
                    case '\t': output += "\\t"; break;
                    default:
                        if ('\x00' <= c && c <= '\x1f') {
                            // Control characters need to be escaped as \uXXXX
                            output += "\\u";
                            output += "0123456789abcdef"[(c >> 12) & 0xF];
                            output += "0123456789abcdef"[(c >> 8) & 0xF];
                            output += "0123456789abcdef"[(c >> 4) & 0xF];
                            output += "0123456789abcdef"[c & 0xF];
                        } else {
                            output += c;
                        }
                }
            }
            return output;
        }
    }
    template <typename DEVICE, typename ENVIRONMENT>
    std::string parameters_message(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIJSON<ENVIRONMENT>& ui){
        std::string parameters_json = json(dev, env, parameters);
        std::string parameters_message = "{";
        parameters_message += "\"namespace\": \"" + ui.ns + "\", ";
        parameters_message += "\"channel\": \"setParameters\", ";
        parameters_message += "\"data\": " + parameters_json;
        parameters_message += "}";
        return parameters_message;
    }
    template <typename DEVICE, typename ENVIRONMENT>
    std::string set_ui_message(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UIJSON<ENVIRONMENT>& ui, std::string render_function){
        std::string message = "{";
        message += "\"namespace\": \"" + ui.ns + "\", ";
        message += "\"channel\": \"setUI\", ";
        message += "\"latch\": true, ";
        std::string data = "{";
        data += "\"type\": \"2d\", ";
        data += "\"render_function\": \"" + ui_server::client::escape_json_string(render_function) + "\"";
        data += "}";
        message += "\"data\": " + data;
        message += "}";
        return message;
    }
    template <typename DEVICE, typename ENVIRONMENT>
    std::string set_state_message(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIJSON<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        std::string message = "{";
        message += "\"namespace\": \"" + ui.ns + "\", ";
        message += "\"channel\": \"setState\", ";
        std::string data = "{";
        data += "\"state\": " + std::string(json(dev, env, parameters, state));
        data += "}";
        message += "\"data\": " + data;
        message += "}";
        return message;
    }
    template <typename DEVICE, typename ENVIRONMENT>
    std::string set_truncated_message(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIJSON<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        std::string message = "{";
        message += "\"namespace\": \"" + ui.ns + "\", ";
        message += "\"channel\": \"setTruncated\", ";
        std::string data = "{";
        data += "\"state\": " + std::string(json(dev, env, parameters, state));
        data += "}";
        message += "\"data\": " + data;
        message += "}";
        return message;
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    std::string action_data(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UIJSON<ENVIRONMENT>& ui, const Matrix<ACTION_SPEC>& action){
        static_assert(ACTION_SPEC::COLS == ENVIRONMENT::ACTION_DIM);
//        static_assert(ACTION_SPEC::ROWS == 1);
        using T = typename ACTION_SPEC::T;
        using TI = typename DEVICE::index_t;
        std::string action_data = "[";
        for(TI row_i = 0; row_i < ACTION_SPEC::ROWS; row_i++){
            action_data += "[";
            for(TI action_i = 0; action_i < ACTION_SPEC::COLS; action_i++){
                action_data += std::to_string(get(action, row_i, action_i));
                if(action_i < ACTION_SPEC::COLS - 1){
                    action_data += ", ";
                }
            }
            action_data += "]";
            if(row_i < ACTION_SPEC::ROWS - 1){
                action_data += ", ";
            }
        }
        action_data += "]";
        return action_data;
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    std::string set_state_action_message(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIJSON<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state, const Matrix<ACTION_SPEC>& action){
        static_assert(ACTION_SPEC::COLS == ENVIRONMENT::ACTION_DIM);
//        static_assert(ACTION_SPEC::ROWS == 1);
        using T = typename ACTION_SPEC::T;
        using TI = typename DEVICE::index_t;
        std::string message = set_state_message(dev, env, parameters, ui, state);
        message = message.substr(0, message.size()-2);
        message += ", \"action\": " + action_data(dev, env, ui, action);
        message += "}}";
        return message;
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    std::string set_action_message(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIJSON<ENVIRONMENT>& ui, const Matrix<ACTION_SPEC>& action){
        static_assert(ACTION_SPEC::COLS == ENVIRONMENT::ACTION_DIM);
        static_assert(ACTION_SPEC::ROWS == 1);
        using T = typename ACTION_SPEC::T;
        using TI = typename DEVICE::index_t;
        std::string message = "{";
        message += "\"namespace\": " + ui.ns + ", ";
        message += "\"channel\": \"setState\", ";
        std::string data = "{";
        data += "\"action\": " + action_data(dev, env, ui, action);
        data += "}";
        message += "\"data\": " + data;
        message += "}";
        return message;
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void init(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIBuffered<ENVIRONMENT>& ui, std::string name_space){
        ui.buffer.push(parameters_message(dev, env, parameters, ui));
        ui.ns = name_space;
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void init(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIBuffered<ENVIRONMENT>& ui){
        init(dev, env, parameters, ui, "default");
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void set_state(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIBuffered<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        ui.buffer.push(set_state_message(dev, env, parameters, ui, state));
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void set_truncated(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIBuffered<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        ui.buffer.push(set_truncated_message(dev, env, parameters, ui, state));
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    void set_state(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIBuffered<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state, const Matrix<ACTION_SPEC>& action){
        ui.buffer.push(set_state_action_message(dev, env, parameters, ui, state, action));
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    void set_action(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIBuffered<ENVIRONMENT>& ui, const Matrix<ACTION_SPEC>& action){
        ui.buffer.push(set_action_message(dev, env, parameters, ui, action));
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void render(DEVICE& dev, ENVIRONMENT& env, typename ENVIRONMENT::Parameters&, ui_server::client::UIBuffered<ENVIRONMENT>& ui){
//        std::this_thread::sleep_for(std::chrono::duration<decltype(env.parameters.dt)>((env.parameters.dt)));
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif