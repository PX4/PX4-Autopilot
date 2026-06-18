#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UI_SERVER_CLIENT_OPERATIONS_BOOST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UI_SERVER_CLIENT_OPERATIONS_BOOST_H

#include "operations_cpu.h"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace ui_server::client{
        namespace beast = boost::beast;
        namespace http = beast::http;
        namespace websocket = beast::websocket;
        namespace net = boost::asio;
        using tcp = boost::asio::ip::tcp;
        template<typename T_ENVIRONMENT>
        struct UI: UIBuffered<T_ENVIRONMENT>{
            using ENVIRONMENT = T_ENVIRONMENT;
            std::string host = "localhost";
            std::string port = "8000";
            net::io_context ioc;
            websocket::stream <tcp::socket> ws{ioc};
        };
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void _flush(DEVICE& dev, ui_server::client::UI<ENVIRONMENT>& ui){
        if (ui.ws.is_open()) {
            while(!ui.buffer.empty()){
                ui.ws.write(boost::beast::net::buffer(ui.buffer.front()));
                ui.buffer.pop();
            }
        }
        else{
            std::cerr << "Error: websocket is not open" << std::endl;
        }
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void init(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UI<ENVIRONMENT>& ui){
        namespace beast = boost::beast;         // from <boost/beast.hpp>
        namespace http = beast::http;           // from <boost/beast/http.hpp>
        namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
        namespace net = boost::asio;            // from <boost/asio.hpp>
        using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
        tcp::resolver resolver{ui.ioc};
        auto const results = resolver.resolve(ui.host, ui.port);

        net::connect(ui.ws.next_layer(), results.begin(), results.end());
        ui.ws.handshake(ui.host, "/backend");
        std::cout << "Waiting for handshake" << std::endl;
        boost::beast::flat_buffer buffer;
        ui.ws.read(buffer);
        std::cout << beast::make_printable(buffer.data()) << std::endl;
        auto message_string = beast::buffers_to_string(buffer.data());
        std::cout << "Handshake received: " << message_string << std::endl;
        buffer.consume(buffer.size());
        auto message = nlohmann::json::parse(message_string);
        ui.ns = message["data"]["namespace"];

        init(dev, env, static_cast<ui_server::client::UIBuffered<ENVIRONMENT>&>(ui));
        _flush(dev, ui);
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void set_state(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UI<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        set_state(dev, env, static_cast<ui_server::client::UIBuffered<ENVIRONMENT>&>(ui), state);
        _flush(dev, ui);
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    void set_state(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UI<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state, const Matrix<ACTION_SPEC>& action){
        set_state(dev, env, static_cast<ui_server::client::UIBuffered<ENVIRONMENT>&>(ui), state, action);
        _flush(dev, ui);
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    void set_action(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UI<ENVIRONMENT>& ui, const Matrix<ACTION_SPEC>& action){
        set_action(dev, env, static_cast<ui_server::client::UIBuffered<ENVIRONMENT>&>(ui), action);
        _flush(dev, ui);
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void render(DEVICE& dev, ENVIRONMENT& env, ui_server::client::UI<ENVIRONMENT>& ui){
        std::this_thread::sleep_for(std::chrono::duration<decltype(env.parameters.dt)>((env.parameters.dt)));
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
