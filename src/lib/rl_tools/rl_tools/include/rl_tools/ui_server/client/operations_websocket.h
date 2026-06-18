#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UI_SERVER_CLIENT_OPERATIONS_WEBSOCKET_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UI_SERVER_CLIENT_OPERATIONS_WEBSOCKET_H

#include "client.h"

#include <thread>
#include <chrono>
#include <cstring>

#include "operations_cpu.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace ui_server::client::websocket{
        template <typename ENVIRONMENT>
        void connect(ui_server::client::UIWebSocket<ENVIRONMENT>& ui){
            ui.conn_info.context = ui.context;
            ui.conn_info.address = ui.address.c_str();
            ui.conn_info.port = ui.port;
            ui.conn_info.path = "/backend";
            ui.conn_info.host = lws_canonical_hostname(ui.context);
            ui.conn_info.origin = "origin";
            ui.conn_info.protocol = "ui_server";
            ui.conn_info.ssl_connection = 0;
            ui.conn_info.userdata = &ui;

            if(ui.verbose){
                lws_set_log_level(LLL_ERR | LLL_WARN | LLL_NOTICE | LLL_INFO | LLL_DEBUG | LLL_PARSER | LLL_HEADER | LLL_EXT | LLL_CLIENT | LLL_LATENCY, NULL);
            }

            ui.wsi = lws_client_connect_via_info(&ui.conn_info);
        }
        template <typename ENVIRONMENT>
        static int callback(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
            using UI = ui_server::client::UIWebSocket<ENVIRONMENT>;
            UI *ui = (UI*) user;
            switch (reason) {
                case LWS_CALLBACK_CLIENT_ESTABLISHED:
                    lwsl_user("Client connected\n");
                    ui->connected = true;
                    lws_callback_on_writable(ui->wsi);
                    break;
                case LWS_CALLBACK_CLIENT_WRITEABLE:
                    {
                        std::string message;
                        bool messages_left = false;
                        bool found_message = false;
                        {
                            std::lock_guard guard(ui->message_queue_mutex);
                            if(!ui->message_queue.empty()){
                                found_message = true;
                                message = ui->message_queue.front();
                                ui->message_queue.pop();
                            }
                            if(!ui->message_queue.empty()){
                                messages_left = true;
                            }
                        }
                        size_t buf_size = LWS_PRE + message.size() + 1; // +1 for null terminator
                        std::vector<unsigned char> buf(buf_size); // Use a vector for dynamic allocation
                        std::memcpy(buf.data() + LWS_PRE, message.c_str(), message.size() + 1);
                        lws_write(ui->wsi, buf.data() + LWS_PRE, message.size(), LWS_WRITE_TEXT);
                        if(ui->verbose){
                            std::cout << std::string("Message \"") + message + "\" sent." << std::endl;
                        }
                        if(messages_left){
                            lws_callback_on_writable(ui->wsi);
                        }
                    }
                    break;
                case LWS_CALLBACK_CLIENT_RECEIVE:
                    lwsl_user("Received: %s\n", (char *)in);
                    break;
                case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
                    std::cerr << "Client connection error: " << (char *)in << std::endl;
                    ui->connected = false;
                    ui->error = true;
                    lws_cancel_service(ui->context);
                    break;
                case LWS_CALLBACK_CLIENT_CLOSED:
                    lwsl_user("Client closed\n");
                    ui->connected = false;
                    lws_cancel_service(ui->context);
                    break;
                default:
                    if(ui && ui->verbose){
                        std::cout << "Unhandled callback: " << reason << std::endl;
                    }
                    break;
            }
            return 0;
        }
        template <typename ENVIRONMENT>
        const struct lws_protocols protocols[] = {{"ui_server", callback<ENVIRONMENT>, sizeof(UI*), 0, 0, nullptr, 0}, LWS_PROTOCOL_LIST_TERM};

        template <typename DEVICE, typename ENVIRONMENT>
        void send_message(DEVICE& device, ui_server::client::UIWebSocket<ENVIRONMENT>& ui, std::string message){
            utils::assert_exit(device, message.length() > 0, "Message is empty");
            {
                std::lock_guard guard(ui.message_queue_mutex);
                ui.message_queue.push(message);
            }
            lws_cancel_service(ui.context);
        }
    }

    template <typename DEVICE, typename ENVIRONMENT>
    void init(DEVICE& device, ENVIRONMENT& env, typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIWebSocket<ENVIRONMENT>& ui, std::string name_space = "default", bool verbose = false){
        // note the &env needs to be alive until free is called
        using UI = ui_server::client::UIWebSocket<ENVIRONMENT>;
        ui.ns = name_space;
        ui.verbose = verbose;
        ui.connected = false;
        ui.interrupt = false;
        if(ui.address.empty()){
            ui.address = "localhost";
        }
        if(ui.port == 0){
            ui.port = 13337; // default is 13337 so it does not clash with other services at e.g. 8000 or 8888
        }
        memset(&ui.ctx_info, 0, sizeof(ui.ctx_info));
        memset(&ui.conn_info, 0, sizeof(ui.conn_info));

        ui.ctx_info.port = CONTEXT_PORT_NO_LISTEN;
        ui.ctx_info.protocols = ui_server::client::websocket::protocols<ENVIRONMENT>;
        ui.ctx_info.options = 0;
        ui.context = lws_create_context(&ui.ctx_info);
        utils::assert_exit(device, ui.context, "lws_create_context failed");

        ui.thread = std::thread([&device, &env, &parameters, &ui](){
            std::chrono::steady_clock::time_point last_sync_time;
            bool last_sync_time_set = false;
            while (!ui.interrupt) {
                bool was_not_connected = false;
                if(!ui.connected){
                    ui.error = false;
                    log(device, device.logger, "Not connected, wiping message buffer");
                    {
                        std::lock_guard guard(ui.message_queue_mutex);
                        while(!ui.message_queue.empty()){
                            ui.message_queue.pop();
                        }
                    }
                    log(device, device.logger, "Waiting for connection to the ui_server at " + ui.address + ":" + std::to_string(ui.port) + " ...");
                    ui_server::client::websocket::connect(ui);
                    was_not_connected = true;
                }
                while (!ui.connected && !ui.error && !ui.interrupt) {
                    lws_service(ui.context, 250);
                }
                if(ui.error || ui.interrupt){
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }
                if(was_not_connected){
                    log(device, device.logger, "UI connected.");
                }
                auto now = std::chrono::steady_clock::now();
                if(was_not_connected || !last_sync_time_set || (now- last_sync_time) > std::chrono::milliseconds(ui.sync_interval)){
                    last_sync_time_set = true;
                    last_sync_time = now;
                    ui_server::client::websocket::send_message(device, ui, parameters_message(device, env, parameters, ui));
                    std::string render_function = get_ui(device, env);
                    if(!render_function.empty()){
                        ui_server::client::websocket::send_message(device, ui, set_ui_message(device, env, ui, render_function));
                    }
                }
                lws_service(ui.context, 50);
                bool messages_left = false;
                {
                    std::lock_guard guard(ui.message_queue_mutex);
                    messages_left = !ui.message_queue.empty();
                }
                if(messages_left){
                    lws_callback_on_writable(ui.wsi);
                }
            }
        });


    }
    template <typename DEVICE, typename ENVIRONMENT>
    void free(DEVICE& device, ENVIRONMENT& env, ui_server::client::UIWebSocket<ENVIRONMENT>& ui){
        ui.interrupt = true;
        lws_cancel_service(ui.context);
        ui.thread.join();
        lws_context_destroy(ui.context);
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void set_state(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIWebSocket<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state){
        ui_server::client::websocket::send_message(dev, ui, set_state_message(dev, env, parameters, ui, state));
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    void set_state(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIWebSocket<ENVIRONMENT>& ui, const typename ENVIRONMENT::State& state, const Matrix<ACTION_SPEC>& action){
        ui_server::client::websocket::send_message(dev, ui, set_state_action_message(dev, env, parameters, ui, state, action));
    }
    template <typename DEVICE, typename ENVIRONMENT, typename ACTION_SPEC>
    void set_action(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIWebSocket<ENVIRONMENT>& ui, const Matrix<ACTION_SPEC>& action){
        ui_server::client::websocket::send_message(dev, ui, set_action_message(dev, env, ui, action));
    }
    template <typename DEVICE, typename ENVIRONMENT>
    void render(DEVICE& dev, ENVIRONMENT& env, const typename ENVIRONMENT::Parameters& parameters, ui_server::client::UIWebSocket<ENVIRONMENT>& ui){
//        std::this_thread::sleep_for(std::chrono::duration<decltype(env.parameters.dt)>((env.parameters.dt)));
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
