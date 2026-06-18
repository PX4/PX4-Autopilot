#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UI_SERVER_SERVER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UI_SERVER_SERVER_H

#include "../rl_tools.h"
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio.hpp>
#include <boost/beast/websocket.hpp>
#include <ctime>
#include <iostream>
#include <string>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include <thread>
#include <chrono>




RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::ui_server{
    namespace beast = boost::beast;
    namespace http = beast::http;
    namespace net = boost::asio;
    using tcp = boost::asio::ip::tcp;

    class websocket_session;

    class State{
    public:
        std::string scenario;
        std::vector<int> namespaces;
        std::vector<std::weak_ptr<websocket_session>> ui_sessions;
        std::vector<std::weak_ptr<websocket_session>> backend_sessions;
        std::mutex mutex;
        int request_count = 0;
        std::string now(){
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::string now_s = std::ctime(&now_c);
            now_s.pop_back();
            return now_s;
        }
    };

    class websocket_session : public std::enable_shared_from_this<websocket_session> {
    public:
        enum class TYPE{
            UI,
            BACKEND
        };
    private:
        beast::websocket::stream<tcp::socket> ws_;
        std::vector<std::string> queue_;

        State& state;
        TYPE type;
        bool verbose_;
    public:
        explicit websocket_session(tcp::socket socket, State& state, TYPE type, bool verbose) : ws_(std::move(socket)), state(state), type(type), verbose_(verbose) {}

        template<class Body>
        void run(http::request<Body>&& req) {
            ws_.async_accept(req, beast::bind_front_handler(&websocket_session::on_accept, shared_from_this()));
        }

        void on_accept(beast::error_code ec){
            if(ec) return;
            if(type == TYPE::UI){
                std::cout << "UI connected" << std::endl;
            }
            else if(type == TYPE::BACKEND){
                int new_namespace;
                state.mutex.lock();
                new_namespace = state.namespaces.size();
                state.namespaces.push_back(new_namespace);
                state.mutex.unlock();
                std::cout << "Backend connected: " << new_namespace << std::endl;
                nlohmann::json message;
                message["channel"] = "handshake";
                message["data"]["namespace"] = std::to_string(new_namespace);
//            ws_.async_write(net::buffer(message.dump()), beast::bind_front_handler(&websocket_session::on_write, shared_from_this()));
                send_message(message.dump());
            }
            state.mutex.lock();
            if(type == websocket_session::TYPE::UI){
                state.ui_sessions.push_back(shared_from_this());
            }
            else{
                state.backend_sessions.push_back(shared_from_this());
            }
            state.mutex.unlock();
            do_read();
        }

        void do_read() {
            ws_.async_read(buffer_, beast::bind_front_handler(&websocket_session::on_read, shared_from_this()));
        }

        void refresh(){
            if(false){
                // terminate connection here
                ws_.async_close(beast::websocket::close_code::normal, beast::bind_front_handler(&websocket_session::on_close, shared_from_this()));
            }
        }

        void on_close(beast::error_code ec) {
            if(ec) {
                std::cerr << "WebSocket close failed: " << ec.message() << std::endl;
                return;
            }
            ws_.next_layer().shutdown(tcp::socket::shutdown_both, ec);
            ws_.next_layer().close(ec);
        }

        void send_message(std::string message){
            boost::asio::post(ws_.get_executor(), beast::bind_front_handler( &websocket_session::on_send_message, shared_from_this(), message));
//        ws_.async_write(net::buffer(message.dump()), beast::bind_front_handler(&websocket_session::on_write, shared_from_this()));
        }
        void on_send_message(std::string message) {
            // Always add to queue
            queue_.push_back(message);

            // Are we already writing?
            if(queue_.size() > 1)
                return;

            // We are not currently writing, so send this immediately
            ws_.async_write(
                    net::buffer(queue_.front()),
                    beast::bind_front_handler(
                            &websocket_session::on_write,
                            shared_from_this()));
        }


        void on_read(beast::error_code ec, std::size_t bytes_transferred) {
            boost::ignore_unused(bytes_transferred);
            if(ec) return;

            auto message_string = beast::buffers_to_string(buffer_.data());
            buffer_.consume(buffer_.size());
            if(message_string.empty()){
                do_read();
                return;
            }
            nlohmann::json message;
            try{
                message = nlohmann::json::parse(message_string);
            }
            catch(nlohmann::json::parse_error& e){
                std::cerr << "Error parsing message: " << e.what() << std::endl;
                std::cerr << "Malformed message: " << message_string << std::endl;
                do_read();
                return;
            }

            state.mutex.lock();
            if(type == TYPE::UI){
                for(auto& backend_session : state.backend_sessions){
                    if(auto backend_session_ptr = backend_session.lock()){
                        backend_session_ptr->send_message(message.dump());
                    }
                }
            }
            else if(type == TYPE::BACKEND){
                for(auto& ui_session : state.ui_sessions){
                    if(auto ui_session_ptr = ui_session.lock()){
                        ui_session_ptr->send_message(message.dump());
                    }
                }
            }
            state.mutex.unlock();

            if(verbose_){
                std::cout << "Message: " << message.dump() << std::endl;
            }
            if(message["channel"] == "startTraining"){
                std::cout << "startTraining message received" << std::endl;
            }
            do_read();
        }

        void on_write(beast::error_code ec, std::size_t bytes_transferred) {
            boost::ignore_unused(bytes_transferred);
            // Handle the error, if any
            if(ec)
                return;

            // Remove the string from the queue
            queue_.erase(queue_.begin());

            // Send the next message if any
            if(! queue_.empty())
                ws_.async_write(
                        net::buffer(queue_.front()),
                        beast::bind_front_handler(
                                &websocket_session::on_write,
                                shared_from_this()));
        }

    private:
        beast::flat_buffer buffer_;
    };


    class http_connection: public std::enable_shared_from_this<http_connection>
    {
    public:
        http_connection(tcp::socket socket, State& state, std::string static_path, bool verbose): static_path(static_path), state(state), socket_(std::move(socket)), verbose_(verbose){ }
        void start(){
            read_request();
            check_deadline();
        }

    private:
        std::string static_path;
        State& state;
        tcp::socket socket_;
        beast::flat_buffer buffer_{8192};
        http::request<http::dynamic_body> request_;
        http::response<http::dynamic_body> response_;
        bool verbose_;
        net::steady_timer deadline_{socket_.get_executor(), std::chrono::seconds(60)};
        void read_request(){
            auto self = shared_from_this();

            http::async_read(
                    socket_,
                    buffer_,
                    request_,
                    [self](beast::error_code ec,
                           std::size_t bytes_transferred)
                    {
                        boost::ignore_unused(bytes_transferred);
                        if(!ec)
                            self->process_request();
                    });
        }

        void process_request(){
            response_.version(request_.version());
            response_.keep_alive(false);

            switch(request_.method())
            {
                case http::verb::get:
                    response_.result(http::status::ok);
                    response_.set(http::field::server, "Beast");
                    create_response();
                    break;

                default:
                    // We return responses indicating an error if
                    // we do not recognize the request method.
                    response_.result(http::status::bad_request);
                    response_.set(http::field::content_type, "text/plain");
                    beast::ostream(response_.body())
                            << "Invalid request-method '"
                            << std::string(request_.method_string())
                            << "'";
                    break;
            }
            write_response();
        }

        void create_response(){
            if(request_.target() == "/count"){
                response_.set(http::field::content_type, "text/html");
                beast::ostream(response_.body())
                        << "<html>\n"
                        <<  "<head><title>Request count</title></head>\n"
                        <<  "<body>\n"
                        <<  "<h1>Request count</h1>\n"
                        <<  "<p>There have been "
                        <<  state.request_count
                        <<  " requests so far.</p>\n"
                        <<  "</body>\n"
                        <<  "</html>\n";
            }
            else if(request_.target() == "/time"){
                response_.set(http::field::content_type, "text/html");
                beast::ostream(response_.body())
                        <<  "<html>\n"
                        <<  "<head><title>Current time</title></head>\n"
                        <<  "<body>\n"
                        <<  "<h1>Current time</h1>\n"
                        <<  "<p>The current time is "
                        <<  state.now()
                        <<  " seconds since the epoch.</p>\n"
                        <<  "</body>\n"
                        <<  "</html>\n";
            }
            else if(request_.target() == "/ui"){
                maybe_upgrade(websocket_session::TYPE::UI);
            }
            else if(request_.target() == "/backend"){
                maybe_upgrade(websocket_session::TYPE::BACKEND);
            }
            else if(request_.target() == "/scenario"){
                response_.set(http::field::content_type, "text/html");
                beast::ostream(response_.body())
                        <<  state.scenario;
            }
            else{
                std::filesystem::path path(std::string(request_.target()));
                if(path.empty() || path == "/"){
                    path = "/index.html";
                }
                path = static_path + path.string();
                // check if file at path exists

                if(std::filesystem::exists(path)){
                    response_.result(http::status::ok);
                    // check extension and use correct content_type
                    if(path.extension() == ".html")
                        response_.set(http::field::content_type, "text/html");
                    else if(path.extension() == ".js")
                        response_.set(http::field::content_type, "application/javascript");
                    else if(path.extension() == ".wasm")
                        response_.set(http::field::content_type, "application/wasm");
                    else if(path.extension() == ".css")
                        response_.set(http::field::content_type, "text/css");
                    else if(path.extension() == ".png")
                        response_.set(http::field::content_type, "image/png");
                    else if(path.extension() == ".jpg")
                        response_.set(http::field::content_type, "image/jpeg");
                    else if(path.extension() == ".gif")
                        response_.set(http::field::content_type, "image/gif");
                    else if(path.extension() == ".ico")
                        response_.set(http::field::content_type, "image/x-icon");
                    else if(path.extension() == ".txt")
                        response_.set(http::field::content_type, "text/plain");
                    else
                        response_.set(http::field::content_type, "application/octet-stream");
                    beast::ostream(response_.body()) << std::ifstream(path).rdbuf();
                }
                else{
                    response_.result(http::status::not_found);
                    response_.set(http::field::content_type, "text/plain");
                    beast::ostream(response_.body()) << "File not found\r\n";
                    std::cout << "File not found: " << path << " (you might need to run \"download_dependencies.sh\" to download the UI dependencies into the static folder)" << std::endl;
                }

//            response_.result(http::status::not_found);
//            response_.set(http::field::content_type, "text/plain");
//            beast::ostream(response_.body()) << "File not found\r\n";
            }
        }
        void maybe_upgrade(websocket_session::TYPE type) {
            if (beast::websocket::is_upgrade(request_)) {
                // Construct the WebSocket session and run it
                auto ws_session = std::make_shared<websocket_session>(std::move(socket_), state, type, verbose_);
                ws_session->run(std::move(request_));
            }
        }


        void write_response(){
            auto self = shared_from_this();

            response_.content_length(response_.body().size());

            http::async_write(
                    socket_,
                    response_,
                    [self](beast::error_code ec, std::size_t)
                    {
                        self->socket_.shutdown(tcp::socket::shutdown_send, ec);
                        self->deadline_.cancel();
                    });
        }

        void check_deadline(){
            auto self = shared_from_this();

            deadline_.async_wait(
                    [self](beast::error_code ec){
                        if(!ec){
                            self->socket_.close(ec);
                        }
                    });
        }
    };

    void http_server(tcp::acceptor& acceptor, tcp::socket& socket, State& state, std::string& static_path, bool verbose){
        acceptor.async_accept(socket, [&, verbose](beast::error_code ec){
            if(!ec)
                std::make_shared<http_connection>(std::move(socket), state, static_path, verbose)->start();
            http_server(acceptor, socket, state, static_path, verbose);
        });
    }

    struct UIServer{
        boost::asio::ip::address address;
        unsigned short port;
        net::io_context* ioc;
        tcp::acceptor* acceptor;
        boost::asio::ip::tcp::socket* socket;
        State state;
        std::thread thread;
        std::string static_path;
        bool running;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE>
    void init(DEVICE& device, ui_server::UIServer& ui_server, std::string address, unsigned short port, std::string static_path, std::string scenario, bool verbose){
        namespace beast = boost::beast;
        namespace http = beast::http;
        namespace net = boost::asio;
        using tcp = boost::asio::ip::tcp;
        ui_server.static_path = static_path;
        ui_server.state.scenario = scenario;
        ui_server.address = net::ip::make_address(address);
        ui_server.port = static_cast<unsigned short>(port);
        ui_server.ioc = new net::io_context{1};
        ui_server.acceptor = new tcp::acceptor{*ui_server.ioc, {ui_server.address, ui_server.port}};
        ui_server.socket = new tcp::socket{*ui_server.ioc};
        ui_server::http_server(*ui_server.acceptor, *ui_server.socket, ui_server.state, ui_server.static_path, verbose);
        std::cout << "Web interface coming up at: http://" << address << ":" << port << std::endl;
        ui_server.thread = std::thread([&](){

            boost::asio::signal_set signals(*ui_server.ioc, SIGINT);

            signals.async_wait(
                    [&](const boost::system::error_code& error, int signal_number) {
                        ui_server.ioc->stop();
                        ui_server.running = false;
                    }
            );
            ui_server.ioc->run();
        });
        ui_server.thread.detach();
        ui_server.running = true;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif