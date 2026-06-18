//std::thread t([&](){
//    UI_CLIENT client;
//    rlt::init(state->device, state->ts.env_eval, client);
//    while(true){
//        std::cout << "Waiting for message" << std::endl;
//        boost::beast::flat_buffer buffer;
//        client.ws.read(buffer);
//        std::cout << boost::beast::make_printable(buffer.data()) << std::endl;
//        auto message_string = boost::beast::buffers_to_string(buffer.data());
//        std::cout << "message received: " << message_string << std::endl;
//        buffer.consume(buffer.size());
//        {
//            std::lock_guard<std::mutex> lock(mutex);
//            receiving_message_queue.push(message_string);
//        }
//        {
//            std::queue<std::string> out_messages;
//            {
//                std::lock_guard<std::mutex> lock(mutex);
//                while(!sending_message_queue.empty()){
//                    auto message = sending_message_queue.front();
//                    sending_message_queue.pop();
//                    out_messages.push(message);
//                }
//            }
//            while(!out_messages.empty()){
//                auto message = out_messages.front();
//                out_messages.pop();
//                client.ws.write(boost::beast::net::buffer(message));
//            }
//
//        }
//    }
//});
//t.detach();


#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/strand.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <queue>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

void
fail(beast::error_code ec, char const* what)
{
    std::cerr << what << ": " << ec.message() << "\n";
}

class Forwarder : public std::enable_shared_from_this<Forwarder>
{
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::string host_;
public:
    std::mutex mutex;
    bool handshook = false;
    std::queue<std::string> receiving_message_queue, sending_message_queue;
    std::queue<std::string> queue_;

public:
    explicit
    Forwarder(net::io_context& ioc) : resolver_(net::make_strand(ioc)) , ws_(net::make_strand(ioc)){ }
    void run( char const* host, char const* port) {
        host_ = host;
        resolver_.async_resolve(host, port, beast::bind_front_handler( &Forwarder::on_resolve, shared_from_this()));}
    void on_resolve( beast::error_code ec, tcp::resolver::results_type results) {
        if(ec){
            return fail(ec, "resolve");
        }
        // Set the timeout for the operation
        beast::get_lowest_layer(ws_).expires_after(std::chrono::seconds(30));
        // Make the connection on the IP address we get from a lookup
        beast::get_lowest_layer(ws_).async_connect( results, beast::bind_front_handler( &Forwarder::on_connect, shared_from_this())); }

    void on_connect(beast::error_code ec, tcp::resolver::results_type::endpoint_type ep) {
        if(ec){
            return fail(ec, "connect");
        }
        // Turn off the timeout on the tcp_stream, because
        // the websocket stream has its own timeout system.
        beast::get_lowest_layer(ws_).expires_never();
        // Set suggested timeout settings for the websocket
        ws_.set_option( websocket::stream_base::timeout::suggested( beast::role_type::client));

        // Set a decorator to change the User-Agent of the handshake
        ws_.set_option(websocket::stream_base::decorator( [](websocket::request_type& req) {
            req.set(http::field::user_agent, std::string(BOOST_BEAST_VERSION_STRING) + " websocket-client-async");
        }));

        // Update the host_ string. This will provide the value of the
        // Host HTTP header during the WebSocket handshake.
        // See https://tools.ietf.org/html/rfc7230#section-5.4
        host_ += ':' + std::to_string(ep.port());

        // Perform the websocket handshake
        ws_.async_handshake(host_, "/backend", beast::bind_front_handler( &Forwarder::on_handshake, shared_from_this()));
    }

    void on_handshake(beast::error_code ec) {
        if(ec){
            return fail(ec, "handshake");
        }

        handshook = true;

        do_read();
        // Send the message
    }

    void send_message(std::string message){
        boost::asio::post(ws_.get_executor(), beast::bind_front_handler( &Forwarder::on_send_message, shared_from_this(), message));
//        ws_.async_write(net::buffer(message.dump()), beast::bind_front_handler(&websocket_session::on_write, shared_from_this()));
    }
    void on_send_message(std::string message) {
        // Always add to queue
        queue_.push(message);

        // Are we already writing?
        if(queue_.size() > 1)
            return;

        // We are not currently writing, so send this immediately
        ws_.async_write(
                net::buffer(queue_.front()),
                beast::bind_front_handler(
                        &Forwarder::on_write,
                        shared_from_this()));
    }

    void on_write( beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if(ec){
            return fail(ec, "write");
        }

        queue_.pop();

        // Send the next message if any
        if(! queue_.empty())
            ws_.async_write(
                    net::buffer(queue_.front()),
                    beast::bind_front_handler(
                            &Forwarder::on_write,
                            shared_from_this()));
    }
    void do_read() {
        ws_.async_read(buffer_, beast::bind_front_handler(&Forwarder::on_read, shared_from_this()));
    }


    void on_read( beast::error_code ec, std::size_t bytes_transferred){
        boost::ignore_unused(bytes_transferred);

        if(ec){
            return fail(ec, "read");
        }
        std::string message = beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());
        {
            std::lock_guard<std::mutex> lock(mutex);
            receiving_message_queue.push(message);
        }
        std::cout << "Received message: " << beast::buffers_to_string(buffer_.data()) << std::endl;

        do_read();
    }

    void on_close(beast::error_code ec) {
        if(ec){
            return fail(ec, "close");
        }

        // If we get here then the connection is closed gracefully
        // The make_printable() function helps print a ConstBufferSequence
        std::cout << beast::make_printable(buffer_.data()) << std::endl;
    }
};
