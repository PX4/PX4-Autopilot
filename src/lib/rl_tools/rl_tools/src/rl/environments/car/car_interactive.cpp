#define LOAD_TRACK_FROM_FILE
#include "training_ppo.h"
#include "forwarder.h"

int main(int argc, char** argv) {
    TI seed = 0;
    std::string port = "8000";
    if (argc > 1) {
        seed = std::atoi(argv[1]);
    }
    if (argc > 2){
        port = argv[2];
    }

    boost::asio::io_context io_context;
    boost::asio::io_context::strand strand(io_context);
    auto forwarder = std::make_shared<Forwarder>(io_context);
    forwarder->run("localhost", port.c_str());

    auto* state = create(seed);

    std::thread io_thread([&io_context]() {
        io_context.run();
    });

    bool prev_mode_interactive = false;
    T sleep = 0;
    T playbackSpeed = 1;
    while(!state->finished){
        {
            std::string message = "";
            bool new_message = false;
            {
                std::lock_guard<std::mutex> lock(forwarder->mutex);
                if(!forwarder->receiving_message_queue.empty()){
                    message = forwarder->receiving_message_queue.front();
                    forwarder->receiving_message_queue.pop();

                    nlohmann::json message_json = nlohmann::json::parse(message);
                    if(message_json["channel"] == "setPlaybackSpeed"){
                        playbackSpeed = message_json["data"];
                        std::cout << "Setting playback speed to: " << playbackSpeed << std::endl;
                    }
                    else{
                        new_message = true;
                    }
                }
            }

            if(new_message){
                std::cout << "new message: " << message << std::endl;
                sleep = step(state, message.c_str());
            }
            else{
                sleep = step(state);
            }
            while(forwarder->handshook && !state->ts.ui.buffer.empty()){
//                std::lock_guard<std::mutex> lock(mutex);
                std::string message = state->ts.ui.buffer.front();
                state->ts.ui.buffer.pop();
                forwarder->send_message(message);
                if(state->mode_training){
                    std::this_thread::sleep_for(std::chrono::duration<T>(state->ts.env_eval_parameters.dt/playbackSpeed));
                }
            }
        }
        if(sleep > 0){
            std::this_thread::sleep_for(std::chrono::duration<T>(state->ts.env_eval_parameters.dt / 100));
        }
    }

    io_context.stop();
    io_thread.join();
    destroy(state);
    return 0;
}
