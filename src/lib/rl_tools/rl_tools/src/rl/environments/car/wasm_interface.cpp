#include "training_ppo.h"
#include <emscripten.h>

DEVICE device;
extern "C" {
    EMSCRIPTEN_KEEPALIVE
    State* proxy_create(int seed){
        return create(seed);
    }

    EMSCRIPTEN_KEEPALIVE
    float proxy_step(State* ts){
        return step(ts);
    }

    EMSCRIPTEN_KEEPALIVE
    float proxy_step_message(State* ts, char* message){
        return step(ts, message);
    }

    EMSCRIPTEN_KEEPALIVE
    void proxy_destroy(State* ts){
        destroy(ts);
    }
    EMSCRIPTEN_KEEPALIVE
    int proxy_num_messages(State* ts){
        return ts->ts.ui.buffer.size();
    }
    EMSCRIPTEN_KEEPALIVE
    char* proxy_pop_message(State* ts){
        if(ts->ts.ui.buffer.empty()){
            return nullptr;
        }
        std::string message = ts->ts.ui.buffer.front();
        ts->ts.ui.buffer.pop();
        char* cstr = new char[message.length() + 1];
        strcpy(cstr, message.c_str());
        return cstr;
    }
    EMSCRIPTEN_KEEPALIVE
    void proxy_delete_message(char* message){
        delete[] message;
    }
}
