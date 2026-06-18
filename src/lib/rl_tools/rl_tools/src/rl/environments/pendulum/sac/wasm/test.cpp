
#include "full_training.h"
int main(){
    DEVICE device;
    LOOP_STATE ts;

    rlt::malloc(device, ts);
    rlt::init(device, ts, 10);

    bool finished = false;
    while(!finished){
        finished = rlt::step(device, ts);
    }
    rlt::free(device, ts);
    return 0;
}