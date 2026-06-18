#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/car/operations_cpu.h>
#include <rl_tools/rl/environments/car/operations_json.h>

#include <rl_tools/rl/environments/car/operations_cpu.h>
#include <rl_tools/ui_server/client/operations_boost.h>
namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TI = typename DEVICE::index_t;

using ENV_SPEC = rlt::rl::environments::car::SpecificationTrack<T, TI, 100, 100, 20>;
using ENVIRONMENT = rlt::rl::environments::CarTrack<ENV_SPEC>;

using UI = rlt::ui_server::client::UI<ENVIRONMENT>;


#include <chrono>
#include <thread>

int main(){
    DEVICE device;
    RNG rng;
    ENVIRONMENT env;
    UI ui;

    rlt::malloc(device, env);
    rlt::init(device, env);
//    rlt::malloc(device, ui);
    rlt::init(device, env, ui);

    typename ENVIRONMENT::State state, next_state;
    rlt::initial_state(device, env, state);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::malloc(device, action);
    rlt::set_all(device, action, 1);
    rlt::set_state(device, env, ui, state, action);
    while(true){
//        rlt::step(device, env, state, action, next_state, rng);
        rlt::sample_initial_state(device, env, state, rng);
//        rlt::initial_state(device, env, state);
        rlt::set_state(device, env, ui, state, action);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        state = next_state;
    }


    return 0;
}