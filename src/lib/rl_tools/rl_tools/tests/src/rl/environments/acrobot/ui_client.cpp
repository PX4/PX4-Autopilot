#include <rl_tools/operations/cpu.h>
#include <rl_tools/rl/environments/acrobot/operations_cpu.h>
#include <rl_tools/ui_server/client/operations_websocket.h>


namespace rlt = rl_tools;


using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = float;

struct ENVIRONMENT_PARAMETERS: rlt::rl::environments::acrobot::DefaultParameters<T>{
    static constexpr T DT = 0.018;
};
using ENVIRONMENT_SPEC = rlt::rl::environments::acrobot::Specification<T, TI, ENVIRONMENT_PARAMETERS>;
using ENVIRONMENT = rlt::rl::environments::AcrobotSwingup<ENVIRONMENT_SPEC>;

using ENV_UI = rlt::ui_server::client::UIWebSocket<ENVIRONMENT>;

int main(){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    ENVIRONMENT env;
    ENVIRONMENT::Parameters env_parameters;
    ENV_UI ui;
    ui.address = "127.0.0.1";
    ui.port = 13337;
    rlt::init(device, env, env_parameters, ui);
    typename ENVIRONMENT::State state, next_state;
    ENVIRONMENT::Parameters parameters;
    rlt::sample_initial_parameters(device, env, parameters, rng);
    rlt::sample_initial_state(device, env, parameters, state, rng);
    state.theta_1_dot = 0;
    state.theta_2_dot = 0;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM, false>> action;
//    rlt::randn(device, action, rng);
    rlt::set_all(device, action, -1);
    rlt::clamp(device, action, -1, 1);
    while(true){
        T dt = rlt::step(device, env, parameters, state, action, next_state, rng);
        rlt::set_state(device, env, parameters, ui, state, action);
        std::this_thread::sleep_for(std::chrono::duration<T>(dt));
        state = next_state;
        state.theta_1_dot *= 0.99;
        state.theta_2_dot *= 0.99;
    }
    return 0;
}
