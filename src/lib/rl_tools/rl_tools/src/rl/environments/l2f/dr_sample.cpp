#include <rl_tools/operations/cpu.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/environments/l2f/operations_helper_generic.h>


namespace rlt = rl_tools;


#include <iostream>
#include <chrono>


using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;

using T = float;


using ENVIRONMENT = rlt::rl::environments::Multirotor<rlt::rl::environments::l2f::Specification<T, TI>>;



int main(int argc, char** argv) {
    DEVICE device;
    TI seed;
    if(argc < 2) {
        seed = std::chrono::system_clock::now().time_since_epoch().count();
    }
    else {
        seed = std::stoi(argv[1]);
    }
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);

    ENVIRONMENT::Parameters nominal_parameters, parameters;
    ENVIRONMENT::State state;
    ENVIRONMENT env;

    rlt::initial_parameters(device, env, nominal_parameters);
    rlt::initial_parameters(device, env, parameters);
    parameters.domain_randomization = {
        1.5, // thrust_to_weight_min;
        5.0, // thrust_to_weight_max;
        0.0, //torque_to_inertia_min; // cf: torque_to_inertia ~[536, 933]
        0.0, //torque_to_inertia_max;
        0.027, // mass_min;
        5.00, // mass_max;
        1.0, // mass_size_deviation;
        0.0, // rotor_time_constant_rising_min; // cf: rising: ~[0.05, 0.09], falling: ~[0.07, 0.3]
        0.0, // rotor_time_constant_rising_max;
        0.0, // rotor_time_constant_falling_min;
        0.0, // rotor_time_constant_falling_max;
        0.0, // rotor_torque_constant_min; // cf: ~0.005
        0.0, // rotor_torque_constant_max;
        0.0, // orientation_offset_angle_max;
        0.0 // disturbance_force_max; // in multiples of the surplus thrust to weight ratio max(0, t2w - 1.0)
    };
    rlt::sample_initial_parameters(device, env, parameters, rng);



    std::string nominal_parameters_json = rlt::json(device, env, nominal_parameters);
    std::string parameters_json = rlt::json(device, env, parameters);

    rlt::compare_parameters(device, nominal_parameters, parameters);

    std::cout << parameters_json << std::endl;
}