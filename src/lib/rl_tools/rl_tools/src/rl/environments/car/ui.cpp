#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/car/operations_cpu.h>
#include <rl_tools/rl/environments/car/ui.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <SDL2/SDL.h>

int main(int argc, char** argv){
    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        fprintf(stderr, "Unable to initialize SDL: %s\n", SDL_GetError());
        return 1;
    }

    if (SDL_NumJoysticks() < 1) {
        printf("No joysticks connected!\n");
        return 1;
    }

    SDL_Joystick* joystick = SDL_JoystickOpen(0);
    if (joystick == NULL) {
        printf("Could not open joystick 0: %s\n", SDL_GetError());
        return 1;
    }

    printf("Opened joystick %s\n", SDL_JoystickName(joystick));


    using DEV_SPEC = rlt::devices::DefaultCPUSpecification;
    using DEVICE = rlt::devices::CPU<DEV_SPEC>;
    using T = float;
    using TI = typename DEVICE::index_t;


    TI throttle_axis = 3;
    TI steering_axis = 0;
    bool invert_throttle = false, invert_steering = false;
    bool test_axes = false;
    if(argc == 3){
        int throttle_axis_signed = std::atoi(argv[1]);
        int steering_axis_signed = std::atoi(argv[2]);
        if(steering_axis_signed < 0){
            steering_axis = -steering_axis_signed;
            invert_steering = true;
        }
        else{
            steering_axis = steering_axis_signed;
        }
        if(throttle_axis_signed < 0){
            throttle_axis = -throttle_axis_signed;
            invert_throttle = true;
        }
        else{
            throttle_axis = throttle_axis_signed;
        }
        std::cout << "Using throttle axis " << throttle_axis << " and steering axis " << steering_axis << std::endl;
    }
    else{
        std::cout << "No gamepad axes provided, you can provide as arguments to the program: \"rl_environments_car_ui [steering_axis] [throttle_axis]\" (wiggle your gamepad to find the axis values)" << std::endl;
        test_axes = true;
    }
//    using ENV_SPEC = rlt::rl::environments::car::SpecificationTrack<T, DEVICE::index_t>;
    using ENV_SPEC = rlt::rl::environments::car::SpecificationTrack<T, DEVICE::index_t, 100, 100, 20>;
    using ENVIRONMENT = rlt::rl::environments::CarTrack<ENV_SPEC>;

    using UI_SPEC = rlt::rl::environments::car::ui::Specification<T, TI, ENVIRONMENT, 200, 60>;
    using UI = rlt::rl::environments::car::UI<UI_SPEC>;

    DEVICE device;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters env_parameters;
    ENVIRONMENT::State state, next_state;
    UI ui;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM>> observation;
    rlt::malloc(device, action);
    rlt::malloc(device, observation);

    SDL_Event event;

    T color = 0;
    bool forward = true;

    rlt::init(device, env, env_parameters);
    rlt::init(device, env, env_parameters, ui);
//    rlt::initial_state(device, env, state);
    T steering = 0, throttle = 0;
    rlt::sample_initial_parameters(device, env, env_parameters, rng);
    rlt::sample_initial_state(device, env, env_parameters, state, rng);
    while(true){
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) return 0;
            if (event.type == SDL_JOYAXISMOTION) {
                if(!test_axes){
                    if(event.jaxis.axis == steering_axis){
                        steering = -event.jaxis.value / 32768.0 * 60.0/180.0*rlt::math::PI<T>;
                        steering *= invert_steering ? -1 : 1;
                    }
                    if(event.jaxis.axis == throttle_axis){
                        throttle = event.jaxis.value / 32768.0;
                        throttle *= invert_throttle ? -1 : 1;
                    }
                }
                else{
                    if(std::abs(event.jaxis.value) > 32768 / 2){
                        printf("Joystick %d axis %d value: %d\n", event.jaxis.which, event.jaxis.axis, event.jaxis.value);
                    }
                }
            }
//            if (event.type == SDL_JOYBUTTONDOWN || event.type == SDL_JOYBUTTONUP) {
//                printf("Joystick %d button %d state: %d\n", event.jbutton.which, event.jbutton.button, event.jbutton.state);
//            }
        }

//        std::cout << "throttle " << throttle << " steering " << steering << std::endl;
        set(action, 0, 0, throttle);
        set(action, 0, 1, steering);
        rlt::step(device, env, env_parameters, state, action, next_state, rng);
        state = next_state;
        rlt::set_action(device, env, env_parameters, ui, action);
        rlt::set_state(device, env, env_parameters, ui, state);
        rlt::render(device, env, env_parameters, ui);

        rlt::observe(device, env, env_parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);
        if(rlt::terminated(device, env, env_parameters, state, rng)){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            rlt::sample_initial_state(device, env, env_parameters, state, rng);
        }

    }

    SDL_JoystickClose(joystick);
    SDL_Quit();
    return 0;
}


