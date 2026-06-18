#include <vector>

#pragma push_macro("slots")
#undef slots
#include "matplotlib/matplotlibcpp.h"
#pragma pop_macro("slots")
namespace plt = matplotlibcpp;
#include <iostream>


template<typename T, typename ENVIRONMENT, typename ACTOR, typename CRITIC>
void plot_policy_and_value_function(ACTOR& a, CRITIC& c, std::string dir, int step=0){
    std::cout << "plotting policy and value function for step " << step << std::endl;
    static_assert(CRITIC::INPUT_DIM == 4);
    static_assert(ACTOR::INPUT_DIM == 3);
    constexpr int H = 50;
    constexpr int W = 50;
    float state_values[H][W];
    float state_actions[H][W];
    std::vector<float> yticks(H);
    std::vector<float> xticks(W);
    T abs_max = -std::numeric_limits<T>::infinity();
    T abs_min = std::numeric_limits<T>::infinity();
    for(int theta_i=0; theta_i<H; theta_i++){
        for(int theta_dot_i=0; theta_dot_i<W; theta_dot_i++){
            T theta = theta_i * 2 * M_PI / H;
            T theta_dot = theta_dot_i * 1.0 / W * 16 - 8;
            yticks[theta_i] = theta;
            xticks[theta_dot_i] = theta_dot;
            typename ENVIRONMENT::State state = {theta, theta_dot};
            T critic_input[ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM];
            rlt::observe(env, state, critic_input);
            T max_value = -std::numeric_limits<T>::infinity();
            T max_value_action = 0;
            for(T action=-1; action<=1; action+=0.1){
                critic_input[ENVIRONMENT::Observation::DIM] = action;
                T value = rlt::evaluate(c, critic_input);
                if(value > max_value){
                    max_value = value;
                    max_value_action = action;
                }
                if(value > abs_max){
                    abs_max = value;
                }
                if(value < abs_min){
                    abs_min = value;
                }
            }
            state_values[theta_i][theta_dot_i] = max_value;
            state_actions[theta_i][theta_dot_i] = rlt::evaluate(a, critic_input);
        }
    }
    for(typename DEVICE::index_t i=0; i<H; i++){
        for(typename DEVICE::index_t j=0; j<W; j++){
            state_values[i][j] = (state_values[i][j] - abs_min) / (abs_max - abs_min);
        }
    }
    std::string plots_dir = "plots";
    mkdir(plots_dir.c_str(), 0777);
    std::string this_plots_dir = plots_dir + "/" + dir;
    mkdir(this_plots_dir.c_str(), 0777);
    std::string value_function_dir = this_plots_dir + "/value_function";
    mkdir(value_function_dir.c_str(), 0777);
    std::string policy_dir = this_plots_dir + "/policy";
    mkdir(policy_dir.c_str(), 0777);

    plt::figure();
    plt::title("Value function");
    plt::imshow((float*)state_values, H, W, 1);
    plt::xlabel("theta_dot [-8, 8]");
    plt::ylabel("theta [0, 2*pi]");
    std::string value_function_filename = value_function_dir + "/" + std::to_string(step) + ".png";
    plt::save(value_function_filename);
    plt::close();

    plt::figure();
    plt::title("Policy");
    plt::imshow((float*)state_actions, H, W, 1);
    plt::xlabel("theta_dot [-8, 8]");
    plt::ylabel("theta [0, 2*pi]");
    std::string policy_filename = policy_dir + "/" + std::to_string(step) + ".png";
    plt::save(policy_filename);
    plt::close();
}
