#include <rl_tools/inference/applications/l2f/c_interface.h>

#include <gtest/gtest.h>

TEST(RL_TOOLS_INFERENCE_APPLICATIONS_L2F, MAIN){
    RLtoolsInferenceApplicationsL2FObservation observation;
    RLtoolsInferenceApplicationsL2FAction action;

    rl_tools_inference_applications_l2f_init();
    float diff = rl_tools_inference_applications_l2f_test(&action);
    std::cout << "test: " << diff << std::endl;
    observation.position[0] = 0.0f;
    observation.position[1] = 0.0f;
    observation.position[2] = 0.0f;
    observation.orientation[0] = 1.0f;
    observation.orientation[1] = 0.0f;
    observation.orientation[2] = 0.0f;
    observation.orientation[3] = 0.0f;
    observation.linear_velocity[0] = 0.0f;
    observation.linear_velocity[1] = 0.0f;
    observation.linear_velocity[2] = 0.0f;
    observation.angular_velocity[0] = 0.0f;
    observation.angular_velocity[1] = 0.0f;
    observation.angular_velocity[2] = 0.0f;
    for(int j = 0; j < RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM; j++){
        observation.previous_action[j] = 0.0f;
    }
    RLtoolsInferenceTimestamp timestamp = 0;
    char message[256];
    for (int step_i=0; step_i < 10000; step_i++){
        auto status = rl_tools_inference_applications_l2f_control(timestamp, &observation, &action);
        rl_tools_inference_executor_status_message(status, message, sizeof(message));
        std::cout << "status message: " << message << std::endl;
        if (status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL && status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_NATIVE){
            std::cout << "Native bias: " << status.timing_bias.MAGNITUDE << std::endl;
        }
        timestamp += 1000 * 1000;
    }
}
