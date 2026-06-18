#ifndef RL_TOOLS_INFERENCE_APPLICATIONS_L2F_C_INTERFACE_H
#define RL_TOOLS_INFERENCE_APPLICATIONS_L2F_C_INTERFACE_H

#include <stdint.h>

#include "../../executor/c_interface.h"
#define RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM 4

#ifdef __cplusplus
extern "C" {
#endif
    typedef struct {
        float position[3];
        float orientation[4]; // Quaternion: w, x, y, z
        float linear_velocity[3];
        float angular_velocity[3];
        float previous_action[RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM];
    } RLtoolsInferenceApplicationsL2FObservation;
    typedef struct {
        float action[RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM];
    } RLtoolsInferenceApplicationsL2FAction;

    void rl_tools_inference_applications_l2f_init();
    void rl_tools_inference_applications_l2f_reset();
    float rl_tools_inference_applications_l2f_test(RLtoolsInferenceApplicationsL2FAction* action);
    // note: DON'T pass an uint32 timestamp here, which might wrap around after ~1h
    void rl_tools_inference_applications_l2f_set_force_sync_native(uint32_t force_sync_native);
    RLtoolsInferenceExecutorStatus rl_tools_inference_applications_l2f_control(uint64_t nanoseconds, RLtoolsInferenceApplicationsL2FObservation* observation, RLtoolsInferenceApplicationsL2FAction* action);
    const char* rl_tools_inference_applications_l2f_checkpoint_name();
#ifdef __cplusplus
}
#endif



#endif
