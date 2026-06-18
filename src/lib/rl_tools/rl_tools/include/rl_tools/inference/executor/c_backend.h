#include <stdio.h>
#include "executor.h"
#include "c_interface.h"
#include "helper.h"

static int portable_strlen(const char* str) {
    const char* ptr = str;
    while (*ptr != '\0') {
        ptr++;
    }
    return ptr - str;
}

static char * portable_strcpy(char *dest, const char *src) {
    char *original_dest = dest;
    while ((*dest++ = *src++) != '\0');
    return original_dest;
}

static void append(char* target, int target_size, const char* message, int &position){
    if(position + portable_strlen(message) < target_size){
        portable_strcpy(target + position, message);
        position += portable_strlen(message);
    }
}


void rl_tools_inference_executor_status_message(RLtoolsInferenceExecutorStatus status, char* target, int target_size){
    int position = 0;
    if(status.OK){
        append(target, target_size, "OK", position);
        if(status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL){
            append(target, target_size, " CONTROL", position);
            if(status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_INTERMEDIATE){
                append(target, target_size, " INTERMEDIATE", position);
            }
            else{
                append(target, target_size, " NATIVE", position);
            }
        }
    }
    else{
        append(target, target_size, "PROBLEM", position);
        if(status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL){
            append(target, target_size, " CONTROL", position);
            if(status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_INTERMEDIATE){
                append(target, target_size, " INTERMEDIATE", position);
            }
            else{
                append(target, target_size, " NATIVE", position);
            }
        }
        else{
            append(target, target_size, " UNKNOWN", position);
        }
        if(status.TIMESTAMP_INVALID){
            append(target, target_size, " TIMESTAMP_INVALID", position);
        }
        else{
            if(!status.timing_bias.OK || !status.timing_jitter.OK){
                append(target, target_size, " TIMING_ISSUE", position);
                if(!status.timing_jitter.OK){
                    append(target, target_size, " JITTER ", position);
                    char buffer[50];
                    rl_tools_inference_executor_float_to_str(status.timing_jitter.MAGNITUDE, buffer, 50);
                    append(target, target_size, buffer, position);
                }
                if(!status.timing_bias.OK){
                    append(target, target_size, " BIAS ", position);
                    char buffer[50];
                    rl_tools_inference_executor_float_to_str(status.timing_bias.MAGNITUDE, buffer, 50);
                    append(target, target_size, buffer, position);
                }
            }
            else{
                append(target, target_size, " UNKNOWN", position);
            }
        }

    }
}

namespace rl_tools{
    template <typename SPEC>
    RLtoolsInferenceExecutorStatus convert(const inference::executor::Status<SPEC>& status){
        RLtoolsInferenceExecutorStatus output;
        output.OK = status.OK;
        output.TIMESTAMP_INVALID = status.TIMESTAMP_INVALID;
        output.LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP = status.LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP;
        output.source = status.source == inference::executor::Status<SPEC>::OBSERVATION ? RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_OBSERVATION : RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL;
        output.step_type = status.step_type == inference::executor::Status<SPEC>::INTERMEDIATE ? RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_INTERMEDIATE : RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_NATIVE;
        output.timing_jitter.OK = status.timing_jitter.OK;
        output.timing_jitter.MAGNITUDE = status.timing_jitter.MAGNITUDE;
        output.timing_bias.OK = status.timing_bias.OK;
        output.timing_bias.MAGNITUDE = status.timing_bias.MAGNITUDE;
        return output;
    }
}
