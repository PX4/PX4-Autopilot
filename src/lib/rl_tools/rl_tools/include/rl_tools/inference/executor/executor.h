#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_INFERENCE_EXECUTOR_EXECUTOR_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_INFERENCE_EXECUTOR_EXECUTOR_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace inference{
        namespace executor{
            template <typename TYPE_POLICY>
            struct WarningLevelsDefault{
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr T INTERMEDIATE_TIMING_JITTER_HIGH_THRESHOLD = 1.2;
                static constexpr T INTERMEDIATE_TIMING_JITTER_LOW_THRESHOLD = 0.8;
                static constexpr T INTERMEDIATE_TIMING_BIAS_HIGH_THRESHOLD = 1.2;
                static constexpr T INTERMEDIATE_TIMING_BIAS_LOW_THRESHOLD = 0.8;
                static constexpr T NATIVE_TIMING_JITTER_HIGH_THRESHOLD = 1.2;
                static constexpr T NATIVE_TIMING_JITTER_LOW_THRESHOLD = 0.8;
                static constexpr T NATIVE_TIMING_BIAS_HIGH_THRESHOLD = 1.2;
                static constexpr T NATIVE_TIMING_BIAS_LOW_THRESHOLD = 0.8;
            };
            template <typename T_TYPE_POLICY, typename T_TI, typename T_TIMESTAMP, typename T_POLICY, T_TIMESTAMP T_CONTROL_INTERVAL_INTERMEDIATE_NS, T_TIMESTAMP T_CONTROL_INTERVAL_NATIVE_NS, bool T_FORCE_SYNC_INTERMEDIATE=false, T_TI T_FORCE_SYNC_NATIVE=0, bool T_FORCE_SYNC_NATIVE_RUNTIME=false,typename T_WARNING_LEVELS=WarningLevelsDefault<T_TYPE_POLICY>, bool T_DYNAMIC_ALLOCATION=true>
            struct Specification{
                using TYPE_POLICY = T_TYPE_POLICY;
                using T = typename TYPE_POLICY::DEFAULT;
                using TI = T_TI;
                using TIMESTAMP = T_TIMESTAMP;
                static_assert(sizeof(TIMESTAMP) >= 8, "The TIMESTAMP should be unsigned and at least 8 bytes to prevent overflow (it shall be measured in nanoseconds)");
                using POLICY = T_POLICY;
                static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
                static constexpr TI TIMING_STATS_NUM_STEPS = 100;
                static constexpr TIMESTAMP CONTROL_INTERVAL_INTERMEDIATE_NS = T_CONTROL_INTERVAL_INTERMEDIATE_NS;
                static constexpr TIMESTAMP CONTROL_INTERVAL_NATIVE_NS = T_CONTROL_INTERVAL_NATIVE_NS; // the control interval native to the policy (that it was trained at)
                static constexpr bool FORCE_SYNC_INTERMEDIATE = T_FORCE_SYNC_INTERMEDIATE; // forcing the sync of intermediate steps with the observations: for each observation => run intermediate control
                static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = T_FORCE_SYNC_NATIVE_RUNTIME; // runtime variante of FORCE_SYNC_NATIVE, allows to react to different inference/control frequencies without recompiling when setting
                static constexpr TI FORCE_SYNC_NATIVE = T_FORCE_SYNC_NATIVE; // 0 means not forcing, != 0 means forcing every FORCE_SYNC_TRAINING inference control steps
                static constexpr TI INPUT_DIM = POLICY::INPUT_SHAPE::LAST;
                static constexpr TI OUTPUT_DIM = POLICY::OUTPUT_SHAPE::LAST;
                using WARNING_LEVELS = T_WARNING_LEVELS;
            };
            template <typename T_SPEC>
            struct JitterStatus{
                using SPEC = T_SPEC;
                using T = typename SPEC::T;
                bool OK;
                T MAGNITUDE;

            };
            template <typename T_SPEC>
            struct BiasStatus{
                using SPEC = T_SPEC;
                using T = typename SPEC::T;
                bool OK;
                T MAGNITUDE;
            };
            template <typename T_SPEC>
            struct Status{
                using SPEC = T_SPEC;
                using T = typename SPEC::T;
                bool OK;
                bool TIMESTAMP_INVALID;
                bool LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP;
                enum Source{
                    OBSERVATION,
                    CONTROL
                };
                Source source;
                enum StepType{
                    INTERMEDIATE,
                    NATIVE
                };
                StepType step_type;
                struct ControlReasons{
                    bool reset;
                    bool time_diff;
                    bool force_sync;
                };
                ControlReasons control_reasons_intermediate;
                ControlReasons control_reasons_native;
                JitterStatus<SPEC> timing_jitter;
                BiasStatus<SPEC> timing_bias;
            };
            namespace assert{
                struct TestSpec{
                    using T = float;
                };
#ifndef _MSC_VER
                static_assert(sizeof(Status<TestSpec>) <= 36);
#endif
            }
        }
        template <typename T_SPEC>
        struct Executor{
            // The executor gets a (potentially high-frequency) stream of observations and timestamps.
            // Then decides when to run the policy forward pass (at inference frequency, e.g. 4x faster than during training: 400Hz vs 100Hz)
            // In between forward passes the observations are averaged (probably the optimal behavior assuming Gaussian additive noise)
            // Every once in a while the time comes to advance the policy state (at the training frequency, e.g. 100Hz)
            // This is required because running e.g. a trained RNN at a faster frequency than experienced in training would be heavily OOD
            using SPEC = T_SPEC;
            using TI = typename SPEC::TI;
            using POLICY = typename SPEC::POLICY;
            static constexpr bool DYNAMIC_ALLOCATION = SPEC::DYNAMIC_ALLOCATION;
            using TIMESTAMP = typename SPEC::TIMESTAMP;

            TIMESTAMP last_observation_timestamp, last_control_timestamp, last_control_timestamp_original; // last_control_timestamp runs at the higher rate, while last_control_timestamp_original runs at the original control rate of the simulation
            bool last_observation_timestamp_set, last_control_timestamp_set, last_control_timestamp_original_set;
            TIMESTAMP control_dt[SPEC::TIMING_STATS_NUM_STEPS];
            TIMESTAMP control_dt_index = 0;
            TIMESTAMP control_original_dt[SPEC::TIMING_STATS_NUM_STEPS];
            TIMESTAMP control_original_dt_index = 0;

            TI intermediate_step = 0;
            bool force_sync_native_initialized = false;
            TI force_sync_native = SPEC::FORCE_SYNC_NATIVE; // runtime variante of FORCE_SYNC_NATIVE, allows to react to different inference/control frequencies without recompiling when setting

            Tensor<tensor::Specification<typename SPEC::T, typename SPEC::TI, tensor::Shape<typename SPEC::TI, 1, SPEC::INPUT_DIM>, DYNAMIC_ALLOCATION>> observation;
            typename POLICY::template State<DYNAMIC_ALLOCATION> policy_state, policy_state_temp;
            typename POLICY::template Buffer<DYNAMIC_ALLOCATION> policy_buffer;
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
