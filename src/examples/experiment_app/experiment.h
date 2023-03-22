#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <poll.h>
#include <float.h>
#include <stdint.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/experiment.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>
#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

using namespace time_literals;

class experiment_params : public ModuleParams
{
public:


private:

    // Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::EXP_MAG>) _param_experiment_mag,
        (ParamFloat<px4::params::EXP_DUR>) _param_experiment_dur,
        (ParamFloat<px4::params::EXP_FSTART>) _param_experiment_fstart,
        (ParamFloat<px4::params::EXP_FEND>) _param_experiment_fend,
        (ParamInt<px4::params::EXP_INPUT>) _param_experiment_input,
        (ParamInt<px4::params::EXP_OUTPUT>) _param_experiment_output,
        (ParamInt<px4::params::EXP_FLAG>) _param_experiment_flag
    )


};
