/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
// #include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
// #include <uORB/topics/input_rc.h>
// #include <uORB/topics/actuator_test.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/water_detection.h>

#include <uORB/uORB.h>

#include <px4_platform_common/log.h>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector3f;

using uORB::SubscriptionData;

using namespace time_literals;

class RobosubPosControl : public ModuleBase<RobosubPosControl>, public ModuleParams, public px4::WorkItem {
      public:
        RobosubPosControl();
        ~RobosubPosControl();

        /** @see ModuleBase */
        static int task_spawn(int argc, char *argv[]);

        static int custom_command(int argc, char *argv[]);

        /** @see ModuleBase */
        static int print_usage(const char *reason = nullptr);

        bool init();

      private:
        void publishTorqueSetpoint(const hrt_abstime &timestamp_sample);
        void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

        uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

        uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
        uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

        uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

        uORB::Subscription _vehicle_attitude_setpoint_sub{
            ORB_ID(vehicle_attitude_setpoint)}; /**< vehicle attitude setpoint */
        uORB::Subscription _vehicle_rates_setpoint_sub{
            ORB_ID(vehicle_rates_setpoint)}; /**< vehicle bodyrates setpoint subscriber */
        uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
        uORB::Subscription _angular_velocity_sub{
            ORB_ID(vehicle_angular_velocity)}; /**< vehicle angular velocity subscription */
        uORB::Subscription _manual_control_setpoint_sub{
            ORB_ID(manual_control_setpoint)}; /**< notification of manual control updates */
        uORB::Subscription _vcontrol_mode_sub{
            ORB_ID(vehicle_control_mode)}; /**< vehicle status subscription TaskModeManager */


        uORB::Subscription _water_detection_sub{
            ORB_ID(water_detection)}; /* Mainbrain and power exeterior water dectection sensor outside */

        uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
        uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

        vehicle_attitude_s _vehicle_attitude{};

        vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{};
        vehicle_torque_setpoint_s _vehicle_torque_setpoint{};
        manual_control_setpoint_s _manual_control_setpoint{};
        vehicle_attitude_setpoint_s _attitude_setpoint{};
        vehicle_rates_setpoint_s _rates_setpoint{}; // vechile bodyrates setpoint */
        trajectory_setpoint_s _trajectory_setpoint{};
        vehicle_control_mode_s _vcontrol_mode{};

  	water_detection_s _water_detection{};

        perf_counter_t _loop_perf;

        DEFINE_PARAMETERS(
            (ParamFloat<px4::params::RS_GAIN_X_P>)_param_pose_gain_x,
            (ParamFloat<px4::params::RS_GAIN_Y_P>)_param_pose_gain_y,
            (ParamFloat<px4::params::RS_GAIN_Z_P>)_param_pose_gain_z,
            (ParamFloat<px4::params::RS_GAIN_X_D>)_param_pose_gain_d_x,
            (ParamFloat<px4::params::RS_GAIN_Y_D>)_param_pose_gain_d_y,
            (ParamFloat<px4::params::RS_GAIN_Z_D>)_param_pose_gain_d_z,

            (ParamFloat<px4::params::RS_ROLL_P>)_param_roll_p, (ParamFloat<px4::params::RS_ROLL_D>)_param_roll_d,
            (ParamFloat<px4::params::RS_PITCH_P>)_param_pitch_p, (ParamFloat<px4::params::RS_PITCH_D>)_param_pitch_d,
            (ParamFloat<px4::params::RS_YAW_P>)_param_yaw_p, (ParamFloat<px4::params::RS_YAW_D>)_param_yaw_d,
            // control/input modes
            (ParamInt<px4::params::RS_INPUT_MODE>)_param_input_mode,
            (ParamInt<px4::params::RS_STAB_MODE>)_param_stabilization,
            (ParamInt<px4::params::RS_SKIP_CTRL>)_param_skip_ctrl, /** < Whether to skip geometric controller */
            // direct access to inputs
            (ParamFloat<px4::params::RS_DIRCT_ROLL>)_param_direct_roll,
            (ParamFloat<px4::params::RS_DIRCT_PITCH>)_param_direct_pitch,
            (ParamFloat<px4::params::RS_DIRCT_YAW>)_param_direct_yaw,
            (ParamFloat<px4::params::RS_DIRCT_THRUST>)_param_direct_thrust)

        void Run() override;
        /**
         * Update our local parameter cache.
         */
        void parameters_update(bool force = false);

        void publish_attitude_setpoint(const float thrust_x, const float thrust_y, const float thrust_z,
                                       const float roll_des, const float pitch_des, const float yaw_des);

        /**
         * @brief In-/underwater actuator safety factor
         */
        void apply_water_safety(float &roll_u, float &pitch_u, float &yaw_u, float &thrust_x, float &thrust_y,
                                float &thrust_z);
        /**
         * @brief Control Attitude geometric controller
         */
        void control_attitude_geo(const vehicle_attitude_s &attitude,
                                  const vehicle_attitude_setpoint_s &attitude_setpoint,
                                  const vehicle_angular_velocity_s &angular_velocity,
                                  const vehicle_rates_setpoint_s &rates_setpoint);

        void constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_x, float thrust_y,
                                         float thrust_z);

        /* TODO_RS 6DOF controller*/
        void pos_controller_6dof(const Vector3f &pos_des, const float roll_des, const float pitch_des,
                                 const float yaw_des, vehicle_attitude_s &vehicle_attitude,
                                 vehicle_local_position_s &vlocal_pos);

        void stabilization_controller_6dof(const Vector3f &pos_des, const float roll_des, const float pitch_des,
                                           const float yaw_des, vehicle_attitude_s &vehicle_attitude,
                                           vehicle_local_position_s &vlocal_pos);
};
