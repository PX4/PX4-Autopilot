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

// LIBRARY INCLUDES
#include <float.h>                      // floating point
#include <drivers/drv_hrt.h>			// High Resolution Timer
#include <lib/perf/perf_counter.h>		// Performance Counters
// #include <matrix/math.hpp>

// PX4 INCLUDES
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// PX4 UORB INCLUDES
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

// PX4 TOPICS
#include <uORB/topics/vehicle_position_attitude_setpoint.h> // vehicle position and attitude setpoint subscription
#include <uORB/topics/drone_task.h>                         // drone task subscription
#include <uORB/topics/vehicle_thrust_setpoint.h>            // vehicle thrust setpoint publication
#include <uORB/topics/vehicle_torque_setpoint.h>            // vehicle torque setpoint publication

#include <uORB/uORB.h>

// PX4 LOG
#include <px4_platform_common/log.h>

// ENTRY POINT
extern "C" __EXPORT int rs_pos_control_main(int argc, char *argv[]);

// NAMESPACES
using uORB::SubscriptionData;

using namespace time_literals;

// MAIN CLASS
class RobosubPosControl : public ModuleBase<RobosubPosControl>,
                          public ModuleParams,
                          public px4::WorkItem
{
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

  uORB::SubscriptionCallback _manual_control_setpoint_sub{ORB_ID(rs_manual_setpoint)};  // manual control setpoint
  uORB::SubscriptionCallback _auto_control_setpoint_sub{ORB_ID(rs_auto_setpoint)};      // auto control setpoint
  uORB::SubscriptionCallback _drone_task_sub{ORB_ID(drone_task)};                       // drone task subscription

  uORB::Publication<vehicle_thrust_setpoint_s> _thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)}; // vehicle thrust setpoint publication
  uORB::Publication<vehicle_torque_setpoint_s> _torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)}; // vehicle torque setpoint publication


// add curreent position and attitude setpoint variables

  pos_setpoint_s _pos_setpoint{};
  drone_task_s _drone_task{};

  perf_counter_t _loop_perf;


  DEFINE_PARAMETERS((ParamFloat<px4::params::RS_GAIN_X_P>)_gain_p_x,
                    (ParamFloat<px4::params::RS_GAIN_X_I>)_gain_i_x,
                    (ParamFloat<px4::params::RS_GAIN_X_D>)_gain_d_x,

                    (ParamFloat<px4::params::RS_GAIN_Y_P>)_gain_p_y,
                    (ParamFloat<px4::params::RS_GAIN_Y_I>)_gain_i_y,
                    (ParamFloat<px4::params::RS_GAIN_Y_D>)_gain_d_y,

                    (ParamFloat<px4::params::RS_GAIN_Z_P>)_gain_p_z,
                    (ParamFloat<px4::params::RS_GAIN_Z_I>)_gain_i_z,
                    (ParamFloat<px4::params::RS_GAIN_Z_D>)_gain_d_z,

                    (ParamFloat<px4::params::RS_GAIN_ROLL_P>)_gain_p_roll,
                    (ParamFloat<px4::params::RS_GAIN_ROLL_I>)_gain_i_roll,
                    (ParamFloat<px4::params::RS_GAIN_ROLL_D>)_gain_d_roll,

                    (ParamFloat<px4::params::RS_GAIN_PITCH_P>)_gain_p_pitch,
                    (ParamFloat<px4::params::RS_GAIN_PITCH_I>)_gain_i_pitch,
                    (ParamFloat<px4::params::RS_GAIN_PITCH_D>)_gain_d_pitch,

                    (ParamFloat<px4::params::RS_GAIN_YAW_P>)_gain_p_yaw,
                    (ParamFloat<px4::params::RS_GAIN_YAW_I>)_gain_i_yaw,
                    (ParamFloat<px4::params::RS_GAIN_YAW_D>)_gain_d_yaw);

  void Run() override;
  /**
   * Update our local parameter cache.
   */
  void parameters_update(bool force = false);

  void publish_attitude_setpoint(const float thrust_x, const float thrust_y,
                                                    const float thrust_z, const float roll_des,
                                                    const float pitch_des, const float yaw_des);
  /**
   * @brief Control Attitude geometric controller
   */
  void control_attitude_geo(const vehicle_attitude_s &attitude,
                            const vehicle_attitude_setpoint_s &attitude_setpoint,
                            const vehicle_angular_velocity_s &angular_velocity,
                            const vehicle_rates_setpoint_s &rates_setpoint);

  void constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_x,
                                   float thrust_y, float thrust_z);

  /* TODO_RS 6DOF controller*/
  void pos_controller_6dof(const Vector3f &pos_des, const float roll_des, const float pitch_des,
                           const float yaw_des, vehicle_attitude_s &vehicle_attitude,
                           vehicle_local_position_s &vlocal_pos);

  void stabilization_controller_6dof(const Vector3f &pos_des, const float roll_des,
                                     const float pitch_des, const float yaw_des,
                                     vehicle_attitude_s &vehicle_attitude,
                                     vehicle_local_position_s &vlocal_pos);
};
