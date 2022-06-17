/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "ext_state_est_main.h"

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ext_core_state.h>
#include <uORB/topics/ext_core_state_lite.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>


// Circular buffer class implementation

template <class T>
void circBuf_t<T>::put(value_type timest, value_type delay) {
    // add elements and increment head
    buf_[head_] = timest;
    delay_buf_[head_] = delay;
    increment_head();
    // only if it is full increment tail
    if (full_) {
        increment_tail();
    }
    // keep the buffers in the selected time window (+ tolerance)
    keep_in_win();
    full_ = ( head_ == tail_ );
    // if full and less than time window - tolerance add space to buffers
    if (full_ && ((front() - back()) < (time_win_ - tol_))) {
        resize(max_size_ + resize_increment_);
    }
}

template <class T>
size_t circBuf_t<T>::size() const {
    size_t size = max_size_;

    if (!full_) {
        if (head_ >= tail_) {
            size = head_ - tail_;
        }
        else {
            size = max_size_ + head_ - tail_;
        }
    }

    return size;
}

template <class T>
double circBuf_t<T>::rate() const {
    double rate = 0.0;

    // if buffer has less than 2 elements return 0
    if (size() > 1) {
        // rate [Hz] = 1000000 * N / time window [us]
        rate = 1000000.0 * double(size() - 1) / double(front() - back());
    }

    return rate;
}

template <class T>
double circBuf_t<T>::mean_delay() const {
    double mean = 0.0;

    if (full_) {
        mean = double(sum_delays()) / double(max_size_);
    }
    else if (size() > 0){
        mean = double(sum_delays()) / double(size());
    }

    return mean;
}

template<class T>
bool circBuf_t<T>::comm_lost() const {
    bool comm_lost = false;

    if (!isEmpty() && ((hrt_absolute_time() - front()) > (time_win_ + tol_))) {
        comm_lost = true;
    }

    return comm_lost;
}

template <class T>
T circBuf_t<T>::sum_delays() const {
    value_type sum = 0;

    if (!full_) {
        if (head_ >= tail_) {
            for (size_t i = tail_; i < head_; i++) {
                sum += delay_buf_[i];
            }
        }
        else {
            for (size_t i = tail_; i < max_size_+head_; i++) {
                sum += delay_buf_[i % max_size_];
            }
        }
    }
    else {
        for (size_t i = 0; i < max_size_; i++) {
            sum += delay_buf_[i];
        }
    }

    return sum;
}

template <class T>
void circBuf_t<T>::resize(size_t max_size_new) {

    if (max_size_new == max_size_) return;

    if (max_size_new < 1) {
        reset();
        return;
    }

    size_t num_items = size();
    size_t offset = 0;
    if (num_items > max_size_new) {
        offset = num_items - max_size_new;
        num_items = max_size_new;
    }

    pointer buf_new = new T[max_size_new];
    pointer delay_buf_new = new T[max_size_new];
    for (size_t i = 0; i < num_items; i++) {
        buf_new[i] = buf_[(tail_ + offset + i) % max_size_];
        delay_buf_new[i] = delay_buf_[(tail_ + offset + i) % max_size_];
    }

    buf_ = buf_new;
    delay_buf_ = delay_buf_new;
    max_size_ = max_size_new;
    head_ = (num_items % max_size_);
    tail_ = 0;
    full_ = (num_items == max_size_);
}

template<class T>
void circBuf_t<T>::keep_in_win() {
    if ((front() - back()) > (time_win_ + tol_)) {
        increment_tail();
        keep_in_win();
    }
    else {
        return;
    }
}


// ExtStateEst class implementation

int ExtStateEst::print_status() {
  perf_print_counter(_ext_state_perf);

  if (_timestBuf_lite_5s.comm_lost()) {
      _timestBuf_lite_5s.reset();
      _timestBuf_lite_1s.reset();
      dprintf(1, "ext_state_lite: communication lost.\n");
  }
  else {
      dprintf(1, "avg rate ext_state_lite (1s, 5s): %.2f [Hz], %.2f [Hz]\n",
              _timestBuf_lite_1s.rate(),
              _timestBuf_lite_5s.rate());
      dprintf(1, "avg delay ext_state_lite (1s, 5s): %.2f [us], %.2f [us]\n",
              _timestBuf_lite_1s.mean_delay(),
              _timestBuf_lite_5s.mean_delay());
  }

  if (_timestBuf_5s.comm_lost()) {
      _timestBuf_5s.reset();
      _timestBuf_1s.reset();
      dprintf(1, "ext_state: communication lost.\n");
  }
  else {
      dprintf(1, "avg rate ext_state (1s, 5s): %.2f [Hz], %.2f [Hz]\n",
              _timestBuf_1s.rate(),
              _timestBuf_5s.rate());
      dprintf(1, "avg delay ext_state (1s, 5s): %.2f [us], %.2f [us]\n",
              _timestBuf_1s.mean_delay(),
              _timestBuf_5s.mean_delay());
  }

  return 0;
}

int ExtStateEst::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int ExtStateEst::task_spawn(int argc, char *argv[]) {
  ExtStateEst *instance = new ExtStateEst();

  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

bool ExtStateEst::init() {
  bool callback_status = true;
  if (!_ext_state_sub.registerCallback()) {
    callback_status = false;
  }
  if (!_ext_state_lite_sub.registerCallback()) {
    callback_status = false;
  }

  if (callback_status) {
    PX4_INFO("Callbacks registered");
    _callback_registered = true;
    return true;
  }

  PX4_WARN("failed to register callback, retrying in 1 second");
  ScheduleDelayed(1000000); // retry in 1 second

  return false;
}

ExtStateEst::ExtStateEst()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME,
                        px4::wq_configurations::nav_and_controllers),
      _ext_state_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": update")),
      _ext_state_sub{this, ORB_ID(ext_core_state)},
      _ext_state_lite_sub{this, ORB_ID(ext_core_state_lite)},
      _att_pub{ORB_ID(vehicle_attitude)},
      _vehicle_global_position_pub{ORB_ID(vehicle_global_position)},
      _vehicle_local_position_pub{ORB_ID(vehicle_local_position)},
      _estimator_status_pub{ORB_ID(estimator_status)},
      _vehicle_odometry_pub{ORB_ID(vehicle_odometry)}, _unitq(matrix::Quatf()),
      _timestBuf_lite_5s(5000000), _timestBuf_lite_1s(),
      _timestBuf_5s(5000000), _timestBuf_1s() {
}

ExtStateEst::~ExtStateEst() { 
px4_lockstep_unregister_component(_lockstep_component);
perf_free(_ext_state_perf);
}

void ExtStateEst::Run() {

  if (should_exit()) {
    _ext_state_sub.unregisterCallback();
    _ext_state_lite_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  if (!_callback_registered) {
    init();
    return;
  }

  perf_begin(_ext_state_perf);

  //  vehicle_odometry_s odom{};


  ext_core_state_lite_s ext_state_lite_in{};

  if (_ext_state_lite_sub.update(&ext_state_lite_in)) {

    uint64_t timestamp = ext_state_lite_in.timestamp;
    uint64_t delay = uint64_t(labs(int64_t(hrt_absolute_time()) - int64_t(timestamp)));

    // push timestamp and delay to buffers
    _timestBuf_lite_5s.put(timestamp, delay);
    _timestBuf_lite_1s.put(timestamp, delay);

    //    uint64_t timestamp_sample = ext_state_in.timestamp_sample;

    //    // Odometry for PX4 -> MAVLINK -> MAVROS -> ROS
    //    odom.timestamp = timestamp;
    //    odom.timestamp_sample = timestamp_sample;
    //    odom.x = ext_state_in.p_wi[0];
    //    odom.y = ext_state_in.p_wi[1];
    //    odom.z = ext_state_in.p_wi[2];

    //    matrix::Quatf q_odom(ext_state_in.q_wi);
    //    q_odom.copyTo(odom.q);

    //    odom.local_frame = odom.LOCAL_FRAME_NED;

    //    odom.vx = NAN;
    //    odom.vy = NAN;
    //    odom.vz = NAN;
    //    odom.rollspeed = NAN;
    //    odom.pitchspeed = NAN;
    //    odom.yawspeed = NAN;

    //    //TODO: add covariance
    //    _vehicle_odometry_pub.publish(odom);

    // Local position for control
    vehicle_local_position_s &position = _vehicle_local_position_pub.get();
    position.timestamp = timestamp;
    position.x = ext_state_lite_in.p_wi[0];
    position.y = ext_state_lite_in.p_wi[1];
    position.z = ext_state_lite_in.p_wi[2];

    position.epv = 0;
    position.eph = 0;
    position.xy_valid = true;
    position.z_valid = true;

    position.vx = ext_state_lite_in.v_wi[0];
    position.vy = ext_state_lite_in.v_wi[1];
    position.vz = ext_state_lite_in.v_wi[2];

    position.evv = 0;
    position.evh = 0;
    position.v_xy_valid = true;
    position.v_z_valid = true;

    position.vxy_max = INFINITY;
    position.vz_max = INFINITY;
    position.hagl_min = INFINITY;
    position.hagl_max = INFINITY;

    position.ax = INFINITY;
    position.ay = INFINITY;
    position.az = INFINITY;

    const matrix::Quatf q_att(ext_state_lite_in.q_wi);

    position.heading = matrix::Eulerf(q_att).psi();
    position.delta_heading = 0;
    position.heading_reset_counter = 0;

    // Attitude for control
    vehicle_attitude_s attitude;
    attitude.timestamp = timestamp;
    attitude.quat_reset_counter = 0;
    _unitq.copyTo(attitude.delta_q_reset);
    q_att.copyTo(attitude.q);

    _vehicle_local_position_pub.update();
    _att_pub.publish(attitude);

    // Estimator Status
    // TODO: for now we only fullfill components needed by the commander
    estimator_status_s status;
    status.timestamp = hrt_absolute_time();

    // status.states;
    status.n_states = 0;
    // status.covariances;
    status.control_mode_flags = 0;
    status.filter_fault_flags = 0;
    status.innovation_check_flags = 0;
    status.mag_test_ratio = 0.1f;
    status.vel_test_ratio = 0.1f;
    status.pos_test_ratio = 0.1f;
    status.hgt_test_ratio = 0.1f;
    status.tas_test_ratio = 0.1f;
    status.hagl_test_ratio = 0.1f;
    status.beta_test_ratio = 0.1f;

    status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
    status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
    status.solution_status_flags = 0;

    status.time_slip = 0;
    status.pre_flt_fail_innov_heading = false;
    status.pre_flt_fail_innov_vel_horiz = false;
    status.pre_flt_fail_innov_vel_vert = false;
    status.pre_flt_fail_innov_height = false;
    status.pre_flt_fail_mag_field_disturbed = false;
    _estimator_status_pub.publish(status);

    perf_end(_ext_state_perf);

   if (_lockstep_component == -1)
   {
      _lockstep_component = px4_lockstep_register_component();
   }

   px4_lockstep_progress(_lockstep_component);
  }

  ext_core_state_s ext_state_in{};

  if (_ext_state_sub.update(&ext_state_in)) {

    uint64_t timestamp = ext_state_in.timestamp;
    uint64_t delay = uint64_t(labs(int64_t(hrt_absolute_time()) - int64_t(timestamp)));

    // push timestamp and delay to buffers
    _timestBuf_5s.put(timestamp, delay);
    _timestBuf_1s.put(timestamp, delay);

    //    uint64_t timestamp_sample = ext_state_in.timestamp_sample;

    //    // Odometry for PX4 -> MAVLINK -> MAVROS -> ROS
    //    odom.timestamp = timestamp;
    //    odom.timestamp_sample = timestamp_sample;
    //    odom.x = ext_state_in.p_wi[0];
    //    odom.y = ext_state_in.p_wi[1];
    //    odom.z = ext_state_in.p_wi[2];

    //    matrix::Quatf q_odom(ext_state_in.q_wi);
    //    q_odom.copyTo(odom.q);

    //    odom.local_frame = odom.LOCAL_FRAME_NED;

    //    odom.vx = NAN;
    //    odom.vy = NAN;
    //    odom.vz = NAN;
    //    odom.rollspeed = NAN;
    //    odom.pitchspeed = NAN;
    //    odom.yawspeed = NAN;

    //    //TODO: add covariance
    //    _vehicle_odometry_pub.publish(odom);

    // Local position for control
    vehicle_local_position_s &position = _vehicle_local_position_pub.get();
    position.timestamp = timestamp;
    position.x = ext_state_in.p_wi[0];
    position.y = ext_state_in.p_wi[1];
    position.z = ext_state_in.p_wi[2];

    position.epv = 0;
    position.eph = 0;
    position.xy_valid = true;
    position.z_valid = true;

    position.vx = ext_state_in.v_wi[0];
    position.vy = ext_state_in.v_wi[1];
    position.vz = ext_state_in.v_wi[2];

    position.evv = 0;
    position.evh = 0;
    position.v_xy_valid = true;
    position.v_z_valid = true;

    position.vxy_max = INFINITY;
    position.vz_max = INFINITY;
    position.hagl_min = INFINITY;
    position.hagl_max = INFINITY;

    position.ax = INFINITY;
    position.ay = INFINITY;
    position.az = INFINITY;

    const matrix::Quatf q_att(ext_state_in.q_wi);

    position.heading = matrix::Eulerf(q_att).psi();
    position.delta_heading = 0;
    position.heading_reset_counter = 0;

    // Attitude for control
    vehicle_attitude_s attitude;
    attitude.timestamp = timestamp;
    attitude.quat_reset_counter = 0;
    _unitq.copyTo(attitude.delta_q_reset);
    q_att.copyTo(attitude.q);

    _vehicle_local_position_pub.update();
    _att_pub.publish(attitude);

    // Estimator Status
    // TODO: for now we only fullfill components needed by the commander
    estimator_status_s status;
    status.timestamp = hrt_absolute_time();

    // status.states;
    status.n_states = 0;
    // status.covariances;
    status.control_mode_flags = 0;
    status.filter_fault_flags = 0;
    status.innovation_check_flags = 0;
    status.mag_test_ratio = 0.1f;
    status.vel_test_ratio = 0.1f;
    status.pos_test_ratio = 0.1f;
    status.hgt_test_ratio = 0.1f;
    status.tas_test_ratio = 0.1f;
    status.hagl_test_ratio = 0.1f;
    status.beta_test_ratio = 0.1f;

    status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
    status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
    status.solution_status_flags = 0;

    status.time_slip = 0;
    status.pre_flt_fail_innov_heading = false;
    status.pre_flt_fail_innov_vel_horiz = false;
    status.pre_flt_fail_innov_vel_vert = false;
    status.pre_flt_fail_innov_height = false;
    status.pre_flt_fail_mag_field_disturbed = false;
    _estimator_status_pub.publish(status);

    perf_end(_ext_state_perf);

   if (_lockstep_component == -1)
   {
      _lockstep_component = px4_lockstep_register_component();
   }

   px4_lockstep_progress(_lockstep_component);
  }
}

int ExtStateEst::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  return 0;
}
