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

/**
 * @file ext_state_est_main.h
 * Passtrough for external state to the controller
 *
 * @author Christian Brommer, Alessandro Fornasier and Giulio Delama
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ext_core_state.h>
#include <uORB/topics/ext_core_state_lite.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>


// Circular buffer class
template <class T>
class circBuf_t {
public:
    typedef T           value_type;
    typedef T*          pointer;
    typedef const T*    const_pointer;
    typedef T&          reference;
    typedef const T&    const_reference;
    typedef ptrdiff_t   difference_type;

    enum {  default_win = 1000000, default_tol = 500000, default_size = 1000 };
    explicit circBuf_t(value_type time_win = default_win, // default time window [us]
                       value_type tol = default_tol,      // default tolerance for time window [us]
                       size_t max_size = default_size)  // default capacity
        : buf_(new value_type[max_size]),
          delay_buf_(new value_type[max_size]),
          max_size_(max_size),
          time_win_(time_win),
          tol_(tol) { }
    ~circBuf_t()
        { delete[] buf_; }

    void put(value_type timest, value_type delay);
    size_t size() const;

    const_reference front() const { return buf_[(max_size_ + head_ - 1) % max_size_]; }
    const_reference back() const { return buf_[tail_]; }
    void reset() { head_ = tail_ = full_ = 0; }
    bool isEmpty() const { return (!full_ && (head_ == tail_)); }
    bool isFull() const { return full_; }
    size_t capacity() const { return max_size_; }

    double rate() const;
    double mean_delay() const;
    bool comm_lost() const;

private:
    pointer buf_;           // buffer for time stamps
    pointer delay_buf_;     // buffer for delays
    size_t max_size_;
    size_t head_ = 0;
    size_t tail_ = 0;
    bool full_ = false;
    value_type time_win_;               // time window [us]
    value_type tol_;                    // time window tolerance [us]
    size_t resize_increment_ = 100;     // size increment if buf is full and less than time_w

    void increment_tail() { tail_ = (tail_ + 1) % max_size_; }
    void increment_head() { head_ = (head_ + 1) % max_size_; }
    value_type sum_delays() const;
    void resize(size_t max_size_new);
    void keep_in_win();
};


// ExtStateEst class header
class ExtStateEst final : public ModuleBase<ExtStateEst>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem {

public:
  explicit ExtStateEst();
  ~ExtStateEst() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  bool init();

  int print_status() override;

private:
  /** @see ModuleBase::run() */
  void Run() override;

  perf_counter_t _ext_state_perf;
  int _lockstep_component{-1};

  uORB::SubscriptionCallbackWorkItem _ext_state_sub;
  uORB::SubscriptionCallbackWorkItem _ext_state_lite_sub;
  uORB::Publication<vehicle_attitude_s> _att_pub;
  uORB::PublicationData<vehicle_global_position_s> _vehicle_global_position_pub;
  uORB::PublicationData<vehicle_local_position_s> _vehicle_local_position_pub;
  uORB::Publication<estimator_status_s> _estimator_status_pub;

  uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub;

  bool _callback_registered{false};
  const matrix::Quatf _unitq;

  // circular buffer to store timestamps
  circBuf_t<uint64_t> _timestBuf_lite_5s;
  circBuf_t<uint64_t> _timestBuf_lite_1s;

  circBuf_t<uint64_t> _timestBuf_5s;
  circBuf_t<uint64_t> _timestBuf_1s;
};

extern "C" __EXPORT int ext_state_est_main(int argc, char *argv[]) {
  return ExtStateEst::main(argc, argv);
}
