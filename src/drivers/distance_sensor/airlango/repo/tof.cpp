//
// Copyright (c) 2016 Airlango Ltd. All rights reserved.
//
// @file tof.cpp
//
// Basic implementation for TOF device driver.
//
#include <errno.h>
#include <systemlib/err.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include "tof.h"
#include "lanbao_isl.h"
#include "lanbao_isl_v2.h"


#define TOF_MEASUREMENT_TIMEOUT_USEC      200000
#define TOF_MEASUREMENT_TIMER_SIGNAL      (SIGRTMAX-1)

#define TIME_DIFF_USEC(start, end) \
  (((end).tv_sec - (start).tv_sec)*1E6 + ((end).tv_nsec - (start).tv_nsec)/1E3)

Tof* Tof::instance_ = NULL;

Tof::Tof()
  : is_initialized_(false),
    callback_(NULL),
    context_(NULL),
    tx_thread_(0),
    measurement_interval_ms_(0),
    measurement_should_stop_(true),
    periodic_measurement_running_(false) {
  PX4_DEBUG("Tof ctor");

  memset(&last_measurement_ts_, 0, sizeof(struct timespec));
  memset(&last_measurement_, 0, sizeof(struct TofData));

  // NOTE(jintao): Must initialize mutex with the following functions.
  // Using INITIALIZER results in DSP crash while calling
  // and pthread_mutex_destory.
  pthread_mutex_init(&mutex_, NULL);
}

Tof::~Tof() {
  PX4_DEBUG("Tof dtor");
  instance_ = NULL;

  pthread_mutex_destroy(&mutex_);
}

Tof* Tof::GetInstance(TofModel model) {
  PX4_DEBUG("Tof::GetInstance() for model %s", TofModelToStr(model));

  if (model >= TOF_MODEL_NUM) {
    PX4_ERR("Unknown model");
    return NULL;
  }

  if (instance_ != NULL) {
    TofModel curr_model = instance_->model();
    if (curr_model != model) {
      PX4_ERR("Tof already initialized for model %s",
              TofModelToStr(curr_model));
      return NULL;
    }

    PX4_DEBUG("return existing instance %s", TofModelToStr(curr_model));
    return instance_;
  }

  switch (model) {
  case LANBAO_ISL:
    instance_ = new LanbaoIsl();
    break;
  case LANBAO_ISL_V2:
    instance_ = new LanbaoIslV2();
    break;
  default:
    return NULL;
  }

  if (instance_ == NULL) {
    PX4_ERR("Failed to instantiate Tof model %s", TofModelToStr(model));
    return NULL;
  }

  PX4_DEBUG("Created instance for %s", TofModelToStr(model));
  return instance_;
}

int Tof::Initialize(const char* device, int baudrate) {
  int ret;

  if (is_initialized_) {
    PX4_ERR("Initialize() can be called only once!");
    return -1;
  }

  // Initialize the specified serial device with baudrate
  ret = InitializeInternal(device, baudrate);
  if (ret < 0) {
    PX4_ERR("Failed to initialize device %s baudrate %d", device, baudrate);
    return -1;
  }

  // Start RX thread to parse the incoming frame
  ret = StartRxThread(Tof::DataReadyCb, (void*)this);
  if (ret < 0) {
    PX4_ERR("Failed to register Data Ready Callback. Fatal!");
    return -1;
  }

  is_initialized_ = true;
  return 0;
}

void Tof::DataReadyCb(const struct TofData* data, void* context) {
  PX4_DEBUG("Tof::DataReadyCb(): new data %d mm", data->distance_mm);
  Tof* obj = (Tof*)context;
  obj->HandleNewMeasurement(data);
}

void Tof::HandleNewMeasurement(const struct TofData* data) {
  PX4_DEBUG("Tof::HandleNewMeasurement()");

  struct TofData m;
  int ret;
  struct timespec ts;

  // remember the latest measurement result. Call user callback
  // if available.

  pthread_mutex_lock(&mutex_);

  memcpy(&last_measurement_, data, sizeof(struct TofData));
  clock_gettime(CLOCK_REALTIME, &last_measurement_ts_);

  // save a snapshot to avoid race condition.
  m = last_measurement_;
  ts = last_measurement_ts_;

  pthread_mutex_unlock(&mutex_);

  PX4_DEBUG("last_measurement updated distance %d mm, ts %llu ms",
            m.distance_mm, ts.tv_sec*1000+(uint64_t)(ts.tv_nsec/1E6));

  if (callback_) {
    PX4_DEBUG("Notifying user of new measurement result");
    callback_(&m, context_);
  }
}

int Tof::Start(int interval_ms, DataReadyCallback callback, void* context) {
  PX4_DEBUG("Tof::Start() internval %d ms", interval_ms);
  int ret;

  if (!is_initialized_) {
    PX4_ERR("Start() cannot be called without device initialized!");
    return -1;
  }

  if (interval_ms <= 0) {
    PX4_ERR("Invalid measurement interval %d", interval_ms);
    return -1;
  }

  callback_ = callback;
  context_ = context;
  measurement_interval_ms_ = interval_ms;

  measurement_should_stop_ = false;
  periodic_measurement_running_ = true;

  // If device requires user to actively send measurement commands, we create
  // a thread (timer) to do that periodically

  if (NeedTriggerManually()) {
    pthread_attr_t attr;
    size_t stacksize = -1;
    pthread_attr_init(&attr);
    pthread_attr_getstacksize(&attr, &stacksize);
    PX4_DEBUG("TX thread stack size: %d", stacksize);
    stacksize = 8 * 1024;

    PX4_DEBUG("setting the thread stack size to[%d]", stacksize);
    pthread_attr_setstacksize(&attr, stacksize);
    ret = pthread_create(&tx_thread_, &attr, &Tof::TxTrampoline, this);
    if (ret != 0) {
      periodic_measurement_running_ = false;
      PX4_ERR("Failed to create TX thread in Tof: %d", ret);
      return -1;
    }
  }

  return 0;
}

void* Tof::TxTrampoline(void* arg) {
  Tof* obj = (Tof*)arg;
  return obj->DoPeriodicMeasurement();
}

void* Tof::DoPeriodicMeasurement() {
  struct itimerspec timer_spec;
  struct sigevent sigev;
  sigset_t set;
  timer_t timer_id;
  int sig;
  int rv;

  sigev.sigev_notify           = SIGEV_SIGNAL;
  sigev.sigev_signo            = TOF_MEASUREMENT_TIMER_SIGNAL;
  sigev.sigev_value.sival_int  = TOF_MEASUREMENT_TIMER_SIGNAL;
  sigev.sigev_notify_function  = 0;
  sigev.sigev_notify_attributes = 0;

  // create timer
  if (timer_create(CLOCK_REALTIME, &sigev, &timer_id) != 0) {
    PX4_ERR("timer_create failed");
    return NULL;
  }

  timer_spec.it_value.tv_sec     = 0;
  timer_spec.it_value.tv_nsec    = measurement_interval_ms_*1E6;
  timer_spec.it_interval.tv_sec  = 0;
  timer_spec.it_interval.tv_nsec = measurement_interval_ms_*1E6;

  // start the timer
  if (timer_settime(timer_id, 0, &timer_spec, NULL) != 0) {
    PX4_ERR("timer_settime failed");
    timer_delete(timer_id);
    return NULL;
  }

  sigemptyset(&set);
  sigaddset(&set, TOF_MEASUREMENT_TIMER_SIGNAL);

  PX4_DEBUG("start periodic measurement");
  while(!measurement_should_stop_) {
    rv = sigwait(&set, &sig);
    if (rv != 0 || sig != TOF_MEASUREMENT_TIMER_SIGNAL) {
      PX4_ERR("sigwait failed rv %d sig %d", rv, sig);
      continue;
    }
    PX4_DEBUG("waken up by signal %d", sig);

    rv = SendMeasurementCommand();
    if (rv < 0) {
      PX4_ERR("SendMeasurementCommand() failed: %d", rv);
    } else {
      PX4_DEBUG("Sent measurement command");
    }
  }

  PX4_DEBUG("stop periodic measurement");

  // delete the timer
  timer_delete(timer_id);
  return NULL;
}

int Tof::Stop() {
  PX4_DEBUG("Tof::Stop()");
  PX4_DEBUG("stopping measurement thread");

  measurement_should_stop_ = true;
  if (tx_thread_ != 0) {
    pthread_join(tx_thread_, NULL);
    tx_thread_ = 0;
  }

  periodic_measurement_running_ = false;

  PX4_DEBUG("measurement thread stopped");
  return 0;
}

int Tof::DoMeasurement() {
  int ret;
  struct timespec ts_start, ts_now, old_ts;
  bool stop_wait = false;
  uint64_t time_lapse_usec;
  int i;

  PX4_DEBUG("Tof::DoMeasurement()");

  if (!is_initialized_) {
    PX4_ERR("DoMeasurement() cannot be called without device initialized!");
    return -1;
  }

  // get a snapshot of previous last_measurement_ts_
  pthread_mutex_lock(&mutex_);
  old_ts = last_measurement_ts_;
  pthread_mutex_unlock(&mutex_);


  // Send measurement command only if the device is not in
  // periodic measurement mode
  if (!periodic_measurement_running_) {
    ret = SendMeasurementCommand();
    if (ret < 0) {
      PX4_DEBUG("SendMeasurementCommand() failed: %d", ret);
      return -1;
    }
  }

  clock_gettime(CLOCK_REALTIME, &ts_start);

  // Wait on new data arrival until new measurement is ready or time out occurs.
  // Periodically poll the last measurement result. If the last measurement
  // timestamp is within max RTT 20ms, it is considered the new measurement
  // data.

  while (!stop_wait) {
    pthread_mutex_lock(&mutex_);

    if (last_measurement_ts_.tv_sec != old_ts.tv_sec ||
        last_measurement_ts_.tv_nsec != old_ts.tv_nsec) {
      stop_wait = true;
      ret = last_measurement_.distance_mm;
      pthread_mutex_unlock(&mutex_);
      break;
    }

    pthread_mutex_unlock(&mutex_);

    clock_gettime(CLOCK_REALTIME, &ts_now);
    time_lapse_usec = TIME_DIFF_USEC(ts_start, ts_now);

    PX4_DEBUG("time_lapse_usec %llu", time_lapse_usec);

    if (time_lapse_usec > TOF_MEASUREMENT_TIMEOUT_USEC) {
      stop_wait = true;
      ret = -2;
    }

    usleep(10000);
  }

  PX4_DEBUG("DoMeasurement() return %d", ret);

  return ret;
}
