/****************************************************************************
 * Copyright (c) 2016 Airlango, Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tof_main.cpp
 *
 * TOF device driver task
 */
#include <px4_includes.h>
#include <px4_getopt.h>
#include <px4_tasks.h>
#include <px4_log.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <drivers/drv_hrt.h>
#include <tof.h>
#include <systemlib/param/param.h>

//#define NO_ADIAG_LOG
//#define NO_ADIAG_STATS
#include <adiag.h>

/** driver 'main' command */
extern "C" { __EXPORT int tof_main(int argc, char *argv[]); }

/*
 * Default parameters for tof driver
 */
#define TOF_DEFAULT_BAUDRATE      115200
#define TOF_DEFAULT_INTERVAL      100   /* milliseconds */

#define MAX_LEN_DEV_PATH  32

static param_t algo_aes;

namespace tof
{

/** device path that TOF is connected to */
static char _device[MAX_LEN_DEV_PATH];

/** serial device speed (uart baudrate) */
static int _baudrate = TOF_DEFAULT_BAUDRATE;

/** sampling frequency in Hz */
static int _frequency = 10;

/** tof device model */
static int _device_model = 0;

/** flag indicating if TOF driver task is running */
static bool _is_running = false;

/** flag indicating if TOF driver task should stop */
static bool _task_should_stop = false;

/** handle to the task main thread */
static px4_task_t _task_handle = -1;

/** TOF measurement data */
static struct TofData _data;

/** TOF data publication */
static orb_advert_t _tof_pub = nullptr;

/** Print out the usage information */
static void usage();

/** TOF start measurement */
static void start();

/** TOF stop measurement */
static void stop();

/** task main trampoline function */
static void	task_main_trampoline(int argc, char *argv[]);

/** TOF measurement thread primary entry point */
static void task_main(int argc, char *argv[]);

void tof_rx_callback(const TofData* data, void* context)
{
  (void)context;

  /* copy out the TOF data */
  _data = *data;

  /* send signal to measurement thread */
  px4_task_kill(_task_handle, SIGRTMIN);
}

void publish_reports()
{
  algo_aes = param_find("ALGO_AES");
  int aes;

  if (param_get(algo_aes, &aes) == 0) {
    // AD_DEBUG("ALGO_AES %d", aes);
  }

  struct distance_sensor_s report;
  report.timestamp = hrt_absolute_time();
  report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
  report.current_distance =
    static_cast<float>(_data.distance_mm) / 1000; /* in metre */

//  PX4_WARN("tof-avg %d, tof-raw %d", _data.distance_mm, _data.raw_distance_mm);
  AD_STATS("tof_time %llu, tof-avg %d, tof-raw %d",report.timestamp, _data.distance_mm, _data.raw_distance_mm);
  // TODO: subject to tune
  if (report.current_distance < 0.17f) {
    // NOTE(xiaoming@airlango.com): add a random noise to avoid round up error in blockstats deviation calculation
    report.current_distance = 0.15f + float(rand())/100000000000000;
    // AD_DEBUG("RANDOM %f", float(rand())/100000000000000);
  }

  switch (aes) {
    case 0:
    report.min_distance = 0.1f;
    break;

    case 1:
    report.min_distance = 0.01f;
    break;

    default:
    report.min_distance = 0.17f;
    break;
  }

  report.max_distance = 10.0f;
  report.orientation = 25;//downward facing
  report.covariance = 0.002f;
  // report.covariance = 0.01f;

  /* TODO: set proper ID */
  report.id = 90;

  if (_tof_pub == nullptr) {
    _tof_pub = orb_advertise(ORB_ID(distance_sensor), &report);
  } else {
    orb_publish(ORB_ID(distance_sensor), _tof_pub, &report);
  }

  // PX4_WARN("Published distance sensor data: %.3f m", report.current_distance);
}

void task_main(int argc, char *argv[])
{
  PX4_WARN("enter tof task_main");

  int interval_ms = TOF_DEFAULT_INTERVAL;
  if (_frequency > 0) {
    interval_ms = 1000 / _frequency;
  }

  /*
   * initialize signal
   */
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGRTMIN);

  /*
   * start tof driver
   */
  TofModel tof_model = static_cast<TofModel>(_device_model);

  Tof* driver = Tof::GetInstance(tof_model);
  if (driver == nullptr) {
    PX4_ERR("fail to instantiate tof driver");
    goto _failure;
  }

  if (driver->Initialize(_device, _baudrate) < 0) {
    PX4_ERR("fail to initialize tof driver");
    goto _failure;
  }

  if (driver->Start(interval_ms, tof_rx_callback, nullptr) < 0) {
    PX4_ERR("fail to start tof driver");
    goto _failure;
  }

  /*
   * enter working loop
   */
  while (!_task_should_stop) {
    /* wait on signal */
    int sig = 0;
    int rv = sigwait(&set, &sig);

    /* check if we are waken up by the proper signal */
    if (rv != 0 || sig != SIGRTMIN) {
      PX4_WARN("sigwait failed rv %d sig %d", rv, sig);
      continue;
    }

    /* publish distance sensor reports */
    publish_reports();
  }

_failure:
  PX4_WARN("closing tof");
  if (driver != nullptr) {
    driver->Stop();
    delete driver;
  }

  _is_running = false;
}

void task_main_trampoline(int argc, char *argv[])
{
  PX4_WARN("task_main_trampoline");
  task_main(argc, argv);
}

void start()
{
  ASSERT(_task_handle == -1);

  _task_handle = px4_task_spawn_cmd("tof_main",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_MAX,
                                    1500,
                                    (px4_main_t)&task_main_trampoline,
                                    nullptr);
  if (_task_handle < 0) {
    PX4_WARN("tof task start failed");
    return;
  }

  _is_running = true;
}

void stop()
{
  _task_should_stop = true;

  _is_running = false;

  _task_handle = -1;
}

void usage()
{
  PX4_WARN("missing command: try 'start', 'stop', 'status'");
  PX4_WARN("options:");
  PX4_WARN("    -D device          device path, e.g. /dev/tty-1");
  PX4_WARN("    -F frequency       sampling frequency (Hz), default to 10");
  PX4_WARN("    -M model           device hardware model (0: LANBAO_ISL)");
}

}; // namespace tof


int tof_main(int argc, char* argv[])
{
  int ch;
  int myoptind = 1;
  const char* myoptarg = nullptr;
  const char* device = nullptr;
  const char* frequency = nullptr;
  const char* device_model = nullptr;
  const char* baudrate = nullptr;

  if (argc < 2) {
    tof::usage();
    return -1;
  }

  while ((ch = px4_getopt(argc, argv, "D:F:M:B:", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case 'D':
      device = myoptarg;
      break;

    case 'F':
      frequency = myoptarg;
      break;

    case 'M':
      device_model = myoptarg;
      break;

    case 'B':
      baudrate = myoptarg;
      break;

    default:
      tof::usage();
      return -1;
    }
  }

  if (device == NULL || strlen(device) == 0) {
    tof::usage();
    return -1;
  }

  memset(tof::_device, 0, MAX_LEN_DEV_PATH);
  strncpy(tof::_device, device, MAX_LEN_DEV_PATH - 1);

  if (frequency != nullptr) {
    char *endptr;
    long val = strtoul(frequency, &endptr, 0);
    if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
        || (errno != 0 && val == 0)
        || (*endptr != '\0')) {
      PX4_WARN("Invalid parameter for frequency, ignore");
    } else {
      tof::_frequency = val;
    }
  }

  if (device_model != nullptr) {
    char *endptr;
    long val = strtoul(device_model, &endptr, 0);
    if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
        || (errno != 0 && val == 0)
        || (*endptr != '\0')) {
      PX4_WARN("Invalid parameter for device_model, ignore");
    } else {
      tof::_device_model = val;
    }
  }

  if (baudrate != nullptr) {
    char *endptr;
    long val = strtoul(baudrate, &endptr, 0);
    if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
        || (errno != 0 && val == 0)
        || (*endptr != '\0')) {
      PX4_WARN("Invalid parameter for baudrate, ignore");
    } else {
      tof::_baudrate = val;
    }
  }

  const char* verb = argv[myoptind];
  if (!strcmp(verb, "start")) {
    if (tof::_is_running) {
      PX4_WARN("tof already running");
      return 1;
    }
    tof::start();

  } else if (!strcmp(verb, "stop")) {
    if (!tof::_is_running) {
      PX4_WARN("tof is not running");
      return 1;
    }
    tof::stop();

  } else if (!strcmp(verb, "status")) {
    PX4_WARN("tof is %s", tof::_is_running ? "running" : "stopped");
    return 0;

  } else {
    tof::usage();
    return -1;
  }

  return 0;
}
