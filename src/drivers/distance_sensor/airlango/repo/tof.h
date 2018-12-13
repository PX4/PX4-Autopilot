//
// Copyright (c) 2016 Airlango Ltd. All rights reserved.
//
// @file tof.h
//
// TOF (time of flight) device driver interfaces
//
#pragma once
#include <pthread.h>

// Data info reported to up-layer user
struct TofData {
  int distance_mm;
  int raw_distance_mm;
};

// Supported TOF device model
enum TofModel {
  LANBAO_ISL,
  LANBAO_ISL_V2, // hw version 2, with crc16
  BENEWAKE_TF_MINI, // Benewake TF_MINI
  TOF_MODEL_NUM
};

static inline const char* TofModelToStr(enum TofModel model) {
  switch (model) {
  case LANBAO_ISL:
    return "LANBAO_ISL";
  case LANBAO_ISL_V2:
    return "LANBAO_ISL_V2";
  case BENEWAKE_TF_MINI:
    return "BENEWAKE_TF_MINI";
  default:
    return "Unknown";
  }
}

typedef void (*DataReadyCallback)(const struct TofData* data, void* context);

// Tof is a base class to define interfaces for TOF device driver. Sub-classes
// shall provide detailed implementation for specific device to complete
// the measurement functionality.
class Tof {
 public:

  // Tof class method to get the singleton Tof instance.
  // If no device model has been instantiated, this function instantiates
  // specified model and return the pointer to the new instance. If the
  // specified type has been instantiated, this function returns the
  // pointer to the existing instance. If a different model has been
  // instantiated, this function returns NULL.
  // NOTE:
  // 1. This function only instantiates the singleton object. To initialize
  // the tof device, need to call Initialize().
  // 2. Tof driver is currently not implemented to be thread-safe. We assume
  // there is up to 1 user that uses tof driver at a moment.
  //
  // @param[in] model the tof model enumeration value
  //
  // @return
  // - pointer to the instance on success
  // - NULL on error.
  static Tof* GetInstance(TofModel model);

  virtual ~Tof();

  // @brief
  // Initialize the Tof driver instance for the specified device path. On
  // successful initialization, the Tof device is ready to do measurement.
  // For periodic measurement, see Start() function.
  //
  // @param[in]  device
  // the dspal serial device path which the tof is connected to
  // @param[in]  baudrate
  // the UART baud rate in bit per second
  // @return
  // - 0 on success
  // - -1 on error
  int Initialize(const char* device, int baudrate);

  // @brief
  // Start periodic measurement at specified interval_ms.
  // @param[in] interval_ms measurement interval_ms in millisecond
  // @param[in] callback data ready interrupt service routine. This is
  //            the callback function to be invoked when new measurement data
  //            is ready.
  // @param[in] context address where the context data for callback is stored at
  // @return
  // - 0 successfully started the measurement
  // - -1 device not initialized
  // - -2 other errors
  int Start(int interval_ms, DataReadyCallback callback, void* context);

  // Stop the periodic measurement. If this function is called when measurement
  // is not running, this function takes no effect.
  // @return
  // - 0 on success
  // - -1 on error
  int Stop();

  // Do one time measurement and return the measurement results. This is a
  // blocking call.
  //
  // @return
  // - positive integer indicating the distance to the object in millimeter
  // - 0 if no object is detected
  // - -1 on error
  // - -2 on timeout
  int DoMeasurement();

  virtual TofModel model() const = 0;

 protected:
  Tof();

  static void DataReadyCb(const struct TofData* data, void* context);

  void HandleNewMeasurement(const struct TofData* data);

  // Initialize the specified serial device with given baudrate. The
  // actual initialization operation is model specific, thus this virtual
  // method should be implemented by the tof subclass.
  // This function is called in Initialize().
  //
  // @return
  // - 0 if device is successfully initialized.
  // - -1 on error.
  virtual int InitializeInternal(const char* device, int baudrate) = 0;

  // Start RX thread to process incoming byte stream amd register the provided
  // callback function as data ready callback. On success, when
  // As the tof data parser is model dependent, thus this virtual method
  // shall be implemented by the tof subclass. When new tof measurement
  // data is ready, the subclass instance invokes the callback immediately.
  // context is pass to the callback function as argument.
  // This method is called in Initialize().
  //
  // @return
  // - 0 on success
  // -1 on error.
  virtual int StartRxThread(DataReadyCallback callback, void* context) = 0;

  // Send one time measurement command to tof device. This tof communication
  // protocol is device specific and thus this virtual method needs to be
  // implemented by tof subclass.
  //
  // @return
  // - 0 on success,
  // - -1 on error
  virtual int SendMeasurementCommand() = 0;

  // Some devices will periodically response measurement data once it gets
  // initialized. While some other devices might need user trigger measurement
  // operation manually.
  virtual bool NeedTriggerManually() const { return false; }

  static void* TxTrampoline(void* arg);

  void* DoPeriodicMeasurement();

 private:
  static Tof* instance_;
  bool is_initialized_;
  pthread_mutex_t mutex_;
  struct TofData last_measurement_;
  struct timespec last_measurement_ts_;
  DataReadyCallback callback_;
  void* context_;
  pthread_t tx_thread_;
  int measurement_interval_ms_;
  bool measurement_should_stop_;
  bool periodic_measurement_running_;
};
