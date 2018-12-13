//
// Copyright (c) 2016 Airlango Ltd. All rights reserved.
//
// @file lanbao_isl.h
//
// Device driver implementation for ISL29501 from www.shlanbao.com
//
#pragma once
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include "tof.h"
#include "adiag.h"

// Average filter
template <typename T>
class AverageFilter {
 public:
  AverageFilter(int window_size = 12)
    : window_full_(false),
      window_size_(window_size),
      item_index_(0),
      items_(nullptr) {
    // ASSERT(window_size > 0);
    items_ = new T[window_size];
    if (items_ != nullptr) {
      for (int i = 0; i < window_size; i++) {
        items_[i] = 0;
      }
    }
  }

  ~AverageFilter() {
    if (items_ != nullptr) {
      delete[] items_;
    }
  }

 public:
  T Filter(const T& item) {
#if 0
    sum_ -= items_[item_index_];
    sum_ += item;
    items_[item_index_] = item;
    if (++item_index_ == window_size_) {
      item_index_ = 0;
      window_full_ = true;
    }

    T result = sum_ / (window_full_ ? window_size_ : item_index_);
    return result;
#else
    items_[item_index_] = item;
    if (++item_index_ == window_size_) {
      item_index_ = 0;
      window_full_ = true;
    }

    int num = NumOfSamplingForCalc(item);
    T result = CalcAverage(num);

    return result;
#endif
  }

 protected:
  T CalcAverage(int num) {
    int valid_index; // (latest) valid index

    if (item_index_ == 0) {
      valid_index = window_size_ - 1;
    } else  {
      valid_index = item_index_ - 1;
    }

    // IF data not fully collected, we return the original sampling data,
    // this wil only occur at the very earlier stage
    if (!window_full_) {
      return items_[valid_index];
    }

    sum_ = 0;

    for (int i = 0; i < num; i++) {
      sum_ += items_[valid_index];

      if (--valid_index < 0) {
        valid_index = window_size_ - 1;
      }
    }

    return sum_ / num;
  }

  int NumOfSamplingForCalc(const T& item) {
    int num = (item / 1000) + 3;
    if (num > window_size_) {
      num = window_size_;
    }
    return num;
  }

 protected:
  bool window_full_ = false;
  int window_size_ = 0;
  int item_index_ = 0;
  T* items_ = nullptr;
  T sum_ = 0;
};

// frame start delimiter
#define TOF_SFD1      0xA5
#define TOF_SFD2      0x5A

typedef enum {
  TFS_NOT_STARTED = 0,
  TFS_GOT_SFD1,
  TFS_GOT_SFD2,
  TFS_GOT_DATA1,
  TFS_GOT_DATA2,
  TFS_GOT_CHECKSUM1,
  TFS_GOT_CHECKSUM2,
} TofFramingState;

enum IslWorkingMode {
  KEEP_HEIGHT = 0,
  NUM_WORKING_MODE
};

// Lanbao ISL29501 to provide tof functionality
class LanbaoIsl : public Tof {
 public:
  LanbaoIsl();
  virtual ~LanbaoIsl();
  virtual TofModel model() const { return LANBAO_ISL; };

 protected:
  virtual int InitializeInternal(const char* device, int baudrate);
  virtual int StartRxThread(DataReadyCallback callback, void* context);
  virtual int SendMeasurementCommand();
  virtual int Parse(const uint8_t* buffer, int length, bool* full_frame);

  static void* RxTrampoline(void* arg);
  void* RxMain();
  void CloseDevice();
  int SetDeviceMode(IslWorkingMode mode = KEEP_HEIGHT);

  DataReadyCallback callback_;
  void* context_;
  int fd_;
  pthread_t rx_thread_;
  volatile bool rx_should_stop_;
  struct TofData data_;
  AverageFilter<int> distance_filter_;
};
