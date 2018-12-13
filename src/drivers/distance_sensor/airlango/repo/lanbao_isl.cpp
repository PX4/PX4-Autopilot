//
// Copyright (c) 2016 Airlango Ltd. All rights reserved.
//
// @file lanbao_isl.cpp
//
// Device driver implementaion for Lanbao ISL29501
//
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#include <systemlib/err.h>
#include "lanbao_isl.h"

static const char _height_instructions[] = {
  0xA5, 0xF0, 0x0F, 0x5A, 0xFE, 0xA5, 0x05, 0x02, 0x5A, 0x06
};

static const int _height_instructions_len =
  sizeof(_height_instructions) / sizeof(_height_instructions[0]);

static enum DSPAL_SERIAL_BITRATES IntToDspalSerialBitratesEnum(int bitrate) {
  // NOTE: here we only support the data rate we possibly use
  switch (bitrate) {
  case 115200:
    return DSPAL_SIO_BITRATE_115200;
  default:
    return DSPAL_SIO_BITRATE_MAX;
  }
}

LanbaoIsl::LanbaoIsl()
  : callback_(NULL),
    context_(NULL),
    fd_(-1),
    rx_should_stop_(false) {
  PX4_DEBUG("LanbaoIsl ctor");
}

LanbaoIsl::~LanbaoIsl() {
  PX4_DEBUG("LanbaoIsl dtor");
  PX4_DEBUG("waiting on rx thread");
  rx_should_stop_ = true;
  pthread_join(rx_thread_, NULL);
  PX4_DEBUG("RX thread done");

  Stop();
  CloseDevice();
}

int LanbaoIsl::SetDeviceMode(IslWorkingMode mode) {
  if (mode == KEEP_HEIGHT) { // actively config mode to HEIGHT
    int ret = write(fd_, _height_instructions, _height_instructions_len);
    if (ret != _height_instructions_len) {
      PX4_ERR("failed to write working mode command to device");
      return -1;
    }
  }
  return 0;
}

int LanbaoIsl::InitializeInternal(const char* device, int baudrate) {
  PX4_DEBUG("LanbaoIsl::InitializeInternal()");

  fd_ = open(device, O_RDWR);
  if (fd_ < 0) {
    PX4_ERR("failed to open %s", device);
    return -1;
  }

  // set the serial baud rate
  struct dspal_serial_ioctl_data_rate rate_cfg = {
    .bit_rate  = IntToDspalSerialBitratesEnum(baudrate)
  };

  if (rate_cfg.bit_rate == DSPAL_SIO_BITRATE_MAX) {
    PX4_ERR("baudrate not supported: %d", baudrate);
    goto failure;
  }

  if (ioctl(fd_, SERIAL_IOCTL_SET_DATA_RATE, (void*)&rate_cfg) < 0) {
    PX4_ERR("failed to set baudrate %d on %s", baudrate, device);
    goto failure;
  }

#if 0
  {
    // This device requires us to manually change its working mode
    IslWorkingMode mode = KEEP_HEIGHT;
    if (SetDeviceMode(mode) < 0) {
      PX4_ERR("failed to set device working mode as KEEP_HEIGHT");
      goto failure;
    }
  }
#endif

  PX4_DEBUG("LanbaoIsl::InitializeInternal() succeeded");
  return 0;

failure:
  PX4_DEBUG("LanbaoIsl::InitializeInternal() failed");

  CloseDevice();
  return -1;
}

void* LanbaoIsl::RxTrampoline(void *arg) {
  LanbaoIsl* instance = reinterpret_cast<LanbaoIsl*>(arg);
  return instance->RxMain();
}

void* LanbaoIsl::RxMain() {
  PX4_DEBUG("enter LanbaoIsl::RxMain()");
  uint8_t buffer[50];
  int bytes_available = 0;
  int bytes_processed = 0;
  int bytes_read = 0;
  bool full_frame;

  while (!rx_should_stop_) {
    // sleep for a short while. This is b/c the
    // dspal serial driver does not support blocking
    // read.
    usleep(10 * 1000);

    // read incoming bytes into buffer
    // On Eagle board, `read()` returns:
    // 0   means there's no more data available to read
    // < 0  means something error on hardware
    // > 0  means successfully get data
    bytes_read = read(fd_, buffer + bytes_available, 50 - bytes_available);
    PX4_DEBUG("read() returns %d", bytes_read);

    if (bytes_read < 0) {
      PX4_ERR("error on read(): %d", bytes_read);
      rx_should_stop_ = true;
    } else if (bytes_read == 0) {
      continue;
    }

    bytes_available += bytes_read;

    // parse the buffer data
    full_frame = false;
    bytes_processed = Parse(buffer, bytes_available, &full_frame);

    PX4_DEBUG("Parse() processed %d bytes, full_frame %d",
              bytes_processed, full_frame);

    // discard the processed bytes and move the buffer content to the head
    bytes_available -= bytes_processed;
    memcpy(buffer, buffer + bytes_processed, bytes_available);

    // if full frame is identified, invoke the
    // callback_ if available
    if (full_frame) {
      int raw_measurement = data_.distance_mm;

      // TODO:ATTENTION:
      //
#if 0
      raw_measurement -= 80;
      if (raw_measurement < 0) {
        raw_measurement = 0;
      }
#endif

      data_.raw_distance_mm = raw_measurement;
      data_.distance_mm = distance_filter_.Filter(raw_measurement);
      PX4_DEBUG("tof measurement data, raw: %d mm, filtered: %d mm",
                raw_measurement, data_.distance_mm);
      if (callback_) {
        callback_(&data_, context_);
      }
    }
  }

  PX4_DEBUG("exit RxMain()");
  return NULL;
}

void LanbaoIsl::CloseDevice() {
  PX4_DEBUG("LanbaoIsl::CloseDevice()");

  if (fd_ >= 0) {
    PX4_DEBUG("close device handle %d", fd_);
    close(fd_);
    fd_ = -1;
  }
}

int LanbaoIsl::StartRxThread(DataReadyCallback callback, void* context) {
  PX4_DEBUG("LanbaoIsl::StartRxThread()");
  int ret;

  callback_ = callback;
  context_ = context;

  rx_should_stop_ = false;

  pthread_attr_t attr;
  size_t stacksize = -1;
  pthread_attr_init(&attr);
  pthread_attr_getstacksize(&attr, &stacksize);
  PX4_DEBUG("RX thread stack size: %d", stacksize);
  stacksize = 8 * 1024;

  PX4_DEBUG("setting the thread stack size to[%d]", stacksize);
  pthread_attr_setstacksize(&attr, stacksize);

  ret = pthread_create(&rx_thread_, &attr, &LanbaoIsl::RxTrampoline, this);
  if (ret != 0) {
    PX4_ERR("Failed to create RX thread in LanbaoIsl: %d", ret);
    return -1;
  }

  PX4_DEBUG("RX thread created in LanbaoIsl");
  return 0;
}

int LanbaoIsl::SendMeasurementCommand() {
  PX4_DEBUG("SendMeasurementCommand(), nothing need to do since device will.");
  return 0;
}

int LanbaoIsl::Parse(const uint8_t* buffer, int length, bool* full_frame) {
  static TofFramingState state = TFS_NOT_STARTED;
  static uint16_t crc16 = 0;
  int bytes_processed = 0;

  PX4_DEBUG("LanbaoTof::Parse()");

  while (bytes_processed < length) {
    uint8_t b = buffer[bytes_processed++];
    PX4_DEBUG("parse byte 0x%02X", b);

    switch (state) {
    case TFS_NOT_STARTED:
      if (b == TOF_SFD1) {
        crc16 = b;
        state = TFS_GOT_SFD1;
        PX4_DEBUG("Got SFD1");
      }
      break;

    case TFS_GOT_SFD1:
      if (b == TOF_SFD2) {
        crc16 += b;
        state = TFS_GOT_SFD2;
        PX4_DEBUG("Got SFD2");
      } else if (b == TOF_SFD1) {
        crc16 = b;
        state = TFS_GOT_SFD1;
        PX4_DEBUG("Discard previous SFD1, Got new SFD1");
      } else {
        state = TFS_NOT_STARTED;
      }
      break;

    case TFS_GOT_SFD2:
      crc16 += b;
      data_.distance_mm = b;
      state = TFS_GOT_DATA1;
      PX4_DEBUG("Got DATA1 0x%02X", b);
      break;

    case TFS_GOT_DATA1:
      crc16 += b;
      data_.distance_mm = (data_.distance_mm << 8) + b;
      state = TFS_GOT_DATA2;
      PX4_DEBUG("Got DATA2 0x%02X", b);
      break;

    case TFS_GOT_DATA2:
      if (b == (crc16 >> 8)) {
        state = TFS_GOT_CHECKSUM1;
      } else {
        PX4_DEBUG("Checksum invalid on high byte: 0x%02X, calculated: 0x%04X",
                  b, crc16);
        state = TFS_NOT_STARTED;
      }
      break;

    case TFS_GOT_CHECKSUM1:
      // Here, reset state to `NOT-STARTED` no matter crc ok or not
      state = TFS_NOT_STARTED;
      if (b == (crc16 & 0xFF)) {
        PX4_DEBUG("Checksum verified");
        *full_frame = true;
        return bytes_processed;
      } else {
        PX4_DEBUG("Checksum invalidon low byte: 0x%02X, calculated: 0x%04X",
                  b, crc16);
      }
      break;

    default:
      PX4_DEBUG("This should never happen.")
      break;
    }
  }

  return bytes_processed;
}
