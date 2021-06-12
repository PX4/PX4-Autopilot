/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2020 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>

#include <Eigen/Eigen>

#include <mavlink/v2.0/common/mavlink.h>
#include "msgbuffer.h"

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultMavlinkTcpPort = 4560;
static const uint32_t kDefaultGCSUdpPort = 14550;
static const uint32_t kDefaultSDKUdpPort = 14540;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
  incomplete = MAVLINK_FRAMING_INCOMPLETE,
  ok = MAVLINK_FRAMING_OK,
  bad_crc = MAVLINK_FRAMING_BAD_CRC,
  bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

//! Enumeration to use on the bitmask in HIL_SENSOR
enum class SensorSource {
  ACCEL = 0b111,
  GYRO = 0b111000,
  MAG = 0b111000000,
  BARO = 0b1101000000000,
  DIFF_PRESS = 0b10000000000,
};

namespace SensorData {
struct Imu {
  Eigen::Vector3d accel_b;
  Eigen::Vector3d gyro_b;
};

struct Barometer {
  double temperature;
  double abs_pressure;
  double pressure_alt;
};

struct Magnetometer {
  Eigen::Vector3d mag_b;
};

struct Airspeed {
  double diff_pressure;
};

struct Gps {
  int time_utc_usec;
  int fix_type;
  double latitude_deg;
  double longitude_deg;
  double altitude;
  double eph;
  double epv;
  double velocity;
  double velocity_north;
  double velocity_east;
  double velocity_down;
  double cog;
  double satellites_visible;
  int id;
};
}  // namespace SensorData

class MavlinkInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MavlinkInterface();
  ~MavlinkInterface();
  void pollForMAVLinkMessages();
  void pollFromGcsAndSdk();
  void send_mavlink_message(const mavlink_message_t *message);
  void forward_mavlink_message(const mavlink_message_t *message);
  void open();
  void close();
  void Load();
  void SendSensorMessages(int time_usec);
  void SendGpsMessages(const SensorData::Gps &data);
  void UpdateBarometer(const SensorData::Barometer &data);
  void UpdateAirspeed(const SensorData::Airspeed &data);
  void UpdateIMU(const SensorData::Imu &data);
  void UpdateMag(const SensorData::Magnetometer &data);
  Eigen::VectorXd GetActuatorControls();
  bool GetArmedState();
  void onSigInt();
  inline bool GetReceivedFirstActuator() { return received_first_actuator_; }
  inline void SetBaudrate(int baudrate) { baudrate_ = baudrate; }
  inline void SetSerialEnabled(bool serial_enabled) { serial_enabled_ = serial_enabled; }
  inline void SetUseTcp(bool use_tcp) { use_tcp_ = use_tcp; }
  inline void SetDevice(std::string device) { device_ = device; }
  inline void SetEnableLockstep(bool enable_lockstep) { enable_lockstep_ = enable_lockstep; }
  inline void SetMavlinkAddr(std::string mavlink_addr) { mavlink_addr_str_ = mavlink_addr; }
  inline void SetMavlinkTcpPort(int mavlink_tcp_port) { mavlink_tcp_port_ = mavlink_tcp_port; }
  inline void SetMavlinkUdpPort(int mavlink_udp_port) { mavlink_udp_port_ = mavlink_udp_port; }
  inline void SetGcsAddr(std::string gcs_addr) { gcs_addr_ = gcs_addr; }
  inline void SetGcsUdpPort(int gcs_udp_port) { gcs_udp_port_ = gcs_udp_port; }
  inline void SetSdkAddr(std::string sdk_addr) { sdk_addr_ = sdk_addr; }
  inline void SetSdkUdpPort(int sdk_udp_port) { sdk_udp_port_ = sdk_udp_port; }
  inline void SetHILMode(bool hil_mode) { hil_mode_ = hil_mode; }
  inline void SetHILStateLevel(bool hil_state_level) { hil_state_level_ = hil_state_level; }

 private:
  bool received_first_actuator_;
  bool armed_;
  Eigen::VectorXd input_reference_;

  void handle_message(mavlink_message_t *msg, bool &received_actuator);
  void acceptConnections();

  // Serial interface
  void do_read();
  void parse_buffer(const boost::system::error_code &err, std::size_t bytes_t);
  inline bool is_open() { return serial_dev.is_open(); }
  void do_write(bool check_tx_state);

  static const unsigned n_out_max = 16;

  int input_index_[n_out_max];

  struct sockaddr_in local_simulator_addr_;
  socklen_t local_simulator_addr_len_;
  struct sockaddr_in remote_simulator_addr_;
  socklen_t remote_simulator_addr_len_;

  int gcs_udp_port_;
  struct sockaddr_in remote_gcs_addr_;
  socklen_t remote_gcs_addr_len_;
  struct sockaddr_in local_gcs_addr_;
  std::string gcs_addr_;
  socklen_t local_gcs_addr_len_;

  int sdk_udp_port_;
  struct sockaddr_in remote_sdk_addr_;
  socklen_t remote_sdk_addr_len_;
  struct sockaddr_in local_sdk_addr_;
  socklen_t local_sdk_addr_len_;
  std::string sdk_addr_;

  unsigned char _buf[65535];
  enum FD_TYPES { LISTEN_FD, CONNECTION_FD, N_FDS };
  struct pollfd fds_[N_FDS];
  bool use_tcp_ = false;
  bool close_conn_ = false;

  in_addr_t mavlink_addr_;
  std::string mavlink_addr_str_;
  int mavlink_udp_port_;  // MAVLink refers to the PX4 simulator interface here
  int mavlink_tcp_port_;  // MAVLink refers to the PX4 simulator interface here

  boost::asio::io_service io_service;
  boost::asio::serial_port serial_dev;

  int simulator_socket_fd_;
  int simulator_tcp_client_fd_;

  int gcs_socket_fd_{-1};
  int sdk_socket_fd_{-1};

  bool enable_lockstep_ = false;

  // Serial interface
  mavlink_status_t m_status;
  mavlink_message_t m_buffer;
  bool serial_enabled_;
  std::thread io_thread;
  std::string device_;

  std::recursive_mutex mutex;
  std::mutex actuator_mutex;

  std::array<uint8_t, MAX_SIZE> rx_buf;
  unsigned int baudrate_;
  std::atomic<bool> tx_in_progress;
  std::deque<MsgBuffer> tx_q;

  bool hil_mode_;
  bool hil_state_level_;

  bool baro_updated_;
  bool diff_press_updated_;
  bool mag_updated_;
  bool imu_updated_;

  double temperature_;
  double pressure_alt_;
  double abs_pressure_;
  double diff_pressure_;
  Eigen::Vector3d mag_b_;
  Eigen::Vector3d accel_b_;
  Eigen::Vector3d gyro_b_;

  std::atomic<bool> gotSigInt_{false};
};
