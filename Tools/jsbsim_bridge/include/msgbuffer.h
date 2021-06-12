/**
 * @brief MAVConn message buffer class (internal)
 * @file msgbuffer.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Amy Wagoner <arwagoner@gmail.com> (adaptation to Gazebo)
 */
/**
 * Copyright 2017 MAVROS dev team. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cassert>
#include "mavlink/v2.0/common/mavlink.h"

/**
 * @brief Message buffer for internal use in libmavconn
 */
struct MsgBuffer {
  //! Maximum buffer size with padding for CRC bytes (280 + padding)
  static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
  uint8_t data[MAX_SIZE];
  ssize_t len;
  ssize_t pos;

  MsgBuffer() : pos(0), len(0) {}

  /**
   * @brief Buffer constructor from mavlink_message_t
   */
  explicit MsgBuffer(const mavlink_message_t *msg) : pos(0) {
    len = mavlink_msg_to_send_buffer(data, msg);
    // paranoic check, it must be less than MAVLINK_MAX_PACKET_LEN
    assert(len < MAX_SIZE);
  }

  /**
   * @brief Buffer constructor for send_bytes()
   * @param[in] nbytes should be less than MAX_SIZE
   */
  MsgBuffer(const uint8_t *bytes, ssize_t nbytes) : pos(0), len(nbytes) {
    assert(0 < nbytes && nbytes < MAX_SIZE);
    memcpy(data, bytes, nbytes);
  }

  virtual ~MsgBuffer() {
    pos = 0;
    len = 0;
  }

  uint8_t *dpos() { return data + pos; }

  ssize_t nbytes() { return len - pos; }
};
