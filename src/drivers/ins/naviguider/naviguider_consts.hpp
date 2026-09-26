/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

// Raw FIFO payload layouts (must match device FIFO packing)
#ifndef DRV_INS_DEVTYPE_NAVIGUIDER
# define DRV_INS_DEVTYPE_NAVIGUIDER 0xF0
#endif

#ifndef DRV_ACC_DEVTYPE_NAVIGUIDER
# define DRV_ACC_DEVTYPE_NAVIGUIDER 0xF1
#endif

#ifndef DRV_GYR_DEVTYPE_NAVIGUIDER
# define DRV_GYR_DEVTYPE_NAVIGUIDER 0xF2
#endif

#ifndef DRV_MAG_DEVTYPE_NAVIGUIDER
# define DRV_MAG_DEVTYPE_NAVIGUIDER 0xF3
#endif

// Register addresses
static constexpr uint8_t FIFO_REG_BASE = 0x00;
static constexpr uint8_t BYTES_REMAINING_REG = 0x38;
static constexpr uint8_t PARAM_ACK_REG = 0x3A;
static constexpr uint8_t PARAM_SAVED_REG = 0x3B;
static constexpr uint8_t FIFO_FLUSH_REG = 0x32;
static constexpr uint8_t CHIP_CONTROL_REG = 0x34;
static constexpr uint8_t HOST_STATUS_REG = 0x35;
static constexpr uint8_t CHIP_STATUS_REG = 0x37;
static constexpr uint8_t ERR_REG = 0x50;
static constexpr uint8_t PARAM_PAGE_SELECT_REG = 0x54;
static constexpr uint8_t HOST_INTERFACE_CTRL_REG = 0x55;
static constexpr uint8_t PARAM_LOAD_REG = 0x5C;
static constexpr uint8_t PARAM_REQUEST_REG = 0x64;
static constexpr uint8_t RESET_REQ_REG = 0x9B;

// Special FIFO sensor type identifiers
static constexpr uint8_t SENSOR_TYPE_TIMESTAMP = 0xFC;
static constexpr uint8_t SENSOR_TYPE_TIMESTAMP_OVERFLOW = 0xFD;
static constexpr uint8_t SENSOR_TYPE_META = 0xFE;
static constexpr uint8_t SENSOR_TYPE_TIMESTAMP_WAKE = 0xF6;
static constexpr uint8_t SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE = 0xF7;
static constexpr uint8_t SENSOR_TYPE_META_WAKE = 0xF8;

// FIFO read constraints
static constexpr uint32_t FIFO_WINDOW_BYTES = 50;
static constexpr uint32_t I2C_MAX_READ = 128;

// Control bit values / commands
static constexpr uint8_t CHIP_CONTROL_CPU_RUN = 0x01;
static constexpr uint8_t FIFO_FLUSH_DISCARD_ALL = 0xFE;
static constexpr uint8_t HOST_STATUS_RESET = 0x01;
static constexpr uint8_t HOST_STATUS_ALGORITHM_STANDBY = 0x02;
static constexpr uint8_t CHIP_STATUS_EEPROM_UPLOAD_DONE = 0x02;
static constexpr uint8_t CHIP_STATUS_FIRMWARE_IDLE = 0x08;
static constexpr uint8_t CHIP_STATUS_EEPROM_UPLOAD_ERROR = 0x04;

// Timing constants
static constexpr uint32_t RESET_DELAY_US = 200000;  // 200 ms
static constexpr uint32_t STATUS_CHECK_DELAY_US = 10000;  // 10 ms
static constexpr uint32_t BOOT_TIMEOUT_ITERATIONS = 200;  // ~2 seconds total
static constexpr uint32_t PARAM_POLL_DELAY_US = 1000;  // 1 ms
static constexpr uint32_t PARAM_TIMEOUT_ITERATIONS = 50;

// Meta Events
static constexpr uint8_t META_FLUSH_COMPLETE        = 0x01;
static constexpr uint8_t META_SAMPLE_RATE_CHANGED   = 0x02;
static constexpr uint8_t META_POWER_MODE_CHANGED    = 0x03;
static constexpr uint8_t META_ERROR                 = 0x04;
static constexpr uint8_t META_MAGNETIC_TRANSIENT    = 0x05;
static constexpr uint8_t META_CAL_STATUS_CHANGED    = 0x06;
static constexpr uint8_t META_STILLNESS_CHANGED     = 0x07;
static constexpr uint8_t META_AVAILABLE             = 0x08;
static constexpr uint8_t META_CALIBRATION_STABLE    = 0x09;
//                       META_RESERVED              = 0x0A;
static constexpr uint8_t META_SENSOR_ERROR          = 0x0B;
static constexpr uint8_t META_FIFO_OVERFLOW         = 0x0C;
static constexpr uint8_t META_DYNAMIC_RANGE_CHANGED = 0x0D;
static constexpr uint8_t META_FIFO_WATERMARK        = 0x0E;
static constexpr uint8_t META_SELF_TEST_RESULTS     = 0x0F;
static constexpr uint8_t META_INITIALIZED           = 0x10;
static constexpr uint8_t META_TRANSFER_CAUSE        = 0x11;

static constexpr uint8_t PARAM_PAGE_SYSTEM      = 1;
static constexpr uint8_t PARAM_NUM_PHYS_SENSOR_STATUS = 31;
static constexpr uint8_t PARAM_NUM_PHYS_SENSORS_PRESENT = 32;
static constexpr uint8_t PARAM_PAGE_WARM_START  = 2;
static constexpr uint8_t PARAM_WARM_START_CAL_SCORE = 29;
static constexpr uint8_t PARAM_PAGE_SENSOR_INFO = 3;
static constexpr uint8_t PARAM_PAGE_SENSOR_CONF = 5;

// ---- Host Interface Control Masks ----
static constexpr uint8_t HIC_MASK_ABORT_TX        = 0b00000010;
static constexpr uint8_t HIC_MASK_UPDATE_TX_COUNT = 0b00000100;
static constexpr uint8_t HIC_MASK_NED_COORD       = 0b00010000;

static constexpr uint32_t FIFO_BUF_SIZE = 24 * 1024;
