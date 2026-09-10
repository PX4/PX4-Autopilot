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

#include "TdkIcm42x.hpp"
#include "TdkIcm42xRegisters.hpp"
#include "registers/TdkICM40609DRegisters.hpp"
#include "registers/TdkICM42605Registers.hpp"
#include "registers/TdkICM42670PRegisters.hpp"
#include "registers/TdkICM45686Registers.hpp"
#include <drivers/drv_hrt.h>

using namespace frequency_literals;

namespace
{
using Variant = TdkIcm42x::Variant;
using Protocol = TdkIcm42x::Protocol;
using PacketFormat = TdkIcm42x::PacketFormat;
using namespace time_literals;

#if defined(CONFIG_TDK_ICM42X_ICM40609D)
constexpr TdkIcm42x::Profile icm40609DProfile()
{
	using namespace tdk_icm40609d_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIcm40609D;
	profile.whoami              = 0x3b;
	profile.protocol            = Protocol::kBanked;
	profile.packet_format       = PacketFormat::kStandard16;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 8_kHz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 32.f;
	profile.gyro_range_dps      = 2000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::DEVICE_CONFIG);
	profile.reset_bit             = static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
	profile.reset_status_bit      = 0x00;
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 4;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = true;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = false;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = false;

	profile.device.name                 = "icm40609d";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM40609D;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_ICM40609D

#if defined(CONFIG_TDK_ICM42X_ICM42605)
constexpr TdkIcm42x::Profile icm42605Profile()
{
	using namespace tdk_icm42605_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIcm42605;
	profile.whoami              = 0x42;
	profile.protocol            = Protocol::kBanked;
	profile.packet_format       = PacketFormat::kStandard16;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 8_kHz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 16.f;
	profile.gyro_range_dps      = 2000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::DEVICE_CONFIG);
	profile.reset_bit             = static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
	profile.reset_status_bit      = static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT);
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 4;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = false;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = false;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = false;

	profile.device.name                 = "icm42605";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM42605;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_ICM42605

#if defined(CONFIG_TDK_ICM42X_ICM42670P)
constexpr TdkIcm42x::Profile icm42670PProfile()
{
	using namespace tdk_icm42670p_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIcm42670P;
	profile.whoami              = 0x67;
	profile.protocol            = Protocol::kMreg;
	profile.packet_format       = PacketFormat::kStandard16;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 1600_Hz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 16.f;
	profile.gyro_range_dps      = 2000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.reset_bit             = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::SOFT_RESET_DEVICE_CONFIG);
	profile.reset_status_bit      = static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT);
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 6;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = false;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = false;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = false;

	profile.device.name                 = "icm42670p";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM42670P;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_ICM42670P

#if defined(CONFIG_TDK_ICM42X_ICM42688P)
constexpr TdkIcm42x::Profile icm42688PProfile()
{
	using namespace tdk_icm42x_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIcm42688P;
	profile.whoami              = 0x47;
	profile.protocol            = Protocol::kBanked;
	profile.packet_format       = PacketFormat::kHighRes20;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 8_kHz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 16.f;
	profile.gyro_range_dps      = 2000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::DEVICE_CONFIG);
	profile.reset_bit             = static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
	profile.reset_status_bit      = static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT);
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 4;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = false;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = true;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = true;

	profile.device.name                 = "icm42688p";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM42688P;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.min_clock_hz         = 31_kHz;
	profile.device.max_clock_hz         = 50_kHz;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_ICM42688P

#if defined(CONFIG_TDK_ICM42X_ICM42686P)
constexpr TdkIcm42x::Profile icm42686PProfile()
{
	using namespace tdk_icm42x_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIcm42686P;
	profile.whoami              = 0x44;
	profile.protocol            = Protocol::kBanked;
	profile.packet_format       = PacketFormat::kHighRes20;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 8_kHz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 32.f;
	profile.gyro_range_dps      = 4000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::DEVICE_CONFIG);
	profile.reset_bit             = static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
	profile.reset_status_bit      = static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT);
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 4;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = false;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = true;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = true;

	profile.device.name                 = "icm42686p";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM42686P;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.min_clock_hz         = 31_kHz;
	profile.device.max_clock_hz         = 50_kHz;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_ICM42686P

#if defined(CONFIG_TDK_ICM42X_IIM42652)
constexpr TdkIcm42x::Profile iim42652Profile()
{
	using namespace tdk_icm42x_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIim42652;
	profile.whoami              = 0x6f;
	profile.protocol            = Protocol::kBanked;
	profile.packet_format       = PacketFormat::kHighRes20;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 8_kHz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 16.f;
	profile.gyro_range_dps      = 2000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::DEVICE_CONFIG);
	profile.reset_bit             = static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
	profile.reset_status_bit      = static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT);
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 4;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = false;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = true;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = true;

	profile.device.name                 = "iim42652";
	profile.device.device_type          = DRV_IMU_DEVTYPE_IIM42652;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.min_clock_hz         = 31_kHz;
	profile.device.max_clock_hz         = 50_kHz;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_IIM42652

#if defined(CONFIG_TDK_ICM42X_IIM42653)
constexpr TdkIcm42x::Profile iim42653Profile()
{
	using namespace tdk_icm42x_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIim42653;
	profile.whoami              = 0x56;
	profile.protocol            = Protocol::kBanked;
	profile.packet_format       = PacketFormat::kHighRes20;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 8_kHz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 32.f;
	profile.gyro_range_dps      = 4000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::DEVICE_CONFIG);
	profile.reset_bit             = static_cast<uint8_t>(DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
	profile.reset_status_bit      = static_cast<uint8_t>(INT_STATUS_BIT::RESET_DONE_INT);
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT_STATUS);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT_STATUS_BIT::FIFO_FULL_INT);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 4;
	profile.signal_path_reset_reg = static_cast<uint8_t>(Register::BANK_0::SIGNAL_PATH_RESET);
	profile.fifo_flush_bit        = static_cast<uint8_t>(SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1);

	profile.fifo_count_is_records    = false;
	profile.fifo_count_little_endian = false;
	profile.fifo_temperature         = true;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = true;

	profile.device.name                 = "iim42653";
	profile.device.device_type          = DRV_IMU_DEVTYPE_IIM42653;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.min_clock_hz         = 31_kHz;
	profile.device.max_clock_hz         = 50_kHz;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_IIM42653

#if defined(CONFIG_TDK_ICM42X_ICM45686)
constexpr TdkIcm42x::Profile icm45686Profile()
{
	using namespace tdk_icm45686_registers;
	TdkIcm42x::Profile profile {};

	profile.variant             = Variant::kIcm45686;
	profile.whoami              = 0xe9;
	profile.protocol            = Protocol::kDirect456;
	profile.packet_format       = PacketFormat::kStandard16LittleEndian;
	profile.fifo_capacity       = FIFO::SIZE;
	profile.output_data_rate_hz = 6400_Hz;
	profile.packet_size         = sizeof(FIFO::DATA);
	profile.accel_range_g       = 32.f;
	profile.gyro_range_dps      = 4000.f;

	profile.temperature_sensitivity = TEMPERATURE_SENSITIVITY;
	profile.temperature_offset      = TEMPERATURE_OFFSET;

	profile.whoami_reg            = static_cast<uint8_t>(Register::BANK_0::WHO_AM_I);
	profile.reset_reg             = static_cast<uint8_t>(Register::BANK_0::REG_MISC2);
	profile.reset_bit             = static_cast<uint8_t>(REG_MISC2_BIT::SOFT_RST);
	profile.reset_status_bit      = 0x00;
	profile.power_reg             = static_cast<uint8_t>(Register::BANK_0::PWR_MGMT0);
	profile.int_status_reg        = static_cast<uint8_t>(Register::BANK_0::INT1_STATUS0);
	profile.reset_status_reg      = profile.int_status_reg;
	profile.fifo_full_bit         = static_cast<uint8_t>(INT1_STATUS0::INT1_STATUS_FIFO_FULL);
	profile.fifo_count_reg        = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNT_0);
	profile.fifo_data_reg         = static_cast<uint8_t>(Register::BANK_0::FIFO_DATA);
	profile.fifo_transfer_prefix  = 1;
	profile.signal_path_reset_reg = 0x00;
	profile.fifo_flush_bit        = 0x00;
	profile.temperature_reg       = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1_UI);

	profile.fifo_count_is_records    = true;
	profile.fifo_count_little_endian = true;
	profile.fifo_temperature         = false;
	profile.data_ready_interrupt     = true;
	profile.clock_input              = true;

	profile.device.name                 = "icm45686";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM45686;
	profile.device.frequency            = 24_MHz;
	profile.device.data_frequency       = 24_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm42x::maxTransferSize(profile.packet_size, profile.fifo_transfer_prefix);
	profile.device.data_prefix_bytes    = profile.fifo_transfer_prefix;
	profile.device.min_clock_hz         = 20_kHz;
	profile.device.max_clock_hz         = 40_kHz;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	return profile;
}
#endif // CONFIG_TDK_ICM42X_ICM45686

constexpr TdkIcm42x::Profile kModels[] {
#if defined(CONFIG_TDK_ICM42X_ICM40609D)
	icm40609DProfile(),
#endif // CONFIG_TDK_ICM42X_ICM40609D

#if defined(CONFIG_TDK_ICM42X_ICM42605)
	icm42605Profile(),
#endif // CONFIG_TDK_ICM42X_ICM42605

#if defined(CONFIG_TDK_ICM42X_ICM42670P)
	icm42670PProfile(),
#endif // CONFIG_TDK_ICM42X_ICM42670P

#if defined(CONFIG_TDK_ICM42X_ICM42688P)
	icm42688PProfile(),
#endif // CONFIG_TDK_ICM42X_ICM42688P

#if defined(CONFIG_TDK_ICM42X_ICM42686P)
	icm42686PProfile(),
#endif // CONFIG_TDK_ICM42X_ICM42686P

#if defined(CONFIG_TDK_ICM42X_IIM42652)
	iim42652Profile(),
#endif // CONFIG_TDK_ICM42X_IIM42652

#if defined(CONFIG_TDK_ICM42X_IIM42653)
	iim42653Profile(),
#endif // CONFIG_TDK_ICM42X_IIM42653

#if defined(CONFIG_TDK_ICM42X_ICM45686)
	icm45686Profile(),
#endif // CONFIG_TDK_ICM42X_ICM45686
};
static_assert(sizeof(kModels) > 0, "Select at least one headered FIFO model");

// Reset completion is signaled in the configured status register, never the zero-initialized profile field.
constexpr bool resetStatusRegistersValid()
{
	for (const auto &profile : kModels) {
		if (profile.reset_status_bit != 0
		    && (profile.reset_status_reg == 0 || profile.reset_status_reg != profile.int_status_reg)) {
			return false;
		}
	}

	return true;
}

static_assert(resetStatusRegistersValid(), "Reset completion must read the configured status register");
}

void TdkIcm42x::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for the TDK packet-FIFO family, including ICM42670P and ICM45686.
Wire framing, register access and initialization remain model-specific; validated
batches use the native PX4 accel/gyro publishers.
Select ICM42686P with -T icm42686p; the former -6 option is not supported.
-C supplies the external reference-clock frequency in Hz only for profiles that support it.
Omit -C to use the internal clock; an explicit zero is rejected.

Model availability depends on Kconfig. Start requires an exact -T model; stop/status
may omit -T to visit all compiled family instances matching the native bus selectors.
Without a bus selector, start uses board-registered internal SPI devices of the selected type.

### Examples
```
tdk_icm42x -T icm42688p start
tdk_icm42x -T icm42686p status
tdk_icm42x stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("tdk_icm42x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
		"icm40609d | icm42605 | icm42670p | icm42688p | icm42686p | iim42652 | iim42653 | icm45686",
		"Exact model (required for start; availability depends on Kconfig)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('C', 0, 1_Hz, 1_MHz, "Reference clock Hz; omit for internal clock; model limits also apply", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances; omit -T for all compiled family models");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances; omit -T for all compiled family models");

	for (const auto &model : kModels) {
		PX4_INFO("-T %s", model.device.name);
	}
}

extern "C" int tdk_icm42x_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<TdkIcm42x>(argc, argv, MODULE_NAME, kModels);
}
