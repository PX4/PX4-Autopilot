/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file messages.h
 *
 * @brief Septentrio binary format (SBF) protocol message definitions.
 *
 * @author Thomas Frans
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace septentrio
{

namespace sbf
{

constexpr char k_sync1 {'$'};
constexpr char k_sync2 {'@'};

// Do-Not-Use values for fields in SBF blocks. The receiver can set certain fields to these values to signal that they are invalid.
// Most fields of a certain type will use these values (u2 representing a value often has DNU value of 65535).
// For the ones that do not use these, custom ones can be specified together with the blocks.
constexpr uint32_t k_dnu_u4_value {4294967295};
constexpr uint32_t k_dnu_u4_bitfield {0};
constexpr uint16_t k_dnu_u2_value {65535};
constexpr uint16_t k_dnu_u2_bitfield {0};
constexpr float k_dnu_f4_value {-2.0f * 10000000000.0f};
constexpr double k_dnu_f8_value {-2.0 * 10000000000.0};

/// Maximum size of all expected messages.
/// This needs to be bigger than the maximum size of all declared SBF blocks so that `memcpy()` can safely copy from the decoding buffer using this value into messages.
constexpr size_t k_max_message_size {300};

/// Maximum expected number of sub-blocks.
constexpr uint8_t k_max_quality_indicators {9};
constexpr uint8_t k_max_rfband_blocks {4};

/**
 * IDs of all the available SBF messages that we care about.
*/
enum class BlockID : uint16_t {
	Invalid        = 0,
	DOP            = 4001,
	PVTGeodetic    = 4007,
	ReceiverStatus = 4014,
	QualityInd     = 4082,
	RFStatus       = 4092,
	GALAuthStatus  = 4245,
	VelCovGeodetic = 5908,
	EndOfPVT       = 5921,
	GEOIonoDelay   = 5933,
	AttEuler       = 5938,
	AttCovEuler    = 5939,
};

#pragma pack(push, 1)

/**
 * Common SBF message header.
 */
struct Header {
	uint8_t sync1;
	uint8_t sync2;
	uint16_t crc;
	BlockID id_number: 13;
	uint16_t id_revision: 3;
	uint16_t length;
	uint32_t tow;
	uint16_t wnc;
};

struct DOP {
	uint8_t nr_sv;
	uint8_t reserved;
	uint16_t p_dop;
	uint16_t t_dop;
	uint16_t h_dop;
	uint16_t v_dop;
	float hpl;
	float vpl;
};

struct PVTGeodetic {
	static constexpr uint8_t k_dnu_nr_sv {255};
	enum class ModeType : uint8_t {
		NoPVT                   = 0,
		StandAlonePVT           = 1,
		DifferentialPVT         = 2,
		FixedLocation           = 3,
		RTKFixed                = 4,
		RTKFloat                = 5,
		PVTWithSBAS             = 6,
		MovingBaseRTKFixed      = 7,
		MovingBaseRTKFloat      = 8,
		PrecisePointPositioning = 10,
	};
	enum class Error : uint8_t {
		None = 0,
		InsufficientMeasurements = 1,
		InsufficientEphemerides = 2,
		DOPTooLarge = 3,
		SquaredResidualSumTooLarge = 4,
		NoConvergence = 5,
		InsufficientMeasurementsAfterOutlierRejection = 6,
		ExportLawsForbidPositionOutput = 7,
		InsufficientDifferentialCorrections = 8,
		MissingBaseStationCoordinates = 9,
		MissingRequiredRTKFixedPosition = 10,
	};
	ModeType mode_type: 4;
	uint8_t mode_reserved: 2;
	uint8_t mode_base_fixed: 1;
	uint8_t mode_2d: 1;
	Error error;
	double latitude;
	double longitude;
	double height;
	float undulation;
	float vn;
	float ve;
	float vu;
	float cog;
	double rx_clk_bias;
	float rx_clk_drift;
	uint8_t time_system;
	uint8_t datum;
	uint8_t nr_sv;
	uint8_t wa_corr_info;
	uint16_t reference_id;
	uint16_t mean_corr_age;
	uint32_t signal_info;
	uint8_t alert_flag;
	uint8_t nr_bases;
	uint16_t ppp_info;
	uint16_t latency;
	uint16_t h_accuracy;
	uint16_t v_accuracy;
};

struct ReceiverStatus {
	uint8_t  cpu_load;
	uint8_t  ext_error_siserror: 1;
	uint8_t  ext_error_diff_corr_error: 1;
	uint8_t  ext_error_ext_sensor_error: 1;
	uint8_t  ext_error_setup_error: 1;
	uint8_t  ext_error_reserved: 4;
	uint32_t uptime;
	uint32_t rx_state_reserved1: 1;
	uint32_t rx_state_active_antenna: 1;
	uint32_t rx_state_ext_freq: 1;
	uint32_t rx_state_ext_time: 1;
	uint32_t rx_state_wn_set: 1;
	uint32_t rx_state_tow_set: 1;
	uint32_t rx_state_fine_time: 1;
	uint32_t rx_state_internal_disk_activity: 1;
	uint32_t rx_state_internal_disk_full: 1;
	uint32_t rx_state_internal_disk_mounted: 1;
	uint32_t rx_state_int_ant: 1;
	uint32_t rx_state_refout_locked: 1;
	uint32_t rx_state_reserved2: 1;
	uint32_t rx_state_external_disk_activity: 1;
	uint32_t rx_state_external_disk_full: 1;
	uint32_t rx_state_external_disk_mounted: 1;
	uint32_t rx_state_pps_in_cal: 1;
	uint32_t rx_state_diff_corr_in: 1;
	uint32_t rx_state_internet: 1;
	uint32_t rx_state_reserved3: 13;
	uint32_t rx_error_reserved1: 3;
	uint32_t rx_error_software: 1;
	uint32_t rx_error_watchdog: 1;
	uint32_t rx_error_antenna: 1;
	uint32_t rx_error_congestion: 1;
	uint32_t rx_error_reserved2: 1;
	uint32_t rx_error_missed_event: 1;
	uint32_t rx_error_cpu_overload: 1;
	uint32_t rx_error_invalid_config: 1;
	uint32_t rx_error_out_of_geofence: 1;
	uint32_t rx_error_reserved3: 22;
	uint8_t n;
	uint8_t sb_length;
	uint8_t cmd_count;
	uint8_t temperature;
};

struct QualityIndicator {
	enum class Type : uint8_t {
		Overall                 = 0,
		GNSSSignalsMainAntenna  = 1,
		GNSSSignalsAuxAntenna   = 2,
		RFPowerMainAntenna      = 11,
		RFPowerAuxAntenna       = 12,
		CPUHeadroom             = 21,
		OCXOStability           = 25,
		BaseStationMeasurements = 30,
		RTKPostProcessing       = 31,
	};
	Type type: 8;
	uint16_t value: 4;
	uint16_t reserved: 4;
};

struct QualityInd {
	uint8_t n;
	uint8_t reserved1;
	// Only support the currently available indicators for now so we don't have to allocate.
	QualityIndicator indicators[k_max_quality_indicators];
};

struct RFBand {
	uint32_t frequency;
	uint16_t bandwidth;
	uint8_t  info_mode: 4;
	uint8_t  info_reserved: 2;
	uint8_t  info_antenna_id: 2;
};

struct RFStatus {
	uint8_t n;
	uint8_t sb_length;
	uint8_t flags_inauthentic_gnss_signals: 1;
	uint8_t flags_inauthentic_navigation_message: 1;
	uint8_t flags_reserved: 6;
	uint8_t reserved[3];
	RFBand rf_band[k_max_rfband_blocks];
};

struct GALAuthStatus {
	uint16_t osnma_status_status: 3;
	uint16_t osnma_status_initialization_progress: 8;
	uint16_t osnma_status_trusted_time_source: 3;
	uint16_t osnma_status_merkle_tree_busy: 1;
	uint16_t osnma_status_reserved: 1;
	float trusted_time_delta;
	uint64_t gal_active_mask;
	uint64_t gal_authentic_mask;
	uint64_t gps_active_mask;
	uint64_t gps_authentic_mask;
};

struct VelCovGeodetic {
	uint8_t mode_type: 4;
	uint8_t mode_reserved: 2;
	uint8_t mode_base_fixed: 1;
	uint8_t mode_2d: 1;
	uint8_t error;
	float cov_vn_vn;
	float cov_ve_ve;
	float cov_vu_vu;
	float cov_dt_dt;
	float cov_vn_ve;
	float cov_vn_vu;
	float cov_vn_dt;
	float cov_ve_vu;
	float cov_ve_dt;
	float cov_vu_dt;
};

struct IDC {
	uint8_t igp_mask_no;
	uint8_t givei;
	uint8_t reserved[2];
	float vertical_delay;
};

struct GEOIonoDelay {
	uint8_t prn;
	uint8_t bandnbr;
	uint8_t iodi;
	uint8_t n;
	uint8_t sb_length;
	uint8_t reserved;
	IDC idc[4];
};

struct AttEuler {
	enum class Error : uint8_t {
		None                     = 0,
		InsufficientMeasurements = 1,
	};
	uint8_t nr_sv;
	Error error_aux1: 2;
	Error error_aux2: 2;
	uint8_t error_reserved: 3;
	uint8_t error_not_requested: 1;
	uint16_t mode;
	uint16_t reserved;
	float heading;
	float pitch;
	float roll;
	float pitch_dot;
	float roll_dot;
	float heading_dot;
};

struct AttCovEuler {
	enum class Error : uint8_t {
		None                     = 0,
		InsufficientMeasurements = 1,
	};
	uint8_t reserved;
	Error error_aux1: 2;
	Error error_aux2: 2;
	uint8_t error_reserved: 3;
	uint8_t error_not_requested: 1;
	float cov_headhead;
	float cov_pitchpitch;
	float cov_rollroll;
	float cov_headpitch;
	float cov_headroll;
	float cov_pitchroll;
};

#pragma pack(pop)

} // namespace sbf

} // namespace septentrio
