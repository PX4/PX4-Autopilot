/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * The header is a modified version of a header from this repo:
 * https://github.com/euler-nav/bahrs-oss
 *
 * Copyright 2023 AMS Advanced Air Mobility Sensors UG
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>

#define PROTOCOL_WORD_LEN (4)
#define PADDING_SIZE(SPayloadType) (PROTOCOL_WORD_LEN - ((sizeof(SMessageHeader) + sizeof(SPayloadType)) % PROTOCOL_WORD_LEN))

#define BIT_VALID_HEIGHT           (0x01)
#define BIT_VALID_VELOCITY_DOWN    (0x02)
#define BIT_VALID_ROLL             (0x04)
#define BIT_VALID_PITCH            (0x08)
#define BIT_VALID_MAGNETIC_HEADING (0x10)
#define BIT_VALID_SPECIFIC_FORCE_X (0x01)
#define BIT_VALID_SPECIFIC_FORCE_Y (0x02)
#define BIT_VALID_SPECIFIC_FORCE_Z (0x04)
#define BIT_VALID_ANGULAR_RATE_X   (0x08)
#define BIT_VALID_ANGULAR_RATE_Y   (0x10)
#define BIT_VALID_ANGULAR_RATE_Z   (0x20)

/**
 * @brief The class that describes and implements the BAHRS serial protocol.
*/
class CSerialProtocol
{
public:
	static constexpr uint8_t uMarker1_ { 0x4E }; ///< Sync byte 1, symbol 'N'
	static constexpr uint8_t uMarker2_ { 0x45 }; ///< Sync char 2, symbol 'E'
	static constexpr uint16_t uVersion_ { 2U }; ///< Protocol version

	static constexpr float skfSpecificForceScale_ { 1.495384e-3F }; ///< Integer to float, +-5g range
	static constexpr float skfAngularRateScale_ { 1.597921e-4F }; ///< Integer to float, +-300 deg/s range
	static constexpr float skfHeightScale_ { 0.16784924F }; ///< Integer to float, -1000 to 10000 m range
	static constexpr float skfHeighOffset_ { 1000.0F }; ///< An offset to convert unsigned integer to float
	static constexpr float skfVelocityDownScale_ { 9.155413e-3F }; ///< Integer to float, -300 to 300 m/s range
	static constexpr float skfAngleScale_ { 9.587526e-5F }; ///< Integer to float, -pi to pi or 0 to 2 pi range

	// Signal ranges
	static constexpr float skfMaxHeight_ { 10000.0F };
	static constexpr float skfMinHeight_ { -1000.0F };
	static constexpr float skfMaxVelocityDown_ { 300.0F };
	static constexpr float skfMinVelocityDown_ { -300.0F };

	enum class EMessageIds : uint8_t {
		eInvalid = 0x00, ///< Invalid
		eInertialData = 0x01, ///< Inertial data message
		eNavigationData = 0x02, ///< Navigation data message
		eAccuracy = 0x03, ///< Attitude accuracy information
		eTimeOfNavigationData = 0x04, ///< "Time of navigation data" message
		eTimeOfInertialData = 0x05, ///< "Time of inertial data" message
		eTimeOfSyncPulse = 0x06, ///< "Time of the latest sync pulse" message
		eSoftwareVersion = 0x0F, ///< "Software version" message

		eDebugEventWriteToPort = 0xC0, ///< Debug information: SWC port data
		eDebugEventRunnableCall = 0xC1, ///< Debug information: SWC API call

		eTypeOpenDiagnostic = 0xF0, ///< Request to enter diagnostics mode
		eTypeCloseDiagnostic = 0xF1, ///< Request to exit diagnostics mode
		eTypeReadNVMPage = 0xF2, ///< Request to read NVM page
		eTypeNVMPageData = 0xF3, ///< A message with NVM page data
		eTypeAccept = 0xFF ///< Acknowledgment of diagnostics message reception
	};

	typedef uint32_t CrcType_t;

#pragma pack(push, 1)

	/**
	 * @brief Message header struct.
	*/
	struct SMessageHeader {
		uint8_t uMarker1_ { 0x4E };
		uint8_t uMarker2_ { 0x45 };
		uint16_t uVersion_ { CSerialProtocol::uVersion_ };
		uint8_t uMsgType_ { 0U };
	};

	/**
	 * @brief Inertial data message payload.
	*/
	struct SInertialData {
		uint8_t uSequenceCounter_ { 0U };
		int16_t iSpecificForceX_ { 0 };
		int16_t iSpecificForceY_ { 0 };
		int16_t iSpecificForceZ_ { 0 };
		int16_t iAngularRateX_ { 0 };
		int16_t iAngularRateY_ { 0 };
		int16_t iAngularRateZ_ { 0 };
		uint8_t uValidity_ { 0U };
	};

	/**
	 * @brief Payload of the time of inertial data message.
	*/
	struct STimeOfInertialData {
		uint8_t uSequenceCounter_ { 0U };
		uint8_t uInertialDataSequenceCounter_ { 0U };
		uint64_t uTimestampUs_ { 0U };
	};

	/**
	 * @brief Navigation data message payload.
	*/
	struct SNavigationData {
		uint8_t uSequenceCounter_ { 0U };
		uint16_t uPressureHeight_ { 0 };
		int16_t iVelocityDown_ { 0 };
		int16_t iRoll_ { 0 };
		int16_t iPitch_ { 0 };
		uint16_t uMagneticHeading_ { 0U };
		uint8_t uValidity_ { 0 };
	};

	/**
	 * @brief Payload of the "time of navigation data" message.
	*/
	struct STimeOfNavigationData {
		uint8_t uSequenceCounter_ { 0U };
		uint8_t uNavigationDataSequenceCounter_ { 0U };
		uint64_t uTimestampUs_ { 0U };
	};

	/**
	 * @brief Accuracy message payload.
	*/
	struct SAccuracyData {
		uint8_t uSequenceCounter_ { 0U };
		uint16_t uAttitudeStdN_ { 0U };
		uint16_t uAttitudeStdE_ { 0U };
		uint16_t uMagneticHeadingStd_ { 0U };
		uint64_t uTimestampUs_ { 0U };
	};

	/**
	 * @brief Payload of the "time of the latest pulse" message.
	*/
	struct STimeOfLatestSyncPulse {
		uint8_t uSequenceCounter_ { 0U };
		uint64_t uTimestampUs_ { 0U };
	};

	/**
	 * @brief Payload of the "software version" message.
	*/
	struct SSoftwareVersionData {
		char acProjectCode_[3] { '\0', '\0', '\0' };
		uint16_t uMajor_ { 0U };
		uint16_t uMinor_ { 0U };
	};

	/**
	 * @brief Partial data of the "write to port event".
	 * The struct does not include a variable size array of data written to a port.
	 */
	struct SWriteToPortEventPartialData {
		uint8_t uPortId_ { 0U };
		uint64_t uTimestampUs_ { 0U };
		uint16_t uDataLen_ { 0U };
	};

	/**
	 * @brief Debug information on runnable call.
	 */
	struct SRunnableCallEventData {
		uint8_t uRunnableId_ { 0U };
		uint64_t uTimestampUs_ { 0U };
	};

	/**
	 * @brief Inertial data message.
	*/
	struct SInertialDataMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eInertialData) };

		SInertialData oInertialData_;
		uint8_t auPadding_[PADDING_SIZE(SInertialData)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief Time of inertial data message.
	*/
	struct STimeOfInertialDataMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eTimeOfInertialData) };

		STimeOfInertialData oTimeOfInertialData_;
		uint8_t auPadding_[PADDING_SIZE(STimeOfInertialData)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief Navigation data message.
	*/
	struct SNavigationDataMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eNavigationData) };

		SNavigationData oNavigationData_;
		uint8_t auPadding_[PADDING_SIZE(SNavigationData)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief Navigation data accuracy message.
	*/
	struct SAccuracyDataMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eAccuracy) };

		SAccuracyData oAccuracy_;
		uint8_t auPadding_[PADDING_SIZE(SAccuracyData)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief Time of navigation data message.
	*/
	struct STimeOfNavigationDataMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eTimeOfNavigationData) };

		STimeOfNavigationData oTimeOfNavigationData_;
		uint8_t auPadding_[PADDING_SIZE(STimeOfNavigationData)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief Time of the latest sync pulse message.
	*/
	struct STimeOfLatestSyncPulseMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eTimeOfSyncPulse) };

		STimeOfLatestSyncPulse oTimeOfLatestSyncPulse_;
		uint8_t auPadding_[PADDING_SIZE(STimeOfLatestSyncPulse)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief Software version message.
	*/
	struct SSoftwareVersionMessage {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eSoftwareVersion) };

		SSoftwareVersionData oSoftwareVersion_;
		uint8_t auPadding_[PADDING_SIZE(SSoftwareVersionData)];
		CrcType_t uCrc_ { 0U };
	};

	/**
	 * @brief A debug message with runnable call data.
	*/
	struct SRunnableCallEventDebugMessage {
		SMessageHeader oHeader_{ CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eDebugEventRunnableCall) };

		SRunnableCallEventData oRunnableCallData_;
		uint8_t auPadding_[PADDING_SIZE(oRunnableCallData_)];
		CrcType_t uCrc_{ 0U };
	};

	/**
	 * @brief Diagnostic Mode Messages
	 ****************************************************************************/

	/**
	 * @brief Diagnostic message: Page request from NVM.
	*/
	struct SPacketReadNVMPage {
		SMessageHeader oHeader_;
		uint8_t uPageNumber { 0U };
		CrcType_t uCrc32 { 0U };
	};

	/**
	 * @brief Diagnostic message: confirmation of request processing by the device.
	*/
	struct SPacketReceiveConfirmation {
		SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
				       CSerialProtocol::uMarker2_,
				       CSerialProtocol::uVersion_,
				       static_cast<uint8_t>(EMessageIds::eTypeAccept) };
		EMessageIds eMessageType;    // message type to confirm.
		uint8_t uStatus { 0U };      // 0 - OK, other values - error codes
		CrcType_t uCrc32 { 0U };
	};

#pragma pack(pop)

};
