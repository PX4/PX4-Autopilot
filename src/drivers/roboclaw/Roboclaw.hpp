/****************************************************************************
 *
 *   Copyright (C) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file Roboclaw.hpp
 *
 * Roboclaw motor control driver
 *
 * Product page: https://www.basicmicro.com/motor-controller
 * Manual: https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf
 */

#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <sys/select.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_encoders.h>

// Roboclaw 클래스 생성, cpp파일에 쓰임
// ModuleBase<Roboclaw>와 OutputModuleInterface 상속 받음. PX4 파일들과 연결되기 위해 필요함.
class Roboclaw : public ModuleBase<Roboclaw>, public OutputModuleInterface
{
public:
	/**
	 * @param device_name Name of the serial port e.g. "/dev/ttyS2"
	 * @param bad_rate_parameter Name of the parameter that holds the baud rate of this serial port
	 */
	// 생성자, 위에 지정한 포트 이름과 보드레이트를 받음
	Roboclaw(const char *device_name, const char *bad_rate_parameter);
	// 파괴자, 뒤처리
	virtual ~Roboclaw();

	// 모터 두개 정의, 우측이 0, 좌측이 1
	enum class Motor {
		Right = 0,
		Left = 1
	};

	// 모듈 관련
	static int task_spawn(int argc, char *argv[]); ///< @see ModuleBase
	static int custom_command(int argc, char *argv[]); ///< @see ModuleBase
	static int print_usage(const char *reason = nullptr); ///< @see ModuleBase
	int print_status() override; ///< @see ModuleBase

	// 메인 루프 함수 정의, 기존 모듈의 run을 덮어씀
	void Run() override;

	/** @see OutputModuleInterface */
	// cpp에 쓰일 핵심 함수 중 하나. setMotorSpeed()에 명령 하달. 마찬가지로 오버라이드로 덮어씀
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;
	// 모터 속도 제어 함수
	void setMotorSpeed(Motor motor, float value); ///< rev/sec
	// 이걸로 제어는 안하는 듯
	void setMotorDutyCycle(Motor motor, float value);
	// 엔코더 값 읽기
	int readEncoder();
	// 엔코더 초기화
	void resetEncoders();

private:
	// 모터 드라이버를 위한 각각의 명령어 정의
	enum class Command : uint8_t {
		ReadStatus = 90,

		DriveForwardMotor1 = 0,
		DriveBackwardsMotor1 = 1,
		DriveForwardMotor2 = 4,
		DriveBackwardsMotor2 = 5,
		DutyCycleMotor1 = 32,
		DutyCycleMotor2 = 33,

		ReadSpeedMotor1 = 18,
		ReadSpeedMotor2 = 19,
		ResetEncoders = 20,
		ReadEncoderCounters = 78,
	};

	static constexpr int MAX_ACTUATORS = 2;
	// 믹서를 위한 인스턴스 '_mixing_output'. Run() 즉 메인 루프에서 쓰임
	MixingOutput _mixing_output{"RBCLW", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	// actuator_armed, parameter_update 두 uORB 토픽 구독
	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	// wheel_encoders_s 토픽 퍼블리시
	uORB::Publication<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};

	char _stored_device_name[256]; // Adjust size as necessary
	char _stored_baud_rate_parameter[256]; // Adjust size as necessary

	// setMotorSpeed()에서 사용하는 통신
	void sendUnsigned7Bit(Command command, float data);
	// setMotorDutyCycle()에서 사용하는 통신
	void sendSigned16Bit(Command command, float data);
	// 위 두 함수 모두 sendTransaction()로 전송

	// sendTransaction()이 위에 두 통신법에서 받아 writeCommandWithPayload()로 보냄. 달되면 readAcknowledgement()으로
	int sendTransaction(Command cmd, uint8_t *write_buffer, size_t bytes_to_write);
	int writeCommandWithPayload(Command cmd, uint8_t *wbuff, size_t bytes_to_write);
	int readAcknowledgement();

	// initializeUART(), readEncoder() 발동시 receiveTransaction 실행, writeCommand()로 통신 확인, 잘되면 readResponce() 실행
	int receiveTransaction(Command cmd, uint8_t *read_buffer, size_t bytes_to_read);
	int writeCommand(Command cmd);
	int readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read);

	// 통신 데이터 신뢰성 보장을 위한 CRC 계산
	static uint16_t _calcCRC(const uint8_t *buf, size_t n, uint16_t init = 0);
	int32_t swapBytesInt32(uint8_t *buffer);

	// UART handling
	int initializeUART();
	bool _uart_initialized{false};
	int _uart_fd{0};
	fd_set _uart_fd_set;
	struct timeval _uart_fd_timeout;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RBCLW_ADDRESS>) _param_rbclw_address,
		(ParamInt<px4::params::RBCLW_COUNTS_REV>) _param_rbclw_counts_rev
	)
};
