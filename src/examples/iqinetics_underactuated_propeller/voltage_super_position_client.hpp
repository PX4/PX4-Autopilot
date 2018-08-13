/*
 * propeller_motor_control_client.hpp
 *
 *  Created on: Aug 13, 2017
 *      Author: Raphael Van hoffelen
 */

#ifndef COMMON_CPP_INC_VOLTAGE_SUPERPOSITION_CLIENT_HPP_
#define COMMON_CPP_INC_VOLTAGE_SUPERPOSITION_CLIENT_HPP_

#include "communication_interface.h"
#include "client_communication.hpp"

const uint8_t kTypeVoltageSuperposition = 74;

class VoltageSuperPositionClient: public ClientAbstract{
	public:
		VoltageSuperPositionClient(uint8_t obj_idn):
			ClientAbstract(  kTypeVoltageSuperposition, obj_idn),
			zero_angle_(     kTypeVoltageSuperposition, obj_idn, kSubZeroAngle),
			frequency_(      kTypeVoltageSuperposition, obj_idn, kSubFrequency),
			phase_(          kTypeVoltageSuperposition, obj_idn, kSubPhase),
			amplitude_(      kTypeVoltageSuperposition, obj_idn, kSubAmplitude),
			voltage_(        kTypeVoltageSuperposition, obj_idn, kSubVoltage)
			{};

		// Client Entries
		ClientEntry<float>    zero_angle_;
		ClientEntry<uint8_t>  frequency_;
		ClientEntry<float>    phase_;
		ClientEntry<float>    amplitude_;
		ClientEntry<float>    voltage_;

		void ReadMsg(CommunicationInterface& com,
          uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubVoltage+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &zero_angle_,        // 0
        &frequency_,       	 // 1
        &phase_,       			 // 2
        &amplitude_,         // 3
        &voltage_,      		 // 4
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

 private:
		static const uint8_t kSubZeroAngle = 0;
		static const uint8_t kSubFrequency = 1;
		static const uint8_t kSubPhase     = 2;
		static const uint8_t kSubAmplitude = 3;
		static const uint8_t kSubVoltage   = 4;
};

#endif