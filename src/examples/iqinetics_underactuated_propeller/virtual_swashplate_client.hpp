/*
 * virtual_swashplate_client.hpp
 *
 *  Created on: Aug 9, 2017
 *      Author: Matthew Piccoli
 */

#ifndef VIRTUAL_SWASHPLATE_CLIENT_HPP_
#define VIRTUAL_SWASHPLATE_CLIENT_HPP_

#include "communication_interface.h"
#include "client_communication.hpp"

const uint8_t kTypeVirtualSwashplate = 230;

class VirtualSwashplateClient: public ClientAbstract{
  public:
    VirtualSwashplateClient(uint8_t obj_idn):
      ClientAbstract(               kTypeVirtualSwashplate, obj_idn),
      is_positive_rotation_(        kTypeVirtualSwashplate, obj_idn, kSubIsPositiveRotation),
      advance_angle(                kTypeVirtualSwashplate, obj_idn, kSubAdvanceAngle),
      pulse_zero_threshold_(        kTypeVirtualSwashplate, obj_idn, kSubPulseZeroThreshold),
      pulse_deadband_compensation_( kTypeVirtualSwashplate, obj_idn, kSubPulseDeadbandCompensation),
      pulse_saturate_(              kTypeVirtualSwashplate, obj_idn, kSubPulseSaturate),
      speed_(                       kTypeVirtualSwashplate, obj_idn, kSubSpeed),
      roll_(                        kTypeVirtualSwashplate, obj_idn, kSubRoll),
      pitch_(                       kTypeVirtualSwashplate, obj_idn, kSubPitch)
      {};

    // Client Entries
    // Drive values
    ClientEntry<bool>       is_positive_rotation_;
    ClientEntry<float>      advance_angle;
    ClientEntry<float>      pulse_zero_threshold_;
    ClientEntry<float>      pulse_deadband_compensation_;
    ClientEntry<float>      pulse_saturate_;
    ClientEntry<float>      speed_;
    ClientEntry<float>      roll_;
    ClientEntry<float>      pitch_;

    void ReadMsg(CommunicationInterface& com,
      uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubPitch+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &is_positive_rotation_,         // 0
        &advance_angle,                 // 1
        &pulse_zero_threshold_,         // 2
        &pulse_deadband_compensation_,  // 3
        &pulse_saturate_,               // 4
        &speed_,                        // 5
        &roll_,                         // 6
        &pitch_                        // 7
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubIsPositiveRotation         = 0;
    static const uint8_t kSubAdvanceAngle               = 1;
    static const uint8_t kSubPulseZeroThreshold         = 2;
    static const uint8_t kSubPulseDeadbandCompensation  = 3;
    static const uint8_t kSubPulseSaturate              = 4;
    static const uint8_t kSubSpeed                      = 5;
    static const uint8_t kSubRoll                       = 6;
    static const uint8_t kSubPitch                      = 7;
};

#endif /* VIRTUAL_SWASHPLATE_CLIENT_HPP_ */
