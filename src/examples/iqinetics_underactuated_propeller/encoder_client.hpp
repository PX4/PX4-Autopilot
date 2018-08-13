#ifndef ENCODER_CLIENT_H
#define ENCODER_CLIENT_H

#include "communication_interface.h"
#include "client_communication.hpp"

//TODO::Cleanup then include common_message_types and delete the below line
const uint8_t kTypeEncoder = 53;

class EncoderClient: public ClientAbstract{
  public:
    EncoderClient(uint8_t obj_idn):
      ClientAbstract(     kTypeEncoder, obj_idn),
      zero_angle_(        kTypeEncoder, obj_idn, kSubZeroAngle),
      velocity_filter_fs_(kTypeEncoder, obj_idn, kSubVelocityFilterFs),
      velocity_filter_fc_(kTypeEncoder, obj_idn, kSubVelocityFilterFc),
      rev_(                kTypeEncoder, obj_idn, kSubRev),
      absolute_rev_(      kTypeEncoder, obj_idn, kSubAbsoluteRev),
      rad_(               kTypeEncoder, obj_idn, kSubRad),
      absolute_rad_(      kTypeEncoder, obj_idn, kSubAbsoluteRad),
      velocity_(          kTypeEncoder, obj_idn, kSubVelocity)
      {};

    // Client Entries
    ClientEntry<float>    zero_angle_;
    ClientEntry<uint32_t> velocity_filter_fs_;
    ClientEntry<uint32_t> velocity_filter_fc_;
    ClientEntry<uint32_t> rev_;
    ClientEntry<uint32_t> absolute_rev_;
    ClientEntry<float>    rad_;
    ClientEntry<float>    absolute_rad_;
    ClientEntry<float>    velocity_;

    void ReadMsg(CommunicationInterface& com,
      uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubVelocity+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &zero_angle_,         // 0
        &velocity_filter_fs_, // 1
        &velocity_filter_fc_, // 2
        &rev_,                // 3
        &absolute_rev_,       // 4
        &rad_,                // 5
        &absolute_rad_,       // 6
        &velocity_            // 7
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubZeroAngle        = 0;
    static const uint8_t kSubVelocityFilterFs = 1;
    static const uint8_t kSubVelocityFilterFc = 2;
    static const uint8_t kSubRev              = 3;
    static const uint8_t kSubAbsoluteRev      = 4;
    static const uint8_t kSubRad              = 5;
    static const uint8_t kSubAbsoluteRad      = 6;
    static const uint8_t kSubVelocity         = 7;
};

#endif // ENCODER_CLIENT_H
