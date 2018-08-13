#ifndef SYSTEM_CONTROL_CLIENT_H
#define SYSTEM_CONTROL_CLIENT_H

#include "communication_interface.h"
#include "client_communication.hpp"

//TODO::Cleanup then include common_message_types and delete the below line
const uint8_t kTypeSystemControl = 5;

class SystemControlClient: public ClientAbstract{
  public:
    SystemControlClient(uint8_t obj_idn):
      ClientAbstract(     kTypeSystemControl, obj_idn),
      reboot_program_(    kTypeSystemControl, obj_idn,kSubRebootProgram),
      reboot_boot_loader_(kTypeSystemControl, obj_idn,kSubRebootBootLoader),
      dev_id_(            kTypeSystemControl, obj_idn, kSubDevId),
      rev_id_(            kTypeSystemControl, obj_idn, kSubRevId),
      uid1_(              kTypeSystemControl, obj_idn, kSubUid1),
      uid2_(              kTypeSystemControl, obj_idn, kSubUid2),
      uid3_(              kTypeSystemControl, obj_idn, kSubUid3),
      mem_size_(          kTypeSystemControl, obj_idn, kSubMemSize),
      build_year_(        kTypeSystemControl, obj_idn, kSubBuildYear),
      build_month_(       kTypeSystemControl, obj_idn, kSubBuildMonth),
      build_day_(         kTypeSystemControl, obj_idn, kSubBuildDay),
      build_hour_(        kTypeSystemControl, obj_idn, kSubBuildHour),
      build_minute_(      kTypeSystemControl, obj_idn, kSubBuildMinuite),
      build_second_(      kTypeSystemControl, obj_idn, kSubBuildSecond),
      module_id_(         kTypeSystemControl, obj_idn, kSubModuleId)
      {};

    // Client Entries
    ClientEntryVoid       reboot_program_;
    ClientEntryVoid       reboot_boot_loader_;
    ClientEntry<uint16_t> dev_id_;
    ClientEntry<uint16_t> rev_id_;
    ClientEntry<uint32_t> uid1_;
    ClientEntry<uint32_t> uid2_;
    ClientEntry<uint32_t> uid3_;
    ClientEntry<uint16_t> mem_size_;
    ClientEntry<uint16_t> build_year_;
    ClientEntry<uint8_t>  build_month_;
    ClientEntry<uint8_t>  build_day_;
    ClientEntry<uint8_t>  build_hour_;
    ClientEntry<uint8_t>  build_minute_;
    ClientEntry<uint8_t>  build_second_;
    ClientEntry<uint8_t>  module_id_;

    void ReadMsg(CommunicationInterface& com,
      uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubBuildSecond+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &reboot_program_,     // 0
        &reboot_boot_loader_, // 1
        &dev_id_,             // 2
        &rev_id_,             // 3
        &uid1_,               // 4
        &uid2_,               // 5
        &uid3_,               // 6
        &mem_size_,           // 7
        &build_year_,         // 8
        &build_month_,        // 9
        &build_day_,          // 10
        &build_hour_,         // 11
        &build_minute_,       // 12
        &build_second_,       // 13
        &module_id_           // 14
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubRebootProgram    = 0;
    static const uint8_t kSubRebootBootLoader = 1;
    static const uint8_t kSubDevId            = 2;
    static const uint8_t kSubRevId            = 3;
    static const uint8_t kSubUid1             = 4;
    static const uint8_t kSubUid2             = 5;
    static const uint8_t kSubUid3             = 6;
    static const uint8_t kSubMemSize          = 7;
    static const uint8_t kSubBuildYear        = 8;
    static const uint8_t kSubBuildMonth       = 9;
    static const uint8_t kSubBuildDay         = 10;
    static const uint8_t kSubBuildHour        = 11;
    static const uint8_t kSubBuildMinuite     = 12;
    static const uint8_t kSubBuildSecond      = 13;
    static const uint8_t kSubModuleId         = 14;
};

#endif // SYSTEM_CONTROL_CLIENT_H
