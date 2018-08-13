/*
 * propeller_motor_control_client.hpp
 *
 *  Created on: Dec 28, 2017
 *      Author: Matthew Piccoli
 */

#ifndef COMMON_CPP_INC_PROPELLER_MOTOR_CONTROL_CLIENT_HPP_
#define COMMON_CPP_INC_PROPELLER_MOTOR_CONTROL_CLIENT_HPP_

#include "communication_interface.h"
#include "client_communication.hpp"

//TODO::Cleanup then include common_message_types and delete the below line
const uint8_t kTypePropellerMotorControl  =   52;

class PropellerMotorControlClient: public ClientAbstract{
  public:
    PropellerMotorControlClient(uint8_t obj_idn):
      ClientAbstract(   kTypePropellerMotorControl, obj_idn),
      ctrl_mode_(       kTypePropellerMotorControl, obj_idn, kSubCtrlMode),
      ctrl_brake_(      kTypePropellerMotorControl, obj_idn, kSubCtrlBrake),
      ctrl_coast_(      kTypePropellerMotorControl, obj_idn, kSubCtrlCoast),
      ctrl_pwm_(        kTypePropellerMotorControl, obj_idn, kSubCtrlPwm),
      ctrl_volts_(      kTypePropellerMotorControl, obj_idn, kSubCtrlVolts),
      ctrl_velocity_(   kTypePropellerMotorControl, obj_idn, kSubCtrlVelocity),
      ctrl_thrust_(     kTypePropellerMotorControl, obj_idn, kSubCtrlThrust),
      velocity_Kp_(     kTypePropellerMotorControl, obj_idn, kSubVelocityKp),
      velocity_Ki_(     kTypePropellerMotorControl, obj_idn, kSubVelocityKi),
      velocity_Kd_(     kTypePropellerMotorControl, obj_idn, kSubVelocityKd),
      velocity_ff0_(    kTypePropellerMotorControl, obj_idn, kSubVelocityFF0),
      velocity_ff1_(    kTypePropellerMotorControl, obj_idn, kSubVelocityFF1),
      velocity_ff2_(    kTypePropellerMotorControl, obj_idn, kSubVelocityFF2),
      propeller_kt_pos_(kTypePropellerMotorControl, obj_idn, kSubPropellerKtPos),
      propeller_kt_neg_(kTypePropellerMotorControl, obj_idn, kSubPropellerKtNeg),
      timeout_(         kTypePropellerMotorControl, obj_idn, kSubTimeout),
      input_filter_fc_( kTypePropellerMotorControl, obj_idn, kSubInputFilterFc)
      {};

    // Client Entries
    // Control commands
    ClientEntry<uint8_t>    ctrl_mode_;
    ClientEntryVoid         ctrl_brake_;
    ClientEntryVoid         ctrl_coast_;
    ClientEntry<float>      ctrl_pwm_;
    ClientEntry<float>      ctrl_volts_;
    ClientEntry<float>      ctrl_velocity_;
    ClientEntry<float>      ctrl_thrust_;
    // Velocity control
    ClientEntry<float>      velocity_Kp_;
    ClientEntry<float>      velocity_Ki_;
    ClientEntry<float>      velocity_Kd_;
    ClientEntry<float>      velocity_ff0_;
    ClientEntry<float>      velocity_ff1_;
    ClientEntry<float>      velocity_ff2_;
    // Propeller values
    ClientEntry<float>      propeller_kt_pos_;
    ClientEntry<float>      propeller_kt_neg_;
    // Timeout
    ClientEntry<float>      timeout_;
    // Filter
    ClientEntry<uint32_t>   input_filter_fc_;

    void ReadMsg(CommunicationInterface& com,
          uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubInputFilterFc+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &ctrl_mode_,        // 0
        &ctrl_brake_,       // 1
        &ctrl_coast_,       // 2
        &ctrl_pwm_,         // 3
        &ctrl_volts_,       // 4
        &ctrl_velocity_,    // 5
        &ctrl_thrust_,      // 6
        &velocity_Kp_,      // 7
        &velocity_Ki_,      // 8
        &velocity_Kd_,      // 9
        &velocity_ff0_,     // 10
        &velocity_ff1_,     // 11
        &velocity_ff2_,     // 12
        &propeller_kt_pos_, // 13
        &propeller_kt_neg_, // 14
        &timeout_,          // 15
        &input_filter_fc_   // 16
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubCtrlMode         =  0;
    static const uint8_t kSubCtrlBrake        =  1;
    static const uint8_t kSubCtrlCoast        =  2;
    static const uint8_t kSubCtrlPwm          =  3;
    static const uint8_t kSubCtrlVolts        =  4;
    static const uint8_t kSubCtrlVelocity     =  5;
    static const uint8_t kSubCtrlThrust       =  6;
    static const uint8_t kSubVelocityKp       =  7;
    static const uint8_t kSubVelocityKi       =  8;
    static const uint8_t kSubVelocityKd       =  9;
    static const uint8_t kSubVelocityFF0      = 10;
    static const uint8_t kSubVelocityFF1      = 11;
    static const uint8_t kSubVelocityFF2      = 12;
    static const uint8_t kSubPropellerKtPos   = 13;
    static const uint8_t kSubPropellerKtNeg   = 14;
    static const uint8_t kSubTimeout          = 15;
    static const uint8_t kSubInputFilterFc    = 16;
};

#endif /* COMMON_CPP_INC_PROPELLER_MOTOR_CONTROL_CLIENT_HPP_ */
