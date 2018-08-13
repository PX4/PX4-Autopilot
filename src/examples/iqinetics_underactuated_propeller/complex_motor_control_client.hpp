#ifndef COMPLEX_MOTOR_CONTROL_CLIENT_H
#define COMPLEX_MOTOR_CONTROL_CLIENT_H

#include "communication_interface.h"
#include "client_communication.hpp"

//TODO::Cleanup then include common_message_types and delete the below line
const uint8_t kTypeComplexMotorControl = 51;

class ComplexMotorControlClient: public ClientAbstract{
  public:
    ComplexMotorControlClient(uint8_t obj_idn):
      ClientAbstract(         kTypeComplexMotorControl, obj_idn),
      cmd_mode_(              kTypeComplexMotorControl, obj_idn, kSubCmdMode),
      cmd_phase_pwm_(         kTypeComplexMotorControl, obj_idn, kSubCmdPhasePwm),
      cmd_phase_volts_(       kTypeComplexMotorControl, obj_idn, kSubCmdPhaseVolts),
      cmd_spin_pwm_(          kTypeComplexMotorControl, obj_idn, kSubCmdSpinPwm),
      cmd_spin_volts_(        kTypeComplexMotorControl, obj_idn, kSubCmdSpinVolts),
      cmd_brake_(             kTypeComplexMotorControl, obj_idn, kSubCmdBrake),
      cmd_coast_(             kTypeComplexMotorControl, obj_idn, kSubCmdCoast),
      cmd_calibrate_(         kTypeComplexMotorControl, obj_idn, kSubCmdCalibrate),
      cmd_velocity_(          kTypeComplexMotorControl, obj_idn, kSubCmdVelocity),
      cmd_angle_(             kTypeComplexMotorControl, obj_idn, kSubCmdAngle),
      drive_pwm_(             kTypeComplexMotorControl, obj_idn, kSubDrivePwm),
      drive_volts_(           kTypeComplexMotorControl, obj_idn, kSubDriveVolts),
      mech_lead_angle_(       kTypeComplexMotorControl, obj_idn, kSubMechLeadAngle),
      obs_supply_volts_(      kTypeComplexMotorControl, obj_idn, kSubObsSupplyVolts),
      obs_supply_amps_(       kTypeComplexMotorControl, obj_idn, kSubObsSupplyAmps),
      obs_angle_(             kTypeComplexMotorControl, obj_idn, kSubObsAngle),
      obs_absolute_angle_(    kTypeComplexMotorControl, obj_idn, kSubObsAbsoluteAngle),
      obs_velocity_(          kTypeComplexMotorControl, obj_idn, kSubObsVelocity),
      motor_pole_pairs_(      kTypeComplexMotorControl, obj_idn, kSubMotorPolePairs),
      motor_emf_shape_(       kTypeComplexMotorControl, obj_idn, kSubMotorEmfShape),
      motor_kv_(              kTypeComplexMotorControl, obj_idn, kSubMotorKv),
      motor_r_ohm_(           kTypeComplexMotorControl, obj_idn, kSubMotorROhm),
      motor_i_max_(           kTypeComplexMotorControl, obj_idn, kSubMotorIMax),
      permute_wires_(         kTypeComplexMotorControl, obj_idn, kSubPermuteWires),
      calibration_angle_(     kTypeComplexMotorControl, obj_idn, kSubCalibrationAngle),
      lead_time_(             kTypeComplexMotorControl, obj_idn, kSubLeadTime),
      commutation_hz_(        kTypeComplexMotorControl, obj_idn, kSubCommutationHz),
      control_hz_(            kTypeComplexMotorControl, obj_idn, kSubControlHz),
      phase_angle_(           kTypeComplexMotorControl, obj_idn, kSubPhaseAngle),
      calibration_time_(      kTypeComplexMotorControl, obj_idn, kSubCalibrationTime),
      velocity_filter_fc_(    kTypeComplexMotorControl, obj_idn, kSubVelocityFilterFc),
      velocity_filter_value_( kTypeComplexMotorControl, obj_idn, kSubVelocityFilterValue),
      velocity_kp_(           kTypeComplexMotorControl, obj_idn, kSubVelocityKp),
      velocity_ki_(           kTypeComplexMotorControl, obj_idn, kSubVelocityKi),
      velocity_kd_(           kTypeComplexMotorControl, obj_idn, kSubVelocityKd),
      velocity_ff0_(          kTypeComplexMotorControl, obj_idn, kSubVelocityFF0),
      velocity_ff1_(          kTypeComplexMotorControl, obj_idn, kSubVelocityFF1),
      velocity_ff2_(          kTypeComplexMotorControl, obj_idn, kSubVelocityFF2),
      angle_kp_(              kTypeComplexMotorControl, obj_idn, kSubAngleKp),
      angle_ki_(              kTypeComplexMotorControl, obj_idn, kSubAngleKi),
      angle_kd_(              kTypeComplexMotorControl, obj_idn, kSubAngleKd),
      est_motor_amps_(        kTypeComplexMotorControl, obj_idn, kSubEstMotorAmps),
      est_motor_torque_(      kTypeComplexMotorControl, obj_idn, kSubEstMotorTorque),
      obs_motor_amps_(        kTypeComplexMotorControl, obj_idn, kSubObsMotorAmps),
      ctrl_spin_amps_(        kTypeComplexMotorControl, obj_idn, kSubCtrlSpinAmps),
      ctrl_spin_torque_(      kTypeComplexMotorControl, obj_idn, kSubCtrlSpinTorque),
      amps_kp_(               kTypeComplexMotorControl, obj_idn, kSubAmpsKp),
      amps_ki_(               kTypeComplexMotorControl, obj_idn, kSubAmpsKi),
      amps_kd_(               kTypeComplexMotorControl, obj_idn, kSubAmpsKd),
      volts_limit_(           kTypeComplexMotorControl, obj_idn, kSubVoltsLimit)
      {};

    // Client Entries
    ClientEntry<uint8_t>  cmd_mode_;
    ClientEntry<float>    cmd_phase_pwm_;
    ClientEntry<float>    cmd_phase_volts_;
    ClientEntry<float>    cmd_spin_pwm_;
    ClientEntry<float>    cmd_spin_volts_;
    ClientEntryVoid       cmd_brake_;
    ClientEntryVoid       cmd_coast_;
    ClientEntry<float>    cmd_calibrate_;
    ClientEntry<float>    cmd_velocity_;
    ClientEntry<float>    cmd_angle_;
    ClientEntry<float>    drive_pwm_;
    ClientEntry<float>    drive_volts_;
    ClientEntry<float>    mech_lead_angle_;
    ClientEntry<float>    obs_supply_volts_;
    ClientEntry<float>    obs_supply_amps_;
    ClientEntry<float>    obs_angle_;
    ClientEntry<float>    obs_absolute_angle_;
    ClientEntry<float>    obs_velocity_;
    ClientEntry<uint16_t> motor_pole_pairs_;
    ClientEntry<uint8_t>  motor_emf_shape_;
    ClientEntry<float>    motor_kv_;
    ClientEntry<float>    motor_r_ohm_;
    ClientEntry<float>    motor_i_max_;
    ClientEntry<uint8_t>  permute_wires_;
    ClientEntry<float>    calibration_angle_;
    ClientEntry<float>    lead_time_;
    ClientEntry<uint32_t> commutation_hz_;
    ClientEntry<uint32_t> control_hz_;
    ClientEntry<float>    phase_angle_;
    ClientEntry<float>    calibration_time_;
    ClientEntry<uint32_t> velocity_filter_fc_;
    ClientEntry<float>    velocity_filter_value_;
    ClientEntry<float>    velocity_kp_;
    ClientEntry<float>    velocity_ki_;
    ClientEntry<float>    velocity_kd_;
    ClientEntry<float>    velocity_ff0_;
    ClientEntry<float>    velocity_ff1_;
    ClientEntry<float>    velocity_ff2_;
    ClientEntry<float>    angle_kp_;
    ClientEntry<float>    angle_ki_;
    ClientEntry<float>    angle_kd_;
    ClientEntry<float>    est_motor_amps_;
    ClientEntry<float>    est_motor_torque_;
    ClientEntry<float>    obs_motor_amps_;
    ClientEntry<float>    ctrl_spin_amps_;
    ClientEntry<float>    ctrl_spin_torque_;
    ClientEntry<float>    amps_kp_;
    ClientEntry<float>    amps_ki_;
    ClientEntry<float>    amps_kd_;
    ClientEntry<float>    volts_limit_;

    void ReadMsg(CommunicationInterface& com,
      uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubVoltsLimit+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &cmd_mode_,             // 0
        &cmd_phase_pwm_,        // 1
        &cmd_phase_volts_,      // 2
        &cmd_spin_pwm_,         // 3
        &cmd_spin_volts_,       // 4
        &cmd_brake_,            // 5
        &cmd_coast_,            // 6
        &cmd_calibrate_,        // 7
        &cmd_velocity_,         // 8
        &cmd_angle_,            // 9
        &drive_pwm_,            // 10
        &drive_volts_,          // 11
        &mech_lead_angle_,      // 12
        &obs_supply_volts_,     // 13
        &obs_supply_amps_,      // 14
        &obs_angle_,            // 15
        &obs_absolute_angle_,   // 16
        &obs_velocity_,         // 17
        &motor_pole_pairs_,     // 18
        &motor_emf_shape_,      // 19
        &motor_kv_,             // 20
        &motor_r_ohm_,          // 21
        &motor_i_max_,          // 22
        &permute_wires_,        // 23
        &calibration_angle_,    // 24
        &lead_time_,            // 25
        &commutation_hz_,       // 26
        &control_hz_,           // 27
        &phase_angle_,          // 28
        &calibration_time_,     // 29
        &velocity_filter_fc_,   // 30
        &velocity_filter_value_,// 31
        &velocity_kp_,          // 32
        &velocity_ki_,          // 33
        &velocity_kd_,          // 34
        &velocity_ff0_,         // 35
        &velocity_ff1_,         // 36
        &velocity_ff2_,         // 37
        &angle_kp_,             // 38
        &angle_ki_,             // 39
        &angle_kd_,             // 40
        nullptr,                // 41
        &est_motor_amps_,       // 42
        &est_motor_torque_,     // 43
        &obs_motor_amps_,       // 44
        &ctrl_spin_amps_,       // 45
        &ctrl_spin_torque_,     // 46
        &amps_kp_,              // 47
        &amps_ki_,              // 48
        &amps_kd_,              // 49
        &volts_limit_           // 50
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubCmdMode             = 0;
    static const uint8_t kSubCmdPhasePwm         = 1;
    static const uint8_t kSubCmdPhaseVolts       = 2;
    static const uint8_t kSubCmdSpinPwm          = 3;
    static const uint8_t kSubCmdSpinVolts        = 4;
    static const uint8_t kSubCmdBrake            = 5;
    static const uint8_t kSubCmdCoast            = 6;
    static const uint8_t kSubCmdCalibrate        = 7;
    static const uint8_t kSubCmdVelocity         = 8;
    static const uint8_t kSubCmdAngle            = 9;
    static const uint8_t kSubDrivePwm            = 10;
    static const uint8_t kSubDriveVolts          = 11;
    static const uint8_t kSubMechLeadAngle       = 12;
    static const uint8_t kSubObsSupplyVolts      = 13;
    static const uint8_t kSubObsSupplyAmps       = 14;
    static const uint8_t kSubObsAngle            = 15;
    static const uint8_t kSubObsAbsoluteAngle    = 16;
    static const uint8_t kSubObsVelocity         = 17;
    static const uint8_t kSubMotorPolePairs      = 18;
    static const uint8_t kSubMotorEmfShape       = 19;
    static const uint8_t kSubMotorKv             = 20;
    static const uint8_t kSubMotorROhm           = 21;
    static const uint8_t kSubMotorIMax           = 22;
    static const uint8_t kSubPermuteWires        = 23;
    static const uint8_t kSubCalibrationAngle    = 24;
    static const uint8_t kSubLeadTime            = 25;
    static const uint8_t kSubCommutationHz       = 26;
    static const uint8_t kSubControlHz           = 27;
    static const uint8_t kSubPhaseAngle          = 28;
    static const uint8_t kSubCalibrationTime     = 29;
    static const uint8_t kSubVelocityFilterFc    = 30;
    static const uint8_t kSubVelocityFilterValue = 31;
    static const uint8_t kSubVelocityKp          = 32;
    static const uint8_t kSubVelocityKi          = 33;
    static const uint8_t kSubVelocityKd          = 34;
    static const uint8_t kSubVelocityFF0         = 35;
    static const uint8_t kSubVelocityFF1         = 36;
    static const uint8_t kSubVelocityFF2         = 37;
    static const uint8_t kSubAngleKp             = 38;
    static const uint8_t kSubAngleKi             = 39;
    static const uint8_t kSubAngleKd             = 40;
    // State deprecated                   = 41
    static const uint8_t kSubEstMotorAmps        = 42;
    static const uint8_t kSubEstMotorTorque      = 43;
    static const uint8_t kSubObsMotorAmps        = 44;
    static const uint8_t kSubCtrlSpinAmps        = 45;
    static const uint8_t kSubCtrlSpinTorque      = 46;
    static const uint8_t kSubAmpsKp              = 47;
    static const uint8_t kSubAmpsKi              = 48;
    static const uint8_t kSubAmpsKd              = 49;
    static const uint8_t kSubVoltsLimit          = 50;
};

#endif // COMPLEX_MOTOR_CONTROL_CLIENT_H
