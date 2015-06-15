uorb start
param load
mavlink start -u 14556
simulator start -s
param set CAL_GYRO0_ID 2293760
param set CAL_ACC0_ID 1310720
param set CAL_ACC1_ID 1376256
param set CAL_MAG0_ID 196608
rgbled start
tone_alarm start
gyrosim start
accelsim start
barosim start
adcsim start
gpssim start
commander start
sensors start
ekf_att_pos_estimator start
mc_pos_control start
mc_att_control start
hil mode_pwm
dataman start
navigator start
param set MAV_TYPE 2
param set RC1_MAX 2015
param set RC1_MIN 996
param set RC1_TRIM 1502
param set RC1_REV -1
param set RC2_MAX 2016 
param set RC2_MIN 995
param set RC2_TRIM 1500
param set RC3_MAX 2003
param set RC3_MIN 992
param set RC3_TRIM 992
param set RC4_MAX 2011
param set RC4_MIN 997
param set RC4_TRIM 1504
param set RC4_REV -1
param set RC6_MAX 2016
param set RC6_MIN 992
param set RC6_TRIM 1504
param set RC_CHAN_CNT 8
param set RC_MAP_MODE_SW 5
param set RC_MAP_POSCTL_SW 7
param set RC_MAP_RETURN_SW 8
param set MC_PITCHRATE_P 0.05
param set MC_ROLLRATE_P 0.05
mixer load /dev/pwm_output0 ../../ROMFS/px4fmu_common/mixers/quad_x.main.mix
