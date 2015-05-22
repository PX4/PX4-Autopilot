uorb start
simulator start -s
barosim start
adcsim start
accelsim start
gyrosim start
param set CAL_GYRO0_ID 2293760
param set CAL_ACC0_ID 1310720
param set CAL_ACC1_ID 1376256
param set CAL_MAG0_ID 196608
rgbled start
mavlink start -d /tmp/ttyS0
sensors start
hil mode_pwm
commander start
list_devices
