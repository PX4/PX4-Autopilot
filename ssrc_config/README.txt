This folder contains extra parameter sets for different flying environments.
The config_<env>.txt for each environment needs to be copied into sdcard as /etc/config.txt

config_outdoor.txt
- Empty configuration

config_indoor.txt
- Contains settings for maximum speeds (very slow), altitudes (very low), disabling GPS, magnetometer etc.

config_hitl.txt
- Contains settings for configure px4 into hardware-in-the-loop mode and mavlink setup for UART gazebo connection

config_hitl_eth.txt
- Contains same settings as config_hitl.txt, but the mavlink for gazebo connection is using ethernet
