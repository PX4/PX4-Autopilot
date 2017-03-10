********************************************
Day: 10-03-2017
---------
sess01
---------
Motivation:
 testing if the gnd pos controller behaves correctly commanding the actuators as tested yesterday in Log 23. Also increased the mixer for the steering from 10000 to 12000 to see whether the yaw/steering has a more amplified pwm out. 
Result:
 The new app does not control yaw as before, now it's zero.
---------
sess02
---------
Motivation:
 reverted back to fw_pos and att control apps, to check if that controls the yaw.
Result: it works, my changes were too drastic.