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
---------
sess03
---------
Motivation:
 enabled yaw control from the beginning of control_position function. 
Result: 
 This seems to work somehow, meaning that now the steering output has something different than zero but it now displays a bang bang behaviour. Apparently the correct behaviour observed yesterday and in session 2 is due to the fact that the app behaves like the car is taking off so it enables the rudder steering to keep the plane straight. This should also explain why the magnitude of the yaw output is not very big and very noisy.
 The trottle in manual mode is 1000 because the disarm switch wasaccidentaly triggered. This test interrupted quickly for unknown reasons, the LED was blinking red and it disarmed. The question is: was it because of some issue or because my disarming code worked? I need to check the log data and mavlink console to really see ths.
---------
sess04
---------
Motivation:
 Unexpected end of session 3 so I decided to repeat the same test. No changes in the code. 
Result: 
 I forgot to choose mission mode at the first run, I had to do a second run and the steering channel behaves as explained above. I need to inspect this or change the whole thing altogether. 

********************************************
Day: 13-03-2017
---------
sess01-2-3-4
---------
Motivation:
 Testing if setting the circuit breaker for the gps, as advised by James, works and runs the mission.   
Result:
 In session 2 it is possible to see that it controls the actuators.
---------