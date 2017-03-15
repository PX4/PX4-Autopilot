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
All the other sessions
---------
Testing the local position estimator lock going around the lab and investigating the sonar fault.

********************************************
Day: 14-03-2017
---------
sess01
---------
Motivation:
 The new ESC arrived and today I will test outside collecting a log of the behaviour.
 This thest was to check that the pwm commands where working.
Result:
 PWM 1 needs to be reversed in QGC, set limits on pwm, it works.
---------
sess02
---------
Motivation:
 Testing if the newly calibrated controller paired with the new mixer reacts to roll.
Result:
 It doesn't, but steering is very noisy. Probably because not in any controlled mode, need to test mission. Enough to collect the data for manual control.
---------
sess03
---------
Motivation:
 Getting the feeling of the new controller
Result:
 ok, quick and easy
---------
sess44
---------
Motivation:
 logging dynamics of the system while driving manually to better tune the controller and keep the data for offline analysis and comparison
Result:
 I did 10 sections in this log, they can be distinguished in the acquisition by 3 turns of the rc car around teh roll axis, usually around 90 degrees. The test done were the following
 1) step response to full throttle forward 3 times
 2) step response to full throttle backwards 1 time as it was very unstable
 3) driving straight at a slow speed and applying a step on the steering to the right
 4) driving straight at a slow speed and applying a step on the steering to the left
 5) driving straight at a medium speed and applying a step on the steering to the right
 6) driving straight at a medium speed and applying a step on the steering to the left
 7) driving straight at a fast speed and applying a step on the steering to the right
 8) driving straight at a fast speed and applying a step on the steering to the left
 9) fast driving in circle at increasing speed, in both right and left directions
 10) breaking down hard from a high velocity
 All these tests were carried out with a pwm limit of 1800 upper and 1200 lower and pwm disarmed of 1500. In addition, the ESC is set in training mode which limits the power to 50%. During these tests I checked repeatedly the temperature all the power cables and the esc heat sink: they were always cold to the touch.
 ---------
sess05
---------
Motivation:
 Testing the mission
Result:
 As soon as the mission starts the wheels start to spin quite fast. I decided to lower the pwm limit even more to 1600. In this session i mistakenly set the PWM_AUX channel instead of the main one.
 ---------
sess06-07-08
---------
Motivation:
 Setting 1600 as limit on the upper pwm
Result:
 This generates a wierd offset on the wheels so that at a neutral position of the remote throttle lever the wheels start spinning in reverse. I figured out it is due to an imbalance in the pwm limits.
 ---------
sess09
---------
Motivation:
 Testing simmetric PWM limits.
Result:
 It now works, at 0 the wheels don't spin.
 ---------
sess10
---------
Motivation:
 Tuning control gains
Result:
 increased itegrator (mistakenly), increased the roll proportional to 0.2, reduced cruising throttle, reduced max throttle
 ---------
sess11
---------
Motivation:
 Testing the mission 
Result:
 They are better but no steering though, I realized that my roll controller was too low and was not commanding the steering. Not sure about gps plot as the pwm was reversed.
---------
sess12
---------
Motivation:
 Testing the mission. Reversed the pwm on the throttle and added 0.2 to the roll feed forward
Result:
 I realized that wheels were spinning in the wrong direction while moving forward. I decided to change the PWM_MAIN_REV4 parameter to 1. This means that the throttle lever on the remote is now inverted but it doesn't hurt as long as the pilot knows it.  I also added 0.2 in the roll feedforward which seems to behave 
 ---------
sess13
---------
Motivation:
 Testing the missio again
Result:
 There must have been an issue, the LED turned flashing red for some reason and the logging stopped.
 ---------
sess14
---------
BAD
---------
sess15
---------
Motivation:
 Testing the mission
Result:
 By looking at the car I saw steering but the log did not work. weird
***************************************************
Day: 15-03-2017

---------
sess01
---------
Motivation:
 First mission test
Result:
 it moves but it steers quite slowly. Increasing the gain to 0.5
---------
sess02
---------
Motivation:
 testing different parameters as inverting the pwm of the steering. This did not improve the quality.
Result:
 x
---------
sess03
---------
Motivation:
 Increasing the pwm limit to allow for more steering.
Result:
 This worked MUCH better! It was actually able to finish the mission and it entered loitering mode but it does not disarm as it was doing in the simulator. It does not steer enough probably, so I'll try to increase the roll gain.
---------
sess04
---------
Motivation:
 Changin airspeed minimum, maximum and trim so to be able to cruise at a decent velocity and allowing the full range of the pwm. 1700 is too high for going full power but hopefully the airspeed trim works. leaving pwm to 1700 for now. also, removing the disarming commando from mission_block.cpp as it does not work and the only effect it has is to kill the logging. Roll gains kept to 0.5
Result:
 The mission is executed but there are 2 issues: acc radius too small and still too fast, trimming down airspeed again.
---------
sess05
---------
Motivation:
 Reducing airspeed trim and reducing acc radius
Result:
still fast but it looks a bit slower, the mission looked better, check daq
---------
sess06
---------
Motivation:
 reducing airspd trim again to 0.2
Result:

---------
sess07
---------
Motivation:
 I could not connect to the px4, tried arming hoping something would happen
Result:
 not working.
++++++++++++++++++++++++++ DELETING ALL LOGS FROM SD ++++++++++++++++++++++++++++++++++++

---------
sess08
---------
Motivation:
 Lorenz suggested setting the throttle trim to 0
Result:
 this was arming to try to get the lock, disregard this session.


---------
sess09
---------
Motivation:
 testing new params
Result:
 looks good, disarming problem, recompiled fw. From log analysis it was found that in all the afternoon sessions (from 8 to 14),   the NAV_DLL_ACT parameter should not be set to 2, but let disabled. This test is overall quite good, the proportional gain of 0.5 and no FF term looks decent. Unfortunately the disarming problem and the parameter issue perturbed the mission.
Msg file: msgs_2017_03_15_13_36_00.txt
Log file: http://logs.uaventure.com/view/TZj2Ym9kvyU6jSe8v8v9Tg#Actuators_PLOT
  

---------
sess10-11
---------
Motivation:
 Increasing pwm limit and roll control
Result:
 two logs, one wasn't getting lock. Looks good but it was kinda entering failsafe mode and loitering. steering was not agressive enough, putting the FF term again. Also increasing the pwm. It looks like the mixer worked. It seems promising except for the issues with the parameter and the 
MSG file: msgs_2017_03_15_13_44_54.txt 
MSG file 2: msgs_2017_03_15_13_54_05.txt
Log file: http://logs.uaventure.com/view/SemB6BwzbBFDf3oE4J5PqE


---------
sess12
---------
Motivation:
 Added back the feedforward and increased the pwm limits
Result:
 The actuators outputs for the steering are saturated all the time, I'm guessing too much amplification. Same problems with wrong parameter causing circles all the time.
MSG File: msgs_2017_03_15_14_02_26.txt
Log file: http://logs.uaventure.com/view/CgSHdcBg3gXQvbGHdX9NaF#Actuators_PLOT

---------
sess13
---------
Motivation:
 Increasing PWM limits to the maximum to see if it still behaves
Result:
 yes it is ok, still same issues as before
MSG file: msgs_2017_03_15_14_08_14.txt
Log file: http://logs.uaventure.com/view/uQsa9SGBc6ejbDFNzmtF39

---------
sess14
---------
Motivation:
 Tested a second time as before but this time I was running along the car to see why it was not doing the mission. I realized the LED was turning purple and flashing while in mission mode.
Result:
 same as before

---------
sess15
---------
Motivation:
 Increased cruising speed, removed Feed forward term, set roll proportional gain to 1 just to log the data.
Result:
 As noted in previous sessions the Proportional gain is now too big. The throotle seems increased a bit. The parameter for the cruising speed is the one responsible now.

---------
sess
---------
Motivation:
 x
Result:
 x