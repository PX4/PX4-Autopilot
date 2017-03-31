***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
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

***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************

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

***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
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

*************************************************************************************************************************************************************************
**************************************************************************************************************************************************************************************************************************************************************************************************************************************************
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
MSG file: msgs_2017_03_15_14_15_34.txt
Log file: http://logs.uaventure.com/view/JyU55jLLaEqCLyeh3YChEE

---------
sess15
---------
Motivation:
 Increased cruising speed, removed Feed forward term, set roll proportional gain to 1 just to log the data.
Result:
 As noted in previous sessions the Proportional gain is now too big. The throotle seems increased a bit. The parameter for the cruising speed is the one responsible now.
MSG file: msgs_2017_03_15_14_35_25.txt
Log file: http://logs.uaventure.com/view/FRocYZ79vHwsi6QJnWWT83

***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************

Day: 16-03-2017
NOTE: battery was not completely charged, which shows some voltage drops. In the last logs this effect was noticeable
---------
sess01
---------
Motivation:
 Testing mission withouth NAV_DLL_ACT and with P gain at 0.5. No disarming code.
Result:
 Very fast but seems ok. The speed is now increased without changing anything from yesterday. That is quite weird. From the logs it looks like the pwm sent out is 1700 while on the last log of yesterday it clearly reads 1600. Mission finished i 3 seconds (?)
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/tTtqMWxcfUmceBPZsbqxPo

---------
sess02
---------
Moivation:
 test lower acc radius and lower cruising throttle. at half log increased P to 0.7.
Result:
The speed did not lower which is even more suspicious. Nevertheless this log is useful to see that even though the roll proportional gain changed from 0.5 to 0.7 the results are pretty much similar. in fact from the map in the log file is possible to see how the first run (with 0.5, the farthest one from the starting location) has the same turning radius as the second one. the car was going towards a waypoint of an unfinished mission so it allowed to record this good comparison. Mission finished i 3 seconds (?)
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/eJZmDdaDB6uofqynzqyfQA

---------
sess03
---------
Motivation:
 Testing higher P and lower speed, changes made before disarming in the previous log. P = 0.7 
Result:
 Best result so far but still going fast for some reason. Mission finished i 3 seconds (?). It could be that it was loitering but this does not seem likely as the shape of the gps curve is quite similar to the path, even to the last moment
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/EpmaLDk2ZJrv3xP84NwvXd

---------
sess04
---------
Motivation:
 changed mission, speed at 3%. The mission was made to be a more squared because the car was passing quite close to the soccer goal.
Result:
 Crashed. changing mission again. It was loitering after the crash.
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/oR6MY5bPyigZxgbLkX6zq4

---------
sess05
---------
Motivation:
 Changing mission and reducing max throttle to 5%
Result:
 The speed definitely did not reduce. Interrupted because too close to the soccer door. Mission took 5 seconds(?)
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/XwsFqTguLNEUthunoBnSD7

---------
sess06
---------
Motivation:
 Reduced airspeed trim to 2% and airspeed max to 4% to see if it slows down.
Result:
 Does not slow down. Had to stop mission to prevent a crash. Looks good though. This looks plausible as mission times are in the range of 15 seconds.
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/MdskidvXE9rt5x5oqVBzSM

---------
sess07
---------
Motivation:
 testing again with reduced airspeed
Result:
 Looks ok even with the midd interruption.
MSG file: msgs_2017_03_16_07_35_57.txt
Log file: http://logs.uaventure.com/view/Lz3voycrX57E4GVnWQfp9f#AS_Roll_PLOT

---------
sess08
---------
Motivation:
 Given the amount of noise in the steering I decided to try a lower gain. P = 0.3
Result:
  It does not steer properly anymore. The msg log doesn't say executing mission which is quite weird. It looks like the setpoints for the angles are not generated. Not sure what happened here. Cannot rule out a low parameter like this.
 MSG file: msgs_2017_03_16_07_59_30.txt
 Log file: http://logs.uaventure.com/view/pr5QAQSKQsmqsKjREGLLm3

---------
sess09
---------
Motivation:
 Restored airspeed as the position control app was crashing and the speed was not reducing anyway. reflashed the fw and it started working again. Also set gain higher to 1.
Result:
 From the logs the angles are still not being generated. why? mission started but unfinished. Steering quite saturated. Power shutdown when I attempted to restart the mission
MSG file: msgs_2017_03_16_08_08_09.txt
Log file: http://logs.uaventure.com/view/FiGZNkHVQWWoxpTKEjGDHd

---------
sess10
---------
Motivation:
 Just driving the car back after shutdown.
Result:
 This is a good example of how the steering should look like.
MSG file: msgs_2017_03_16_08_09_22.txt
Log file: http://logs.uaventure.com/view/KUHEPtRtHMBrWUHCVTsHZE

 ---------
sess11
---------
Motivation:
 Flashing light purple, it was an in-air restore.
Result:
 Short log, not useful
MSG file: msgs_2017_03_16_08_10_13.txt
Log file: http://logs.uaventure.com/view/V5rHD96jNyxNzYLxKwQGWY

---------
sess12
---------
Motivation:
 Added integrator after trying gain 1 (this could be inspected further if roll is being produced) and P is set to .6. 
Result:
 Shutdown for power reason
MSG file: msgs_2017_03_16_08_11_10.txt
Log file: http://logs.uaventure.com/view/Wq4AtuLmhDc6ZytsRKsWYe

---------
sess13
---------
Motivation:
 Removing integrator P to 0.6. different times the whole thing is shutting down when there are power surges.
Result:
 It shut down as soon as I triggered the mission because it required too much power
MSG file: msgs_2017_03_16_08_13_51.txt
Log file: http://logs.uaventure.com/view/mDoTMQgTL2PTxtmEez96vB#Actuators_PLOT

---------
sess14
---------
Motivation:
 trying to test mission
Result:
 it died again
MSG file: msgs_2017_03_16_08_14_46.txt
Log file: http://logs.uaventure.com/view/eU3RPmJT84LXL2kac8YqnB

---------
sess15
---------
Motivation:
 Trying to test mission
Result:
 Still not generated roll setpoint. It died wen I started again. Somehow the SYS_RESTART_TYPE changed by itself
MSG file: msgs_2017_03_16_08_14_46.txt
Log file: http://logs.uaventure.com/view/hg8GgNFb4qGxZsjSKy9krk

---------
sess16
---------
Motivation:
 Driving back
Result:
 Dummy log
MSG file: msgs_2017_03_16_08_15_49.txt
Log file: http://logs.uaventure.com/view/M5J4qu5ter963f5edEoyRg

---------
sess17
---------
Motivation:
 Removed the integrator to see if that was the one responsible.
Result:
 The setpoints are still not being generated so obviously not steering
MSG file: msgs_2017_03_16_08_15_49.txt
Log file: http://logs.uaventure.com/view/Rg2vHMdHNx56Sexap3mj9S

---------
sess18
---------
Motivation:
 Trying to run.
Result:
 here it died again, still not generating sp
MSG file: msgs_2017_03_16_08_15_49.txt
Log file: http://logs.uaventure.com/view/nReKoJXP2K73Nwke6dsArh

---------
sess19
---------
Motivation:
 empty log
Result:
 empty log
 MSG file: msgs_2017_03_16_08_19_27.txt
 Log file: http://logs.uaventure.com/view/KyiMtPdMEKqEQuDmePrDMB

 ---------
sess20
---------
Motivation:
 Driving back to go back to the lab
Result:
 no result
 MSG file: msgs_2017_03_16_08_19_50.txt
 Log file: http://logs.uaventure.com/view/DXZSzLYt6bve6KCqNaeTFg

---------
sess21
---------
Motivation:
 Testing the new mixer with yaw and the response to a change of heading.
Result:
 The new mixer is ok, but cannot see the controller without gps lock and mission mode
 MSG file: x
 Log file: http://logs.uaventure.com/view/zFb6Pq5EAMLDe4ftYYVDtH


---------
sess22
---------
Motivation:
 Went on polyterasse to see wether the new two lines of code were doing something. For some reason the EKF2 parameters changed, I did not realize until 17-03-2017
Result:
 Arming to help to get the lock - not useful
MSG file: x
Log file: http://logs.uaventure.com/view/9WXu9Mhu9DZv2rSvDB8voj

---------
sess23
---------
Motivation:
 as 22
Result:
 as above - not useful
MSG file: x
Log file: http://logs.uaventure.com/view/PyWayE7Az6t7judiLE2JbK

---------
sess24
---------
Motivation:
 as 22
Result:
 The car took off, while trying to recover it crashed and rolled 3 times. Nevertheless the new output does something but it just slams to the negative side. It is not safe to test the car on a hard surface if the speed is not under control.
MSG file: x
Log file: http://logs.uaventure.com/view/aN5r5GmYTxC3567zsHgbM5

---------
sess25
---------
Motivation:
 Trying to test the other channel so to see the behaviour with the other pwm steering. Limited pwm to 1600 and 1400. 
Result:
 The car still ran off like crazy and the steering went to the full side as well. Cannot explain why.
MSG file: x
Log file: http://logs.uaventure.com/view/rsoyhWmJ5iJjSiETLFZ4rQ

---------
sess26
---------
Motivation:
 Testing new mission execution with the old steering channel
Result:
 Still goes off like crazy
MSG file: x
Log file: http://logs.uaventure.com/view/7tC4LuZg48VazLEtv37evM

---------
sess27
---------
Motivation:
 As before
Result:
 Not worth testing in these conditions.
MSG file: x
Log file: http://logs.uaventure.com/view/rKr7zhyw46sDpUjBXjpzF8

***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************

Day: 31-03-2017
---------
sess01
---------
Testing mission wheelspeed

---------
sess02
---------
Crap

---------
sess03
---------
Testing mission just going straight. 

---------
sess04
---------
This was a mission with 3 waypoints, the last one to come back. For some reason it stopped during the mission: maybe app crashed?

---------
sess05
---------
Driving back and triggering mission, ignore.

---------
sess06
---------
Very short mission. proabably testing some minor detail.

---------
sess07-08-09
---------
Mission executed correctly, it works! Last one executed for the video.



---------
sess
---------
Motivation:
 x
Result:
 x
MSG file: x
Log file: x