# [REPORT]Multi-Threaded Race Condition bug found in PX4 src/modules/navigator/navigator_main.cpp which cause drone can not PAUSE

### Background

PX4 (6.9k stars) https://github.com/PX4/PX4-Autopilot

PX4 is a professional autopilot system developed by world-class developers from both the industry and academia. Supported by an active global community, it provides power for a variety of vehicles, including racing and cargo drones, ground vehicles, and submarines. Due to code reusability, this vulnerability should be applicable not only to multirotor drones but also to fixed-wing drones, submarines, ground vehicles, and more.

The open-source project's code is directly utilized by numerous OEM drone manufacturers worldwide, including Freefly Systems, Quantum Systems, Skydio, Auterion, and others.

### Summary
Due to the lack of synchronization mechanism for vehicle_status variable,we identified a Race Condition  vulnerability in the navigator_main.cpp and commander.cpp.This will result in the drone not being able to PAUSE when it executes a mission.

### Details
When the drone is executing a mission and the user clicks PAUSE, the ground control station will send **command::VEHICLE_CMD_DO_REPOSITION** to the drone.
1. In commander.cpp, the drone state is modified to **NAVIGATION_STATE_AUTO_LOITER** based on the received commands and the state is published.
As stated in the code comment, it is designed not to require the navigator and command to receive/process data at the **exact same** time.
https://github.com/Drone-Lab/PX4-Autopilot/blob/24cee812795c669318d3e56e5df9261124e319a7/src/modules/commander/Commander.cpp#L726-L731




2. In navigator_main.cpp, it will subscribe to the drone status and execute **status->run()**
https://github.com/Drone-Lab/PX4-Autopilot/blob/24cee812795c669318d3e56e5df9261124e319a7/src/modules/navigator/navigator_main.cpp#L881

3. Some of the parameters needed in **status->run()** are also calculated in navigator_main.cpp, but the conditions that trigger the execution of these calculated functions are not based on the drone status but on the received commander.Here calculate the break point to pause the mission.
https://github.com/Drone-Lab/PX4-Autopilot/blob/24cee812795c669318d3e56e5df9261124e319a7/src/modules/navigator/navigator_main.cpp#L351
Multi-Threaded Race Condition occurs here.When navigator_main.cpp receives the command from the GCS and calculates the braking point, commander.cpp hasn't yet modified the UAV's status according to the received command, so **status->run()**,doesn't publish the braking point as expected

   By the time commander.cpp finishes modifying the drone's status, navigator_main.cpp has executed to the next round and overwritten the brake point it just calculated, causing **status->run()** (status==loiter) to PUBLISH the wrong data

4. Here overwrite the brake point.Because the Pause command is a compound command.
Pause command==reposition to break point+change altitude
I analyzed the QGC code, and when the user presses the PAUSE button, QGC sends twice the REPOSITION CMD to the drone.The second REPOSITION CMD triggers here and cause overwrite.
https://github.com/Drone-Lab/PX4-Autopilot/blob/f38fe24a98e05e5ecada347c32e108a12afb03aa/src/modules/navigator/navigator_main.cpp#L331-L334

### Verification
I added some debug output to watch the drone's state change.

```
// Navigator.cpp

WHILE true DO
    IF vehicle_command_s EQUALS VEHICLE_CMD_DO_reposition THEN
        PX4_INFO("VEHICLE_CMD_DO_REPOSITION")
        IF user_intent EQUALS pause THEN
            PX4_INFO("calculate_breaking_stop point")
            calculate_breaking_stop(rep.current.lat, rep.current.lon, rep.current.yaw)
        END IF
    END IF

    PX4_INFO("Now vehicle_status=missison/loiter") //output drone status
    vehicle_status_s.Run() // If status EQUALS loiter, publish the break point
END WHILE


```
When the drone is executing a mission and the user clicks PAUSE,the console will output the following content:

```
...
INFO  [navigator] Now  vehicle_status=missison    //The drone is on a mission.
INFO  [navigator] Now  vehicle_status=missison
INFO  [navigator] Now  vehicle_status=missison
INFO  [navigator] VEHICLE_CMD_DO_REPOSITION     //Navigator_main.cpp accept CMD from GCS
INFO  [navigator] calculate_breaking_stop point             
INFO  [navigator] Now  vehicle_status=missison   //!!!!!!Beacuse of Race Condition,Drone is still in the previous status
INFO  [navigator] VEHICLE_CMD_DO_REPOSITION   //Pause command==reposition to break point+change altitude,so here's the VEHICLE_CMD_DO_REPOSITION twice
INFO  [navigator] Now vehicle_status=loiter  //Commander.cpp has modified status according to CMD
INFO  [navigator] Now vehicle_status=loiter
INFO  [navigator] Now vehicle_status=loiter
...
```
### Temporary Patch


```
vehicle_command_s cmd{};
_vehicle_command_sub.copy(&cmd);
if(cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION){
_navigation_mode=&_loiter;
}
```
Add code before this line,assure the status is consistent with CMD before executing status->run()
https://github.com/Drone-Lab/PX4-Autopilot/blob/24cee812795c669318d3e56e5df9261124e319a7/src/modules/navigator/navigator_main.cpp#L881


### Impact
- Due to multiple threads accessing the 'vehicle_status' variable, one thread may experience simultaneous modifications by another thread, leading to a data race bug.
- Drones are unable to PAUSE during missions, which can lead to collision crashes in the event of a hazardous encounter or incorrect mission settings.#22492

https://user-images.githubusercontent.com/151698793/291232214-40734dab-363a-43ec-aa02-6f8bf177f3b4.mp4

- After pause,the drone still keep move, but also in hold mode.So the user can not manually enter the hold to stop the aircraft, user need to switch to other modes then manually hold
- An attacker could use the vulnerability to keep the drone from executing certain commands



