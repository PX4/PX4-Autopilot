This page contains logs of RRV and ATV experiments. Those experiments were design to evaluate if their softaware are equivalent (since they have different hardware and operating systems!). Each flight log page should point to the respective software version under experimentation and respective output data files (either here in Github or external servers!).

* [April 25, 2019](#April-25-2019)
* [April 24, 2019](#April-24-2019)
* [April 23, 2019](#April-23-2019)
* [April 11, 2019](#April-11-2019)

# April 25 2019

## Summary

This experiment was performed with a Pixhawk board and the Ironbird. The Pixhawk and the Ironbird's INS were rigidly rotated together to evaluate differences between the controller in the ATV and the RRV. New code has been added to implement anti-windups, maximum setpoint rates, and some additional minor details. The controllers (both ATV and RRV) had their proportional gain multiplied by 5 so to purposely generate saturation (we want to test the correctness of the anti-windups in ATV code with respect to RRV).  Additionally, greater care was taken with regard to the different ATV and RRV timestamps. Results could be seen in real-time in QGroundControl. This experiment is a follow-up to the [April 24, 2019](#April-24-2019) experiment.

## Data collected

![](https://github.com/lustosa-leandro/lustosa-leandro.github.io/blob/master/img/april25-2019.png)

## Comments

In comparison to yesterday's experiment, we can see that this version of the code yields a matching ATV and RRV actuators response (even in case of actuator saturation!). Note that pitot tubes were not connected, and the RRV control law takes the pitot tube reading into account in the controller, if one is present. In this regard, the control law has not yet been completely validated, and we should be careful when implementing the airspeed contribution in the control law.

## Project version

* Athena code: [commit c8f3c8afaf8dce2ec9d128518e853eb690947c45](https://github.com/UM-A2SRL/xhale-avionics/commit/c8f3c8afaf8dce2ec9d128518e853eb690947c45)
* Pixhawk code: [commit 5141a126c16a622e820aa2dae396eecc6ad87ae9](https://github.com/UM-A2SRL/xhale-pixhawk/commit/5141a126c16a622e820aa2dae396eecc6ad87ae9) 
* QGroundStation: v3.2.6
* Post processing scripts: [commit 9af7c3277a1f81aa8104c88d224cc72b14a129c8](https://github.com/UM-A2SRL/xhale-postprocessing/commit/9af7c3277a1f81aa8104c88d224cc72b14a129c8)

# April 24 2019

## Summary

This experiment was performed with a Pixhawk board and the Ironbird. The Pixhawk and the Ironbird's INS were rigidly rotated together to evaluate differences between the controller in the ATV and the RRV. Results could be seen in real-time in QGroundControl. This experiment is a follow-up to the [April 23, 2019](#April-23-2019) experiment.

## Data collected

![](https://github.com/lustosa-leandro/lustosa-leandro.github.io/blob/master/img/april24-2019.png)

## Comments

In comparison to yesterday's experiment, we can now see great improvement in matching ATV and RRV actuators response. The code that was given to me was never tested and so it was expected erroneous behavior. Now that we have the development support tools to properly assess the correctness of the code, I was able to identify bugs in the control law code that was handled to me, and fixed all of them. There are still some mismatches between curves, and those might come from some differences that I've found between RRV and ATV code, including:

* in ATV code anti-windup techniques are not implemented whereas in RRV they are;
* in ATV code maximum integral limits are not implemented, whereas in RRV they are.

Additionally, I've found the shape of the offline curves very similar to the shapes I saw online on the ground station. This makes me wonder if the ground station overwrote the timestamps from the vehicles with its own ground time (msg received time). Next time I really need to make sure the timestamps are correct.

I will implement all integrators details and make sure timestamps are correct, and will re-do this experiment soon.

## Project version

* Athena code: [commit 2387f0a136cde89d3e97df5ec868c56b3908f988](https://github.com/UM-A2SRL/xhale-avionics/commit/2387f0a136cde89d3e97df5ec868c56b3908f988)
* Pixhawk code: [commit 5141a126c16a622e820aa2dae396eecc6ad87ae9](https://github.com/UM-A2SRL/xhale-pixhawk/commit/5141a126c16a622e820aa2dae396eecc6ad87ae9) 
* QGroundStation: v3.2.6
* Post processing scripts: [commit a4a6c789bfdb505bc6692a7f1db8eb27a21a47e2](https://github.com/UM-A2SRL/xhale-postprocessing/commit/a4a6c789bfdb505bc6692a7f1db8eb27a21a47e2)

# April 23 2019

## Summary

This experiment was performed with a Pixhawk board and the Ironbird. The Pixhawk and the Ironbird's INS were rigidly rotated together to evaluate differences between the controller in the ATV and the RRV. Results could be seen in real-time in QGroundControl. This experiment is a follow-up to the [April 11, 2019](#April-11-2019) experiment.

## Data collected

![](https://github.com/lustosa-leandro/lustosa-leandro.github.io/blob/master/img/april23-2019.png)

## Comments

Elevons are not influenced by the control law since we turned off pitch control. In the other channels, we can see great discrepancy. In the ATV, the control law seems extremely agressive and quickly saturating the actuators, and not much can be inferred from this experiment, except that we need to debug the ATV control laws -- they don't seem to be correct yet.

## Project version

* Athena code: [commit e74fd1a8ed4314e916b670fa9ddce5eb3b24af24](https://github.com/UM-A2SRL/xhale-avionics/commit/e74fd1a8ed4314e916b670fa9ddce5eb3b24af24)
* Pixhawk code: [commit 5141a126c16a622e820aa2dae396eecc6ad87ae9](https://github.com/UM-A2SRL/xhale-pixhawk/commit/5141a126c16a622e820aa2dae396eecc6ad87ae9) 
* QGroundStation: v3.2.6
* Post processing scripts: [commit fa6ad41f8e1c57c102b78808ab6043671477f7fa](https://github.com/UM-A2SRL/xhale-postprocessing/commit/fa6ad41f8e1c57c102b78808ab6043671477f7fa)

# April 11 2019

## Summary

This experiment was performed with a Pixhawk board and the Ironbird. The Pixhawk and the Ironbird's INS were rigidly rotated together to evaluate differences between the estimator in the ATV and the RRV. Results could be seen in real-time in QGroundControl. The following figure illustrates the procedure.

![](https://github.com/lustosa-leandro/lustosa-leandro.github.io/blob/master/img/ExpBothObservers.png)

## Data collected

![](https://github.com/lustosa-leandro/lustosa-leandro.github.io/blob/master/img/april11-2019.png)

## Comments

There was noticeable lag between all estimates in real-time due to UDP packet delivery delay (as it can be noticed by the above photo). If one plots the data offline however, the lag does not exist as the above plot demonstrates. Noticeable yaw estimation difference is apparent, and must come from magnetometer calibration. We need to keep in mind that one (or both) of the platforms has substantial yaw error with respect to north.

## Project version

* Athena code: [commit e2a70f6e50ae5bbf9291d7b95d1d456a9a38afca](https://github.com/UM-A2SRL/xhale-avionics/commit/e2a70f6e50ae5bbf9291d7b95d1d456a9a38afca)
* Pixhawk code: [commit 1dc41803ea45c94b7600dd6a2ee353c4215ddf81](https://github.com/UM-A2SRL/xhale-pixhawk/commit/1dc41803ea45c94b7600dd6a2ee353c4215ddf81) 
* QGroundStation: v3.2.6
* Post processing scripts: [commit b3f1f8596279340eaf01de0ceba3bc86233d61aa](https://github.com/UM-A2SRL/xhale-postprocessing/commit/b3f1f8596279340eaf01de0ceba3bc86233d61aa)



