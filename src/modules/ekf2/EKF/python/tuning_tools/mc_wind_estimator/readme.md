# PX4 Drag fusion parameter tuning algorithm

In PX4, drag fusion can be enabled in order to estimate the wind when flying a multirotor, assuming that the body vertical acceleration is produced by the rotors and that the lateral forces are produced by drag.
The model assumes a combination of:
1. **momentum drag:** created by the rotors changing the direction of the incoming air, linear to the relative airspeed.
   Parameter [`EKF2_MCOEF`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_MCOEF)
2. **bluff body drag:** created by the [wetted area](https://en.wikipedia.org/wiki/Wetted_area) of the aircraft, quadratic to the relative airspeed.
   Parameters [`EKF2_BCOEF_X`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_BCOEF_X) and [`EKF2_BCOEF_Y`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_BCOEF_Y)

The python script was created to automate the tuning of the aforementioned parameters using flight test data.

## How to use this script

First, a flight log with enough information is required in order to accurately estimate the parameters.
The best way to do this is to fly the drone in altitude mode, accelerate to a moderate-high speed and let the drone slow-down by its own drag.
Repeat the same maneuver in all directions and several times to obtain a good dataset.

> **NOTE:** the current state of this script assumes no wind. Some modifications are required to estimate both the wind and the parameters at the same time.

Then, install the required python packages:
```
pip install -r requirements.txt
```

and run the script and give it the log file as an argument:
```
python mc_wind_estimator_tuning.py <logfilename.ulg>
```

The estimated parameters are displayed in the console and the fit quality is shown in a plot:
```
param set EKF2_BCOEF_X 0.0
param set EKF2_BCOEF_Y 62.1
param set EKF2_MCOEF 0.16

# EXPERIMENTAL
param set EKF2_DRAG_NOISE 0.31
```
![DeepinScreenshot_matplotlib_20220329100027](https://user-images.githubusercontent.com/14822839/160563024-efddd100-d7db-46f7-8676-cf4296e9f737.png)
