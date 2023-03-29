# xhale-postprocessing

This repo is part of the X-Hale project. Please visit our [wiki](https://github.com/UM-A2SRL/xhale-avionics/wiki) for more info.

This repo should contain material (or links to) on X-Hale postprocessing routines. For instance,

* Plotting data and analysis Scripts for ATV –(plan to merge) 
* Scripts to identify transfer functions from in-flight telemetry data and calculate gains for the pitch, roll and yaw control loops (in another repo for now: X-HALE_autopilot)
* Camera Data (reconstruct shape/deformation)

## How to plot data from Pixhawk log flights

The associated scripts are in folder FlightTestPlotting.

* Obtain .px4log file from microSD card in Pixhawk.
* Run **ImportPX4LogData** to convert .px4log into MATLAB variables in a .mat.
* Run **Plot_PixHawk_data.m** to plot data from .mat variables.

Attention 1: To run the **ImportPX4LogData**, the **sdlog2_dump.py** script is neccessary, along a Python (2/3?) installation capable of running it.

Attention 2: **Plot_PixHawk_data_video.m** is an alternative to **Plot_PixHawk_data.m** that plots data with video.

Attention 3: **Plot_PixHawk_data_video.m** and **Plot_PixHawk_data.m** create animation files in .mp4 which are appropriate for PowerPoint presentations. AVI files are problematic with PowerPoint.

### Example files

* log001.px4log is an example of Pixhawk log data
* log001.mat is an example of MATLAB data obtained from **ImportPX4LogData**
* RollDisturbanceCrash.mp4 is an example of video called by **Plot_PixHawk_data_video.m**
