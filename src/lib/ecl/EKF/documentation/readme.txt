The Matlab script used to derive the autocoded expressions in the EKF can be found here: https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m

A working Matlab model of the filter capable of replaying flight data can be found here: https://github.com/PX4/ecl/tree/master/matlab/EKF_replay

The EKF uses a range of techniques acquired from several years of experience including an original method to handle the delayed time horizon problem. A list of references I have found useful has been included.

Paul Riseborough
https://github.com/priseborough