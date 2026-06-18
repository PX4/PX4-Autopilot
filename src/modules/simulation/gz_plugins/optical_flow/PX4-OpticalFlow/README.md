# PX4 Optical Flow repository
This package provides different algorithms to calculate the optical flow. Currently, there is the PX4Flow algorithm (SAD-block-matching) and an OpenCV based (KLT) version.

Please feel free to improve the current algorithms or add new versions.

## Usage
### PX4Flow version
The constructor `OpticalFlowPX4()` needs the following parameters (in this order):
* Focal length in X-Direction [pixels]
* Focal length in Y-Direction [pixels]
* Output rate [Hz] (Optional, default = 15Hz) If smaller than frame rate, flow gets integrated. set to `-1` to use the image frame rate
* Image width [pixels] (Optional, default = 64)
* Image height [pixels] (Optional, default = 64)
* Search size [pixels] (Optional, default = 6)
* Flow feature threshold (Optional, default = 30)
* Flow value threshold (Optional, default = 3000)

For the actual flow calculation the function `calcFlow()` has to be used. It is the same for both versions and needs the following arguments:

Input:
* Image array [* uint8_t]
* Image time stamp [us, uint32_t]

Output:
* Delta time [us, int]
* Angular flow in X-Direction [rad/s, float]
* Angular flow in Y-Direction [rad/s, float]

Return:
* Flow quality [int], 0-255, -1 if it should not be used yet (output rate)

### OpenCV version
The constructor `OpticalFlowOpenCV()` needs the following parameters (in this order):
* Focal length in X-Direction [pixels]
* Focal length in Y-Direction [pixels]
* Output rate [Hz] (Optional, default = 15Hz) If smaller than frame rate, flow gets integrated. set to `-1` to use the image frame rate
* Image width [pixels] (Optional, default = 64)
* Image height [pixels] (Optional, default = 64)
* Number of tracked features (Optional, default = 20)
* Confidence multiplier for outlier rejection (Optional, default = 1.645, 90% confidence interval)

For the actual flow calculation the function `calcFlow()` has to be used. It is the same for both versions and needs the following arguments:

Input:
* Image array [* uint8_t]
* Image time stamp [us, uint32_t]

Output:
* Delta time [us, int]
* Angular flow in X-Direction [rad/s, float]
* Angular flow in Y-Direction [rad/s, float]

Return:
* Flow quality [int], 0-255, -1 if it should not be used yet (output rate)

If you have a camera calibration, you can [undistort](http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e) the tracked features (only for OpenCV version). To use that functionality you have to set the camera matrix `setCameraMatrix()` AND distortion parameters `setCameraDistortion()` after calling the constructor.

`setCameraMatrix()` needs the following arguments:
* Focal length in X-Direction [pixels]
* Focal length in Y-Direction [pixels]
* Principal point in X-Direction [pixels]
* Principal point in Y-Direction [pixels]

`setCameraDistortion()` needs the following arguments (see [here](http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e)):
* Radial coefficient k1
* Radial coefficient k2
* Radial coefficient k3
* Tangential coefficient p1
* Tangential coefficient p2

An example can be found [here](https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_opticalFlow_plugin.cpp#L141-L195).

## Code style
For code formatting [astyle](http://astyle.sourceforge.net/astyle.html) should be used with the `.astylerc` options file.
```
astyle <file> --options=.astylerc
```
