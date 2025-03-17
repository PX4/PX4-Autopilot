# Binary Size Profiling

The `bloaty_compare_master` build target allows you to get a better understanding of the impact of changes on code size.
When it is used, the toolchain downloads the latest successful master build of a particular firmware and compares it to the local build (using the [bloaty](https://github.com/google/bloaty) size profiler for binaries).

:::tip
This can help analyse changes that (may) cause `px4_fmu-v2_default` to hit the 1MB flash limit.
:::

_Bloaty_ must be in your path and found at _cmake_ configure time.
The PX4 [docker files](https://github.com/PX4/containers/blob/master/docker/Dockerfile_nuttx-bionic) install _bloaty_ as shown:

```sh
git clone --recursive https://github.com/google/bloaty.git /tmp/bloaty \
	&& cd /tmp/bloaty && cmake -GNinja . && ninja bloaty && cp bloaty /usr/local/bin/ \
	&& rm -rf /tmp/*
```

The example below shows how you might see the impact of removing the _mpu9250_ driver from `px4_fmu-v2_default`.
First it locally sets up a build without the driver:

```sh
 % git diff
diff --git a/boards/px4/fmu-v2/default.px4board b/boards/px4/fmu-v2/default.px4board
index 40d7778..2ce7972 100644
--- a/boards/px4/fmu-v2/default.px4board
+++ b/boards/px4/fmu-v2/default.px4board
@@ -36,7 +36,7 @@
-               CONFIG_DRIVERS_IMU_INVENSENSE_MPU9250=y
+               CONFIG_DRIVERS_IMU_INVENSENSE_MPU9250=n
```

Then use the make target, specifying the target build to compare (`px4_fmu-v2_default` in this case):

```sh
% make px4_fmu-v2_default bloaty_compare_master
...
...
...
     VM SIZE                                                                                        FILE SIZE
 --------------                                                                                  --------------
  [DEL]     -52 MPU9250::check_null_data(unsigned int*, unsigned char)                               -52  [DEL]
  [DEL]     -52 MPU9250::test_error()                                                                -52  [DEL]
  [DEL]     -52 MPU9250_gyro::MPU9250_gyro(MPU9250*, char const*)                                    -52  [DEL]
  [DEL]     -56 mpu9250::info(MPU9250_BUS)                                                           -56  [DEL]
  [DEL]     -56 mpu9250::regdump(MPU9250_BUS)                                                        -56  [DEL]
...                                        -336  [DEL]
  [DEL]    -344 MPU9250_mag::_measure(ak8963_regs)                                                  -344  [DEL]
  [DEL]    -684 MPU9250::MPU9250(device::Device*, device::Device*, char const*, char const*, cha    -684  [DEL]
  [DEL]    -684 MPU9250::init()                                                                     -684  [DEL]
  [DEL]   -1000 MPU9250::measure()                                                                 -1000  [DEL]
 -41.3%   -1011 [43 Others]                                                                        -1011 -41.3%
  -1.0% -1.05Ki [Unmapped]                                                                       +24.2Ki  +0.2%
  -1.0% -10.3Ki TOTAL                                                                            +14.9Ki  +0.1%
```

This shows that removing _mpu9250_ from `px4_fmu-v2_default` would save 10.3 kB of flash.
It also shows the sizes of different pieces of the _mpu9250_ driver.
