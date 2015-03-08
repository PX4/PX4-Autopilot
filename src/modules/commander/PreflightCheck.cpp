/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PreflightCheck.cpp
 *
 * Preflight check for main system components
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/rc_check.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_baro.h>

#include <mavlink/mavlink_log.h>

#include "PreflightCheck.h"

namespace Commander
{
    static bool magnometerCheck(int mavlink_fd)
    {
        int fd = open(MAG0_DEVICE_PATH, 0);
        if (fd < 0) {
            warn("failed to open magnetometer - start with 'hmc5883 start' or 'lsm303d start'");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: NO MAG");
            return false;
        }

        int calibration_devid;
        int devid = ioctl(fd, DEVIOCGDEVICEID,0);
        param_get(param_find("CAL_MAG0_ID"), &(calibration_devid));
        if (devid != calibration_devid){
            warnx("magnetometer calibration is for a different device - calibrate magnetometer first (dev: %d vs cal: %d)", devid, calibration_devid);
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: MAG CAL ID");
            return false;
        }

        int ret = ioctl(fd, MAGIOCSELFTEST, 0);
        if (ret != OK) {
            warnx("magnetometer calibration missing or bad - calibrate magnetometer first");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: MAG CHECK/CAL");
            return false;
        }
        close(fd);
        return true;
    }

    static bool accelerometerCheck(int mavlink_fd)
    {
        int fd = open(ACCEL0_DEVICE_PATH, O_RDONLY);
        int ret = ioctl(fd, ACCELIOCSELFTEST, 0);

        int calibration_devid;
        int devid = ioctl(fd, DEVIOCGDEVICEID,0);
        param_get(param_find("CAL_ACC0_ID"), &(calibration_devid));
        if (devid != calibration_devid){
            warnx("accelerometer calibration is for a different device - calibrate accelerometer first");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: ACC CAL ID");
            return false;
        }
        
        if (ret != OK) {
            warnx("accel self test failed");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: ACCEL CHECK/CAL");
            return false;
        }

        // check measurement result range
        struct accel_report acc;
        ret = read(fd, &acc, sizeof(acc));

        if (ret == sizeof(acc)) {
            // evaluate values
            float accel_magnitude = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

            // evaluate values
            if (accel_magnitude > 30.0f) { //m/s^2
                warnx("accel with spurious values");
                mavlink_log_critical(mavlink_fd, "SENSOR FAIL: |ACCEL| > 30 m/s^2");
                //this is frickin' fatal
                return false;
            }
        } else {
            warnx("accel read failed");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: ACCEL READ");
            //this is frickin' fatal
            return false;
        }

        close(fd);
        return true;
    }

    static bool gyroCheck(int mavlink_fd)
    {
        int fd = open(GYRO0_DEVICE_PATH, 0);
            
        int calibration_devid;
        int devid = ioctl(fd, DEVIOCGDEVICEID,0);
        param_get(param_find("CAL_GYRO0_ID"), &(calibration_devid));
        if (devid != calibration_devid){
            warnx("gyro calibration is for a different device - calibrate gyro first");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: GYRO CAL ID");
            return false;
        }        

        int ret = ioctl(fd, GYROIOCSELFTEST, 0);

        if (ret != OK) {
            warnx("gyro self test failed");
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: GYRO CHECK/CAL");
            return false;
        }

        close(fd);
        return true;
    }

    static bool baroCheck(int mavlink_fd)
    {
        int fd = open(BARO0_DEVICE_PATH, 0);
        if(fd < 0) {
            mavlink_log_critical(mavlink_fd, "SENSOR FAIL: Barometer");
            return false;        
        }

        close(fd);
        return true;
    }

    bool preflightCheck(int mavlink_fd, bool checkMag, bool checkAcc, bool checkGyro, bool checkBaro, bool checkRC)
    {
        //give the system some time to sample the sensors in the background
        usleep(150000);

        //Magnetometer
        if(checkMag) {
            if(!magnometerCheck(mavlink_fd)) {
                return false;
            }            
        }

        //Accelerometer
        if(checkAcc) {
            if(!accelerometerCheck(mavlink_fd)) {
                return false;
            }            
        }

        // ---- GYRO ----
        if(checkGyro) {
            if(!gyroCheck(mavlink_fd)) {
                return false;
            }            
        }

        // ---- BARO ----
        if(checkBaro) {
            if(!baroCheck(mavlink_fd)) {
                return false;
            }
        }

        // ---- RC CALIBRATION ----
        if(checkRC) {
            if(rc_calibration_check(mavlink_fd) != OK) {
                return false;
            }
        }

        //All is good!
        return true;
    }
}
