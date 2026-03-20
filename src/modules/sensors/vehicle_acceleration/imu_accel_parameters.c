/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
* Low pass filter cutoff frequency for accel
*
* The cutoff frequency for the 2nd order butterworth filter on the primary accelerometer.
* This only affects the signal sent to the controllers, not the estimators. 0 disables the filter.
*
* When dynamic notch filters are enabled (IMU_ACC_DNF_EN), this can be raised
* (e.g. to 50-80 Hz) to reduce phase lag while the notch filters handle
* motor-frequency vibrations.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_ACCEL_CUTOFF, 30.0f);

/**
* Notch filter frequency for accel
*
* The center frequency for the 2nd order notch filter on the primary accelerometer.
* This filter can be enabled to avoid amplification of structural resonances at a specific frequency.
* See "IMU_ACC_NF0_BW" to set the bandwidth of the filter.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @increment 0.1
* @decimal 1
* @reboot_required false
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_ACC_NF0_FRQ, 0.0f);

/**
* Notch filter bandwidth for accel
*
* The frequency width of the stop band for the 2nd order notch filter on the primary accelerometer.
* See "IMU_ACC_NF0_FRQ" to activate the filter and to set the notch frequency.
*
* @min 0
* @max 100
* @unit Hz
* @increment 0.1
* @decimal 1
* @reboot_required false
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_ACC_NF0_BW, 20.0f);

/**
* Notch filter 2 frequency for accel
*
* The center frequency for the 2nd order notch filter on the primary accelerometer.
* This filter can be enabled to avoid amplification of structural resonances at a specific frequency.
* See "IMU_ACC_NF1_BW" to set the bandwidth of the filter.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @increment 0.1
* @decimal 1
* @reboot_required false
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_ACC_NF1_FRQ, 0.0f);

/**
* Notch filter 2 bandwidth for accel
*
* The frequency width of the stop band for the 2nd order notch filter on the primary accelerometer.
* See "IMU_ACC_NF1_FRQ" to activate the filter and to set the notch frequency.
*
* @min 0
* @max 100
* @unit Hz
* @increment 0.1
* @decimal 1
* @reboot_required false
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_ACC_NF1_BW, 20.0f);

/**
* IMU accel dynamic notch filtering
*
* Enable bank of dynamically updating notch filters.
* Requires ESC RPM feedback or onboard FFT (IMU_GYRO_FFT_EN).
* @group Sensors
* @min 0
* @max 3
* @bit 0 ESC RPM
* @bit 1 FFT
*/
PARAM_DEFINE_INT32(IMU_ACC_DNF_EN, 0);

/**
* IMU accel ESC notch filter bandwidth
*
* Bandwidth per notch filter when using dynamic notch filtering with ESC RPM.
*
* @group Sensors
* @unit Hz
* @increment 0.1
* @decimal 1
* @min 5
* @max 100
*/
PARAM_DEFINE_FLOAT(IMU_ACC_DNF_BW, 15.f);

/**
* IMU accel dynamic notch filter harmonics
*
* ESC RPM number of harmonics (multiples of RPM) for ESC RPM dynamic notch filtering.
*
* @group Sensors
* @min 1
* @max 7
*/
PARAM_DEFINE_INT32(IMU_ACC_DNF_HMC, 3);

/**
* IMU accel dynamic notch filter minimum frequency
*
* Minimum notch filter frequency in Hz.
*
* @group Sensors
* @unit Hz
* @increment 0.1
* @decimal 1
* @min 0
* @max 1000
*/
PARAM_DEFINE_FLOAT(IMU_ACC_DNF_MIN, 25.f);
