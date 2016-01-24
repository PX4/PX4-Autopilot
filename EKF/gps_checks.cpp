/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS  (1<<0)
#define MASK_GPS_GDOP   (1<<1)
#define MASK_GPS_HACC   (1<<2)
#define MASK_GPS_VACC   (1<<3)
#define MASK_GPS_SACC   (1<<4)
#define MASK_GPS_HDRIFT (1<<5)
#define MASK_GPS_VDRIFT (1<<6)
#define MASK_GPS_HSPD   (1<<7)
#define MASK_GPS_VSPD   (1<<8)

/*
 * Return true if the GPS solution quality is adequate to set an origin for the EKF
 * and start GPS aiding.
 * All activated checks must pass for 10 seconds.
 * Checks are activated using the EKF2_GPS_CHECKS bitmask parameter
 * Checks are adjusted using the EKF2_REQ_* parameters
*/
bool EstimatorBase::gps_is_good(struct gps_message *gps)
{
    // Check the number of satellites
    _gpsCheckFailStatus.flags.nsats = (gps->nsats < _params.requiredNsats);

    // Check the geometric dilution of precision
    _gpsCheckFailStatus.flags.gdop = (gps->gdop > _params.requiredGDoP);

    // Check the reported horizontal position accuracy
    _gpsCheckFailStatus.flags.hacc = (gps->eph > _params.requiredEph);

    // Check the reported vertical position accuracy
    _gpsCheckFailStatus.flags.vacc = (gps->epv > _params.requiredEpv);

    // Check the reported speed accuracy
    _gpsCheckFailStatus.flags.sacc = (gps->eph > _params.requiredEph);

    // Calculate position movement since last measurement
    float deltaPosN = 0.0f;
    float deltaPosE = 0.0f;
    double lat = gps->lat * 1.0e-7;
    double lon = gps->lon * 1.0e-7;
    if (_posRef.init_done) {
        map_projection_project(&_posRef, lat, lon, &deltaPosN, &deltaPosE);
    } else {
        map_projection_init(&_posRef, lat, lon);
        _gps_alt_ref = gps->alt * 1e-3f;
    }

    // Calculate time lapsesd since last update, limit to prevent numerical errors and calculate the lowpass filter coefficient
    const float filtTimeConst = 10.0f;
    float dt = fminf(fmaxf(float(hrt_absolute_time() - _last_gps_origin_time_us)*1e-6f,0.001f),filtTimeConst);
    float filterCoef = dt/filtTimeConst;

    // Calculate the horizontal drift velocity components and limit to 10x the threshold
    float velLimit = 10.0f * _params.requiredHdrift;
    float velN = fminf(fmaxf(deltaPosN / dt, -velLimit), velLimit);
    float velE = fminf(fmaxf(deltaPosE / dt, -velLimit), velLimit);

    // Apply a low pass filter
    _gpsDriftVelN = velN * filterCoef + _gpsDriftVelN * (1.0f - filterCoef);
    _gpsDriftVelE = velE * filterCoef + _gpsDriftVelE * (1.0f - filterCoef);

    // Calculate the horizontal drift speed and fail if too high
    // This check can only be used if the vehicle is stationary during alignment
    if(_vehicleArmed) {
        float driftSpeed = sqrtf(_gpsDriftVelN * _gpsDriftVelN + _gpsDriftVelE * _gpsDriftVelE);
        _gpsCheckFailStatus.flags.hdrift = (driftSpeed > _params.requiredHdrift);
    } else {
        _gpsCheckFailStatus.flags.hdrift = false;
    }

    // Save current position as the reference for next time
    map_projection_init(&_posRef, lat, lon);
    _last_gps_origin_time_us = hrt_absolute_time();

    // Calculate the vertical drift velocity and limit to 10x the threshold
    velLimit = 10.0f * _params.requiredVdrift;
    float velD = fminf(fmaxf((_gps_alt_ref - gps->alt * 1e-3f) / dt, -velLimit), velLimit);

    // Save the current height as the reference for next time
    _gps_alt_ref = gps->alt * 1e-3f;

    // Apply a low pass filter to the vertical velocity
    _gpsDriftVelD = velD * filterCoef + _gpsDriftVelD * (1.0f - filterCoef);

    // Fail if the vertical drift speed is too high
    // This check can only be used if the vehicle is stationary during alignment
    if(_vehicleArmed) {
        _gpsCheckFailStatus.flags.vdrift = (fabsf(_gpsDriftVelD) > _params.requiredVdrift);
    } else {
        _gpsCheckFailStatus.flags.vdrift = false;
    }

    // Check the magnitude of the filtered horizontal GPS velocity
    // This check can only be used if the vehicle is stationary during alignment
    if (_vehicleArmed) {
        velLimit = 10.0f * _params.requiredHdrift;
        float velN = fminf(fmaxf(gps->vel_ned[0],-velLimit),velLimit);
        float velE = fminf(fmaxf(gps->vel_ned[1],-velLimit),velLimit);
        _gpsVelNorthFilt = velN * filterCoef + _gpsVelNorthFilt * (1.0f - filterCoef);
        _gpsVelEastFilt  = velE * filterCoef + _gpsVelEastFilt  * (1.0f - filterCoef);
        float horizSpeed = sqrtf(_gpsVelNorthFilt * _gpsVelNorthFilt + _gpsVelEastFilt * _gpsVelEastFilt);
        _gpsCheckFailStatus.flags.hspeed = (horizSpeed > _params.requiredHdrift);
    } else {
        _gpsCheckFailStatus.flags.hspeed = false;
    }

    // Check  the filtered difference between GPS and EKF vertical velocity
    velLimit = 10.0f * _params.requiredVdrift;
    float vertVel = fminf(fmaxf((gps->vel_ned[2] - _state.vel(2)), -velLimit), velLimit);
    _gpsVertVelFilt = vertVel * filterCoef + _gpsVertVelFilt * (1.0f - filterCoef);
    _gpsCheckFailStatus.flags.vspeed = (fabsf(_gpsVertVelFilt) > _params.requiredVdrift);

    // assume failed first time through
    if (_lastGpsFail_us == 0) {
        _lastGpsFail_us = hrt_absolute_time();
    }

    // if any user selected checks have failed, record the fail time
    if (
    (_gpsCheckFailStatus.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
    (_gpsCheckFailStatus.flags.gdop    && (_params.gps_check_mask & MASK_GPS_GDOP)) ||
    (_gpsCheckFailStatus.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
    (_gpsCheckFailStatus.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
    (_gpsCheckFailStatus.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
    (_gpsCheckFailStatus.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
    (_gpsCheckFailStatus.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
    (_gpsCheckFailStatus.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
    (_gpsCheckFailStatus.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD))
    ) {
        _lastGpsFail_us = hrt_absolute_time();
    }

    // continuous period without fail of 10 seconds required to return a healthy status
    if (hrt_absolute_time() - _lastGpsFail_us > 1e7) {
        return true;
    }
    return false;
}

