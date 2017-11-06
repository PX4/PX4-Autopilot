/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file airspeed.c
 * Airspeed estimation
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <stdio.h>
#include <math.h>
#include <geo/geo.h>
#include "airspeed.h"

/**
 * Calculate indicated airspeed.
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param differential_pressure total_ pressure - static pressure
 * @return indicated airspeed in m/s
 */
float calc_indicated_airspeed_corrected(enum AIRSPEED_COMPENSATION_MODEL pmodel, enum AIRSPEED_SENSOR_MODEL smodel,
                                        float tube_len, float tube_dia_mm, float differential_pressure, float pressure_ambient, float temperature_celsius)
{

    // air density in kg/m3
    const double rho_air = get_air_density(pressure_ambient, temperature_celsius);

    const float dp = fabsf(differential_pressure);
    float dp_tot = dp;

    float dv = 0.0f;

    switch (smodel) {

    case AIRSPEED_SENSOR_MODEL_MEMBRANE: {
        // do nothing
    }
    break;

    case AIRSPEED_SENSOR_MODEL_SDP3X: {
        // assumes a metal pitot tube with round tip as here: https://drotek.com/shop/2986-large_default/sdp3x-airspeed-sensor-kit-sdp31.jpg
        // and tubing as provided by px4/drotek (1.5 mm diameter)
        // The tube_len represents the length of the tubes connecting the pitot to the sensor.
        switch (pmodel) {
        case AIRSPEED_COMPENSATION_MODEL_PITOT:
        case AIRSPEED_COMPENSATION_MODEL_NO_PITOT: {
            const double dp_corr = (double)dp * 96600.0 / (double)pressure_ambient;
            // flow through sensor
            double flow_SDP33 = (300.805 - 300.878 / (0.00344205 * pow(dp_corr, 0.68698) + 1)) * 1.29 / rho_air;

            // for too small readings the compensation might result in a negative flow which causes numerical issues
            if (flow_SDP33 < 0.0) {
                flow_SDP33 = 0.0;
            }

            float dp_pitot = 0.0f;
            switch (pmodel) {
            case AIRSPEED_COMPENSATION_MODEL_PITOT:
                dp_pitot = (0.0032 * flow_SDP33 * flow_SDP33 + 0.0123 * flow_SDP33 + 1.0) * 1.29 / rho_air;
                break;

            default:
                // do nothing
                break;
            }

            // pressure drop through tube
            const float dp_tube = (flow_SDP33 * 0.674) / 450.0 * (double)tube_len * rho_air / 1.29;

            // speed at pitot-tube tip due to flow through sensor
            dv = 0.125 * flow_SDP33;

            // sum of all pressure drops
            dp_tot = (float)dp_corr + dp_tube + dp_pitot;
        }
        break;
        case AIRSPEED_COMPENSATION_TUBE_PRESSURE_LOSS: {
            // Pressure loss compensation as defined in https://goo.gl/UHV1Vv.
            // tube_dia_mm: Diameter in mm of the pitot and tubes, must have the same diameter.
            // tube_len: Length of the tubes connecting the pitot to the sensor and the static + dynamic port length of the pitot.

            // check if the tube diameter and dp is nonzero to avoid division by 0
            if ((tube_dia_mm > 0.0f) && (dp > 0.0f)) {
                const double d_tubePow4 = pow((double)tube_dia_mm * 1e-3, 4);
                const double denominator = M_PI * d_tubePow4 * rho_air * (double)dp;

                // avoid division by 0
                double eps = 0.0f;
                if (fabs(denominator)> 1e-32) {
                    const double viscosity = (18.205 + 0.0484 * ((double)temperature_celsius - 20.0)) * 1e-6;

                    // 4.79 * 1e-7 -> mass flow through sensor
                    // 59.5 -> dp sensor constant where linear and quadratic contribution to dp vs flow is equal
                    eps = -64.0 * (double)tube_len * viscosity * 4.79 * 1e-7 * (sqrt(1.0 + 8.0 * (double)dp / 59.3319) - 1.0) / denominator;
                }

                // range check on eps
                if (fabs(eps) >= 1.0)
                    eps = 0.0;

                // pressure correction
                dp_tot = dp / (1.0f + (float)eps);
            }
        }
        break;
        default: {
            // do nothing
        }
        break;
        }

    }
    break;

    default: {
        // do nothing
    }
    break;
    }

    // if (!PX4_ISFINITE(dp_tube)) {
    // 	dp_tube = 0.0f;
    // }

    // if (!PX4_ISFINITE(dp_pitot)) {
    // 	dp_pitot = 0.0f;
    // }

    // if (!PX4_ISFINITE(dv)) {
    // 	dv = 0.0f;
    // }

    // computed airspeed without correction for inflow-speed at tip of pitot-tube
    const float airspeed_uncorrected = sqrtf(2 * dp_tot / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

    // corrected airspeed
    const float airspeed_corrected = airspeed_uncorrected + dv;

    // return result with correct sign
    return (differential_pressure > 0.0f) ? airspeed_corrected : -airspeed_corrected;
}


/**
 * Calculate indicated airspeed.
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param differential_pressure total_ pressure - static pressure
 * @return indicated airspeed in m/s
 */
float calc_indicated_airspeed(float differential_pressure)
{


    if (differential_pressure > 0.0f) {
        return sqrtf((2.0f * differential_pressure) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

    } else {
        return -sqrtf((2.0f * fabsf(differential_pressure)) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
    }

}

/**
 * Calculate true airspeed from indicated airspeed.
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @param speed_indicated current indicated airspeed
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return true airspeed in m/s
 */
float calc_true_airspeed_from_indicated(float speed_indicated, float pressure_ambient, float temperature_celsius)
{
    return speed_indicated * sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / get_air_density(pressure_ambient,
                                   temperature_celsius));
}

/**
 * Directly calculate true airspeed
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind
 *
 * @param total_pressure pressure inside the pitot/prandtl tube
 * @param static_pressure pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return true airspeed in m/s
 */
float calc_true_airspeed(float total_pressure, float static_pressure, float temperature_celsius)
{
    float density = get_air_density(static_pressure, temperature_celsius);

    if (density < 0.0001f || !isfinite(density)) {
        density = CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C;
    }

    float pressure_difference = total_pressure - static_pressure;

    if (pressure_difference > 0) {
        return sqrtf((2.0f * (pressure_difference)) / density);

    } else {
        return -sqrtf((2.0f * fabsf(pressure_difference)) / density);
    }
}

float get_air_density(float static_pressure, float temperature_celsius)
{
    return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
}
