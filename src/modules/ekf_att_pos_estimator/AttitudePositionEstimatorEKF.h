/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file AttitudePositionEstimatorEKF.h
 * Implementation of the attitude and position estimator. This is a PX4 wrapper around
 * the EKF IntertialNav filter of Paul Riseborough (@see estimator_22states.cpp)
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <geo/geo.h>
#include <terrain_estimation/terrain_estimator.h>
#include <systemlib/perf_counter.h>
#include <lib/ecl/validation/data_validator_group.h>
#include "estimator_22states.h"

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

//Forward declaration
class AttPosEKF;

class AttitudePositionEstimatorEKF : public control::SuperBlock
{
public:
    /**
     * Constructor
     */
    AttitudePositionEstimatorEKF();

    /* we do not want people ever copying this class */
    AttitudePositionEstimatorEKF(const AttitudePositionEstimatorEKF& that) = delete;
    AttitudePositionEstimatorEKF operator=(const AttitudePositionEstimatorEKF&) = delete;

    /**
     * Destructor, also kills the sensors task.
     */
    ~AttitudePositionEstimatorEKF();

    /**
     * Start the sensors task.
     *
     * @return  OK on success.
     */
    int start();

    /**
     * Task status
     *
     * @return  true if the mainloop is running
     */
    bool task_running() { return _task_running; }

    /**
     * Print the current status.
     */
    void print_status();

    /**
     * Trip the filter by feeding it NaN values.
     */
    int trip_nan();

    /**
     * Enable logging.
     *
     * @param   enable Set to true to enable logging, false to disable
     */
    int enable_logging(bool enable);

    /**
     * Set debug level.
     *
     * @param   debug Desired debug level - 0 to disable.
     */
    int set_debuglevel(unsigned debug) { _debug = debug; return 0; }

    static constexpr unsigned MAX_PREDICITION_STEPS = 3; /**< maximum number of prediction steps between updates */

private:
    bool        _task_should_exit;      /**< if true, sensor task should exit */
    bool        _task_running;          /**< if true, task is running in its mainloop */
    int     _estimator_task;        /**< task handle for sensor task */

    int     _sensor_combined_sub;
    int     _distance_sub;          /**< distance measurement */
    int     _airspeed_sub;          /**< airspeed subscription */
    int     _baro_sub;          /**< barometer subscription */
    int     _gps_sub;           /**< GPS subscription */
    int     _vehicle_status_sub;
    int     _vehicle_land_detected_sub;
    int     _params_sub;            /**< notification of parameter updates */
    int     _manual_control_sub;        /**< notification of manual control updates */
    int     _mission_sub;
    int     _home_sub;          /**< home position as defined by commander / user */
    int     _landDetectorSub;
    int     _armedSub;

    orb_advert_t    _att_pub;           /**< vehicle attitude */
    orb_advert_t    _ctrl_state_pub;        /**< control state */
    orb_advert_t    _global_pos_pub;        /**< global position */
    orb_advert_t    _local_pos_pub;         /**< position in local frame */
    orb_advert_t    _estimator_status_pub;      /**< status of the estimator */
    orb_advert_t    _wind_pub;          /**< wind estimate */

    struct vehicle_attitude_s           _att;           /**< vehicle attitude */
    struct control_state_s              _ctrl_state;    /**< control state */
    struct gyro_report                  _gyro;
    struct accel_report                 _accel;
    struct mag_report                   _mag;
    struct airspeed_s                   _airspeed;      /**< airspeed */
    struct baro_report                  _baro;          /**< baro readings */
    struct vehicle_status_s		_vehicle_status;
    struct vehicle_land_detected_s      _vehicle_land_detected;
    struct vehicle_global_position_s    _global_pos;        /**< global vehicle position */
    struct vehicle_local_position_s     _local_pos;     /**< local vehicle position */
    struct vehicle_gps_position_s       _gps;           /**< GPS position */
    struct wind_estimate_s              _wind;          /**< wind estimate */
    struct distance_sensor_s            _distance;      /**< distance estimate */
    struct actuator_armed_s             _armed;

    hrt_abstime _last_accel;
    hrt_abstime _last_mag;
    unsigned _prediction_steps;
    uint64_t _prediction_last;

    struct sensor_combined_s            _sensor_combined;

    struct map_projection_reference_s   _pos_ref;

    float                       _filter_ref_offset;   /**< offset between initial baro reference and GPS init baro altitude */
    float                       _baro_gps_offset;   /**< offset between baro altitude (at GPS init time) and GPS altitude */
    hrt_abstime                 _last_debug_print = 0;
    float       _vibration_warning_threshold = 1.0f;
    hrt_abstime _vibration_warning_timestamp = 0;

    perf_counter_t  _loop_perf;         ///< loop performance counter
    perf_counter_t  _loop_intvl;        ///< loop rate counter
    perf_counter_t  _perf_gyro;         ///<local performance counter for gyro updates
    perf_counter_t  _perf_mag;          ///<local performance counter for mag updates
    perf_counter_t  _perf_gps;          ///<local performance counter for gps updates
    perf_counter_t  _perf_baro;         ///<local performance counter for baro updates
    perf_counter_t  _perf_airspeed;     ///<local performance counter for airspeed updates
    perf_counter_t  _perf_reset;        ///<local performance counter for filter resets

    float           _gps_alt_filt;
    float           _baro_alt_filt;
    float           _covariancePredictionDt;  ///< time lapsed since last covariance prediction
    bool            _gpsIsGood;               ///< True if the current GPS fix is good enough for us to use
    uint64_t        _previousGPSTimestamp;    ///< Timestamp of last good GPS fix we have received
    bool            _baro_init;
    bool            _gps_initialized;
    hrt_abstime     _filter_start_time;
    hrt_abstime     _last_sensor_timestamp;
    hrt_abstime     _distance_last_valid;
    DataValidatorGroup _voter_gyro;
    DataValidatorGroup _voter_accel;
    DataValidatorGroup _voter_mag;
    int             _gyro_main;         ///< index of the main gyroscope
    int             _accel_main;        ///< index of the main accelerometer
    int             _mag_main;          ///< index of the main magnetometer
    bool            _data_good;         ///< all required filter data is ok
    bool            _failsafe;          ///< failsafe on one of the sensors
    bool            _vibration_warning; ///< high vibration levels detected
    bool            _ekf_logging;       ///< log EKF state
    unsigned        _debug;             ///< debug level - default 0

    bool            _newHgtData;
    bool            _newAdsData;
    bool            _newDataMag;
    bool            _newRangeData;

    orb_advert_t    _mavlink_log_pub;

    control::BlockParamFloat _mag_offset_x;
    control::BlockParamFloat _mag_offset_y;
    control::BlockParamFloat _mag_offset_z;

    struct {
        int32_t vel_delay_ms;
        int32_t pos_delay_ms;
        int32_t height_delay_ms;
        int32_t mag_delay_ms;
        int32_t tas_delay_ms;
        float velne_noise;
        float veld_noise;
        float posne_noise;
        float posd_noise;
        float mag_noise;
        float gyro_pnoise;
        float acc_pnoise;
        float gbias_pnoise;
        float abias_pnoise;
        float mage_pnoise;
        float magb_pnoise;
        float eas_noise;
        float pos_stddev_threshold;
    }       _parameters;            /**< local copies of interesting parameters */

    struct {
        param_t vel_delay_ms;
        param_t pos_delay_ms;
        param_t height_delay_ms;
        param_t mag_delay_ms;
        param_t tas_delay_ms;
        param_t velne_noise;
        param_t veld_noise;
        param_t posne_noise;
        param_t posd_noise;
        param_t mag_noise;
        param_t gyro_pnoise;
        param_t acc_pnoise;
        param_t gbias_pnoise;
        param_t abias_pnoise;
        param_t mage_pnoise;
        param_t magb_pnoise;
        param_t eas_noise;
        param_t pos_stddev_threshold;
    }       _parameter_handles;     /**< handles for interesting parameters */

    AttPosEKF                   *_ekf;

    TerrainEstimator            *_terrain_estimator;

    /* Low pass filter for attitude rates */
    math::LowPassFilter2p _LP_att_P;
    math::LowPassFilter2p _LP_att_Q;
    math::LowPassFilter2p _LP_att_R;

private:
    /**
     * Update our local parameter cache.
     */
    int     parameters_update();

    /**
     * Update control outputs
     *
     */
    void        control_update();

    /**
     * Check for changes in land detected.
     */
    void        vehicle_status_poll();

    /**
     * Check for changes in land detected.
     */
    void        vehicle_land_detected_poll();

    /**
     * Shim for calling task_main from task_create.
     */
    static void task_main_trampoline(int argc, char *argv[]);

    /**
     * Main filter task.
     */
    void        task_main();

    /**
     * Check filter sanity state
     *
     * @return zero if ok, non-zero for a filter error condition.
     */
    int     check_filter_state();

    /**
    * @brief
    *   Publish the euler and quaternions for attitude estimation
    **/
    void publishAttitude();

    /**
    * @brief
    *   Publish the system state for control modules
    **/
    void publishControlState();

    /**
    * @brief
    *   Publish local position relative to reference point where filter was initialized
    **/
    void publishLocalPosition();

    /**
    * @brief
    *   Publish global position estimation (WSG84 and AMSL).
    *   A global position estimate is only valid if we have a good GPS fix
    **/
    void publishGlobalPosition();

    /**
    * @brief
    *   Publish wind estimates for north and east in m/s
    **/
    void publishWindEstimate();

    /**
    * @brief
    *   Runs the sensor fusion step of the filter. The parameters determine which of the sensors
    *   are fused with each other
    **/
    void updateSensorFusion(const bool fuseGPS, const bool fuseMag, const bool fuseRangeSensor,
            const bool fuseBaro, const bool fuseAirSpeed);

    /**
    * @brief
    *   Initialize first time good GPS fix so we can get a reference point to calculate local position from
    *   Should only be required to call once
    **/
    void initializeGPS();

    /**
     * Initialize the reference position for the local coordinate frame
     */
    void initReferencePosition(hrt_abstime timestamp, bool gps_valid,
            double lat, double lon, float gps_alt, float baro_alt);

    /**
    * @brief
    *   Polls all uORB subscriptions if any new sensor data has been publish and sets the appropriate
    *   flags to true (e.g newDataGps)
    **/
    void pollData();
};
