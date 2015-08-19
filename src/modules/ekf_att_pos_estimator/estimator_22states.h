/****************************************************************************
* Copyright (c) 2014, Paul Riseborough All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met:
* 
* Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer.
* 
* Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution.
* 
* Neither the name of the {organization} nor the names of its contributors 
* may be used to endorse or promote products derived from this software without 
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/**
 * @file estimator_22states.h
 *
 * Definition of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "estimator_utilities.h"
#include <cstddef>

constexpr size_t EKF_STATE_ESTIMATES = 22;
constexpr size_t EKF_DATA_BUFFER_SIZE = 50;

class AttPosEKF {

public:

    AttPosEKF();
    ~AttPosEKF();



    /* ##############################################
     *
     *   M A I N    F I L T E R    P A R A M E T E R S
     *
     * ########################################### */

    /*
     * parameters are defined here and initialised in
     * the InitialiseParameters()
     */

    float covTimeStepMax; // maximum time allowed between covariance predictions
    float covDelAngMax; // maximum delta angle between covariance predictions
    float rngFinderPitch; // pitch angle of laser range finder in radians. Zero is aligned with the Z body axis. Positive is RH rotation about Y body axis.

    float yawVarScale;
    float windVelSigma;
    float dAngBiasSigma;
    float dVelBiasSigma;
    float magEarthSigma;
    float magBodySigma;
    float gndHgtSigma;

    float vneSigma;
    float vdSigma;
    float posNeSigma;
    float posDSigma;
    float magMeasurementSigma;
    float airspeedMeasurementSigma;

    float gyroProcessNoise;
    float accelProcessNoise;

    float EAS2TAS; // ratio f true to equivalent airspeed

    struct mag_state_struct {
        unsigned obsIndex;
        float MagPred[3];
        float SH_MAG[9];
        float q0;
        float q1;
        float q2;
        float q3;
        float magN;
        float magE;
        float magD;
        float magXbias;
        float magYbias;
        float magZbias;
        float R_MAG;
        Mat3f DCM;
    };

    struct mag_state_struct magstate;
    struct mag_state_struct resetMagState;




    // Global variables
    float KH[EKF_STATE_ESTIMATES][EKF_STATE_ESTIMATES]; //  intermediate result used for covariance updates
    float KHP[EKF_STATE_ESTIMATES][EKF_STATE_ESTIMATES]; // intermediate result used for covariance updates
    float P[EKF_STATE_ESTIMATES][EKF_STATE_ESTIMATES]; // covariance matrix
    float Kfusion[EKF_STATE_ESTIMATES]; // Kalman gains
    float states[EKF_STATE_ESTIMATES]; // state matrix
    float resetStates[EKF_STATE_ESTIMATES];
    float storedStates[EKF_STATE_ESTIMATES][EKF_DATA_BUFFER_SIZE]; // state vectors stored for the last 50 time steps
    uint32_t statetimeStamp[EKF_DATA_BUFFER_SIZE]; // time stamp for each state vector stored

    // Times
    uint64_t lastVelPosFusion;  // the time of the last velocity fusion, in the standard time unit of the filter

    float statesAtVelTime[EKF_STATE_ESTIMATES]; // States at the effective measurement time for posNE and velNED measurements
    float statesAtPosTime[EKF_STATE_ESTIMATES]; // States at the effective measurement time for posNE and velNED measurements
    float statesAtHgtTime[EKF_STATE_ESTIMATES]; // States at the effective measurement time for the hgtMea measurement
    float statesAtMagMeasTime[EKF_STATE_ESTIMATES]; // filter satates at the effective measurement time
    float statesAtVtasMeasTime[EKF_STATE_ESTIMATES]; // filter states at the effective measurement time
    float statesAtRngTime[EKF_STATE_ESTIMATES]; // filter states at the effective measurement time
    float statesAtFlowTime[EKF_STATE_ESTIMATES]; // States at the effective optical flow measurement time
    float omegaAcrossFlowTime[3]; // angular rates at the effective optical flow measurement time

    Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
    Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
    Vector3f summedDelAng; // summed delta angles about the xyz body axes corrected for errors (rad)
    Vector3f summedDelVel; // summed delta velocities along the XYZ body axes corrected for errors (m/s)
    Vector3f prevDelAng;    ///< previous delta angle, used for coning correction
    float accNavMag; // magnitude of navigation accel (- used to adjust GPS obs variance (m/s^2)
    Vector3f earthRateNED; // earths angular rate vector in NED (rad/s)
    Vector3f angRate; // angular rate vector in XYZ body axes measured by the IMU (rad/s)
    Vector3f lastGyroOffset;    // Last gyro offset
    Vector3f delAngTotal;

    Mat3f Tbn; // transformation matrix from body to NED coordinatesFuseOptFlow
    Mat3f Tnb; // transformation amtrix from NED to body coordinates

    Vector3f accel; // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f dVelIMU;
    Vector3f dAngIMU;
    float dtIMU; // time lapsed since the last IMU measurement or covariance update (sec), this may have significant jitter
    float dtIMUfilt; // average time between IMU measurements (sec)
    float dtVelPos; // time lapsed since the last position / velocity fusion (seconds), this may have significant jitter
    float dtVelPosFilt; // average time between position / velocity fusion steps
    float dtHgtFilt; // average time between height measurement updates
    float dtGpsFilt; // average time between gps measurement updates
    uint8_t fusionModeGPS; // 0 = GPS outputs 3D velocity, 1 = GPS outputs 2D velocity, 2 = GPS outputs no velocity
    float innovVelPos[6]; // innovation output
    float varInnovVelPos[6]; // innovation variance output

    float velNED[3]; // North, East, Down velocity obs (m/s)
    float posNE[2]; // North, East position obs (m)
    float hgtMea; //  measured height (m)
    float baroHgtOffset;        ///< the baro (weather) offset from normalized altitude
    float rngMea; // Ground distance

    float innovMag[3]; // innovation output
    float varInnovMag[3]; // innovation variance output
    Vector3f magData; // magnetometer flux radings in X,Y,Z body axes
    float flowRadXYcomp[2]; // motion compensated optical flow angular rates(rad/sec)
    float flowRadXY[2]; // raw (non motion compensated) optical flow angular rates (rad/sec)
    float innovVtas; // innovation output
    float innovRng; ///< Range finder innovation
    float innovOptFlow[2]; // optical flow LOS innovations (rad/sec)
    float varInnovOptFlow[2]; // optical flow innovations variances (rad/sec)^2
    float varInnovVtas; // innovation variance output
    float varInnovRng; // range finder innovation variance
    float VtasMeas; // true airspeed measurement (m/s)
    float magDeclination;       ///< magnetic declination
    double latRef; // WGS-84 latitude of reference point (rad)
    double lonRef; // WGS-84 longitude of reference point (rad)
    float hgtRef; // WGS-84 height of reference point (m)
    bool refSet;                ///< flag to indicate if the reference position has been set
    Vector3f magBias; // states representing magnetometer bias vector in XYZ body axes
    unsigned covSkipCount; // Number of state prediction frames (IMU daya updates to skip before doing the covariance prediction
    uint32_t lastFixTime_ms; // Number of msec since last GPS Fix that was used
    uint32_t globalTimeStamp_ms; // time in msec of current prediction cycle

    // GPS input data variables
    double gpsLat;
    double gpsLon;
    float gpsHgt;
    uint8_t GPSstatus;

    // Baro input
    float baroHgt;

    bool statesInitialised;

    bool fuseVelData; // this boolean causes the posNE and velNED obs to be fused
    bool fusePosData; // this boolean causes the posNE and velNED obs to be fused
    bool fuseHgtData; // this boolean causes the hgtMea obs to be fused
    bool fuseMagData; // boolean true when magnetometer data is to be fused
    bool fuseVtasData; // boolean true when airspeed data is to be fused
    bool fuseRngData;   ///< true when range data is fused
    bool fuseOptFlowData; // true when optical flow data is fused

    bool inhibitWindStates; // true when wind states and covariances are to remain constant
    bool inhibitMagStates;  // true when magnetic field states and covariances are to remain constant
    bool inhibitGndState; // true when the terrain ground height offset state and covariances are to remain constant
    bool inhibitScaleState; // true when the focal length scale factor state and covariances are to remain constant

    bool staticMode;    ///< boolean true if no position feedback is fused
    bool useGPS; // boolean true if GPS data is being used
    bool useAirspeed;    ///< boolean true if airspeed data is being used
    bool useCompass;    ///< boolean true if magnetometer data is being used
    bool useRangeFinder;     ///< true when rangefinder is being used
    bool useOpticalFlow; // true when optical flow data is being used

    bool ekfDiverged;
    uint64_t lastReset;

    struct ekf_status_report current_ekf_state;
    struct ekf_status_report last_ekf_error;

    bool numericalProtection;

    unsigned storeIndex;

    // Optical Flow error estimation
    float storedOmega[3][EKF_DATA_BUFFER_SIZE]; // angular rate vector stored for the last 50 time steps used by optical flow eror estimators

    // Two state EKF used to estimate focal length scale factor and terrain position
    float Popt[2][2];                       // state covariance matrix
    float flowStates[2];                    // flow states [scale factor, terrain position]
    float prevPosN;                         // north position at last measurement
    float prevPosE;                         // east position at last measurement
    float auxFlowObsInnov[2];               // optical flow observation innovations from focal length scale factor estimator
    float auxFlowObsInnovVar[2];            // innovation variance for optical flow observations from focal length scale factor estimator
    float fScaleFactorVar;                  // optical flow sensor focal length scale factor variance
    Mat3f Tnb_flow;                         // Transformation matrix from nav to body at the time fo the optical flow measurement
    float R_LOS;                            // Optical flow observation noise variance (rad/sec)^2
    float auxFlowTestRatio[2];              // ratio of X and Y flow observation innovations to fault threshold
    float auxRngTestRatio;                  // ratio of range observation innovations to fault threshold
    float flowInnovGate;                    // number of standard deviations used for the innovation consistency check
    float auxFlowInnovGate;                 // number of standard deviations applied to the optical flow innovation consistency check
    float rngInnovGate;                     // number of standard deviations used for the innovation consistency check
    float minFlowRng;                       // minimum range over which to fuse optical flow measurements
    float moCompR_LOS;                      // scaler from sensor gyro rate to uncertainty in LOS rate

    void updateDtGpsFilt(float dt);

    void updateDtHgtFilt(float dt);

    void UpdateStrapdownEquationsNED();

    void CovariancePrediction(float dt);

    void FuseVelposNED();

    void FuseMagnetometer();

    void FuseAirspeed();

    void FuseOptFlow();

    /**
    * @brief
    *    Estimation of optical flow sensor focal length scale factor and terrain height using a two state EKF
    *    This fiter requires optical flow rates that are not motion compensated
    *    Range to ground measurement is assumed to be via a narrow beam type sensor - eg laser
    **/
    void OpticalFlowEKF();

    void zeroRows(float (&covMat)[EKF_STATE_ESTIMATES][EKF_STATE_ESTIMATES], uint8_t first, uint8_t last);

    void zeroCols(float (&covMat)[EKF_STATE_ESTIMATES][EKF_STATE_ESTIMATES], uint8_t first, uint8_t last);

    void quatNorm(float (&quatOut)[4], const float quatIn[4]);

    // store staes along with system time stamp in msces
    void StoreStates(uint64_t timestamp_ms);

    /**
     * Recall the state vector.
     *
     * Recalls the vector stored at closest time to the one specified by msec
     * @return zero on success, integer indicating the number of invalid states on failure.
     *         Does only copy valid states, if the statesForFusion vector was initialized
     *         correctly by the caller, the result can be safely used, but is a mixture
     *         time-wise where valid states were updated and invalid remained at the old
     *         value.
     */
    int RecallStates(float *statesForFusion, uint64_t msec);

    void RecallOmega(float *omegaForFusion, uint64_t msec);

    void quat2Tbn(Mat3f &TBodyNed, const float (&quat)[4]);

    void calcEarthRateNED(Vector3f &omega, float latitude);

    static void eul2quat(float (&quat)[4], const float (&eul)[3]);

    static void quat2eul(float (&eul)[3], const float (&quat)[4]);

    //static void quat2Tnb(Mat3f &Tnb, const float (&quat)[4]);

    static inline float sq(float valIn) {return valIn * valIn;}

    /**
    * @brief
    *   Tell the EKF if the vehicle has landed
    **/
    void setOnGround(const bool isLanded);

    void CovarianceInit();

    void InitialiseFilter(float (&initvelNED)[3], double referenceLat, double referenceLon, float referenceHgt, float declination);

    float ConstrainFloat(float val, float min, float maxf);

    void ConstrainVariances();

    void ConstrainStates();

    void ForceSymmetry();

    /**
    * @brief
    *   Check the filter inputs and bound its operational state
    *  
    *   This check will reset the filter states if required
    *   due to a failure of consistency or timeout checks.
    *   it should be run after the measurement data has been
    *   updated, but before any of the fusion steps are
    *   executed.
    */
    int CheckAndBound(struct ekf_status_report *last_error);

    /**
     * @brief
     *   Reset the filter position states
     *  
     *   This resets the position to the last GPS measurement
     *   or to zero in case of static position.
     */
    void ResetPosition();

    /**
     * @brief
     *   Reset the velocity state.
     */
    void ResetVelocity();

    void GetFilterState(struct ekf_status_report *state);

    void GetLastErrorState(struct ekf_status_report *last_error);

    bool StatesNaN();

    void InitializeDynamic(float (&initvelNED)[3], float declination);

    /**
    * @brief
    *   Tells the EKF wheter the vehicle is a fixed wing frame or not.
    *   This changes the way the EKF fuses position or attitude calulations
    *   by making some assumptions on movement.
    * @param fixedWing
    *   true if the vehicle moves like a Fixed Wing, false otherwise
    **/
    void setIsFixedWing(const bool fixedWing);
    
    /**
     * @brief
     *   Reset internal filter states and clear all variables to zero value
     */
    void ZeroVariables();

    void get_covariance(float c[28]);

protected:

    /**
    * @brief 
    *   Initializes algorithm parameters to safe default values
    **/
    void InitialiseParameters();

    void updateDtVelPosFilt(float dt);

    bool FilterHealthy();

    bool GyroOffsetsDiverged();

    bool VelNEDDiverged();

    /**
     * @brief
     *   Reset the height state.
     *  
     *   This resets the height state with the last altitude measurements
     */
    void ResetHeight();

    void AttitudeInit(float ax, float ay, float az, float mx, float my, float mz, float declination, float *initQuat);

    void ResetStoredStates();

private:
    bool _isFixedWing;               ///< True if the vehicle is a fixed-wing frame type
    bool _onGround;                  ///< boolean true when the flight vehicle is on the ground (not flying)
    float _accNavMagHorizontal;      ///< First-order low-pass filtered rate of change maneuver velocity
};

uint32_t millis();

uint64_t getMicros();

