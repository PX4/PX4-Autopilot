#pragma once

#include "estimator_utilities.h"

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
     * the InitialiseParameters() (which is just 20 lines down)
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

    void InitialiseParameters()
    {
        covTimeStepMax = 0.07f; // maximum time allowed between covariance predictions
        covDelAngMax = 0.02f; // maximum delta angle between covariance predictions
        rngFinderPitch = 0.0f; // pitch angle of laser range finder in radians. Zero is aligned with the Z body axis. Positive is RH rotation about Y body axis.
        EAS2TAS = 1.0f;

        yawVarScale = 1.0f;
        windVelSigma = 0.1f;
        dAngBiasSigma = 5.0e-7f;
        dVelBiasSigma = 1e-4f;
        magEarthSigma = 3.0e-4f;
        magBodySigma  = 3.0e-4f;
        gndHgtSigma  = 0.02f; // assume 2% terrain gradient 1-sigma

        vneSigma = 0.2f;
        vdSigma = 0.3f;
        posNeSigma = 2.0f;
        posDSigma = 2.0f;

        magMeasurementSigma = 0.05;
        airspeedMeasurementSigma = 1.4f;
        gyroProcessNoise = 1.4544411e-2f;
        accelProcessNoise = 0.5f;
    }

    // Global variables
    float KH[n_states][n_states]; //  intermediate result used for covariance updates
    float KHP[n_states][n_states]; // intermediate result used for covariance updates
    float P[n_states][n_states]; // covariance matrix
    float Kfusion[n_states]; // Kalman gains
    float states[n_states]; // state matrix
    float storedStates[n_states][data_buffer_size]; // state vectors stored for the last 50 time steps
    uint32_t statetimeStamp[data_buffer_size]; // time stamp for each state vector stored

    float statesAtVelTime[n_states]; // States at the effective measurement time for posNE and velNED measurements
    float statesAtPosTime[n_states]; // States at the effective measurement time for posNE and velNED measurements
    float statesAtHgtTime[n_states]; // States at the effective measurement time for the hgtMea measurement
    float statesAtMagMeasTime[n_states]; // filter satates at the effective measurement time
    float statesAtVtasMeasTime[n_states]; // filter states at the effective measurement time

    Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
    Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
    Vector3f summedDelAng; // summed delta angles about the xyz body axes corrected for errors (rad)
    Vector3f summedDelVel; // summed delta velocities along the XYZ body axes corrected for errors (m/s)
    float accNavMag; // magnitude of navigation accel (- used to adjust GPS obs variance (m/s^2)
    Vector3f earthRateNED; // earths angular rate vector in NED (rad/s)
    Vector3f angRate; // angular rate vector in XYZ body axes measured by the IMU (rad/s)
    Vector3f accel; // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f dVelIMU;
    Vector3f dAngIMU;
    float dtIMU; // time lapsed since the last IMU measurement or covariance update (sec)
    uint8_t fusionModeGPS; // 0 = GPS outputs 3D velocity, 1 = GPS outputs 2D velocity, 2 = GPS outputs no velocity
    float innovVelPos[6]; // innovation output
    float varInnovVelPos[6]; // innovation variance output

    float velNED[3]; // North, East, Down velocity obs (m/s)
    float posNE[2]; // North, East position obs (m)
    float hgtMea; //  measured height (m)
    float posNED[3]; // North, East Down position (m)

    float innovMag[3]; // innovation output
    float varInnovMag[3]; // innovation variance output
    Vector3f magData; // magnetometer flux radings in X,Y,Z body axes
    float innovVtas; // innovation output
    float varInnovVtas; // innovation variance output
    float VtasMeas; // true airspeed measurement (m/s)
    float magDeclination;
    float latRef; // WGS-84 latitude of reference point (rad)
    float lonRef; // WGS-84 longitude of reference point (rad)
    float hgtRef; // WGS-84 height of reference point (m)
    Vector3f magBias; // states representing magnetometer bias vector in XYZ body axes
    uint8_t covSkipCount; // Number of state prediction frames (IMU daya updates to skip before doing the covariance prediction

    // GPS input data variables
    float gpsCourse;
    float gpsVelD;
    float gpsLat;
    float gpsLon;
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

    bool onGround;    ///< boolean true when the flight vehicle is on the ground (not flying)
    bool staticMode;    ///< boolean true if no position feedback is fused
    bool useAirspeed;    ///< boolean true if airspeed data is being used
    bool useCompass;    ///< boolean true if magnetometer data is being used

    struct ekf_status_report current_ekf_state;
    struct ekf_status_report last_ekf_error;

    bool numericalProtection;

    unsigned storeIndex;


void  UpdateStrapdownEquationsNED();

void CovariancePrediction(float dt);

void FuseVelposNED();

void FuseMagnetometer();

void FuseAirspeed();

void zeroRows(float (&covMat)[n_states][n_states], uint8_t first, uint8_t last);

void zeroCols(float (&covMat)[n_states][n_states], uint8_t first, uint8_t last);

void quatNorm(float (&quatOut)[4], const float quatIn[4]);

// store staes along with system time stamp in msces
void StoreStates(uint64_t timestamp_ms);

/**
 * Recall the state vector.
 *
 * Recalls the vector stored at closest time to the one specified by msec
 *
 * @return zero on success, integer indicating the number of invalid states on failure.
 *         Does only copy valid states, if the statesForFusion vector was initialized
 *         correctly by the caller, the result can be safely used, but is a mixture
 *         time-wise where valid states were updated and invalid remained at the old
 *         value.
 */
int RecallStates(float statesForFusion[n_states], uint64_t msec);

void ResetStoredStates();

void quat2Tbn(Mat3f &Tbn, const float (&quat)[4]);

void calcEarthRateNED(Vector3f &omega, float latitude);

static void eul2quat(float (&quat)[4], const float (&eul)[3]);

static void quat2eul(float (&eul)[3], const float (&quat)[4]);

static void calcvelNED(float (&velNED)[3], float gpsCourse, float gpsGndSpd, float gpsVelD);

static void calcposNED(float (&posNED)[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef);

static void calcLLH(float (&posNED)[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef);

static void quat2Tnb(Mat3f &Tnb, const float (&quat)[4]);

static float sq(float valIn);

void OnGroundCheck();

void CovarianceInit();

void InitialiseFilter(float (&initvelNED)[3], double referenceLat, double referenceLon, float referenceHgt, float declination);

float ConstrainFloat(float val, float min, float max);

void ConstrainVariances();

void ConstrainStates();

void ForceSymmetry();

int CheckAndBound();

void ResetPosition();

void ResetVelocity();

void ZeroVariables();

void GetFilterState(struct ekf_status_report *state);

void GetLastErrorState(struct ekf_status_report *last_error);

bool StatesNaN(struct ekf_status_report *err_report);
void FillErrorReport(struct ekf_status_report *err);

void InitializeDynamic(float (&initvelNED)[3], float declination);

protected:

bool FilterHealthy();

void ResetHeight(void);

void AttitudeInit(float ax, float ay, float az, float mx, float my, float mz, float declination, float *initQuat);

};

uint32_t millis();

