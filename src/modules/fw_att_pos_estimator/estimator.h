#include <math.h>
#include <stdint.h>

#pragma once

#define GRAVITY_MSS 9.80665f
#define deg2rad 0.017453292f
#define rad2deg 57.295780f
#define pi 3.141592657f
#define earthRate 0.000072921f
#define earthRadius 6378145.0f
#define earthRadiusInv  1.5678540e-7f

class Vector3f
{
private:
public:
    float x;
    float y;
    float z;

    float length(void) const;
    Vector3f zero(void) const;
};

class Mat3f
{
private:
public:
    Vector3f x;
    Vector3f y;
    Vector3f z;

    Mat3f transpose(void) const;
};

Vector3f operator*(float sclIn1, Vector3f vecIn1);
Vector3f operator+( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator-( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator*( Mat3f matIn, Vector3f vecIn);
Vector3f operator%( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator*(Vector3f vecIn1, float sclIn1);

void swap_var(float &d1, float &d2);

const unsigned int n_states = 21;
const unsigned int data_buffer_size = 50;

extern uint32_t statetimeStamp[data_buffer_size]; // time stamp for each state vector stored
extern Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
extern Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
extern Vector3f summedDelAng; // summed delta angles about the xyz body axes corrected for errors (rad)
extern Vector3f summedDelVel; // summed delta velocities along the XYZ body axes corrected for errors (m/s)
extern float accNavMag; // magnitude of navigation accel (- used to adjust GPS obs variance (m/s^2)
extern Vector3f earthRateNED; // earths angular rate vector in NED (rad/s)
extern Vector3f angRate; // angular rate vector in XYZ body axes measured by the IMU (rad/s)
extern Vector3f accel; // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
extern Vector3f dVelIMU;
extern Vector3f dAngIMU;

extern float P[n_states][n_states]; // covariance matrix
extern float Kfusion[n_states]; // Kalman gains
extern float states[n_states]; // state matrix
extern float storedStates[n_states][data_buffer_size]; // state vectors stored for the last 50 time steps

extern Vector3f correctedDelAng; // delta angles about the xyz body axes corrected for errors (rad)
extern Vector3f correctedDelVel; // delta velocities along the XYZ body axes corrected for errors (m/s)
extern Vector3f summedDelAng; // summed delta angles about the xyz body axes corrected for errors (rad)
extern Vector3f summedDelVel; // summed delta velocities along the XYZ body axes corrected for errors (m/s)

extern float dt; // time lapsed since last covariance prediction
extern float dtIMU; // time lapsed since the last IMU measurement or covariance update (sec)

extern bool onGround; // boolean true when the flight vehicle is on the ground (not flying)
extern bool useAirspeed; // boolean true if airspeed data is being used
extern bool useCompass; // boolean true if magnetometer data is being used
extern uint8_t fusionModeGPS ; // 0 = GPS outputs 3D velocity, 1 = GPS outputs 2D velocity, 2 = GPS outputs no velocity
extern float innovVelPos[6]; // innovation output
extern float varInnovVelPos[6]; // innovation variance output

extern bool fuseVelData; // this boolean causes the posNE and velNED obs to be fused
extern bool fusePosData; // this boolean causes the posNE and velNED obs to be fused
extern bool fuseHgtData; // this boolean causes the hgtMea obs to be fused
extern bool fuseMagData; // boolean true when magnetometer data is to be fused

extern float velNED[3]; // North, East, Down velocity obs (m/s)
extern float posNE[2]; // North, East position obs (m)
extern float hgtMea; //  measured height (m)
extern float posNED[3]; // North, East Down position (m)

extern float statesAtVelTime[n_states]; // States at the effective measurement time for posNE and velNED measurements
extern float statesAtPosTime[n_states]; // States at the effective measurement time for posNE and velNED measurements
extern float statesAtHgtTime[n_states]; // States at the effective measurement time for the hgtMea measurement

extern float innovMag[3]; // innovation output
extern float varInnovMag[3]; // innovation variance output
extern Vector3f magData; // magnetometer flux radings in X,Y,Z body axes
extern float statesAtMagMeasTime[n_states]; // filter satates at the effective measurement time
extern float innovVtas; // innovation output
extern float varInnovVtas; // innovation variance output
extern bool fuseVtasData; // boolean true when airspeed data is to be fused
extern float VtasMeas; // true airspeed measurement (m/s)
extern float statesAtVtasMeasTime[n_states]; // filter states at the effective measurement time
extern float latRef; // WGS-84 latitude of reference point (rad)
extern float lonRef; // WGS-84 longitude of reference point (rad)
extern float hgtRef; // WGS-84 height of reference point (m)
extern Vector3f magBias; // states representing magnetometer bias vector in XYZ body axes
extern uint8_t covSkipCount; // Number of state prediction frames (IMU daya updates to skip before doing the covariance prediction
extern float EAS2TAS; // ratio f true to equivalent airspeed

// GPS input data variables
extern float gpsCourse;
extern float gpsGndSpd;
extern float gpsVelD;
extern float gpsLat;
extern float gpsLon;
extern float gpsHgt;
extern uint8_t GPSstatus;

// Baro input
extern float baroHgt;

extern bool statesInitialised;

const float covTimeStepMax = 0.07f; // maximum time allowed between covariance predictions
const float covDelAngMax = 0.05f; // maximum delta angle between covariance predictions

void  UpdateStrapdownEquationsNED();

void CovariancePrediction();

void FuseVelposNED();

void FuseMagnetometer();

void FuseAirspeed();

void zeroRows(float covMat[n_states][n_states], uint8_t first, uint8_t last);

void zeroCols(float covMat[n_states][n_states], uint8_t first, uint8_t last);

float sq(float valIn);

void quatNorm(float quatOut[4], float quatIn[4]);

// store staes along with system time stamp in msces
void StoreStates();

// recall stste vector stored at closest time to the one specified by msec
void RecallStates(float statesForFusion[n_states], uint32_t msec);

void quat2Tnb(Mat3f &Tnb, float quat[4]);

void quat2Tbn(Mat3f &Tbn, float quat[4]);

void calcEarthRateNED(Vector3f &omega, float latitude);

void eul2quat(float quat[4], float eul[3]);

void quat2eul(float eul[3],float quat[4]);

void calcvelNED(float velNED[3], float gpsCourse, float gpsGndSpd, float gpsVelD);

void calcposNED(float posNED[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef);

void calcLLH(float posNED[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef);

void OnGroundCheck();

void CovarianceInit();

void InitialiseFilter();

uint32_t millis();

