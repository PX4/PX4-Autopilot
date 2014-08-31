#include <math.h>
#include <stdint.h>

#pragma once

#define GRAVITY_MSS 9.80665f
#define deg2rad 0.017453292f
#define rad2deg 57.295780f
#define pi 3.141592657f
#define earthRate 0.000072921f
#define earthRadius 6378145.0
#define earthRadiusInv  1.5678540e-7

class Vector3f
{
private:
public:
    float x;
    float y;
    float z;

    Vector3f(float a=0.0f, float b=0.0f, float c=0.0f) :
    x(a),
    y(b),
    z(c)
    {}

    float length(void) const;
    void zero(void);
};

class Mat3f
{
private:
public:
    Vector3f x;
    Vector3f y;
    Vector3f z;

    Mat3f();

    void identity();
    Mat3f transpose(void) const;
};

Vector3f operator*(float sclIn1, Vector3f vecIn1);
Vector3f operator+( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator-( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator*( Mat3f matIn, Vector3f vecIn);
Mat3f operator*( Mat3f matIn1, Mat3f matIn2);
Vector3f operator%( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator*(Vector3f vecIn1, float sclIn1);

void swap_var(float &d1, float &d2);

enum GPS_FIX {
    GPS_FIX_NOFIX = 0,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3
};

struct ekf_status_report {
    bool error;
    bool velHealth;
    bool posHealth;
    bool hgtHealth;
    bool velTimeout;
    bool posTimeout;
    bool hgtTimeout;
    bool imuTimeout;
    bool onGround;
    bool staticMode;
    bool useCompass;
    bool useAirspeed;
    uint32_t velFailTime;
    uint32_t posFailTime;
    uint32_t hgtFailTime;
    float states[32];
    unsigned n_states;
    bool angNaN;
    bool summedDelVelNaN;
    bool KHNaN;
    bool KHPNaN;
    bool PNaN;
    bool covarianceNaN;
    bool kalmanGainsNaN;
    bool statesNaN;
    bool gyroOffsetsExcessive;
    bool covariancesExcessive;
    bool velOffsetExcessive;
};

void ekf_debug(const char *fmt, ...);
