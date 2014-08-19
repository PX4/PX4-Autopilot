#include "estimator_23states.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define EKF_COVARIANCE_DIVERGED 1.0e8f

AttPosEKF::AttPosEKF() :
    covTimeStepMax(0.0f),
    covDelAngMax(0.0f),
    rngFinderPitch(0.0f),
    a1(0.0f),
    a2(0.0f),
    a3(0.0f),
    yawVarScale(0.0f),
    windVelSigma(0.0f),
    dAngBiasSigma(0.0f),
    dVelBiasSigma(0.0f),
    magEarthSigma(0.0f),
    magBodySigma(0.0f),
    gndHgtSigma(0.0f),
    vneSigma(0.0f),
    vdSigma(0.0f),
    posNeSigma(0.0f),
    posDSigma(0.0f),
    magMeasurementSigma(0.0f),
    airspeedMeasurementSigma(0.0f),
    gyroProcessNoise(0.0f),
    accelProcessNoise(0.0f),
    EAS2TAS(1.0f),
    magstate{},
    resetMagState{},
    KH{},
    KHP{},
    P{},
    Kfusion{},
    states{},
    resetStates{},
    storedStates{},
    statetimeStamp{},
    statesAtVelTime{},
    statesAtPosTime{},
    statesAtHgtTime{},
    statesAtMagMeasTime{},
    statesAtVtasMeasTime{},
    statesAtRngTime{},
    statesAtOptFlowTime{},
    correctedDelAng(),
    correctedDelVel(),
    summedDelAng(),
    summedDelVel(),
    accNavMag(),
    earthRateNED(),
    angRate(),
    lastGyroOffset(),
    delAngTotal(),
    Tbn(),
    Tnb(),
    accel(),
    dVelIMU(),
    dAngIMU(),
    dtIMU(0),
    fusionModeGPS(0),
    innovVelPos{},
    varInnovVelPos{},
    velNED{},
    accelGPSNED{},
    posNE{},
    hgtMea(0.0f),
    baroHgtOffset(0.0f),
    rngMea(0.0f),
    innovMag{},
    varInnovMag{},
    magData{},
    losData{},
    innovVtas(0.0f),
    innovRng(0.0f),
    innovOptFlow{},
    varInnovOptFlow{},
    varInnovVtas(0.0f),
    varInnovRng(0.0f),
    VtasMeas(0.0f),
    magDeclination(0.0f),
    latRef(0.0f),
    lonRef(-M_PI_F),
    hgtRef(0.0f),
    refSet(false),
    magBias(),
    covSkipCount(0),
    gpsLat(0.0),
    gpsLon(-M_PI),
    gpsHgt(0.0f),
    GPSstatus(0),
    baroHgt(0.0f),
    statesInitialised(false),
    fuseVelData(false),
    fusePosData(false),
    fuseHgtData(false),
    fuseMagData(false),
    fuseVtasData(false),
    fuseRngData(false),
    fuseOptFlowData(false),

    inhibitWindStates(true),
    inhibitMagStates(true),
    inhibitGndHgtState(true),

    onGround(true),
    staticMode(true),
    useAirspeed(true),
    useCompass(true),
    useRangeFinder(false),
    useOpticalFlow(false),

    ekfDiverged(false),
    lastReset(0),
    current_ekf_state{},
    last_ekf_error{},
    numericalProtection(true),
    storeIndex(0)
{
    memset(&last_ekf_error, 0, sizeof(last_ekf_error));
    memset(&current_ekf_state, 0, sizeof(current_ekf_state));
    ZeroVariables();
    InitialiseParameters();
}

AttPosEKF::~AttPosEKF()
{
}

void AttPosEKF::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;
    float q00;
    float q11;
    float q22;
    float q33;
    float q01;
    float q02;
    float q03;
    float q12;
    float q13;
    float q23;
    float rotationMag;
    float qUpdated[4];
    float quatMag;
    float deltaQuat[4];
    const Vector3f gravityNED(0.0f, 0.0f, GRAVITY_MSS);

// Remove sensor bias errors
    correctedDelAng.x = dAngIMU.x - states[10];
    correctedDelAng.y = dAngIMU.y - states[11];
    correctedDelAng.z = dAngIMU.z - states[12];
    dVelIMU.x = dVelIMU.x;
    dVelIMU.y = dVelIMU.y;
    dVelIMU.z = dVelIMU.z - states[13];

    delAngTotal.x += correctedDelAng.x;
    delAngTotal.y += correctedDelAng.y;
    delAngTotal.z += correctedDelAng.z;

// Save current measurements
    Vector3f  prevDelAng = correctedDelAng;

// Apply corrections for earths rotation rate and coning errors
// * and + operators have been overloaded
    correctedDelAng   = correctedDelAng - Tnb*earthRateNED*dtIMU + 8.333333333333333e-2f*(prevDelAng % correctedDelAng);

// Convert the rotation vector to its equivalent quaternion
    rotationMag = correctedDelAng.length();
    if (rotationMag < 1e-12f)
    {
        deltaQuat[0] = 1.0;
        deltaQuat[1] = 0.0;
        deltaQuat[2] = 0.0;
        deltaQuat[3] = 0.0;
    }
    else
    {
        // We are using double here as we are unsure how small
        // the angle differences are and if we get into numeric
        // issues with float. The runtime impact is not measurable
        // for these quantities.
        deltaQuat[0] = cos(0.5*(double)rotationMag);
        float rotScaler = (sin(0.5*(double)rotationMag))/(double)rotationMag;
        deltaQuat[1] = correctedDelAng.x*rotScaler;
        deltaQuat[2] = correctedDelAng.y*rotScaler;
        deltaQuat[3] = correctedDelAng.z*rotScaler;
    }

// Update the quaternions by rotating from the previous attitude through
// the delta angle rotation quaternion
    qUpdated[0] = states[0]*deltaQuat[0] - states[1]*deltaQuat[1] - states[2]*deltaQuat[2] - states[3]*deltaQuat[3];
    qUpdated[1] = states[0]*deltaQuat[1] + states[1]*deltaQuat[0] + states[2]*deltaQuat[3] - states[3]*deltaQuat[2];
    qUpdated[2] = states[0]*deltaQuat[2] + states[2]*deltaQuat[0] + states[3]*deltaQuat[1] - states[1]*deltaQuat[3];
    qUpdated[3] = states[0]*deltaQuat[3] + states[3]*deltaQuat[0] + states[1]*deltaQuat[2] - states[2]*deltaQuat[1];

// Normalise the quaternions and update the quaternion states
    quatMag = sqrtf(sq(qUpdated[0]) + sq(qUpdated[1]) + sq(qUpdated[2]) + sq(qUpdated[3]));
    if (quatMag > 1e-16f)
    {
        float quatMagInv = 1.0f/quatMag;
        states[0] = quatMagInv*qUpdated[0];
        states[1] = quatMagInv*qUpdated[1];
        states[2] = quatMagInv*qUpdated[2];
        states[3] = quatMagInv*qUpdated[3];
    }

// Calculate the body to nav cosine matrix
    q00 = sq(states[0]);
    q11 = sq(states[1]);
    q22 = sq(states[2]);
    q33 = sq(states[3]);
    q01 =  states[0]*states[1];
    q02 =  states[0]*states[2];
    q03 =  states[0]*states[3];
    q12 =  states[1]*states[2];
    q13 =  states[1]*states[3];
    q23 =  states[2]*states[3];

    Tbn.x.x = q00 + q11 - q22 - q33;
    Tbn.y.y = q00 - q11 + q22 - q33;
    Tbn.z.z = q00 - q11 - q22 + q33;
    Tbn.x.y = 2*(q12 - q03);
    Tbn.x.z = 2*(q13 + q02);
    Tbn.y.x = 2*(q12 + q03);
    Tbn.y.z = 2*(q23 - q01);
    Tbn.z.x = 2*(q13 - q02);
    Tbn.z.y = 2*(q23 + q01);

    Tnb = Tbn.transpose();

// transform body delta velocities to delta velocities in the nav frame
// * and + operators have been overloaded
    //delVelNav = Tbn*dVelIMU + gravityNED*dtIMU;
    delVelNav.x = Tbn.x.x*dVelIMU.x + Tbn.x.y*dVelIMU.y + Tbn.x.z*dVelIMU.z + gravityNED.x*dtIMU;
    delVelNav.y = Tbn.y.x*dVelIMU.x + Tbn.y.y*dVelIMU.y + Tbn.y.z*dVelIMU.z + gravityNED.y*dtIMU;
    delVelNav.z = Tbn.z.x*dVelIMU.x + Tbn.z.y*dVelIMU.y + Tbn.z.z*dVelIMU.z + gravityNED.z*dtIMU;

// calculate the magnitude of the nav acceleration (required for GPS
// variance estimation)
    accNavMag = delVelNav.length()/dtIMU;

// If calculating position save previous velocity
    float lastVelocity[3];
    lastVelocity[0] = states[4];
    lastVelocity[1] = states[5];
    lastVelocity[2] = states[6];

// Sum delta velocities to get velocity
    states[4] = states[4] + delVelNav.x;
    states[5] = states[5] + delVelNav.y;
    states[6] = states[6] + delVelNav.z;

// If calculating postions, do a trapezoidal integration for position
    states[7] = states[7] + 0.5f*(states[4] + lastVelocity[0])*dtIMU;
    states[8] = states[8] + 0.5f*(states[5] + lastVelocity[1])*dtIMU;
    states[9] = states[9] + 0.5f*(states[6] + lastVelocity[2])*dtIMU;

    // Constrain states (to protect against filter divergence)
    ConstrainStates();
}

void AttPosEKF::CovariancePrediction(float dt)
{
    // scalars
    float daxCov;
    float dayCov;
    float dazCov;
    float dvxCov;
    float dvyCov;
    float dvzCov;
    float dvx;
    float dvy;
    float dvz;
    float dax;
    float day;
    float daz;
    float q0;
    float q1;
    float q2;
    float q3;
    float dax_b;
    float day_b;
    float daz_b;
    float dvz_b;

    // arrays
    float processNoise[n_states];
    float SF[15];
    float SG[8];
    float SQ[11];
    float SPP[8] = {0};
    float nextP[n_states][n_states];

    // calculate covariance prediction process noise
    for (uint8_t i= 0; i<4;  i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i= 4; i<10;  i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dt * dAngBiasSigma;
    // scale gyro bias noise when on ground to allow for faster bias estimation
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dt * dAngBiasSigma;
    processNoise[13] = dVelBiasSigma;
    if (!inhibitWindStates) {
        for (uint8_t i=14; i<=15; i++) processNoise[i] = dt * windVelSigma;
    } else {
        for (uint8_t i=14; i<=15; i++) processNoise[i] = 0;
    }
    if (!inhibitMagStates) {
        for (uint8_t i=16; i<=18; i++) processNoise[i] = dt * magEarthSigma;
        for (uint8_t i=19; i<=21; i++) processNoise[i] = dt * magBodySigma;
    } else {
        for (uint8_t i=16; i<=21; i++) processNoise[i] = 0;
    }
    if (!inhibitGndHgtState) {
        processNoise[22] = dt * sqrtf(sq(states[4]) + sq(states[5])) * gndHgtSigma;
    } else {
        processNoise[22] = 0;
    }

    // square all sigmas
    for (unsigned i = 0; i < n_states; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = states[0];
    q1 = states[1];
    q2 = states[2];
    q3 = states[3];
    dax_b = states[10];
    day_b = states[11];
    daz_b = states[12];
    dvz_b =  states[13];
    gyroProcessNoise = ConstrainFloat(gyroProcessNoise, 1e-3f, 5e-2f);
    daxCov = sq(dt*gyroProcessNoise);
    dayCov = sq(dt*gyroProcessNoise);
    dazCov = sq(dt*gyroProcessNoise);
    if (onGround) dazCov = dazCov * sq(yawVarScale);
    accelProcessNoise = ConstrainFloat(accelProcessNoise, 5e-2, 1.0f);
    dvxCov = sq(dt*accelProcessNoise);
    dvyCov = sq(dt*accelProcessNoise);
    dvzCov = sq(dt*accelProcessNoise);

    // Predicted covariance calculation
    SF[0] = dvz - dvz_b;
    SF[1] = 2*q3*SF[0] + 2*dvx*q1 + 2*dvy*q2;
    SF[2] = 2*dvx*q3 - 2*q1*SF[0] + 2*dvy*q0;
    SF[3] = 2*q2*SF[0] + 2*dvx*q0 - 2*dvy*q3;
    SF[4] = day/2 - day_b/2;
    SF[5] = daz/2 - daz_b/2;
    SF[6] = dax/2 - dax_b/2;
    SF[7] = dax_b/2 - dax/2;
    SF[8] = daz_b/2 - daz/2;
    SF[9] = day_b/2 - day/2;
    SF[10] = 2*q0*SF[0];
    SF[11] = q1/2;
    SF[12] = q2/2;
    SF[13] = q3/2;
    SF[14] = 2*dvy*q1;

    SG[0] = q0/2;
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);
    SG[5] = 2*q2*q3;
    SG[6] = 2*q1*q3;
    SG[7] = 2*q1*q2;

    SQ[0] = dvzCov*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyCov*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxCov*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
    SQ[1] = dvzCov*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxCov*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyCov*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
    SQ[2] = dvzCov*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyCov*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxCov*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
    SQ[3] = (dayCov*q1*SG[0])/2 - (dazCov*q1*SG[0])/2 - (daxCov*q2*q3)/4;
    SQ[4] = (dazCov*q2*SG[0])/2 - (daxCov*q2*SG[0])/2 - (dayCov*q1*q3)/4;
    SQ[5] = (daxCov*q3*SG[0])/2 - (dayCov*q3*SG[0])/2 - (dazCov*q1*q2)/4;
    SQ[6] = (daxCov*q1*q2)/4 - (dazCov*q3*SG[0])/2 - (dayCov*q1*q2)/4;
    SQ[7] = (dazCov*q1*q3)/4 - (daxCov*q1*q3)/4 - (dayCov*q2*SG[0])/2;
    SQ[8] = (dayCov*q2*q3)/4 - (daxCov*q1*SG[0])/2 - (dazCov*q2*q3)/4;
    SQ[9] = sq(SG[0]);
    SQ[10] = sq(q1);

    SPP[0] = SF[10] + SF[14] - 2*dvx*q2;
    SPP[1] = 2*q2*SF[0] + 2*dvx*q0 - 2*dvy*q3;
    SPP[2] = 2*dvx*q3 - 2*q1*SF[0] + 2*dvy*q0;
    SPP[3] = 2*q0*q1 - 2*q2*q3;
    SPP[4] = 2*q0*q2 + 2*q1*q3;
    SPP[5] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    SPP[6] = SF[13];
    SPP[7] = SF[12];

    nextP[0][0] = P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6] + (daxCov*SQ[10])/4 + SF[7]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[9]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[8]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SF[11]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) + SPP[7]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) + SPP[6]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) + (dayCov*sq(q2))/4 + (dazCov*sq(q3))/4;
    nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6] + SF[6]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[5]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[9]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SPP[6]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) - SPP[7]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) - (q0*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]))/2;
    nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6] + SF[4]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[8]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[6]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SF[11]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) - SPP[6]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) - (q0*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]))/2;
    nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6] + SF[5]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[4]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[7]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - SF[11]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) + SPP[7]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) - (q0*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]))/2;
    nextP[0][4] = P[0][4] + P[1][4]*SF[7] + P[2][4]*SF[9] + P[3][4]*SF[8] + P[10][4]*SF[11] + P[11][4]*SPP[7] + P[12][4]*SPP[6] + SF[3]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[1]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SPP[0]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - SPP[2]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) - SPP[4]*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
    nextP[0][5] = P[0][5] + P[1][5]*SF[7] + P[2][5]*SF[9] + P[3][5]*SF[8] + P[10][5]*SF[11] + P[11][5]*SPP[7] + P[12][5]*SPP[6] + SF[2]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[1]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[3]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) - SPP[0]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SPP[3]*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
    nextP[0][6] = P[0][6] + P[1][6]*SF[7] + P[2][6]*SF[9] + P[3][6]*SF[8] + P[10][6]*SF[11] + P[11][6]*SPP[7] + P[12][6]*SPP[6] + SF[2]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[1]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SPP[0]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) - SPP[1]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
    nextP[0][7] = P[0][7] + P[1][7]*SF[7] + P[2][7]*SF[9] + P[3][7]*SF[8] + P[10][7]*SF[11] + P[11][7]*SPP[7] + P[12][7]*SPP[6] + dt*(P[0][4] + P[1][4]*SF[7] + P[2][4]*SF[9] + P[3][4]*SF[8] + P[10][4]*SF[11] + P[11][4]*SPP[7] + P[12][4]*SPP[6]);
    nextP[0][8] = P[0][8] + P[1][8]*SF[7] + P[2][8]*SF[9] + P[3][8]*SF[8] + P[10][8]*SF[11] + P[11][8]*SPP[7] + P[12][8]*SPP[6] + dt*(P[0][5] + P[1][5]*SF[7] + P[2][5]*SF[9] + P[3][5]*SF[8] + P[10][5]*SF[11] + P[11][5]*SPP[7] + P[12][5]*SPP[6]);
    nextP[0][9] = P[0][9] + P[1][9]*SF[7] + P[2][9]*SF[9] + P[3][9]*SF[8] + P[10][9]*SF[11] + P[11][9]*SPP[7] + P[12][9]*SPP[6] + dt*(P[0][6] + P[1][6]*SF[7] + P[2][6]*SF[9] + P[3][6]*SF[8] + P[10][6]*SF[11] + P[11][6]*SPP[7] + P[12][6]*SPP[6]);
    nextP[0][10] = P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6];
    nextP[0][11] = P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6];
    nextP[0][12] = P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6];
    nextP[0][13] = P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6];
    nextP[0][14] = P[0][14] + P[1][14]*SF[7] + P[2][14]*SF[9] + P[3][14]*SF[8] + P[10][14]*SF[11] + P[11][14]*SPP[7] + P[12][14]*SPP[6];
    nextP[0][15] = P[0][15] + P[1][15]*SF[7] + P[2][15]*SF[9] + P[3][15]*SF[8] + P[10][15]*SF[11] + P[11][15]*SPP[7] + P[12][15]*SPP[6];
    nextP[0][16] = P[0][16] + P[1][16]*SF[7] + P[2][16]*SF[9] + P[3][16]*SF[8] + P[10][16]*SF[11] + P[11][16]*SPP[7] + P[12][16]*SPP[6];
    nextP[0][17] = P[0][17] + P[1][17]*SF[7] + P[2][17]*SF[9] + P[3][17]*SF[8] + P[10][17]*SF[11] + P[11][17]*SPP[7] + P[12][17]*SPP[6];
    nextP[0][18] = P[0][18] + P[1][18]*SF[7] + P[2][18]*SF[9] + P[3][18]*SF[8] + P[10][18]*SF[11] + P[11][18]*SPP[7] + P[12][18]*SPP[6];
    nextP[0][19] = P[0][19] + P[1][19]*SF[7] + P[2][19]*SF[9] + P[3][19]*SF[8] + P[10][19]*SF[11] + P[11][19]*SPP[7] + P[12][19]*SPP[6];
    nextP[0][20] = P[0][20] + P[1][20]*SF[7] + P[2][20]*SF[9] + P[3][20]*SF[8] + P[10][20]*SF[11] + P[11][20]*SPP[7] + P[12][20]*SPP[6];
    nextP[0][21] = P[0][21] + P[1][21]*SF[7] + P[2][21]*SF[9] + P[3][21]*SF[8] + P[10][21]*SF[11] + P[11][21]*SPP[7] + P[12][21]*SPP[6];
    nextP[0][22] = P[0][22] + P[1][22]*SF[7] + P[2][22]*SF[9] + P[3][22]*SF[8] + P[10][22]*SF[11] + P[11][22]*SPP[7] + P[12][22]*SPP[6];
    nextP[1][0] = P[1][0] + SQ[8] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2 + SF[7]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[9]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[8]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SF[11]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) + SPP[7]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) + SPP[6]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2);
    nextP[1][1] = P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] + daxCov*SQ[9] - (P[10][1]*q0)/2 + SF[6]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[5]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[9]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SPP[6]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) - SPP[7]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2) + (dayCov*sq(q3))/4 + (dazCov*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2))/2;
    nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[8]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[6]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SF[11]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2) - SPP[6]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2))/2;
    nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[4]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[7]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - SF[11]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) + SPP[7]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2))/2;
    nextP[1][4] = P[1][4] + P[0][4]*SF[6] + P[2][4]*SF[5] + P[3][4]*SF[9] + P[11][4]*SPP[6] - P[12][4]*SPP[7] - (P[10][4]*q0)/2 + SF[3]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[1]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - SPP[2]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) - SPP[4]*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
    nextP[1][5] = P[1][5] + P[0][5]*SF[6] + P[2][5]*SF[5] + P[3][5]*SF[9] + P[11][5]*SPP[6] - P[12][5]*SPP[7] - (P[10][5]*q0)/2 + SF[2]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[1]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[3]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SPP[3]*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
    nextP[1][6] = P[1][6] + P[0][6]*SF[6] + P[2][6]*SF[5] + P[3][6]*SF[9] + P[11][6]*SPP[6] - P[12][6]*SPP[7] - (P[10][6]*q0)/2 + SF[2]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[1]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) - SPP[1]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
    nextP[1][7] = P[1][7] + P[0][7]*SF[6] + P[2][7]*SF[5] + P[3][7]*SF[9] + P[11][7]*SPP[6] - P[12][7]*SPP[7] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[6] + P[2][4]*SF[5] + P[3][4]*SF[9] + P[11][4]*SPP[6] - P[12][4]*SPP[7] - (P[10][4]*q0)/2);
    nextP[1][8] = P[1][8] + P[0][8]*SF[6] + P[2][8]*SF[5] + P[3][8]*SF[9] + P[11][8]*SPP[6] - P[12][8]*SPP[7] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[6] + P[2][5]*SF[5] + P[3][5]*SF[9] + P[11][5]*SPP[6] - P[12][5]*SPP[7] - (P[10][5]*q0)/2);
    nextP[1][9] = P[1][9] + P[0][9]*SF[6] + P[2][9]*SF[5] + P[3][9]*SF[9] + P[11][9]*SPP[6] - P[12][9]*SPP[7] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[6] + P[2][6]*SF[5] + P[3][6]*SF[9] + P[11][6]*SPP[6] - P[12][6]*SPP[7] - (P[10][6]*q0)/2);
    nextP[1][10] = P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2;
    nextP[1][11] = P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2;
    nextP[1][12] = P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2;
    nextP[1][13] = P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2;
    nextP[1][14] = P[1][14] + P[0][14]*SF[6] + P[2][14]*SF[5] + P[3][14]*SF[9] + P[11][14]*SPP[6] - P[12][14]*SPP[7] - (P[10][14]*q0)/2;
    nextP[1][15] = P[1][15] + P[0][15]*SF[6] + P[2][15]*SF[5] + P[3][15]*SF[9] + P[11][15]*SPP[6] - P[12][15]*SPP[7] - (P[10][15]*q0)/2;
    nextP[1][16] = P[1][16] + P[0][16]*SF[6] + P[2][16]*SF[5] + P[3][16]*SF[9] + P[11][16]*SPP[6] - P[12][16]*SPP[7] - (P[10][16]*q0)/2;
    nextP[1][17] = P[1][17] + P[0][17]*SF[6] + P[2][17]*SF[5] + P[3][17]*SF[9] + P[11][17]*SPP[6] - P[12][17]*SPP[7] - (P[10][17]*q0)/2;
    nextP[1][18] = P[1][18] + P[0][18]*SF[6] + P[2][18]*SF[5] + P[3][18]*SF[9] + P[11][18]*SPP[6] - P[12][18]*SPP[7] - (P[10][18]*q0)/2;
    nextP[1][19] = P[1][19] + P[0][19]*SF[6] + P[2][19]*SF[5] + P[3][19]*SF[9] + P[11][19]*SPP[6] - P[12][19]*SPP[7] - (P[10][19]*q0)/2;
    nextP[1][20] = P[1][20] + P[0][20]*SF[6] + P[2][20]*SF[5] + P[3][20]*SF[9] + P[11][20]*SPP[6] - P[12][20]*SPP[7] - (P[10][20]*q0)/2;
    nextP[1][21] = P[1][21] + P[0][21]*SF[6] + P[2][21]*SF[5] + P[3][21]*SF[9] + P[11][21]*SPP[6] - P[12][21]*SPP[7] - (P[10][21]*q0)/2;
    nextP[1][22] = P[1][22] + P[0][22]*SF[6] + P[2][22]*SF[5] + P[3][22]*SF[9] + P[11][22]*SPP[6] - P[12][22]*SPP[7] - (P[10][22]*q0)/2;
    nextP[2][0] = P[2][0] + SQ[7] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2 + SF[7]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[9]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[8]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SF[11]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) + SPP[7]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) + SPP[6]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2);
    nextP[2][1] = P[2][1] + SQ[5] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2 + SF[6]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[5]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[9]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SPP[6]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) - SPP[7]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2) - (q0*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2))/2;
    nextP[2][2] = P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] + dayCov*SQ[9] + (dazCov*SQ[10])/4 - (P[11][2]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[8]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[6]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SF[11]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2) - SPP[6]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) + (daxCov*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2))/2;
    nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[4]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[7]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - SF[11]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) + SPP[7]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2))/2;
    nextP[2][4] = P[2][4] + P[0][4]*SF[4] + P[1][4]*SF[8] + P[3][4]*SF[6] + P[12][4]*SF[11] - P[10][4]*SPP[6] - (P[11][4]*q0)/2 + SF[3]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[1]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - SPP[2]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) - SPP[4]*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
    nextP[2][5] = P[2][5] + P[0][5]*SF[4] + P[1][5]*SF[8] + P[3][5]*SF[6] + P[12][5]*SF[11] - P[10][5]*SPP[6] - (P[11][5]*q0)/2 + SF[2]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[1]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[3]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SPP[3]*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
    nextP[2][6] = P[2][6] + P[0][6]*SF[4] + P[1][6]*SF[8] + P[3][6]*SF[6] + P[12][6]*SF[11] - P[10][6]*SPP[6] - (P[11][6]*q0)/2 + SF[2]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[1]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) - SPP[1]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
    nextP[2][7] = P[2][7] + P[0][7]*SF[4] + P[1][7]*SF[8] + P[3][7]*SF[6] + P[12][7]*SF[11] - P[10][7]*SPP[6] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[4] + P[1][4]*SF[8] + P[3][4]*SF[6] + P[12][4]*SF[11] - P[10][4]*SPP[6] - (P[11][4]*q0)/2);
    nextP[2][8] = P[2][8] + P[0][8]*SF[4] + P[1][8]*SF[8] + P[3][8]*SF[6] + P[12][8]*SF[11] - P[10][8]*SPP[6] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[4] + P[1][5]*SF[8] + P[3][5]*SF[6] + P[12][5]*SF[11] - P[10][5]*SPP[6] - (P[11][5]*q0)/2);
    nextP[2][9] = P[2][9] + P[0][9]*SF[4] + P[1][9]*SF[8] + P[3][9]*SF[6] + P[12][9]*SF[11] - P[10][9]*SPP[6] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[4] + P[1][6]*SF[8] + P[3][6]*SF[6] + P[12][6]*SF[11] - P[10][6]*SPP[6] - (P[11][6]*q0)/2);
    nextP[2][10] = P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2;
    nextP[2][11] = P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2;
    nextP[2][12] = P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2;
    nextP[2][13] = P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2;
    nextP[2][14] = P[2][14] + P[0][14]*SF[4] + P[1][14]*SF[8] + P[3][14]*SF[6] + P[12][14]*SF[11] - P[10][14]*SPP[6] - (P[11][14]*q0)/2;
    nextP[2][15] = P[2][15] + P[0][15]*SF[4] + P[1][15]*SF[8] + P[3][15]*SF[6] + P[12][15]*SF[11] - P[10][15]*SPP[6] - (P[11][15]*q0)/2;
    nextP[2][16] = P[2][16] + P[0][16]*SF[4] + P[1][16]*SF[8] + P[3][16]*SF[6] + P[12][16]*SF[11] - P[10][16]*SPP[6] - (P[11][16]*q0)/2;
    nextP[2][17] = P[2][17] + P[0][17]*SF[4] + P[1][17]*SF[8] + P[3][17]*SF[6] + P[12][17]*SF[11] - P[10][17]*SPP[6] - (P[11][17]*q0)/2;
    nextP[2][18] = P[2][18] + P[0][18]*SF[4] + P[1][18]*SF[8] + P[3][18]*SF[6] + P[12][18]*SF[11] - P[10][18]*SPP[6] - (P[11][18]*q0)/2;
    nextP[2][19] = P[2][19] + P[0][19]*SF[4] + P[1][19]*SF[8] + P[3][19]*SF[6] + P[12][19]*SF[11] - P[10][19]*SPP[6] - (P[11][19]*q0)/2;
    nextP[2][20] = P[2][20] + P[0][20]*SF[4] + P[1][20]*SF[8] + P[3][20]*SF[6] + P[12][20]*SF[11] - P[10][20]*SPP[6] - (P[11][20]*q0)/2;
    nextP[2][21] = P[2][21] + P[0][21]*SF[4] + P[1][21]*SF[8] + P[3][21]*SF[6] + P[12][21]*SF[11] - P[10][21]*SPP[6] - (P[11][21]*q0)/2;
    nextP[2][22] = P[2][22] + P[0][22]*SF[4] + P[1][22]*SF[8] + P[3][22]*SF[6] + P[12][22]*SF[11] - P[10][22]*SPP[6] - (P[11][22]*q0)/2;
    nextP[3][0] = P[3][0] + SQ[6] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2 + SF[7]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[9]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[8]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SF[11]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) + SPP[7]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) + SPP[6]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2);
    nextP[3][1] = P[3][1] + SQ[4] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2 + SF[6]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[5]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[9]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SPP[6]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) - SPP[7]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2) - (q0*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2))/2;
    nextP[3][2] = P[3][2] + SQ[3] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[8]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[6]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SF[11]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2) - SPP[6]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) - (q0*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2))/2;
    nextP[3][3] = P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] + (dayCov*SQ[10])/4 + dazCov*SQ[9] - (P[12][3]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[4]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[7]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - SF[11]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) + SPP[7]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) + (daxCov*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2))/2;
    nextP[3][4] = P[3][4] + P[0][4]*SF[5] + P[1][4]*SF[4] + P[2][4]*SF[7] - P[11][4]*SF[11] + P[10][4]*SPP[7] - (P[12][4]*q0)/2 + SF[3]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[1]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - SPP[2]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) - SPP[4]*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
    nextP[3][5] = P[3][5] + P[0][5]*SF[5] + P[1][5]*SF[4] + P[2][5]*SF[7] - P[11][5]*SF[11] + P[10][5]*SPP[7] - (P[12][5]*q0)/2 + SF[2]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[1]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[3]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SPP[3]*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
    nextP[3][6] = P[3][6] + P[0][6]*SF[5] + P[1][6]*SF[4] + P[2][6]*SF[7] - P[11][6]*SF[11] + P[10][6]*SPP[7] - (P[12][6]*q0)/2 + SF[2]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[1]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) - SPP[1]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
    nextP[3][7] = P[3][7] + P[0][7]*SF[5] + P[1][7]*SF[4] + P[2][7]*SF[7] - P[11][7]*SF[11] + P[10][7]*SPP[7] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[5] + P[1][4]*SF[4] + P[2][4]*SF[7] - P[11][4]*SF[11] + P[10][4]*SPP[7] - (P[12][4]*q0)/2);
    nextP[3][8] = P[3][8] + P[0][8]*SF[5] + P[1][8]*SF[4] + P[2][8]*SF[7] - P[11][8]*SF[11] + P[10][8]*SPP[7] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[5] + P[1][5]*SF[4] + P[2][5]*SF[7] - P[11][5]*SF[11] + P[10][5]*SPP[7] - (P[12][5]*q0)/2);
    nextP[3][9] = P[3][9] + P[0][9]*SF[5] + P[1][9]*SF[4] + P[2][9]*SF[7] - P[11][9]*SF[11] + P[10][9]*SPP[7] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[5] + P[1][6]*SF[4] + P[2][6]*SF[7] - P[11][6]*SF[11] + P[10][6]*SPP[7] - (P[12][6]*q0)/2);
    nextP[3][10] = P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2;
    nextP[3][11] = P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2;
    nextP[3][12] = P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2;
    nextP[3][13] = P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2;
    nextP[3][14] = P[3][14] + P[0][14]*SF[5] + P[1][14]*SF[4] + P[2][14]*SF[7] - P[11][14]*SF[11] + P[10][14]*SPP[7] - (P[12][14]*q0)/2;
    nextP[3][15] = P[3][15] + P[0][15]*SF[5] + P[1][15]*SF[4] + P[2][15]*SF[7] - P[11][15]*SF[11] + P[10][15]*SPP[7] - (P[12][15]*q0)/2;
    nextP[3][16] = P[3][16] + P[0][16]*SF[5] + P[1][16]*SF[4] + P[2][16]*SF[7] - P[11][16]*SF[11] + P[10][16]*SPP[7] - (P[12][16]*q0)/2;
    nextP[3][17] = P[3][17] + P[0][17]*SF[5] + P[1][17]*SF[4] + P[2][17]*SF[7] - P[11][17]*SF[11] + P[10][17]*SPP[7] - (P[12][17]*q0)/2;
    nextP[3][18] = P[3][18] + P[0][18]*SF[5] + P[1][18]*SF[4] + P[2][18]*SF[7] - P[11][18]*SF[11] + P[10][18]*SPP[7] - (P[12][18]*q0)/2;
    nextP[3][19] = P[3][19] + P[0][19]*SF[5] + P[1][19]*SF[4] + P[2][19]*SF[7] - P[11][19]*SF[11] + P[10][19]*SPP[7] - (P[12][19]*q0)/2;
    nextP[3][20] = P[3][20] + P[0][20]*SF[5] + P[1][20]*SF[4] + P[2][20]*SF[7] - P[11][20]*SF[11] + P[10][20]*SPP[7] - (P[12][20]*q0)/2;
    nextP[3][21] = P[3][21] + P[0][21]*SF[5] + P[1][21]*SF[4] + P[2][21]*SF[7] - P[11][21]*SF[11] + P[10][21]*SPP[7] - (P[12][21]*q0)/2;
    nextP[3][22] = P[3][22] + P[0][22]*SF[5] + P[1][22]*SF[4] + P[2][22]*SF[7] - P[11][22]*SF[11] + P[10][22]*SPP[7] - (P[12][22]*q0)/2;
    nextP[4][0] = P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4] + SF[7]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[9]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[8]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SF[11]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) + SPP[7]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) + SPP[6]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]);
    nextP[4][1] = P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4] + SF[6]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[5]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[9]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SPP[6]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) - SPP[7]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]) - (q0*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]))/2;
    nextP[4][2] = P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4] + SF[4]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[8]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[6]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SF[11]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]) - SPP[6]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) - (q0*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]))/2;
    nextP[4][3] = P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4] + SF[5]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[4]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[7]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - SF[11]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) + SPP[7]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) - (q0*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]))/2;
    nextP[4][4] = P[4][4] + P[0][4]*SF[3] + P[1][4]*SF[1] + P[2][4]*SPP[0] - P[3][4]*SPP[2] - P[13][4]*SPP[4] + dvyCov*sq(SG[7] - 2*q0*q3) + dvzCov*sq(SG[6] + 2*q0*q2) + SF[3]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[1]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SPP[0]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - SPP[2]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) - SPP[4]*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]) + dvxCov*sq(SG[1] + SG[2] - SG[3] - SG[4]);
    nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[3] + P[1][5]*SF[1] + P[2][5]*SPP[0] - P[3][5]*SPP[2] - P[13][5]*SPP[4] + SF[2]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[1]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[3]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) - SPP[0]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SPP[3]*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]);
    nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[3] + P[1][6]*SF[1] + P[2][6]*SPP[0] - P[3][6]*SPP[2] - P[13][6]*SPP[4] + SF[2]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[1]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SPP[0]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) - SPP[1]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]);
    nextP[4][7] = P[4][7] + P[0][7]*SF[3] + P[1][7]*SF[1] + P[2][7]*SPP[0] - P[3][7]*SPP[2] - P[13][7]*SPP[4] + dt*(P[4][4] + P[0][4]*SF[3] + P[1][4]*SF[1] + P[2][4]*SPP[0] - P[3][4]*SPP[2] - P[13][4]*SPP[4]);
    nextP[4][8] = P[4][8] + P[0][8]*SF[3] + P[1][8]*SF[1] + P[2][8]*SPP[0] - P[3][8]*SPP[2] - P[13][8]*SPP[4] + dt*(P[4][5] + P[0][5]*SF[3] + P[1][5]*SF[1] + P[2][5]*SPP[0] - P[3][5]*SPP[2] - P[13][5]*SPP[4]);
    nextP[4][9] = P[4][9] + P[0][9]*SF[3] + P[1][9]*SF[1] + P[2][9]*SPP[0] - P[3][9]*SPP[2] - P[13][9]*SPP[4] + dt*(P[4][6] + P[0][6]*SF[3] + P[1][6]*SF[1] + P[2][6]*SPP[0] - P[3][6]*SPP[2] - P[13][6]*SPP[4]);
    nextP[4][10] = P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4];
    nextP[4][11] = P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4];
    nextP[4][12] = P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4];
    nextP[4][13] = P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4];
    nextP[4][14] = P[4][14] + P[0][14]*SF[3] + P[1][14]*SF[1] + P[2][14]*SPP[0] - P[3][14]*SPP[2] - P[13][14]*SPP[4];
    nextP[4][15] = P[4][15] + P[0][15]*SF[3] + P[1][15]*SF[1] + P[2][15]*SPP[0] - P[3][15]*SPP[2] - P[13][15]*SPP[4];
    nextP[4][16] = P[4][16] + P[0][16]*SF[3] + P[1][16]*SF[1] + P[2][16]*SPP[0] - P[3][16]*SPP[2] - P[13][16]*SPP[4];
    nextP[4][17] = P[4][17] + P[0][17]*SF[3] + P[1][17]*SF[1] + P[2][17]*SPP[0] - P[3][17]*SPP[2] - P[13][17]*SPP[4];
    nextP[4][18] = P[4][18] + P[0][18]*SF[3] + P[1][18]*SF[1] + P[2][18]*SPP[0] - P[3][18]*SPP[2] - P[13][18]*SPP[4];
    nextP[4][19] = P[4][19] + P[0][19]*SF[3] + P[1][19]*SF[1] + P[2][19]*SPP[0] - P[3][19]*SPP[2] - P[13][19]*SPP[4];
    nextP[4][20] = P[4][20] + P[0][20]*SF[3] + P[1][20]*SF[1] + P[2][20]*SPP[0] - P[3][20]*SPP[2] - P[13][20]*SPP[4];
    nextP[4][21] = P[4][21] + P[0][21]*SF[3] + P[1][21]*SF[1] + P[2][21]*SPP[0] - P[3][21]*SPP[2] - P[13][21]*SPP[4];
    nextP[4][22] = P[4][22] + P[0][22]*SF[3] + P[1][22]*SF[1] + P[2][22]*SPP[0] - P[3][22]*SPP[2] - P[13][22]*SPP[4];
    nextP[5][0] = P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3] + SF[7]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[9]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[8]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SF[11]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) + SPP[7]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) + SPP[6]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]);
    nextP[5][1] = P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3] + SF[6]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[5]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[9]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SPP[6]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) - SPP[7]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]) - (q0*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]))/2;
    nextP[5][2] = P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3] + SF[4]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[8]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[6]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SF[11]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]) - SPP[6]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) - (q0*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]))/2;
    nextP[5][3] = P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3] + SF[5]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[4]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[7]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - SF[11]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) + SPP[7]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) - (q0*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]))/2;
    nextP[5][4] = P[5][4] + SQ[2] + P[0][4]*SF[2] + P[2][4]*SF[1] + P[3][4]*SF[3] - P[1][4]*SPP[0] + P[13][4]*SPP[3] + SF[3]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[1]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SPP[0]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - SPP[2]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) - SPP[4]*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]);
    nextP[5][5] = P[5][5] + P[0][5]*SF[2] + P[2][5]*SF[1] + P[3][5]*SF[3] - P[1][5]*SPP[0] + P[13][5]*SPP[3] + dvxCov*sq(SG[7] + 2*q0*q3) + dvzCov*sq(SG[5] - 2*q0*q1) + SF[2]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[1]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[3]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) - SPP[0]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SPP[3]*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]) + dvyCov*sq(SG[1] - SG[2] + SG[3] - SG[4]);
    nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[2] + P[2][6]*SF[1] + P[3][6]*SF[3] - P[1][6]*SPP[0] + P[13][6]*SPP[3] + SF[2]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[1]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SPP[0]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) - SPP[1]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]);
    nextP[5][7] = P[5][7] + P[0][7]*SF[2] + P[2][7]*SF[1] + P[3][7]*SF[3] - P[1][7]*SPP[0] + P[13][7]*SPP[3] + dt*(P[5][4] + P[0][4]*SF[2] + P[2][4]*SF[1] + P[3][4]*SF[3] - P[1][4]*SPP[0] + P[13][4]*SPP[3]);
    nextP[5][8] = P[5][8] + P[0][8]*SF[2] + P[2][8]*SF[1] + P[3][8]*SF[3] - P[1][8]*SPP[0] + P[13][8]*SPP[3] + dt*(P[5][5] + P[0][5]*SF[2] + P[2][5]*SF[1] + P[3][5]*SF[3] - P[1][5]*SPP[0] + P[13][5]*SPP[3]);
    nextP[5][9] = P[5][9] + P[0][9]*SF[2] + P[2][9]*SF[1] + P[3][9]*SF[3] - P[1][9]*SPP[0] + P[13][9]*SPP[3] + dt*(P[5][6] + P[0][6]*SF[2] + P[2][6]*SF[1] + P[3][6]*SF[3] - P[1][6]*SPP[0] + P[13][6]*SPP[3]);
    nextP[5][10] = P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3];
    nextP[5][11] = P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3];
    nextP[5][12] = P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3];
    nextP[5][13] = P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3];
    nextP[5][14] = P[5][14] + P[0][14]*SF[2] + P[2][14]*SF[1] + P[3][14]*SF[3] - P[1][14]*SPP[0] + P[13][14]*SPP[3];
    nextP[5][15] = P[5][15] + P[0][15]*SF[2] + P[2][15]*SF[1] + P[3][15]*SF[3] - P[1][15]*SPP[0] + P[13][15]*SPP[3];
    nextP[5][16] = P[5][16] + P[0][16]*SF[2] + P[2][16]*SF[1] + P[3][16]*SF[3] - P[1][16]*SPP[0] + P[13][16]*SPP[3];
    nextP[5][17] = P[5][17] + P[0][17]*SF[2] + P[2][17]*SF[1] + P[3][17]*SF[3] - P[1][17]*SPP[0] + P[13][17]*SPP[3];
    nextP[5][18] = P[5][18] + P[0][18]*SF[2] + P[2][18]*SF[1] + P[3][18]*SF[3] - P[1][18]*SPP[0] + P[13][18]*SPP[3];
    nextP[5][19] = P[5][19] + P[0][19]*SF[2] + P[2][19]*SF[1] + P[3][19]*SF[3] - P[1][19]*SPP[0] + P[13][19]*SPP[3];
    nextP[5][20] = P[5][20] + P[0][20]*SF[2] + P[2][20]*SF[1] + P[3][20]*SF[3] - P[1][20]*SPP[0] + P[13][20]*SPP[3];
    nextP[5][21] = P[5][21] + P[0][21]*SF[2] + P[2][21]*SF[1] + P[3][21]*SF[3] - P[1][21]*SPP[0] + P[13][21]*SPP[3];
    nextP[5][22] = P[5][22] + P[0][22]*SF[2] + P[2][22]*SF[1] + P[3][22]*SF[3] - P[1][22]*SPP[0] + P[13][22]*SPP[3];
    nextP[6][0] = P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[7]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[9]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[8]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[11]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[7]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[6]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    nextP[6][1] = P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[6]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[5]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[9]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[6]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[7]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
    nextP[6][2] = P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[4]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[8]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[6]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[11]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[6]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
    nextP[6][3] = P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[5]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[4]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[7]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SF[11]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[7]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
    nextP[6][4] = P[6][4] + SQ[1] + P[1][4]*SF[2] + P[3][4]*SF[1] + P[0][4]*SPP[0] - P[2][4]*SPP[1] - P[13][4]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[3]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[0]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[2]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[4]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    nextP[6][5] = P[6][5] + SQ[0] + P[1][5]*SF[2] + P[3][5]*SF[1] + P[0][5]*SPP[0] - P[2][5]*SPP[1] - P[13][5]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[2]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[3]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[0]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[3]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
    nextP[6][6] = P[6][6] + P[1][6]*SF[2] + P[3][6]*SF[1] + P[0][6]*SPP[0] - P[2][6]*SPP[1] - P[13][6]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + dvxCov*sq(SG[6] - 2*q0*q2) + dvyCov*sq(SG[5] + 2*q0*q1) - SPP[5]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*SPP[5]) + SF[2]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[0]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[1]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + dvzCov*sq(SG[1] - SG[2] - SG[3] + SG[4]);
    nextP[6][7] = P[6][7] + P[1][7]*SF[2] + P[3][7]*SF[1] + P[0][7]*SPP[0] - P[2][7]*SPP[1] - P[13][7]*SPP[5] + dt*(P[6][4] + P[1][4]*SF[2] + P[3][4]*SF[1] + P[0][4]*SPP[0] - P[2][4]*SPP[1] - P[13][4]*SPP[5]);
    nextP[6][8] = P[6][8] + P[1][8]*SF[2] + P[3][8]*SF[1] + P[0][8]*SPP[0] - P[2][8]*SPP[1] - P[13][8]*SPP[5] + dt*(P[6][5] + P[1][5]*SF[2] + P[3][5]*SF[1] + P[0][5]*SPP[0] - P[2][5]*SPP[1] - P[13][5]*SPP[5]);
    nextP[6][9] = P[6][9] + P[1][9]*SF[2] + P[3][9]*SF[1] + P[0][9]*SPP[0] - P[2][9]*SPP[1] - P[13][9]*SPP[5] + dt*(P[6][6] + P[1][6]*SF[2] + P[3][6]*SF[1] + P[0][6]*SPP[0] - P[2][6]*SPP[1] - P[13][6]*SPP[5]);
    nextP[6][10] = P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*SPP[5];
    nextP[6][11] = P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*SPP[5];
    nextP[6][12] = P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*SPP[5];
    nextP[6][13] = P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*SPP[5];
    nextP[6][14] = P[6][14] + P[1][14]*SF[2] + P[3][14]*SF[1] + P[0][14]*SPP[0] - P[2][14]*SPP[1] - P[13][14]*SPP[5];
    nextP[6][15] = P[6][15] + P[1][15]*SF[2] + P[3][15]*SF[1] + P[0][15]*SPP[0] - P[2][15]*SPP[1] - P[13][15]*SPP[5];
    nextP[6][16] = P[6][16] + P[1][16]*SF[2] + P[3][16]*SF[1] + P[0][16]*SPP[0] - P[2][16]*SPP[1] - P[13][16]*SPP[5];
    nextP[6][17] = P[6][17] + P[1][17]*SF[2] + P[3][17]*SF[1] + P[0][17]*SPP[0] - P[2][17]*SPP[1] - P[13][17]*SPP[5];
    nextP[6][18] = P[6][18] + P[1][18]*SF[2] + P[3][18]*SF[1] + P[0][18]*SPP[0] - P[2][18]*SPP[1] - P[13][18]*SPP[5];
    nextP[6][19] = P[6][19] + P[1][19]*SF[2] + P[3][19]*SF[1] + P[0][19]*SPP[0] - P[2][19]*SPP[1] - P[13][19]*SPP[5];
    nextP[6][20] = P[6][20] + P[1][20]*SF[2] + P[3][20]*SF[1] + P[0][20]*SPP[0] - P[2][20]*SPP[1] - P[13][20]*SPP[5];
    nextP[6][21] = P[6][21] + P[1][21]*SF[2] + P[3][21]*SF[1] + P[0][21]*SPP[0] - P[2][21]*SPP[1] - P[13][21]*SPP[5];
    nextP[6][22] = P[6][22] + P[1][22]*SF[2] + P[3][22]*SF[1] + P[0][22]*SPP[0] - P[2][22]*SPP[1] - P[13][22]*SPP[5];
    nextP[7][0] = P[7][0] + P[4][0]*dt + SF[7]*(P[7][1] + P[4][1]*dt) + SF[9]*(P[7][2] + P[4][2]*dt) + SF[8]*(P[7][3] + P[4][3]*dt) + SF[11]*(P[7][10] + P[4][10]*dt) + SPP[7]*(P[7][11] + P[4][11]*dt) + SPP[6]*(P[7][12] + P[4][12]*dt);
    nextP[7][1] = P[7][1] + P[4][1]*dt + SF[6]*(P[7][0] + P[4][0]*dt) + SF[5]*(P[7][2] + P[4][2]*dt) + SF[9]*(P[7][3] + P[4][3]*dt) + SPP[6]*(P[7][11] + P[4][11]*dt) - SPP[7]*(P[7][12] + P[4][12]*dt) - (q0*(P[7][10] + P[4][10]*dt))/2;
    nextP[7][2] = P[7][2] + P[4][2]*dt + SF[4]*(P[7][0] + P[4][0]*dt) + SF[8]*(P[7][1] + P[4][1]*dt) + SF[6]*(P[7][3] + P[4][3]*dt) + SF[11]*(P[7][12] + P[4][12]*dt) - SPP[6]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][11] + P[4][11]*dt))/2;
    nextP[7][3] = P[7][3] + P[4][3]*dt + SF[5]*(P[7][0] + P[4][0]*dt) + SF[4]*(P[7][1] + P[4][1]*dt) + SF[7]*(P[7][2] + P[4][2]*dt) - SF[11]*(P[7][11] + P[4][11]*dt) + SPP[7]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][12] + P[4][12]*dt))/2;
    nextP[7][4] = P[7][4] + P[4][4]*dt + SF[1]*(P[7][1] + P[4][1]*dt) + SF[3]*(P[7][0] + P[4][0]*dt) + SPP[0]*(P[7][2] + P[4][2]*dt) - SPP[2]*(P[7][3] + P[4][3]*dt) - SPP[4]*(P[7][13] + P[4][13]*dt);
    nextP[7][5] = P[7][5] + P[4][5]*dt + SF[2]*(P[7][0] + P[4][0]*dt) + SF[1]*(P[7][2] + P[4][2]*dt) + SF[3]*(P[7][3] + P[4][3]*dt) - SPP[0]*(P[7][1] + P[4][1]*dt) + SPP[3]*(P[7][13] + P[4][13]*dt);
    nextP[7][6] = P[7][6] + P[4][6]*dt + SF[2]*(P[7][1] + P[4][1]*dt) + SF[1]*(P[7][3] + P[4][3]*dt) + SPP[0]*(P[7][0] + P[4][0]*dt) - SPP[1]*(P[7][2] + P[4][2]*dt) - SPP[5]*(P[7][13] + P[4][13]*dt);
    nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
    nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
    nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
    nextP[7][10] = P[7][10] + P[4][10]*dt;
    nextP[7][11] = P[7][11] + P[4][11]*dt;
    nextP[7][12] = P[7][12] + P[4][12]*dt;
    nextP[7][13] = P[7][13] + P[4][13]*dt;
    nextP[7][14] = P[7][14] + P[4][14]*dt;
    nextP[7][15] = P[7][15] + P[4][15]*dt;
    nextP[7][16] = P[7][16] + P[4][16]*dt;
    nextP[7][17] = P[7][17] + P[4][17]*dt;
    nextP[7][18] = P[7][18] + P[4][18]*dt;
    nextP[7][19] = P[7][19] + P[4][19]*dt;
    nextP[7][20] = P[7][20] + P[4][20]*dt;
    nextP[7][21] = P[7][21] + P[4][21]*dt;
    nextP[7][22] = P[7][22] + P[4][22]*dt;
    nextP[8][0] = P[8][0] + P[5][0]*dt + SF[7]*(P[8][1] + P[5][1]*dt) + SF[9]*(P[8][2] + P[5][2]*dt) + SF[8]*(P[8][3] + P[5][3]*dt) + SF[11]*(P[8][10] + P[5][10]*dt) + SPP[7]*(P[8][11] + P[5][11]*dt) + SPP[6]*(P[8][12] + P[5][12]*dt);
    nextP[8][1] = P[8][1] + P[5][1]*dt + SF[6]*(P[8][0] + P[5][0]*dt) + SF[5]*(P[8][2] + P[5][2]*dt) + SF[9]*(P[8][3] + P[5][3]*dt) + SPP[6]*(P[8][11] + P[5][11]*dt) - SPP[7]*(P[8][12] + P[5][12]*dt) - (q0*(P[8][10] + P[5][10]*dt))/2;
    nextP[8][2] = P[8][2] + P[5][2]*dt + SF[4]*(P[8][0] + P[5][0]*dt) + SF[8]*(P[8][1] + P[5][1]*dt) + SF[6]*(P[8][3] + P[5][3]*dt) + SF[11]*(P[8][12] + P[5][12]*dt) - SPP[6]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][11] + P[5][11]*dt))/2;
    nextP[8][3] = P[8][3] + P[5][3]*dt + SF[5]*(P[8][0] + P[5][0]*dt) + SF[4]*(P[8][1] + P[5][1]*dt) + SF[7]*(P[8][2] + P[5][2]*dt) - SF[11]*(P[8][11] + P[5][11]*dt) + SPP[7]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][12] + P[5][12]*dt))/2;
    nextP[8][4] = P[8][4] + P[5][4]*dt + SF[1]*(P[8][1] + P[5][1]*dt) + SF[3]*(P[8][0] + P[5][0]*dt) + SPP[0]*(P[8][2] + P[5][2]*dt) - SPP[2]*(P[8][3] + P[5][3]*dt) - SPP[4]*(P[8][13] + P[5][13]*dt);
    nextP[8][5] = P[8][5] + P[5][5]*dt + SF[2]*(P[8][0] + P[5][0]*dt) + SF[1]*(P[8][2] + P[5][2]*dt) + SF[3]*(P[8][3] + P[5][3]*dt) - SPP[0]*(P[8][1] + P[5][1]*dt) + SPP[3]*(P[8][13] + P[5][13]*dt);
    nextP[8][6] = P[8][6] + P[5][6]*dt + SF[2]*(P[8][1] + P[5][1]*dt) + SF[1]*(P[8][3] + P[5][3]*dt) + SPP[0]*(P[8][0] + P[5][0]*dt) - SPP[1]*(P[8][2] + P[5][2]*dt) - SPP[5]*(P[8][13] + P[5][13]*dt);
    nextP[8][7] = P[8][7] + P[5][7]*dt + dt*(P[8][4] + P[5][4]*dt);
    nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
    nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
    nextP[8][10] = P[8][10] + P[5][10]*dt;
    nextP[8][11] = P[8][11] + P[5][11]*dt;
    nextP[8][12] = P[8][12] + P[5][12]*dt;
    nextP[8][13] = P[8][13] + P[5][13]*dt;
    nextP[8][14] = P[8][14] + P[5][14]*dt;
    nextP[8][15] = P[8][15] + P[5][15]*dt;
    nextP[8][16] = P[8][16] + P[5][16]*dt;
    nextP[8][17] = P[8][17] + P[5][17]*dt;
    nextP[8][18] = P[8][18] + P[5][18]*dt;
    nextP[8][19] = P[8][19] + P[5][19]*dt;
    nextP[8][20] = P[8][20] + P[5][20]*dt;
    nextP[8][21] = P[8][21] + P[5][21]*dt;
    nextP[8][22] = P[8][22] + P[5][22]*dt;
    nextP[9][0] = P[9][0] + P[6][0]*dt + SF[7]*(P[9][1] + P[6][1]*dt) + SF[9]*(P[9][2] + P[6][2]*dt) + SF[8]*(P[9][3] + P[6][3]*dt) + SF[11]*(P[9][10] + P[6][10]*dt) + SPP[7]*(P[9][11] + P[6][11]*dt) + SPP[6]*(P[9][12] + P[6][12]*dt);
    nextP[9][1] = P[9][1] + P[6][1]*dt + SF[6]*(P[9][0] + P[6][0]*dt) + SF[5]*(P[9][2] + P[6][2]*dt) + SF[9]*(P[9][3] + P[6][3]*dt) + SPP[6]*(P[9][11] + P[6][11]*dt) - SPP[7]*(P[9][12] + P[6][12]*dt) - (q0*(P[9][10] + P[6][10]*dt))/2;
    nextP[9][2] = P[9][2] + P[6][2]*dt + SF[4]*(P[9][0] + P[6][0]*dt) + SF[8]*(P[9][1] + P[6][1]*dt) + SF[6]*(P[9][3] + P[6][3]*dt) + SF[11]*(P[9][12] + P[6][12]*dt) - SPP[6]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][11] + P[6][11]*dt))/2;
    nextP[9][3] = P[9][3] + P[6][3]*dt + SF[5]*(P[9][0] + P[6][0]*dt) + SF[4]*(P[9][1] + P[6][1]*dt) + SF[7]*(P[9][2] + P[6][2]*dt) - SF[11]*(P[9][11] + P[6][11]*dt) + SPP[7]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][12] + P[6][12]*dt))/2;
    nextP[9][4] = P[9][4] + P[6][4]*dt + SF[1]*(P[9][1] + P[6][1]*dt) + SF[3]*(P[9][0] + P[6][0]*dt) + SPP[0]*(P[9][2] + P[6][2]*dt) - SPP[2]*(P[9][3] + P[6][3]*dt) - SPP[4]*(P[9][13] + P[6][13]*dt);
    nextP[9][5] = P[9][5] + P[6][5]*dt + SF[2]*(P[9][0] + P[6][0]*dt) + SF[1]*(P[9][2] + P[6][2]*dt) + SF[3]*(P[9][3] + P[6][3]*dt) - SPP[0]*(P[9][1] + P[6][1]*dt) + SPP[3]*(P[9][13] + P[6][13]*dt);
    nextP[9][6] = P[9][6] + P[6][6]*dt + SF[2]*(P[9][1] + P[6][1]*dt) + SF[1]*(P[9][3] + P[6][3]*dt) + SPP[0]*(P[9][0] + P[6][0]*dt) - SPP[1]*(P[9][2] + P[6][2]*dt) - SPP[5]*(P[9][13] + P[6][13]*dt);
    nextP[9][7] = P[9][7] + P[6][7]*dt + dt*(P[9][4] + P[6][4]*dt);
    nextP[9][8] = P[9][8] + P[6][8]*dt + dt*(P[9][5] + P[6][5]*dt);
    nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
    nextP[9][10] = P[9][10] + P[6][10]*dt;
    nextP[9][11] = P[9][11] + P[6][11]*dt;
    nextP[9][12] = P[9][12] + P[6][12]*dt;
    nextP[9][13] = P[9][13] + P[6][13]*dt;
    nextP[9][14] = P[9][14] + P[6][14]*dt;
    nextP[9][15] = P[9][15] + P[6][15]*dt;
    nextP[9][16] = P[9][16] + P[6][16]*dt;
    nextP[9][17] = P[9][17] + P[6][17]*dt;
    nextP[9][18] = P[9][18] + P[6][18]*dt;
    nextP[9][19] = P[9][19] + P[6][19]*dt;
    nextP[9][20] = P[9][20] + P[6][20]*dt;
    nextP[9][21] = P[9][21] + P[6][21]*dt;
    nextP[9][22] = P[9][22] + P[6][22]*dt;
    nextP[10][0] = P[10][0] + P[10][1]*SF[7] + P[10][2]*SF[9] + P[10][3]*SF[8] + P[10][10]*SF[11] + P[10][11]*SPP[7] + P[10][12]*SPP[6];
    nextP[10][1] = P[10][1] + P[10][0]*SF[6] + P[10][2]*SF[5] + P[10][3]*SF[9] + P[10][11]*SPP[6] - P[10][12]*SPP[7] - (P[10][10]*q0)/2;
    nextP[10][2] = P[10][2] + P[10][0]*SF[4] + P[10][1]*SF[8] + P[10][3]*SF[6] + P[10][12]*SF[11] - P[10][10]*SPP[6] - (P[10][11]*q0)/2;
    nextP[10][3] = P[10][3] + P[10][0]*SF[5] + P[10][1]*SF[4] + P[10][2]*SF[7] - P[10][11]*SF[11] + P[10][10]*SPP[7] - (P[10][12]*q0)/2;
    nextP[10][4] = P[10][4] + P[10][1]*SF[1] + P[10][0]*SF[3] + P[10][2]*SPP[0] - P[10][3]*SPP[2] - P[10][13]*SPP[4];
    nextP[10][5] = P[10][5] + P[10][0]*SF[2] + P[10][2]*SF[1] + P[10][3]*SF[3] - P[10][1]*SPP[0] + P[10][13]*SPP[3];
    nextP[10][6] = P[10][6] + P[10][1]*SF[2] + P[10][3]*SF[1] + P[10][0]*SPP[0] - P[10][2]*SPP[1] - P[10][13]*SPP[5];
    nextP[10][7] = P[10][7] + P[10][4]*dt;
    nextP[10][8] = P[10][8] + P[10][5]*dt;
    nextP[10][9] = P[10][9] + P[10][6]*dt;
    nextP[10][10] = P[10][10];
    nextP[10][11] = P[10][11];
    nextP[10][12] = P[10][12];
    nextP[10][13] = P[10][13];
    nextP[10][14] = P[10][14];
    nextP[10][15] = P[10][15];
    nextP[10][16] = P[10][16];
    nextP[10][17] = P[10][17];
    nextP[10][18] = P[10][18];
    nextP[10][19] = P[10][19];
    nextP[10][20] = P[10][20];
    nextP[10][21] = P[10][21];
    nextP[10][22] = P[10][22];
    nextP[11][0] = P[11][0] + P[11][1]*SF[7] + P[11][2]*SF[9] + P[11][3]*SF[8] + P[11][10]*SF[11] + P[11][11]*SPP[7] + P[11][12]*SPP[6];
    nextP[11][1] = P[11][1] + P[11][0]*SF[6] + P[11][2]*SF[5] + P[11][3]*SF[9] + P[11][11]*SPP[6] - P[11][12]*SPP[7] - (P[11][10]*q0)/2;
    nextP[11][2] = P[11][2] + P[11][0]*SF[4] + P[11][1]*SF[8] + P[11][3]*SF[6] + P[11][12]*SF[11] - P[11][10]*SPP[6] - (P[11][11]*q0)/2;
    nextP[11][3] = P[11][3] + P[11][0]*SF[5] + P[11][1]*SF[4] + P[11][2]*SF[7] - P[11][11]*SF[11] + P[11][10]*SPP[7] - (P[11][12]*q0)/2;
    nextP[11][4] = P[11][4] + P[11][1]*SF[1] + P[11][0]*SF[3] + P[11][2]*SPP[0] - P[11][3]*SPP[2] - P[11][13]*SPP[4];
    nextP[11][5] = P[11][5] + P[11][0]*SF[2] + P[11][2]*SF[1] + P[11][3]*SF[3] - P[11][1]*SPP[0] + P[11][13]*SPP[3];
    nextP[11][6] = P[11][6] + P[11][1]*SF[2] + P[11][3]*SF[1] + P[11][0]*SPP[0] - P[11][2]*SPP[1] - P[11][13]*SPP[5];
    nextP[11][7] = P[11][7] + P[11][4]*dt;
    nextP[11][8] = P[11][8] + P[11][5]*dt;
    nextP[11][9] = P[11][9] + P[11][6]*dt;
    nextP[11][10] = P[11][10];
    nextP[11][11] = P[11][11];
    nextP[11][12] = P[11][12];
    nextP[11][13] = P[11][13];
    nextP[11][14] = P[11][14];
    nextP[11][15] = P[11][15];
    nextP[11][16] = P[11][16];
    nextP[11][17] = P[11][17];
    nextP[11][18] = P[11][18];
    nextP[11][19] = P[11][19];
    nextP[11][20] = P[11][20];
    nextP[11][21] = P[11][21];
    nextP[11][22] = P[11][22];
    nextP[12][0] = P[12][0] + P[12][1]*SF[7] + P[12][2]*SF[9] + P[12][3]*SF[8] + P[12][10]*SF[11] + P[12][11]*SPP[7] + P[12][12]*SPP[6];
    nextP[12][1] = P[12][1] + P[12][0]*SF[6] + P[12][2]*SF[5] + P[12][3]*SF[9] + P[12][11]*SPP[6] - P[12][12]*SPP[7] - (P[12][10]*q0)/2;
    nextP[12][2] = P[12][2] + P[12][0]*SF[4] + P[12][1]*SF[8] + P[12][3]*SF[6] + P[12][12]*SF[11] - P[12][10]*SPP[6] - (P[12][11]*q0)/2;
    nextP[12][3] = P[12][3] + P[12][0]*SF[5] + P[12][1]*SF[4] + P[12][2]*SF[7] - P[12][11]*SF[11] + P[12][10]*SPP[7] - (P[12][12]*q0)/2;
    nextP[12][4] = P[12][4] + P[12][1]*SF[1] + P[12][0]*SF[3] + P[12][2]*SPP[0] - P[12][3]*SPP[2] - P[12][13]*SPP[4];
    nextP[12][5] = P[12][5] + P[12][0]*SF[2] + P[12][2]*SF[1] + P[12][3]*SF[3] - P[12][1]*SPP[0] + P[12][13]*SPP[3];
    nextP[12][6] = P[12][6] + P[12][1]*SF[2] + P[12][3]*SF[1] + P[12][0]*SPP[0] - P[12][2]*SPP[1] - P[12][13]*SPP[5];
    nextP[12][7] = P[12][7] + P[12][4]*dt;
    nextP[12][8] = P[12][8] + P[12][5]*dt;
    nextP[12][9] = P[12][9] + P[12][6]*dt;
    nextP[12][10] = P[12][10];
    nextP[12][11] = P[12][11];
    nextP[12][12] = P[12][12];
    nextP[12][13] = P[12][13];
    nextP[12][14] = P[12][14];
    nextP[12][15] = P[12][15];
    nextP[12][16] = P[12][16];
    nextP[12][17] = P[12][17];
    nextP[12][18] = P[12][18];
    nextP[12][19] = P[12][19];
    nextP[12][20] = P[12][20];
    nextP[12][21] = P[12][21];
    nextP[12][22] = P[12][22];
    nextP[13][0] = P[13][0] + P[13][1]*SF[7] + P[13][2]*SF[9] + P[13][3]*SF[8] + P[13][10]*SF[11] + P[13][11]*SPP[7] + P[13][12]*SPP[6];
    nextP[13][1] = P[13][1] + P[13][0]*SF[6] + P[13][2]*SF[5] + P[13][3]*SF[9] + P[13][11]*SPP[6] - P[13][12]*SPP[7] - (P[13][10]*q0)/2;
    nextP[13][2] = P[13][2] + P[13][0]*SF[4] + P[13][1]*SF[8] + P[13][3]*SF[6] + P[13][12]*SF[11] - P[13][10]*SPP[6] - (P[13][11]*q0)/2;
    nextP[13][3] = P[13][3] + P[13][0]*SF[5] + P[13][1]*SF[4] + P[13][2]*SF[7] - P[13][11]*SF[11] + P[13][10]*SPP[7] - (P[13][12]*q0)/2;
    nextP[13][4] = P[13][4] + P[13][1]*SF[1] + P[13][0]*SF[3] + P[13][2]*SPP[0] - P[13][3]*SPP[2] - P[13][13]*SPP[4];
    nextP[13][5] = P[13][5] + P[13][0]*SF[2] + P[13][2]*SF[1] + P[13][3]*SF[3] - P[13][1]*SPP[0] + P[13][13]*SPP[3];
    nextP[13][6] = P[13][6] + P[13][1]*SF[2] + P[13][3]*SF[1] + P[13][0]*SPP[0] - P[13][2]*SPP[1] - P[13][13]*SPP[5];
    nextP[13][7] = P[13][7] + P[13][4]*dt;
    nextP[13][8] = P[13][8] + P[13][5]*dt;
    nextP[13][9] = P[13][9] + P[13][6]*dt;
    nextP[13][10] = P[13][10];
    nextP[13][11] = P[13][11];
    nextP[13][12] = P[13][12];
    nextP[13][13] = P[13][13];
    nextP[13][14] = P[13][14];
    nextP[13][15] = P[13][15];
    nextP[13][16] = P[13][16];
    nextP[13][17] = P[13][17];
    nextP[13][18] = P[13][18];
    nextP[13][19] = P[13][19];
    nextP[13][20] = P[13][20];
    nextP[13][21] = P[13][21];
    nextP[13][22] = P[13][22];
    nextP[14][0] = P[14][0] + P[14][1]*SF[7] + P[14][2]*SF[9] + P[14][3]*SF[8] + P[14][10]*SF[11] + P[14][11]*SPP[7] + P[14][12]*SPP[6];
    nextP[14][1] = P[14][1] + P[14][0]*SF[6] + P[14][2]*SF[5] + P[14][3]*SF[9] + P[14][11]*SPP[6] - P[14][12]*SPP[7] - (P[14][10]*q0)/2;
    nextP[14][2] = P[14][2] + P[14][0]*SF[4] + P[14][1]*SF[8] + P[14][3]*SF[6] + P[14][12]*SF[11] - P[14][10]*SPP[6] - (P[14][11]*q0)/2;
    nextP[14][3] = P[14][3] + P[14][0]*SF[5] + P[14][1]*SF[4] + P[14][2]*SF[7] - P[14][11]*SF[11] + P[14][10]*SPP[7] - (P[14][12]*q0)/2;
    nextP[14][4] = P[14][4] + P[14][1]*SF[1] + P[14][0]*SF[3] + P[14][2]*SPP[0] - P[14][3]*SPP[2] - P[14][13]*SPP[4];
    nextP[14][5] = P[14][5] + P[14][0]*SF[2] + P[14][2]*SF[1] + P[14][3]*SF[3] - P[14][1]*SPP[0] + P[14][13]*SPP[3];
    nextP[14][6] = P[14][6] + P[14][1]*SF[2] + P[14][3]*SF[1] + P[14][0]*SPP[0] - P[14][2]*SPP[1] - P[14][13]*SPP[5];
    nextP[14][7] = P[14][7] + P[14][4]*dt;
    nextP[14][8] = P[14][8] + P[14][5]*dt;
    nextP[14][9] = P[14][9] + P[14][6]*dt;
    nextP[14][10] = P[14][10];
    nextP[14][11] = P[14][11];
    nextP[14][12] = P[14][12];
    nextP[14][13] = P[14][13];
    nextP[14][14] = P[14][14];
    nextP[14][15] = P[14][15];
    nextP[14][16] = P[14][16];
    nextP[14][17] = P[14][17];
    nextP[14][18] = P[14][18];
    nextP[14][19] = P[14][19];
    nextP[14][20] = P[14][20];
    nextP[14][21] = P[14][21];
    nextP[14][22] = P[14][22];
    nextP[15][0] = P[15][0] + P[15][1]*SF[7] + P[15][2]*SF[9] + P[15][3]*SF[8] + P[15][10]*SF[11] + P[15][11]*SPP[7] + P[15][12]*SPP[6];
    nextP[15][1] = P[15][1] + P[15][0]*SF[6] + P[15][2]*SF[5] + P[15][3]*SF[9] + P[15][11]*SPP[6] - P[15][12]*SPP[7] - (P[15][10]*q0)/2;
    nextP[15][2] = P[15][2] + P[15][0]*SF[4] + P[15][1]*SF[8] + P[15][3]*SF[6] + P[15][12]*SF[11] - P[15][10]*SPP[6] - (P[15][11]*q0)/2;
    nextP[15][3] = P[15][3] + P[15][0]*SF[5] + P[15][1]*SF[4] + P[15][2]*SF[7] - P[15][11]*SF[11] + P[15][10]*SPP[7] - (P[15][12]*q0)/2;
    nextP[15][4] = P[15][4] + P[15][1]*SF[1] + P[15][0]*SF[3] + P[15][2]*SPP[0] - P[15][3]*SPP[2] - P[15][13]*SPP[4];
    nextP[15][5] = P[15][5] + P[15][0]*SF[2] + P[15][2]*SF[1] + P[15][3]*SF[3] - P[15][1]*SPP[0] + P[15][13]*SPP[3];
    nextP[15][6] = P[15][6] + P[15][1]*SF[2] + P[15][3]*SF[1] + P[15][0]*SPP[0] - P[15][2]*SPP[1] - P[15][13]*SPP[5];
    nextP[15][7] = P[15][7] + P[15][4]*dt;
    nextP[15][8] = P[15][8] + P[15][5]*dt;
    nextP[15][9] = P[15][9] + P[15][6]*dt;
    nextP[15][10] = P[15][10];
    nextP[15][11] = P[15][11];
    nextP[15][12] = P[15][12];
    nextP[15][13] = P[15][13];
    nextP[15][14] = P[15][14];
    nextP[15][15] = P[15][15];
    nextP[15][16] = P[15][16];
    nextP[15][17] = P[15][17];
    nextP[15][18] = P[15][18];
    nextP[15][19] = P[15][19];
    nextP[15][20] = P[15][20];
    nextP[15][21] = P[15][21];
    nextP[15][22] = P[15][22];
    nextP[16][0] = P[16][0] + P[16][1]*SF[7] + P[16][2]*SF[9] + P[16][3]*SF[8] + P[16][10]*SF[11] + P[16][11]*SPP[7] + P[16][12]*SPP[6];
    nextP[16][1] = P[16][1] + P[16][0]*SF[6] + P[16][2]*SF[5] + P[16][3]*SF[9] + P[16][11]*SPP[6] - P[16][12]*SPP[7] - (P[16][10]*q0)/2;
    nextP[16][2] = P[16][2] + P[16][0]*SF[4] + P[16][1]*SF[8] + P[16][3]*SF[6] + P[16][12]*SF[11] - P[16][10]*SPP[6] - (P[16][11]*q0)/2;
    nextP[16][3] = P[16][3] + P[16][0]*SF[5] + P[16][1]*SF[4] + P[16][2]*SF[7] - P[16][11]*SF[11] + P[16][10]*SPP[7] - (P[16][12]*q0)/2;
    nextP[16][4] = P[16][4] + P[16][1]*SF[1] + P[16][0]*SF[3] + P[16][2]*SPP[0] - P[16][3]*SPP[2] - P[16][13]*SPP[4];
    nextP[16][5] = P[16][5] + P[16][0]*SF[2] + P[16][2]*SF[1] + P[16][3]*SF[3] - P[16][1]*SPP[0] + P[16][13]*SPP[3];
    nextP[16][6] = P[16][6] + P[16][1]*SF[2] + P[16][3]*SF[1] + P[16][0]*SPP[0] - P[16][2]*SPP[1] - P[16][13]*SPP[5];
    nextP[16][7] = P[16][7] + P[16][4]*dt;
    nextP[16][8] = P[16][8] + P[16][5]*dt;
    nextP[16][9] = P[16][9] + P[16][6]*dt;
    nextP[16][10] = P[16][10];
    nextP[16][11] = P[16][11];
    nextP[16][12] = P[16][12];
    nextP[16][13] = P[16][13];
    nextP[16][14] = P[16][14];
    nextP[16][15] = P[16][15];
    nextP[16][16] = P[16][16];
    nextP[16][17] = P[16][17];
    nextP[16][18] = P[16][18];
    nextP[16][19] = P[16][19];
    nextP[16][20] = P[16][20];
    nextP[16][21] = P[16][21];
    nextP[16][22] = P[16][22];
    nextP[17][0] = P[17][0] + P[17][1]*SF[7] + P[17][2]*SF[9] + P[17][3]*SF[8] + P[17][10]*SF[11] + P[17][11]*SPP[7] + P[17][12]*SPP[6];
    nextP[17][1] = P[17][1] + P[17][0]*SF[6] + P[17][2]*SF[5] + P[17][3]*SF[9] + P[17][11]*SPP[6] - P[17][12]*SPP[7] - (P[17][10]*q0)/2;
    nextP[17][2] = P[17][2] + P[17][0]*SF[4] + P[17][1]*SF[8] + P[17][3]*SF[6] + P[17][12]*SF[11] - P[17][10]*SPP[6] - (P[17][11]*q0)/2;
    nextP[17][3] = P[17][3] + P[17][0]*SF[5] + P[17][1]*SF[4] + P[17][2]*SF[7] - P[17][11]*SF[11] + P[17][10]*SPP[7] - (P[17][12]*q0)/2;
    nextP[17][4] = P[17][4] + P[17][1]*SF[1] + P[17][0]*SF[3] + P[17][2]*SPP[0] - P[17][3]*SPP[2] - P[17][13]*SPP[4];
    nextP[17][5] = P[17][5] + P[17][0]*SF[2] + P[17][2]*SF[1] + P[17][3]*SF[3] - P[17][1]*SPP[0] + P[17][13]*SPP[3];
    nextP[17][6] = P[17][6] + P[17][1]*SF[2] + P[17][3]*SF[1] + P[17][0]*SPP[0] - P[17][2]*SPP[1] - P[17][13]*SPP[5];
    nextP[17][7] = P[17][7] + P[17][4]*dt;
    nextP[17][8] = P[17][8] + P[17][5]*dt;
    nextP[17][9] = P[17][9] + P[17][6]*dt;
    nextP[17][10] = P[17][10];
    nextP[17][11] = P[17][11];
    nextP[17][12] = P[17][12];
    nextP[17][13] = P[17][13];
    nextP[17][14] = P[17][14];
    nextP[17][15] = P[17][15];
    nextP[17][16] = P[17][16];
    nextP[17][17] = P[17][17];
    nextP[17][18] = P[17][18];
    nextP[17][19] = P[17][19];
    nextP[17][20] = P[17][20];
    nextP[17][21] = P[17][21];
    nextP[17][22] = P[17][22];
    nextP[18][0] = P[18][0] + P[18][1]*SF[7] + P[18][2]*SF[9] + P[18][3]*SF[8] + P[18][10]*SF[11] + P[18][11]*SPP[7] + P[18][12]*SPP[6];
    nextP[18][1] = P[18][1] + P[18][0]*SF[6] + P[18][2]*SF[5] + P[18][3]*SF[9] + P[18][11]*SPP[6] - P[18][12]*SPP[7] - (P[18][10]*q0)/2;
    nextP[18][2] = P[18][2] + P[18][0]*SF[4] + P[18][1]*SF[8] + P[18][3]*SF[6] + P[18][12]*SF[11] - P[18][10]*SPP[6] - (P[18][11]*q0)/2;
    nextP[18][3] = P[18][3] + P[18][0]*SF[5] + P[18][1]*SF[4] + P[18][2]*SF[7] - P[18][11]*SF[11] + P[18][10]*SPP[7] - (P[18][12]*q0)/2;
    nextP[18][4] = P[18][4] + P[18][1]*SF[1] + P[18][0]*SF[3] + P[18][2]*SPP[0] - P[18][3]*SPP[2] - P[18][13]*SPP[4];
    nextP[18][5] = P[18][5] + P[18][0]*SF[2] + P[18][2]*SF[1] + P[18][3]*SF[3] - P[18][1]*SPP[0] + P[18][13]*SPP[3];
    nextP[18][6] = P[18][6] + P[18][1]*SF[2] + P[18][3]*SF[1] + P[18][0]*SPP[0] - P[18][2]*SPP[1] - P[18][13]*SPP[5];
    nextP[18][7] = P[18][7] + P[18][4]*dt;
    nextP[18][8] = P[18][8] + P[18][5]*dt;
    nextP[18][9] = P[18][9] + P[18][6]*dt;
    nextP[18][10] = P[18][10];
    nextP[18][11] = P[18][11];
    nextP[18][12] = P[18][12];
    nextP[18][13] = P[18][13];
    nextP[18][14] = P[18][14];
    nextP[18][15] = P[18][15];
    nextP[18][16] = P[18][16];
    nextP[18][17] = P[18][17];
    nextP[18][18] = P[18][18];
    nextP[18][19] = P[18][19];
    nextP[18][20] = P[18][20];
    nextP[18][21] = P[18][21];
    nextP[18][22] = P[18][22];
    nextP[19][0] = P[19][0] + P[19][1]*SF[7] + P[19][2]*SF[9] + P[19][3]*SF[8] + P[19][10]*SF[11] + P[19][11]*SPP[7] + P[19][12]*SPP[6];
    nextP[19][1] = P[19][1] + P[19][0]*SF[6] + P[19][2]*SF[5] + P[19][3]*SF[9] + P[19][11]*SPP[6] - P[19][12]*SPP[7] - (P[19][10]*q0)/2;
    nextP[19][2] = P[19][2] + P[19][0]*SF[4] + P[19][1]*SF[8] + P[19][3]*SF[6] + P[19][12]*SF[11] - P[19][10]*SPP[6] - (P[19][11]*q0)/2;
    nextP[19][3] = P[19][3] + P[19][0]*SF[5] + P[19][1]*SF[4] + P[19][2]*SF[7] - P[19][11]*SF[11] + P[19][10]*SPP[7] - (P[19][12]*q0)/2;
    nextP[19][4] = P[19][4] + P[19][1]*SF[1] + P[19][0]*SF[3] + P[19][2]*SPP[0] - P[19][3]*SPP[2] - P[19][13]*SPP[4];
    nextP[19][5] = P[19][5] + P[19][0]*SF[2] + P[19][2]*SF[1] + P[19][3]*SF[3] - P[19][1]*SPP[0] + P[19][13]*SPP[3];
    nextP[19][6] = P[19][6] + P[19][1]*SF[2] + P[19][3]*SF[1] + P[19][0]*SPP[0] - P[19][2]*SPP[1] - P[19][13]*SPP[5];
    nextP[19][7] = P[19][7] + P[19][4]*dt;
    nextP[19][8] = P[19][8] + P[19][5]*dt;
    nextP[19][9] = P[19][9] + P[19][6]*dt;
    nextP[19][10] = P[19][10];
    nextP[19][11] = P[19][11];
    nextP[19][12] = P[19][12];
    nextP[19][13] = P[19][13];
    nextP[19][14] = P[19][14];
    nextP[19][15] = P[19][15];
    nextP[19][16] = P[19][16];
    nextP[19][17] = P[19][17];
    nextP[19][18] = P[19][18];
    nextP[19][19] = P[19][19];
    nextP[19][20] = P[19][20];
    nextP[19][21] = P[19][21];
    nextP[19][22] = P[19][22];
    nextP[20][0] = P[20][0] + P[20][1]*SF[7] + P[20][2]*SF[9] + P[20][3]*SF[8] + P[20][10]*SF[11] + P[20][11]*SPP[7] + P[20][12]*SPP[6];
    nextP[20][1] = P[20][1] + P[20][0]*SF[6] + P[20][2]*SF[5] + P[20][3]*SF[9] + P[20][11]*SPP[6] - P[20][12]*SPP[7] - (P[20][10]*q0)/2;
    nextP[20][2] = P[20][2] + P[20][0]*SF[4] + P[20][1]*SF[8] + P[20][3]*SF[6] + P[20][12]*SF[11] - P[20][10]*SPP[6] - (P[20][11]*q0)/2;
    nextP[20][3] = P[20][3] + P[20][0]*SF[5] + P[20][1]*SF[4] + P[20][2]*SF[7] - P[20][11]*SF[11] + P[20][10]*SPP[7] - (P[20][12]*q0)/2;
    nextP[20][4] = P[20][4] + P[20][1]*SF[1] + P[20][0]*SF[3] + P[20][2]*SPP[0] - P[20][3]*SPP[2] - P[20][13]*SPP[4];
    nextP[20][5] = P[20][5] + P[20][0]*SF[2] + P[20][2]*SF[1] + P[20][3]*SF[3] - P[20][1]*SPP[0] + P[20][13]*SPP[3];
    nextP[20][6] = P[20][6] + P[20][1]*SF[2] + P[20][3]*SF[1] + P[20][0]*SPP[0] - P[20][2]*SPP[1] - P[20][13]*SPP[5];
    nextP[20][7] = P[20][7] + P[20][4]*dt;
    nextP[20][8] = P[20][8] + P[20][5]*dt;
    nextP[20][9] = P[20][9] + P[20][6]*dt;
    nextP[20][10] = P[20][10];
    nextP[20][11] = P[20][11];
    nextP[20][12] = P[20][12];
    nextP[20][13] = P[20][13];
    nextP[20][14] = P[20][14];
    nextP[20][15] = P[20][15];
    nextP[20][16] = P[20][16];
    nextP[20][17] = P[20][17];
    nextP[20][18] = P[20][18];
    nextP[20][19] = P[20][19];
    nextP[20][20] = P[20][20];
    nextP[20][21] = P[20][21];
    nextP[20][22] = P[20][22];
    nextP[21][0] = P[21][0] + P[21][1]*SF[7] + P[21][2]*SF[9] + P[21][3]*SF[8] + P[21][10]*SF[11] + P[21][11]*SPP[7] + P[21][12]*SPP[6];
    nextP[21][1] = P[21][1] + P[21][0]*SF[6] + P[21][2]*SF[5] + P[21][3]*SF[9] + P[21][11]*SPP[6] - P[21][12]*SPP[7] - (P[21][10]*q0)/2;
    nextP[21][2] = P[21][2] + P[21][0]*SF[4] + P[21][1]*SF[8] + P[21][3]*SF[6] + P[21][12]*SF[11] - P[21][10]*SPP[6] - (P[21][11]*q0)/2;
    nextP[21][3] = P[21][3] + P[21][0]*SF[5] + P[21][1]*SF[4] + P[21][2]*SF[7] - P[21][11]*SF[11] + P[21][10]*SPP[7] - (P[21][12]*q0)/2;
    nextP[21][4] = P[21][4] + P[21][1]*SF[1] + P[21][0]*SF[3] + P[21][2]*SPP[0] - P[21][3]*SPP[2] - P[21][13]*SPP[4];
    nextP[21][5] = P[21][5] + P[21][0]*SF[2] + P[21][2]*SF[1] + P[21][3]*SF[3] - P[21][1]*SPP[0] + P[21][13]*SPP[3];
    nextP[21][6] = P[21][6] + P[21][1]*SF[2] + P[21][3]*SF[1] + P[21][0]*SPP[0] - P[21][2]*SPP[1] - P[21][13]*SPP[5];
    nextP[21][7] = P[21][7] + P[21][4]*dt;
    nextP[21][8] = P[21][8] + P[21][5]*dt;
    nextP[21][9] = P[21][9] + P[21][6]*dt;
    nextP[21][10] = P[21][10];
    nextP[21][11] = P[21][11];
    nextP[21][12] = P[21][12];
    nextP[21][13] = P[21][13];
    nextP[21][14] = P[21][14];
    nextP[21][15] = P[21][15];
    nextP[21][16] = P[21][16];
    nextP[21][17] = P[21][17];
    nextP[21][18] = P[21][18];
    nextP[21][19] = P[21][19];
    nextP[21][20] = P[21][20];
    nextP[21][21] = P[21][21];
    nextP[21][22] = P[21][22];
    nextP[22][0] = P[22][0] + P[22][1]*SF[7] + P[22][2]*SF[9] + P[22][3]*SF[8] + P[22][10]*SF[11] + P[22][11]*SPP[7] + P[22][12]*SPP[6];
    nextP[22][1] = P[22][1] + P[22][0]*SF[6] + P[22][2]*SF[5] + P[22][3]*SF[9] + P[22][11]*SPP[6] - P[22][12]*SPP[7] - (P[22][10]*q0)/2;
    nextP[22][2] = P[22][2] + P[22][0]*SF[4] + P[22][1]*SF[8] + P[22][3]*SF[6] + P[22][12]*SF[11] - P[22][10]*SPP[6] - (P[22][11]*q0)/2;
    nextP[22][3] = P[22][3] + P[22][0]*SF[5] + P[22][1]*SF[4] + P[22][2]*SF[7] - P[22][11]*SF[11] + P[22][10]*SPP[7] - (P[22][12]*q0)/2;
    nextP[22][4] = P[22][4] + P[22][1]*SF[1] + P[22][0]*SF[3] + P[22][2]*SPP[0] - P[22][3]*SPP[2] - P[22][13]*SPP[4];
    nextP[22][5] = P[22][5] + P[22][0]*SF[2] + P[22][2]*SF[1] + P[22][3]*SF[3] - P[22][1]*SPP[0] + P[22][13]*SPP[3];
    nextP[22][6] = P[22][6] + P[22][1]*SF[2] + P[22][3]*SF[1] + P[22][0]*SPP[0] - P[22][2]*SPP[1] - P[22][13]*SPP[5];
    nextP[22][7] = P[22][7] + P[22][4]*dt;
    nextP[22][8] = P[22][8] + P[22][5]*dt;
    nextP[22][9] = P[22][9] + P[22][6]*dt;
    nextP[22][10] = P[22][10];
    nextP[22][11] = P[22][11];
    nextP[22][12] = P[22][12];
    nextP[22][13] = P[22][13];
    nextP[22][14] = P[22][14];
    nextP[22][15] = P[22][15];
    nextP[22][16] = P[22][16];
    nextP[22][17] = P[22][17];
    nextP[22][18] = P[22][18];
    nextP[22][19] = P[22][19];
    nextP[22][20] = P[22][20];
    nextP[22][21] = P[22][21];
    nextP[22][22] = P[22][22];

    for (unsigned i = 0; i < n_states; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // If the total position variance exceds 1E6 (1000m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1E6f)
    {
        for (uint8_t i=7; i<=8; i++)
        {
            for (unsigned j = 0; j < n_states; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // Copy covariance
    for (unsigned i = 0; i < n_states; i++) {
        P[i][i] = nextP[i][i];
    }

    // force symmetry for observable states
    for (unsigned i = 1; i < n_states; i++)
    {
        for (uint8_t j = 0; j < i; j++)
        {
            P[i][j] = 0.5f * (nextP[i][j] + nextP[j][i]);
            P[j][i] = P[i][j];
        }
    }

        ConstrainVariances();
}

void AttPosEKF::FuseVelposNED()
{

// declare variables used by fault isolation logic
    uint32_t gpsRetryTime = 3000; // time in msec before GPS fusion will be retried following innovation consistency failure
    uint32_t gpsRetryTimeNoTAS = 500; // retry time if no TAS measurement available
    uint32_t hgtRetryTime = 500; // height measurement retry time
    uint32_t horizRetryTime;

// declare variables used to check measurement errors
    float velInnov[3] = {0.0f,0.0f,0.0f};
    float posInnov[2] = {0.0f,0.0f};
    float hgtInnov = 0.0f;

// declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;
    uint8_t indexLimit = 22;

// declare variables used by state and covariance update calculations
    float velErr;
    float posErr;
    float R_OBS[6];
    float observation[6];
    float SK;
    float quatMag;

// Perform sequential fusion of GPS measurements. This assumes that the
// errors in the different velocity and position components are
// uncorrelated which is not true, however in the absence of covariance
// data from the GPS receiver it is the only assumption we can make
// so we might as well take advantage of the computational efficiencies
// associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        // set the GPS data timeout depending on whether airspeed data is present
        if (useAirspeed) horizRetryTime = gpsRetryTime;
        else horizRetryTime = gpsRetryTimeNoTAS;

        // Form the observation vector
        for (uint8_t i=0; i<=2; i++) observation[i] = velNED[i];
        for (uint8_t i=3; i<=4; i++) observation[i] = posNE[i-3];
        observation[5] = -(hgtMea);

        // Estimate the GPS Velocity, GPS horiz position and height measurement variances.
        velErr = 0.2f*accNavMag; // additional error in GPS velocities caused by manoeuvring
        posErr = 0.2f*accNavMag; // additional error in GPS position caused by manoeuvring
        R_OBS[0] = sq(vneSigma) + sq(velErr);
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = sq(vdSigma) + sq(velErr);
        R_OBS[3] = sq(posNeSigma) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(posDSigma) + sq(posErr);

        // calculate innovations and check GPS data validity using an innovation consistency check
        if (fuseVelData)
        {
            // test velocity measurements
            uint8_t imax = 2;
            if (fusionModeGPS == 1) imax = 1;
            for (uint8_t i = 0; i<=imax; i++)
            {
                velInnov[i] = statesAtVelTime[i+4] - velNED[i];
                stateIndex = 4 + i;
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];
            }
            // apply a 5-sigma threshold
            current_ekf_state.velHealth = (sq(velInnov[0]) + sq(velInnov[1]) + sq(velInnov[2])) < 25.0f * (varInnovVelPos[0] + varInnovVelPos[1] + varInnovVelPos[2]);
            current_ekf_state.velTimeout = (millis() - current_ekf_state.velFailTime) > horizRetryTime;
            if (current_ekf_state.velHealth || staticMode) {
                current_ekf_state.velHealth = true;
                current_ekf_state.velFailTime = millis();
            } else if (current_ekf_state.velTimeout || !current_ekf_state.posHealth) {
                // XXX check
                current_ekf_state.velHealth = true;
                ResetVelocity();
                ResetStoredStates();
                // do not fuse bad data
                fuseVelData = false;
            }
            else
            {
                current_ekf_state.velHealth = false;
            }
        }
        if (fusePosData)
        {
            // test horizontal position measurements
            posInnov[0] = statesAtPosTime[7] - posNE[0];
            posInnov[1] = statesAtPosTime[8] - posNE[1];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply a 10-sigma threshold
            current_ekf_state.posHealth = (sq(posInnov[0]) + sq(posInnov[1])) < 100.0f*(varInnovVelPos[3] + varInnovVelPos[4]);
            current_ekf_state.posTimeout = (millis() - current_ekf_state.posFailTime) > horizRetryTime;
            if (current_ekf_state.posHealth || current_ekf_state.posTimeout)
            {
                current_ekf_state.posHealth = true;
                current_ekf_state.posFailTime = millis();

                if (current_ekf_state.posTimeout) {
                    ResetPosition();
                    
                    // XXX cross-check the state reset
                    ResetStoredStates();

                    // do not fuse position data on this time
                    // step
                    fusePosData = false;
                }
            }
            else
            {
                current_ekf_state.posHealth = false;
            }
        }
        // test height measurements
        if (fuseHgtData)
        {
            hgtInnov = statesAtHgtTime[9] + hgtMea;
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // apply a 10-sigma threshold
            current_ekf_state.hgtHealth = sq(hgtInnov) < 100.0f*varInnovVelPos[5];
            current_ekf_state.hgtTimeout = (millis() - current_ekf_state.hgtFailTime) > hgtRetryTime;
            if (current_ekf_state.hgtHealth || current_ekf_state.hgtTimeout || staticMode)
            {
                current_ekf_state.hgtHealth = true;
                current_ekf_state.hgtFailTime = millis();

                // if we just reset from a timeout, do not fuse
                // the height data, but reset height and stored states
                if (current_ekf_state.hgtTimeout) {
                    ResetHeight();
                    ResetStoredStates();
                    fuseHgtData = false;
                }
            }
            else
            {
                current_ekf_state.hgtHealth = false;
            }
        }
        // Set range for sequential fusion of velocity and position measurements depending
        // on which data is available and its health
        if (fuseVelData && fusionModeGPS == 0 && current_ekf_state.velHealth)
        {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }
        if (fuseVelData && fusionModeGPS == 1 && current_ekf_state.velHealth)
        {
            fuseData[0] = true;
            fuseData[1] = true;
        }
        if (fusePosData && fusionModeGPS <= 2 && current_ekf_state.posHealth)
        {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if (fuseHgtData && current_ekf_state.hgtHealth)
        {
            fuseData[5] = true;
        }
        // Fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++)
        {
            if (fuseData[obsIndex])
            {
                stateIndex = 4 + obsIndex;
                // Calculate the measurement innovation, using states from a
                // different time coordinate if fusing height data
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = statesAtVelTime[stateIndex] - observation[obsIndex];
                }
                else if (obsIndex == 3 || obsIndex == 4)
                {
                    innovVelPos[obsIndex] = statesAtPosTime[stateIndex] - observation[obsIndex];
                }
                else if (obsIndex == 5)
                {
                    innovVelPos[obsIndex] = statesAtHgtTime[stateIndex] - observation[obsIndex];
                }
                // Calculate the Kalman Gain
                // Calculate innovation variances - also used for data logging
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0/(double)varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // Don't update Z accel bias state unless using a height observation (GPS velocities can be biased)
                if (obsIndex != 5) {
                    Kfusion[13] = 0;
                }
                // Don't update wind states if inhibited
                if (inhibitWindStates) {
                    Kfusion[14] = 0;
                    Kfusion[15] = 0;
                }
                // Don't update magnetic field states if inhibited
                if (inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++)
                    {
                        Kfusion[i] = 0;
                    }
                }
                // Don't update terrain state if inhibited
                if (inhibitGndHgtState) {
                    Kfusion[22] = 0;
               }

                // Calculate state corrections and re-normalise the quaternions
                for (uint8_t i = 0; i<=indexLimit; i++)
                {
                    states[i] = states[i] - Kfusion[i] * innovVelPos[obsIndex];
                }
                quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
                if (quatMag > 1e-12f) // divide by  0 protection
                {
                    for (uint8_t i = 0; i<=3; i++)
                    {
                        states[i] = states[i] / quatMag;
                    }
                }
                // Update the covariance - take advantage of direct observation of a
                // single state at index = stateIndex to reduce computations
                // Optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    ForceSymmetry();
    ConstrainVariances();

}

void AttPosEKF::FuseMagnetometer()
{

    float &q0 = magstate.q0;
    float &q1 = magstate.q1;
    float &q2 = magstate.q2;
    float &q3 = magstate.q3;
    float &magN = magstate.magN;
    float &magE = magstate.magE;
    float &magD = magstate.magD;
    float &magXbias = magstate.magXbias;
    float &magYbias = magstate.magYbias;
    float &magZbias = magstate.magZbias;
    unsigned &obsIndex = magstate.obsIndex;
    Mat3f &DCM = magstate.DCM;
    float *MagPred = &magstate.MagPred[0];
    float &R_MAG = magstate.R_MAG;
    float *SH_MAG = &magstate.SH_MAG[0];

    float SK_MX[6];
    float SK_MY[5];
    float SK_MZ[6];
    float H_MAG[n_states];
    for (uint8_t i = 0; i < n_states; i++) {
        H_MAG[i] = 0.0f;
    }

// Perform sequential fusion of Magnetometer measurements.
// This assumes that the errors in the different components are
// uncorrelated which is not true, however in the absence of covariance
// data fit is the only assumption we can make
// so we might as well take advantage of the computational efficiencies
// associated with sequential fusion
    if (useCompass && fuseMagData && (obsIndex < 3))
    {
        // Calculate observation jacobians and Kalman gains
        if (obsIndex == 0)
        {
            // Copy required states to local variable names
            q0       = statesAtMagMeasTime[0];
            q1       = statesAtMagMeasTime[1];
            q2       = statesAtMagMeasTime[2];
            q3       = statesAtMagMeasTime[3];
            magN     = statesAtMagMeasTime[16];
            magE     = statesAtMagMeasTime[17];
            magD     = statesAtMagMeasTime[18];
            magXbias = statesAtMagMeasTime[19];
            magYbias = statesAtMagMeasTime[20];
            magZbias = statesAtMagMeasTime[21];

            // rotate predicted earth components into body axes and calculate
            // predicted measurments
            DCM.x.x = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            DCM.x.y = 2*(q1*q2 + q0*q3);
            DCM.x.z = 2*(q1*q3-q0*q2);
            DCM.y.x = 2*(q1*q2 - q0*q3);
            DCM.y.y = q0*q0 - q1*q1 + q2*q2 - q3*q3;
            DCM.y.z = 2*(q2*q3 + q0*q1);
            DCM.z.x = 2*(q1*q3 + q0*q2);
            DCM.z.y = 2*(q2*q3 - q0*q1);
            DCM.z.z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            MagPred[0] = DCM.x.x*magN + DCM.x.y*magE  + DCM.x.z*magD + magXbias;
            MagPred[1] = DCM.y.x*magN + DCM.y.y*magE  + DCM.y.z*magD + magYbias;
            MagPred[2] = DCM.z.x*magN + DCM.z.y*magE  + DCM.z.z*magD + magZbias;

            // scale magnetometer observation error with total angular rate
            R_MAG = sq(magMeasurementSigma) + sq(0.05f*dAngIMU.length()/dtIMU);

            // Calculate observation jacobians
            SH_MAG[0] = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
            SH_MAG[1] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
            SH_MAG[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
            SH_MAG[3] = sq(q3);
            SH_MAG[4] = sq(q2);
            SH_MAG[5] = sq(q1);
            SH_MAG[6] = sq(q0);
            SH_MAG[7] = 2*magN*q0;
            SH_MAG[8] = 2*magE*q3;

            for (uint8_t i = 0; i < n_states; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            H_MAG[1] = SH_MAG[0];
            H_MAG[2] = 2*magE*q1 - 2*magD*q0 - 2*magN*q2;
            H_MAG[3] = SH_MAG[2];
            H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
            H_MAG[17] = 2*q0*q3 + 2*q1*q2;
            H_MAG[18] = 2*q1*q3 - 2*q0*q2;
            H_MAG[19] = 1.0f;

            // Calculate Kalman gain
            float temp = (P[19][19] + R_MAG + P[1][19]*SH_MAG[0] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) - (2*magD*q0 - 2*magE*q1 + 2*magN*q2)*(P[19][2] + P[1][2]*SH_MAG[0] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(2*q0*q3 + 2*q1*q2) - P[18][2]*(2*q0*q2 - 2*q1*q3) - P[2][2]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(2*q0*q3 + 2*q1*q2) - P[18][0]*(2*q0*q2 - 2*q1*q3) - P[2][0]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(2*q0*q3 + 2*q1*q2) - P[18][1]*(2*q0*q2 - 2*q1*q3) - P[2][1]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(2*q0*q3 + 2*q1*q2) - P[18][3]*(2*q0*q2 - 2*q1*q3) - P[2][3]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(2*q0*q3 + 2*q1*q2) - P[18][16]*(2*q0*q2 - 2*q1*q3) - P[2][16]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[17][19]*(2*q0*q3 + 2*q1*q2) - P[18][19]*(2*q0*q2 - 2*q1*q3) - P[2][19]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + (2*q0*q3 + 2*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(2*q0*q3 + 2*q1*q2) - P[18][17]*(2*q0*q2 - 2*q1*q3) - P[2][17]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q2 - 2*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(2*q0*q3 + 2*q1*q2) - P[18][18]*(2*q0*q2 - 2*q1*q3) - P[2][18]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            if (temp >= R_MAG) {
                SK_MX[0] = 1.0f / temp;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[19][19] += 0.1f*R_MAG;
                obsIndex = 1;
                return;
            }
            SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
            SK_MX[2] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
            SK_MX[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MX[4] = 2*q0*q2 - 2*q1*q3;
            SK_MX[5] = 2*q0*q3 + 2*q1*q2;
            Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][1]*SH_MAG[0] + P[0][3]*SH_MAG[2] + P[0][0]*SK_MX[3] - P[0][2]*SK_MX[2] - P[0][16]*SK_MX[1] + P[0][17]*SK_MX[5] - P[0][18]*SK_MX[4]);
            Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][1]*SH_MAG[0] + P[1][3]*SH_MAG[2] + P[1][0]*SK_MX[3] - P[1][2]*SK_MX[2] - P[1][16]*SK_MX[1] + P[1][17]*SK_MX[5] - P[1][18]*SK_MX[4]);
            Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][1]*SH_MAG[0] + P[2][3]*SH_MAG[2] + P[2][0]*SK_MX[3] - P[2][2]*SK_MX[2] - P[2][16]*SK_MX[1] + P[2][17]*SK_MX[5] - P[2][18]*SK_MX[4]);
            Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][1]*SH_MAG[0] + P[3][3]*SH_MAG[2] + P[3][0]*SK_MX[3] - P[3][2]*SK_MX[2] - P[3][16]*SK_MX[1] + P[3][17]*SK_MX[5] - P[3][18]*SK_MX[4]);
            Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][1]*SH_MAG[0] + P[4][3]*SH_MAG[2] + P[4][0]*SK_MX[3] - P[4][2]*SK_MX[2] - P[4][16]*SK_MX[1] + P[4][17]*SK_MX[5] - P[4][18]*SK_MX[4]);
            Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][1]*SH_MAG[0] + P[5][3]*SH_MAG[2] + P[5][0]*SK_MX[3] - P[5][2]*SK_MX[2] - P[5][16]*SK_MX[1] + P[5][17]*SK_MX[5] - P[5][18]*SK_MX[4]);
            Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][1]*SH_MAG[0] + P[6][3]*SH_MAG[2] + P[6][0]*SK_MX[3] - P[6][2]*SK_MX[2] - P[6][16]*SK_MX[1] + P[6][17]*SK_MX[5] - P[6][18]*SK_MX[4]);
            Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][1]*SH_MAG[0] + P[7][3]*SH_MAG[2] + P[7][0]*SK_MX[3] - P[7][2]*SK_MX[2] - P[7][16]*SK_MX[1] + P[7][17]*SK_MX[5] - P[7][18]*SK_MX[4]);
            Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][1]*SH_MAG[0] + P[8][3]*SH_MAG[2] + P[8][0]*SK_MX[3] - P[8][2]*SK_MX[2] - P[8][16]*SK_MX[1] + P[8][17]*SK_MX[5] - P[8][18]*SK_MX[4]);
            Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][1]*SH_MAG[0] + P[9][3]*SH_MAG[2] + P[9][0]*SK_MX[3] - P[9][2]*SK_MX[2] - P[9][16]*SK_MX[1] + P[9][17]*SK_MX[5] - P[9][18]*SK_MX[4]);
            Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][1]*SH_MAG[0] + P[10][3]*SH_MAG[2] + P[10][0]*SK_MX[3] - P[10][2]*SK_MX[2] - P[10][16]*SK_MX[1] + P[10][17]*SK_MX[5] - P[10][18]*SK_MX[4]);
            Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][1]*SH_MAG[0] + P[11][3]*SH_MAG[2] + P[11][0]*SK_MX[3] - P[11][2]*SK_MX[2] - P[11][16]*SK_MX[1] + P[11][17]*SK_MX[5] - P[11][18]*SK_MX[4]);
            Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][1]*SH_MAG[0] + P[12][3]*SH_MAG[2] + P[12][0]*SK_MX[3] - P[12][2]*SK_MX[2] - P[12][16]*SK_MX[1] + P[12][17]*SK_MX[5] - P[12][18]*SK_MX[4]);
            // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
            Kfusion[13] = 0.0f;//SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[3] - P[13][2]*SK_MX[2] - P[13][16]*SK_MX[1] + P[13][17]*SK_MX[5] - P[13][18]*SK_MX[4]);
            // Estimation of selected states is inhibited by setting their Kalman gains to zero
            if (!inhibitWindStates) {
                Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][1]*SH_MAG[0] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[3] - P[14][2]*SK_MX[2] - P[14][16]*SK_MX[1] + P[14][17]*SK_MX[5] - P[14][18]*SK_MX[4]);
                Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][1]*SH_MAG[0] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[3] - P[15][2]*SK_MX[2] - P[15][16]*SK_MX[1] + P[15][17]*SK_MX[5] - P[15][18]*SK_MX[4]);
            } else {
                Kfusion[14] = 0;
                Kfusion[15] = 0;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][1]*SH_MAG[0] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[3] - P[16][2]*SK_MX[2] - P[16][16]*SK_MX[1] + P[16][17]*SK_MX[5] - P[16][18]*SK_MX[4]);
                Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][1]*SH_MAG[0] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[3] - P[17][2]*SK_MX[2] - P[17][16]*SK_MX[1] + P[17][17]*SK_MX[5] - P[17][18]*SK_MX[4]);
                Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][1]*SH_MAG[0] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[3] - P[18][2]*SK_MX[2] - P[18][16]*SK_MX[1] + P[18][17]*SK_MX[5] - P[18][18]*SK_MX[4]);
                Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][1]*SH_MAG[0] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[3] - P[19][2]*SK_MX[2] - P[19][16]*SK_MX[1] + P[19][17]*SK_MX[5] - P[19][18]*SK_MX[4]);
                Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][1]*SH_MAG[0] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[3] - P[20][2]*SK_MX[2] - P[20][16]*SK_MX[1] + P[20][17]*SK_MX[5] - P[20][18]*SK_MX[4]);
                Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][1]*SH_MAG[0] + P[21][3]*SH_MAG[2] + P[21][0]*SK_MX[3] - P[21][2]*SK_MX[2] - P[21][16]*SK_MX[1] + P[21][17]*SK_MX[5] - P[21][18]*SK_MX[4]);
            } else {
                for (uint8_t i=16; i <= 21; i++) {
                    Kfusion[i] = 0;
                }
            }
            if (!inhibitGndHgtState) {
                Kfusion[22] = SK_MX[0]*(P[22][19] + P[22][1]*SH_MAG[0] + P[22][3]*SH_MAG[2] + P[22][0]*SK_MX[3] - P[22][2]*SK_MX[2] - P[22][16]*SK_MX[1] + P[22][17]*SK_MX[5] - P[22][18]*SK_MX[4]);
            } else {
                Kfusion[22] = 0;
            }
            varInnovMag[0] = 1.0f/SK_MX[0];
            innovMag[0] = MagPred[0] - magData.x;
        }
        else if (obsIndex == 1) // we are now fusing the Y measurement
        {
            // Calculate observation jacobians
            for (unsigned int i = 0; i < n_states; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[2];
            H_MAG[1] = SH_MAG[1];
            H_MAG[2] = SH_MAG[0];
            H_MAG[3] = 2*magD*q2 - SH_MAG[8] - SH_MAG[7];
            H_MAG[16] = 2*q1*q2 - 2*q0*q3;
            H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
            H_MAG[18] = 2*q0*q1 + 2*q2*q3;
            H_MAG[20] = 1;

            // Calculate Kalman gain
            float temp = (P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2*q0*q3 - 2*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(2*q0*q3 - 2*q1*q2) + P[18][16]*(2*q0*q1 + 2*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (2*q0*q1 + 2*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(2*q0*q3 - 2*q1*q2) + P[18][18]*(2*q0*q1 + 2*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(2*q0*q3 - 2*q1*q2) + P[18][3]*(2*q0*q1 + 2*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[16][20]*(2*q0*q3 - 2*q1*q2) + P[18][20]*(2*q0*q1 + 2*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(2*q0*q3 - 2*q1*q2) + P[18][0]*(2*q0*q1 + 2*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(2*q0*q3 - 2*q1*q2) + P[18][1]*(2*q0*q1 + 2*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(2*q0*q3 - 2*q1*q2) + P[18][2]*(2*q0*q1 + 2*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(2*q0*q3 - 2*q1*q2) + P[18][17]*(2*q0*q1 + 2*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            if (temp >= R_MAG) {
                SK_MY[0] = 1.0f / temp;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[20][20] += 0.1f*R_MAG;
                obsIndex = 2;
                return;
            }
            SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
            SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MY[3] = 2*q0*q3 - 2*q1*q2;
            SK_MY[4] = 2*q0*q1 + 2*q2*q3;
            Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][0]*SH_MAG[2] + P[0][1]*SH_MAG[1] + P[0][2]*SH_MAG[0] - P[0][3]*SK_MY[2] - P[0][17]*SK_MY[1] - P[0][16]*SK_MY[3] + P[0][18]*SK_MY[4]);
            Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][0]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[1][2]*SH_MAG[0] - P[1][3]*SK_MY[2] - P[1][17]*SK_MY[1] - P[1][16]*SK_MY[3] + P[1][18]*SK_MY[4]);
            Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][0]*SH_MAG[2] + P[2][1]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[2][3]*SK_MY[2] - P[2][17]*SK_MY[1] - P[2][16]*SK_MY[3] + P[2][18]*SK_MY[4]);
            Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][0]*SH_MAG[2] + P[3][1]*SH_MAG[1] + P[3][2]*SH_MAG[0] - P[3][3]*SK_MY[2] - P[3][17]*SK_MY[1] - P[3][16]*SK_MY[3] + P[3][18]*SK_MY[4]);
            Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][0]*SH_MAG[2] + P[4][1]*SH_MAG[1] + P[4][2]*SH_MAG[0] - P[4][3]*SK_MY[2] - P[4][17]*SK_MY[1] - P[4][16]*SK_MY[3] + P[4][18]*SK_MY[4]);
            Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][0]*SH_MAG[2] + P[5][1]*SH_MAG[1] + P[5][2]*SH_MAG[0] - P[5][3]*SK_MY[2] - P[5][17]*SK_MY[1] - P[5][16]*SK_MY[3] + P[5][18]*SK_MY[4]);
            Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][0]*SH_MAG[2] + P[6][1]*SH_MAG[1] + P[6][2]*SH_MAG[0] - P[6][3]*SK_MY[2] - P[6][17]*SK_MY[1] - P[6][16]*SK_MY[3] + P[6][18]*SK_MY[4]);
            Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][0]*SH_MAG[2] + P[7][1]*SH_MAG[1] + P[7][2]*SH_MAG[0] - P[7][3]*SK_MY[2] - P[7][17]*SK_MY[1] - P[7][16]*SK_MY[3] + P[7][18]*SK_MY[4]);
            Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][0]*SH_MAG[2] + P[8][1]*SH_MAG[1] + P[8][2]*SH_MAG[0] - P[8][3]*SK_MY[2] - P[8][17]*SK_MY[1] - P[8][16]*SK_MY[3] + P[8][18]*SK_MY[4]);
            Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][0]*SH_MAG[2] + P[9][1]*SH_MAG[1] + P[9][2]*SH_MAG[0] - P[9][3]*SK_MY[2] - P[9][17]*SK_MY[1] - P[9][16]*SK_MY[3] + P[9][18]*SK_MY[4]);
            Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][0]*SH_MAG[2] + P[10][1]*SH_MAG[1] + P[10][2]*SH_MAG[0] - P[10][3]*SK_MY[2] - P[10][17]*SK_MY[1] - P[10][16]*SK_MY[3] + P[10][18]*SK_MY[4]);
            Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][0]*SH_MAG[2] + P[11][1]*SH_MAG[1] + P[11][2]*SH_MAG[0] - P[11][3]*SK_MY[2] - P[11][17]*SK_MY[1] - P[11][16]*SK_MY[3] + P[11][18]*SK_MY[4]);
            Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][0]*SH_MAG[2] + P[12][1]*SH_MAG[1] + P[12][2]*SH_MAG[0] - P[12][3]*SK_MY[2] - P[12][17]*SK_MY[1] - P[12][16]*SK_MY[3] + P[12][18]*SK_MY[4]);
            // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
            Kfusion[13] = 0.0f;//SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] - P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
            // Estimation of selected states is inhibited by setting their Kalman gains to zero
            if (!inhibitWindStates) {
                Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][17]*SK_MY[1] - P[14][16]*SK_MY[3] + P[14][18]*SK_MY[4]);
                Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][17]*SK_MY[1] - P[15][16]*SK_MY[3] + P[15][18]*SK_MY[4]);
            } else {
                Kfusion[14] = 0;
                Kfusion[15] = 0;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][17]*SK_MY[1] - P[16][16]*SK_MY[3] + P[16][18]*SK_MY[4]);
                Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][17]*SK_MY[1] - P[17][16]*SK_MY[3] + P[17][18]*SK_MY[4]);
                Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][17]*SK_MY[1] - P[18][16]*SK_MY[3] + P[18][18]*SK_MY[4]);
                Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][17]*SK_MY[1] - P[19][16]*SK_MY[3] + P[19][18]*SK_MY[4]);
                Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][17]*SK_MY[1] - P[20][16]*SK_MY[3] + P[20][18]*SK_MY[4]);
                Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][0]*SH_MAG[2] + P[21][1]*SH_MAG[1] + P[21][2]*SH_MAG[0] - P[21][3]*SK_MY[2] - P[21][17]*SK_MY[1] - P[21][16]*SK_MY[3] + P[21][18]*SK_MY[4]);
            } else {
                Kfusion[16] = 0;
                Kfusion[17] = 0;
                Kfusion[18] = 0;
                Kfusion[19] = 0;
                Kfusion[20] = 0;
                Kfusion[21] = 0;
            }
            if (!inhibitGndHgtState) {
                Kfusion[22] = SK_MY[0]*(P[22][20] + P[22][0]*SH_MAG[2] + P[22][1]*SH_MAG[1] + P[22][2]*SH_MAG[0] - P[22][3]*SK_MY[2] - P[22][17]*SK_MY[1] - P[22][16]*SK_MY[3] + P[22][18]*SK_MY[4]);
            } else {
                Kfusion[22] = 0;
            }
            varInnovMag[1] = 1.0f/SK_MY[0];
            innovMag[1] = MagPred[1] - magData.y;
        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // Calculate observation jacobians
            for (uint8_t i = 0; i < n_states; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[1];
            H_MAG[1] = 2*magN*q3 - 2*magE*q0 - 2*magD*q1;
            H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            H_MAG[3] = SH_MAG[0];
            H_MAG[16] = 2*q0*q2 + 2*q1*q3;
            H_MAG[17] = 2*q2*q3 - 2*q0*q1;
            H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            H_MAG[21] = 1;

            // Calculate Kalman gain
            float temp = (P[21][21] + R_MAG + P[0][21]*SH_MAG[1] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) - (2*magD*q1 + 2*magE*q0 - 2*magN*q3)*(P[21][1] + P[0][1]*SH_MAG[1] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(2*q0*q2 + 2*q1*q3) - P[17][1]*(2*q0*q1 - 2*q2*q3) - P[1][1]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(2*q0*q2 + 2*q1*q3) - P[17][2]*(2*q0*q1 - 2*q2*q3) - P[1][2]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(2*q0*q2 + 2*q1*q3) - P[17][0]*(2*q0*q1 - 2*q2*q3) - P[1][0]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(2*q0*q2 + 2*q1*q3) - P[17][3]*(2*q0*q1 - 2*q2*q3) - P[1][3]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(2*q0*q2 + 2*q1*q3) - P[17][18]*(2*q0*q1 - 2*q2*q3) - P[1][18]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[16][21]*(2*q0*q2 + 2*q1*q3) - P[17][21]*(2*q0*q1 - 2*q2*q3) - P[1][21]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + (2*q0*q2 + 2*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(2*q0*q2 + 2*q1*q3) - P[17][16]*(2*q0*q1 - 2*q2*q3) - P[1][16]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q1 - 2*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(2*q0*q2 + 2*q1*q3) - P[17][17]*(2*q0*q1 - 2*q2*q3) - P[1][17]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            if (temp >= R_MAG) {
                SK_MZ[0] = 1.0f / temp;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[21][21] += 0.1f*R_MAG;
                obsIndex = 3;
                return;
            }
            SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            SK_MZ[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
            SK_MZ[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MZ[4] = 2*q0*q1 - 2*q2*q3;
            SK_MZ[5] = 2*q0*q2 + 2*q1*q3;
            Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][0]*SH_MAG[1] + P[0][3]*SH_MAG[0] - P[0][1]*SK_MZ[2] + P[0][2]*SK_MZ[3] + P[0][18]*SK_MZ[1] + P[0][16]*SK_MZ[5] - P[0][17]*SK_MZ[4]);
            Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][0]*SH_MAG[1] + P[1][3]*SH_MAG[0] - P[1][1]*SK_MZ[2] + P[1][2]*SK_MZ[3] + P[1][18]*SK_MZ[1] + P[1][16]*SK_MZ[5] - P[1][17]*SK_MZ[4]);
            Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][0]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[2][1]*SK_MZ[2] + P[2][2]*SK_MZ[3] + P[2][18]*SK_MZ[1] + P[2][16]*SK_MZ[5] - P[2][17]*SK_MZ[4]);
            Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][0]*SH_MAG[1] + P[3][3]*SH_MAG[0] - P[3][1]*SK_MZ[2] + P[3][2]*SK_MZ[3] + P[3][18]*SK_MZ[1] + P[3][16]*SK_MZ[5] - P[3][17]*SK_MZ[4]);
            Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][0]*SH_MAG[1] + P[4][3]*SH_MAG[0] - P[4][1]*SK_MZ[2] + P[4][2]*SK_MZ[3] + P[4][18]*SK_MZ[1] + P[4][16]*SK_MZ[5] - P[4][17]*SK_MZ[4]);
            Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][0]*SH_MAG[1] + P[5][3]*SH_MAG[0] - P[5][1]*SK_MZ[2] + P[5][2]*SK_MZ[3] + P[5][18]*SK_MZ[1] + P[5][16]*SK_MZ[5] - P[5][17]*SK_MZ[4]);
            Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][0]*SH_MAG[1] + P[6][3]*SH_MAG[0] - P[6][1]*SK_MZ[2] + P[6][2]*SK_MZ[3] + P[6][18]*SK_MZ[1] + P[6][16]*SK_MZ[5] - P[6][17]*SK_MZ[4]);
            Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][0]*SH_MAG[1] + P[7][3]*SH_MAG[0] - P[7][1]*SK_MZ[2] + P[7][2]*SK_MZ[3] + P[7][18]*SK_MZ[1] + P[7][16]*SK_MZ[5] - P[7][17]*SK_MZ[4]);
            Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][0]*SH_MAG[1] + P[8][3]*SH_MAG[0] - P[8][1]*SK_MZ[2] + P[8][2]*SK_MZ[3] + P[8][18]*SK_MZ[1] + P[8][16]*SK_MZ[5] - P[8][17]*SK_MZ[4]);
            Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][0]*SH_MAG[1] + P[9][3]*SH_MAG[0] - P[9][1]*SK_MZ[2] + P[9][2]*SK_MZ[3] + P[9][18]*SK_MZ[1] + P[9][16]*SK_MZ[5] - P[9][17]*SK_MZ[4]);
            Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][0]*SH_MAG[1] + P[10][3]*SH_MAG[0] - P[10][1]*SK_MZ[2] + P[10][2]*SK_MZ[3] + P[10][18]*SK_MZ[1] + P[10][16]*SK_MZ[5] - P[10][17]*SK_MZ[4]);
            Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][0]*SH_MAG[1] + P[11][3]*SH_MAG[0] - P[11][1]*SK_MZ[2] + P[11][2]*SK_MZ[3] + P[11][18]*SK_MZ[1] + P[11][16]*SK_MZ[5] - P[11][17]*SK_MZ[4]);
            Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][0]*SH_MAG[1] + P[12][3]*SH_MAG[0] - P[12][1]*SK_MZ[2] + P[12][2]*SK_MZ[3] + P[12][18]*SK_MZ[1] + P[12][16]*SK_MZ[5] - P[12][17]*SK_MZ[4]);
            // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
            Kfusion[13] = 0.0f;//SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] + P[13][3]*SH_MAG[0] - P[13][1]*SK_MZ[2] + P[13][2]*SK_MZ[3] + P[13][18]*SK_MZ[1] + P[13][16]*SK_MZ[5] - P[13][17]*SK_MZ[4]);
            // Estimation of selected states is inhibited by setting their Kalman gains to zero
            if (!inhibitWindStates) {
                Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][0]*SH_MAG[1] + P[14][3]*SH_MAG[0] - P[14][1]*SK_MZ[2] + P[14][2]*SK_MZ[3] + P[14][18]*SK_MZ[1] + P[14][16]*SK_MZ[5] - P[14][17]*SK_MZ[4]);
                Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][0]*SH_MAG[1] + P[15][3]*SH_MAG[0] - P[15][1]*SK_MZ[2] + P[15][2]*SK_MZ[3] + P[15][18]*SK_MZ[1] + P[15][16]*SK_MZ[5] - P[15][17]*SK_MZ[4]);
            } else {
                Kfusion[14] = 0;
                Kfusion[15] = 0;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][0]*SH_MAG[1] + P[16][3]*SH_MAG[0] - P[16][1]*SK_MZ[2] + P[16][2]*SK_MZ[3] + P[16][18]*SK_MZ[1] + P[16][16]*SK_MZ[5] - P[16][17]*SK_MZ[4]);
                Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][0]*SH_MAG[1] + P[17][3]*SH_MAG[0] - P[17][1]*SK_MZ[2] + P[17][2]*SK_MZ[3] + P[17][18]*SK_MZ[1] + P[17][16]*SK_MZ[5] - P[17][17]*SK_MZ[4]);
                Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][0]*SH_MAG[1] + P[18][3]*SH_MAG[0] - P[18][1]*SK_MZ[2] + P[18][2]*SK_MZ[3] + P[18][18]*SK_MZ[1] + P[18][16]*SK_MZ[5] - P[18][17]*SK_MZ[4]);
                Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][0]*SH_MAG[1] + P[19][3]*SH_MAG[0] - P[19][1]*SK_MZ[2] + P[19][2]*SK_MZ[3] + P[19][18]*SK_MZ[1] + P[19][16]*SK_MZ[5] - P[19][17]*SK_MZ[4]);
                Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][0]*SH_MAG[1] + P[20][3]*SH_MAG[0] - P[20][1]*SK_MZ[2] + P[20][2]*SK_MZ[3] + P[20][18]*SK_MZ[1] + P[20][16]*SK_MZ[5] - P[20][17]*SK_MZ[4]);
                Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][0]*SH_MAG[1] + P[21][3]*SH_MAG[0] - P[21][1]*SK_MZ[2] + P[21][2]*SK_MZ[3] + P[21][18]*SK_MZ[1] + P[21][16]*SK_MZ[5] - P[21][17]*SK_MZ[4]);
            } else {
                Kfusion[16] = 0;
                Kfusion[17] = 0;
                Kfusion[18] = 0;
                Kfusion[19] = 0;
                Kfusion[20] = 0;
                Kfusion[21] = 0;
            }
            if (!inhibitGndHgtState) {
                Kfusion[22] = SK_MZ[0]*(P[22][21] + P[22][0]*SH_MAG[1] + P[22][3]*SH_MAG[0] - P[22][1]*SK_MZ[2] + P[22][2]*SK_MZ[3] + P[22][18]*SK_MZ[1] + P[22][16]*SK_MZ[5] - P[22][17]*SK_MZ[4]);
            } else {
                Kfusion[22] = 0;
            }
            varInnovMag[2] = 1.0f/SK_MZ[0];
            innovMag[2] = MagPred[2] - magData.z;

        }

        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovMag[obsIndex]*innovMag[obsIndex]/varInnovMag[obsIndex]) < 25.0f)
        {
            // correct the state vector
            for (uint8_t j= 0; j < n_states; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovMag[obsIndex];
            }
            // normalise the quaternion states
            float quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quatMag > 1e-12f)
            {
                for (uint8_t j= 0; j<=3; j++)
                {
                    float quatMagInv = 1.0f/quatMag;
                    states[j] = states[j] * quatMagInv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i < n_states; i++)
            {
                for (uint8_t j = 0; j <= 3; j++)
                {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                }
                for (uint8_t j = 4; j <= 15; j++) KH[i][j] = 0.0f;
                if (!onGround)
                {
                    for (uint8_t j = 16; j <= 21; j++)
                    {
                        KH[i][j] = Kfusion[i] * H_MAG[j];
                    }
                }
                else
                {
                    for (uint8_t j = 16; j <= 21; j++)
                    {
                        KH[i][j] = 0.0f;
                    }
                }
            }
            for (uint8_t i = 0; i < n_states; i++)
            {
                for (uint8_t j = 0; j < n_states; j++)
                {
                    KHP[i][j] = 0.0f;
                    for (uint8_t k = 0; k <= 3; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    if (!onGround)
                    {
                        for (uint8_t k = 16; k<=21; k++)
                        {
                            KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                        }
                    }
                }
            }
        }
        for (uint8_t i = 0; i < n_states; i++)
        {
            for (uint8_t j = 0; j < n_states; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }
    obsIndex = obsIndex + 1;

    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::FuseAirspeed()
{
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    float R_TAS = sq(airspeedMeasurementSigma);
    float SH_TAS[3];
    float SK_TAS;
    float VtasPred;

    // Copy required states to local variable names
    vn = statesAtVtasMeasTime[4];
    ve = statesAtVtasMeasTime[5];
    vd = statesAtVtasMeasTime[6];
    vwn = statesAtVtasMeasTime[14];
    vwe = statesAtVtasMeasTime[15];

    // Need to check that it is flying before fusing airspeed data
    // Calculate the predicted airspeed
    VtasPred = sqrtf((ve - vwe)*(ve - vwe) + (vn - vwn)*(vn - vwn) + vd*vd);
    // Perform fusion of True Airspeed measurement
    if (useAirspeed && fuseVtasData && (VtasPred > 1.0f) && (VtasMeas > 8.0f))
    {
        // Calculate observation jacobians
        SH_TAS[0] = 1/(sqrt(sq(ve - vwe) + sq(vn - vwn) + sq(vd)));
        SH_TAS[1] = (SH_TAS[0]*(2.0f*ve - 2*vwe))/2.0f;
        SH_TAS[2] = (SH_TAS[0]*(2.0f*vn - 2*vwn))/2.0f;

        float H_TAS[n_states];
        for (uint8_t i = 0; i < n_states; i++) H_TAS[i] = 0.0f;
        H_TAS[4] = SH_TAS[2];
        H_TAS[5] = SH_TAS[1];
        H_TAS[6] = vd*SH_TAS[0];
        H_TAS[14] = -SH_TAS[2];
        H_TAS[15] = -SH_TAS[1];

        // Calculate Kalman gains
        float temp = (R_TAS + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[14][4]*SH_TAS[2] - P[15][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[14][5]*SH_TAS[2] - P[15][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][14]*SH_TAS[2] + P[5][14]*SH_TAS[1] - P[14][14]*SH_TAS[2] - P[15][14]*SH_TAS[1] + P[6][14]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][15]*SH_TAS[2] + P[5][15]*SH_TAS[1] - P[14][15]*SH_TAS[2] - P[15][15]*SH_TAS[1] + P[6][15]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[14][6]*SH_TAS[2] - P[15][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
        if (temp >= R_TAS) {
            SK_TAS = 1.0f / temp;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we increase the wind state variances and try again next time
            P[14][14] += 0.05f*R_TAS;
            P[15][15] += 0.05f*R_TAS;
            return;
        }
        Kfusion[0] = SK_TAS*(P[0][4]*SH_TAS[2] - P[0][14]*SH_TAS[2] + P[0][5]*SH_TAS[1] - P[0][15]*SH_TAS[1] + P[0][6]*vd*SH_TAS[0]);
        Kfusion[1] = SK_TAS*(P[1][4]*SH_TAS[2] - P[1][14]*SH_TAS[2] + P[1][5]*SH_TAS[1] - P[1][15]*SH_TAS[1] + P[1][6]*vd*SH_TAS[0]);
        Kfusion[2] = SK_TAS*(P[2][4]*SH_TAS[2] - P[2][14]*SH_TAS[2] + P[2][5]*SH_TAS[1] - P[2][15]*SH_TAS[1] + P[2][6]*vd*SH_TAS[0]);
        Kfusion[3] = SK_TAS*(P[3][4]*SH_TAS[2] - P[3][14]*SH_TAS[2] + P[3][5]*SH_TAS[1] - P[3][15]*SH_TAS[1] + P[3][6]*vd*SH_TAS[0]);
        Kfusion[4] = SK_TAS*(P[4][4]*SH_TAS[2] - P[4][14]*SH_TAS[2] + P[4][5]*SH_TAS[1] - P[4][15]*SH_TAS[1] + P[4][6]*vd*SH_TAS[0]);
        Kfusion[5] = SK_TAS*(P[5][4]*SH_TAS[2] - P[5][14]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[5][15]*SH_TAS[1] + P[5][6]*vd*SH_TAS[0]);
        Kfusion[6] = SK_TAS*(P[6][4]*SH_TAS[2] - P[6][14]*SH_TAS[2] + P[6][5]*SH_TAS[1] - P[6][15]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]);
        Kfusion[7] = SK_TAS*(P[7][4]*SH_TAS[2] - P[7][14]*SH_TAS[2] + P[7][5]*SH_TAS[1] - P[7][15]*SH_TAS[1] + P[7][6]*vd*SH_TAS[0]);
        Kfusion[8] = SK_TAS*(P[8][4]*SH_TAS[2] - P[8][14]*SH_TAS[2] + P[8][5]*SH_TAS[1] - P[8][15]*SH_TAS[1] + P[8][6]*vd*SH_TAS[0]);
        Kfusion[9] = SK_TAS*(P[9][4]*SH_TAS[2] - P[9][14]*SH_TAS[2] + P[9][5]*SH_TAS[1] - P[9][15]*SH_TAS[1] + P[9][6]*vd*SH_TAS[0]);
        Kfusion[10] = SK_TAS*(P[10][4]*SH_TAS[2] - P[10][14]*SH_TAS[2] + P[10][5]*SH_TAS[1] - P[10][15]*SH_TAS[1] + P[10][6]*vd*SH_TAS[0]);
        Kfusion[11] = SK_TAS*(P[11][4]*SH_TAS[2] - P[11][14]*SH_TAS[2] + P[11][5]*SH_TAS[1] - P[11][15]*SH_TAS[1] + P[11][6]*vd*SH_TAS[0]);
        Kfusion[12] = SK_TAS*(P[12][4]*SH_TAS[2] - P[12][14]*SH_TAS[2] + P[12][5]*SH_TAS[1] - P[12][15]*SH_TAS[1] + P[12][6]*vd*SH_TAS[0]);
        // Only height measurements are allowed to modify the Z delta velocity bias state. This improves the stability of the estimate
        Kfusion[13] = 0.0f;//SK_TAS*(P[13][4]*SH_TAS[2] - P[13][14]*SH_TAS[2] + P[13][5]*SH_TAS[1] - P[13][15]*SH_TAS[1] + P[13][6]*vd*SH_TAS[0]);
        // Estimation of selected states is inhibited by setting their Kalman gains to zero
        if (!inhibitWindStates) {
            Kfusion[14] = SK_TAS*(P[14][4]*SH_TAS[2] - P[14][14]*SH_TAS[2] + P[14][5]*SH_TAS[1] - P[14][15]*SH_TAS[1] + P[14][6]*vd*SH_TAS[0]);
            Kfusion[15] = SK_TAS*(P[15][4]*SH_TAS[2] - P[15][14]*SH_TAS[2] + P[15][5]*SH_TAS[1] - P[15][15]*SH_TAS[1] + P[15][6]*vd*SH_TAS[0]);
        } else {
            Kfusion[14] = 0;
            Kfusion[15] = 0;
        }
        if (!inhibitMagStates) {
            Kfusion[16] = SK_TAS*(P[16][4]*SH_TAS[2] - P[16][14]*SH_TAS[2] + P[16][5]*SH_TAS[1] - P[16][15]*SH_TAS[1] + P[16][6]*vd*SH_TAS[0]);
            Kfusion[17] = SK_TAS*(P[17][4]*SH_TAS[2] - P[17][14]*SH_TAS[2] + P[17][5]*SH_TAS[1] - P[17][15]*SH_TAS[1] + P[17][6]*vd*SH_TAS[0]);
            Kfusion[18] = SK_TAS*(P[18][4]*SH_TAS[2] - P[18][14]*SH_TAS[2] + P[18][5]*SH_TAS[1] - P[18][15]*SH_TAS[1] + P[18][6]*vd*SH_TAS[0]);
            Kfusion[19] = SK_TAS*(P[19][4]*SH_TAS[2] - P[19][14]*SH_TAS[2] + P[19][5]*SH_TAS[1] - P[19][15]*SH_TAS[1] + P[19][6]*vd*SH_TAS[0]);
            Kfusion[20] = SK_TAS*(P[20][4]*SH_TAS[2] - P[20][14]*SH_TAS[2] + P[20][5]*SH_TAS[1] - P[20][15]*SH_TAS[1] + P[20][6]*vd*SH_TAS[0]);
            Kfusion[21] = SK_TAS*(P[21][4]*SH_TAS[2] - P[21][14]*SH_TAS[2] + P[21][5]*SH_TAS[1] - P[21][15]*SH_TAS[1] + P[21][6]*vd*SH_TAS[0]);
        } else {
            for (uint8_t i=16; i <= 21; i++) {
                Kfusion[i] = 0;
            }
        }
        if (!inhibitGndHgtState) {
            Kfusion[22] = SK_TAS*(P[22][4]*SH_TAS[2] - P[22][14]*SH_TAS[2] + P[22][5]*SH_TAS[1] - P[22][15]*SH_TAS[1] + P[22][6]*vd*SH_TAS[0]);
        } else {
            Kfusion[22] = 0;
        }
        varInnovVtas = 1.0f/SK_TAS;

        // Calculate the measurement innovation
        innovVtas = VtasPred - VtasMeas;
        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovVtas*innovVtas*SK_TAS) < 25.0f)
        {
            // correct the state vector
            for (uint8_t j=0; j <= 22; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovVtas;
            }
            // normalise the quaternion states
            float quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quatMag > 1e-12f)
            {
                for (uint8_t j= 0; j <= 3; j++)
                {
                    float quatMagInv = 1.0f/quatMag;
                    states[j] = states[j] * quatMagInv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in H to reduce the
            // number of operations
            for (uint8_t i = 0; i <= 22; i++)
            {
                for (uint8_t j = 0; j <= 3; j++) KH[i][j] = 0.0;
                for (uint8_t j = 4; j <= 6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 7; j <= 13; j++) KH[i][j] = 0.0;
                for (uint8_t j = 14; j <= 15; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 16; j <= 22; j++) KH[i][j] = 0.0;
            }
            for (uint8_t i = 0; i <= 22; i++)
            {
                for (uint8_t j = 0; j <= 22; j++)
                {
                    KHP[i][j] = 0.0;
                    for (uint8_t k = 4; k <= 6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    for (uint8_t k = 14; k <= 15; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
            for (uint8_t i = 0; i <= 22; i++)
            {
                for (uint8_t j = 0; j <= 22; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::zeroRows(float (&covMat)[n_states][n_states], uint8_t first, uint8_t last)
{
    uint8_t row;
    uint8_t col;
    for (row=first; row<=last; row++)
    {
        for (col=0; col<n_states; col++)
        {
            covMat[row][col] = 0.0;
        }
    }
}

void AttPosEKF::FuseRangeFinder()
{

    // Local variables
    float rngPred;
    float SH_RNG[5];
    float H_RNG[23];
    float SK_RNG[6];
    float cosRngTilt;
    const float R_RNG = 0.25f; // 0.5 m2 rangefinder measurement variance

    // Copy required states to local variable names
    float q0 = statesAtRngTime[0];
    float q1 = statesAtRngTime[1];
    float q2 = statesAtRngTime[2];
    float q3 = statesAtRngTime[3];
    float pd = statesAtRngTime[9];
    float ptd = statesAtRngTime[22];

    // Need to check that our range finder tilt angle is less than 30 degrees and we are using range finder data
    SH_RNG[4] = sinf(rngFinderPitch);
    cosRngTilt = - Tbn.z.x * SH_RNG[4] + Tbn.z.z * cosf(rngFinderPitch);
    if (useRangeFinder && fuseRngData && cosRngTilt > 0.87f)
    {
        // Calculate observation jacobian and Kalman gain ignoring all states other than the terrain offset
        // This prevents the range finder measurement modifying any of the other filter states and significantly reduces computations
        SH_RNG[0] = SH_RNG[4]*(2*q0*q2 - 2*q1*q3) - sq(q0) + sq(q1) + sq(q2) - sq(q3);
        SH_RNG[1] = pd - ptd;
        SH_RNG[2] = 1/sq(SH_RNG[0]);
        SH_RNG[3] = 1/SH_RNG[0];
        for (uint8_t i = 0; i < n_states; i++) {
            H_RNG[i] = 0.0f;
            Kfusion[i] = 0.0f;
        }
        H_RNG[22] = -SH_RNG[3];
        SK_RNG[0] = 1/(R_RNG + SH_RNG[3]*(P[9][9]*SH_RNG[3] - P[22][9]*SH_RNG[3] + P[0][9]*SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4]) - P[1][9]*SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4]) - P[2][9]*SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4]) + P[3][9]*SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])) - SH_RNG[3]*(P[9][22]*SH_RNG[3] - P[22][22]*SH_RNG[3] + P[0][22]*SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4]) - P[1][22]*SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4]) - P[2][22]*SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4]) + P[3][22]*SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])) + SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4])*(P[9][0]*SH_RNG[3] - P[22][0]*SH_RNG[3] + P[0][0]*SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4]) - P[1][0]*SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4]) - P[2][0]*SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4]) + P[3][0]*SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])) - SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4])*(P[9][1]*SH_RNG[3] - P[22][1]*SH_RNG[3] + P[0][1]*SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4]) - P[1][1]*SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4]) - P[2][1]*SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4]) + P[3][1]*SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])) - SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4])*(P[9][2]*SH_RNG[3] - P[22][2]*SH_RNG[3] + P[0][2]*SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4]) - P[1][2]*SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4]) - P[2][2]*SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4]) + P[3][2]*SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])) + SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])*(P[9][3]*SH_RNG[3] - P[22][3]*SH_RNG[3] + P[0][3]*SH_RNG[1]*SH_RNG[2]*(2*q0 - 2*q2*SH_RNG[4]) - P[1][3]*SH_RNG[1]*SH_RNG[2]*(2*q1 - 2*q3*SH_RNG[4]) - P[2][3]*SH_RNG[1]*SH_RNG[2]*(2*q2 + 2*q0*SH_RNG[4]) + P[3][3]*SH_RNG[1]*SH_RNG[2]*(2*q3 + 2*q1*SH_RNG[4])));
        SK_RNG[1] = 2*q1 - 2*q3*SH_RNG[4];
        SK_RNG[2] = 2*q0 - 2*q2*SH_RNG[4];
        SK_RNG[3] = 2*q3 + 2*q1*SH_RNG[4];
        SK_RNG[4] = 2*q2 + 2*q0*SH_RNG[4];
        SK_RNG[5] = SH_RNG[2];
        Kfusion[22] = SK_RNG[0]*(P[22][9]*SH_RNG[3] - P[22][22]*SH_RNG[3] + P[22][0]*SH_RNG[1]*SK_RNG[2]*SK_RNG[5] - P[22][1]*SH_RNG[1]*SK_RNG[1]*SK_RNG[5] - P[22][2]*SH_RNG[1]*SK_RNG[4]*SK_RNG[5] + P[22][3]*SH_RNG[1]*SK_RNG[3]*SK_RNG[5]);

        // Calculate the innovation variance for data logging
        varInnovRng = 1.0f/SK_RNG[0];

        // Calculate the measurement innovation
        rngPred = (ptd - pd)/cosRngTilt;
        innovRng = rngPred - rngMea;

        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovRng*innovRng*SK_RNG[0]) < 25)
        {
            // correct the state vector
            states[22] = states[22] - Kfusion[22] * innovRng;

            // correct the covariance P = (I - K*H)*P
            P[22][22] = P[22][22] - Kfusion[22] * H_RNG[22] * P[22][22];
            P[22][22] = ConstrainFloat(P[22][22], 0.0f, 10000.0f);
        }
    }

}

void AttPosEKF::FuseOptFlow()
{
    static uint8_t obsIndex;
    static float SH_LOS[13];
    static float SKK_LOS[15];
    static float SK_LOS[2];
    static float q0 = 0.0f;
    static float q1 = 0.0f;
    static float q2 = 0.0f;
    static float q3 = 1.0f;
    static float vn = 0.0f;
    static float ve = 0.0f;
    static float vd = 0.0f;
    static float pd = 0.0f;
    static float ptd = 0.0f;
    static float R_LOS = 0.01f;
    static float losPred[2];

    // Transformation matrix from nav to body axes
    Mat3f Tnb_local;
    // Transformation matrix from body to sensor axes
    // assume camera is aligned with Z body axis plus a misalignment
    // defined by 3 small angles about X, Y and Z body axis
    Mat3f Tbs;
    Tbs.x.y =  a3;
    Tbs.y.x = -a3;
    Tbs.x.z = -a2;
    Tbs.z.x =  a2;
    Tbs.y.z =  a1;
    Tbs.z.y = -a1;
    // Transformation matrix from navigation to sensor axes
    Mat3f Tns;
    float H_LOS[n_states];
    for (uint8_t i = 0; i < n_states; i++) {
        H_LOS[i] = 0.0f;
    }
    Vector3f velNED_local;
    Vector3f relVelSensor;

// Perform sequential fusion of optical flow measurements only when in the air and tilt is less than 30 deg.
    if (useOpticalFlow && (fuseOptFlowData || obsIndex == 1) && !onGround && Tbs.z.z > 0.866f && rngMea > 5.0f && rngMea < 39.0f)
    {
        // Sequential fusion of XY components to spread processing load across
        // two prediction time steps.

        // Calculate observation jacobians and Kalman gains
        if (fuseOptFlowData)
        {
            // Copy required states to local variable names
            q0       = statesAtOptFlowTime[0];
            q1       = statesAtOptFlowTime[1];
            q2       = statesAtOptFlowTime[2];
            q3       = statesAtOptFlowTime[3];
            vn       = statesAtOptFlowTime[4];
            ve       = statesAtOptFlowTime[5];
            vd       = statesAtOptFlowTime[6];
            pd       = statesAtOptFlowTime[9];
            ptd      = statesAtOptFlowTime[22];
            velNED_local.x = vn;
            velNED_local.y = ve;
            velNED_local.z = vd;

            // calculate rotation from NED to body axes
            float q00 = sq(q0);
            float q11 = sq(q1);
            float q22 = sq(q2);
            float q33 = sq(q3);
            float q01 = q0 * q1;
            float q02 = q0 * q2;
            float q03 = q0 * q3;
            float q12 = q1 * q2;
            float q13 = q1 * q3;
            float q23 = q2 * q3;
            Tnb_local.x.x = q00 + q11 - q22 - q33;
            Tnb_local.y.y = q00 - q11 + q22 - q33;
            Tnb_local.z.z = q00 - q11 - q22 + q33;
            Tnb_local.y.x = 2*(q12 - q03);
            Tnb_local.z.x = 2*(q13 + q02);
            Tnb_local.x.y = 2*(q12 + q03);
            Tnb_local.z.y = 2*(q23 - q01);
            Tnb_local.x.z = 2*(q13 - q02);
            Tnb_local.y.z = 2*(q23 + q01);

            // calculate transformation from NED to sensor axes
            Tns = Tbs*Tnb_local;

            // calculate range from ground plain to centre of sensor fov assuming flat earth
            float range = ConstrainFloat(((ptd - pd)/Tns.z.z),0.5f,100.0f);

            // calculate relative velocity in sensor frame
            relVelSensor = Tns*velNED_local;

            // divide velocity by range  and include angular rate effects to get predicted angular LOS rates relative to X and Y axes
            losPred[0] =  relVelSensor.y/range;
            losPred[1] = -relVelSensor.x/range;

            //printf("relVelSensor.x=%5.1f, relVelSensor.y=%5.1f\n", relVelSensor.x, relVelSensor.y);
            //printf("Xpred=%5.2f, Xmea=%5.2f, Ypred=%5.2f, Ymea=%5.2f, delAng.x=%4.4f, delAng.y=%4.4f\n", losPred[0], losData[0], losPred[1], losData[1], delAng.x, delAng.y);
            //printf("omegaX=%5.2f, omegaY=%5.2f, velY=%5.1f velX=%5.1f\n, range=%5.1f\n", delAngRel.x/dt, delAngRel.y/dt, relVelSensor.y, relVelSensor.x, range);

            // Calculate observation jacobians
            SH_LOS[0] = a1*(2*q0*q1 + 2*q2*q3) + a2*(2*q0*q2 - 2*q1*q3) - sq(q0) + sq(q1) + sq(q2) - sq(q3);
            SH_LOS[1] = vd*(a2*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + 2*q0*q2 - 2*q1*q3 - a3*(2*q0*q1 + 2*q2*q3)) - ve*(a3*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + 2*q0*q3 + 2*q1*q2 + a2*(2*q0*q1 - 2*q2*q3)) + vn*(a2*(2*q0*q2 + 2*q1*q3) + a3*(2*q0*q3 - 2*q1*q2) - sq(q0) - sq(q1) + sq(q2) + sq(q3));
            SH_LOS[2] = ve*(a1*(2*q0*q1 - 2*q2*q3) + a3*(2*q0*q3 + 2*q1*q2) - sq(q0) + sq(q1) - sq(q2) + sq(q3)) - vd*(a1*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + 2*q0*q1 + 2*q2*q3 + a3*(2*q0*q2 - 2*q1*q3)) + vn*(a3*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) + 2*q0*q3 - 2*q1*q2 - a1*(2*q0*q2 + 2*q1*q3));
            SH_LOS[3] = 1/(pd - ptd);
            SH_LOS[4] = 2*q1 - 2*a2*q3 + 2*a3*q2;
            SH_LOS[5] = 2*a2*q2 - 2*q0 + 2*a3*q3;
            SH_LOS[6] = 2*q2 + 2*a2*q0 - 2*a3*q1;
            SH_LOS[7] = 1/sq(pd - ptd);
            SH_LOS[8] = 2*q2 + 2*a1*q3 - 2*a3*q1;
            SH_LOS[9] = 2*q3 - 2*a1*q2 + 2*a3*q0;
            SH_LOS[10] = 2*a1*q1 - 2*q0 + 2*a3*q3;
            SH_LOS[11] = 2*q3 + 2*a2*q1 + 2*a3*q0;
            SH_LOS[12] = 2*q1 + 2*a1*q0 + 2*a3*q2;

            for (uint8_t i = 0; i < n_states; i++) H_LOS[i] = 0;
            H_LOS[0] = - SH_LOS[2]*SH_LOS[3]*(2*a1*q1 - 2*q0 + 2*a2*q2) - SH_LOS[0]*SH_LOS[3]*(ve*SH_LOS[10] - vd*SH_LOS[12] + vn*SH_LOS[9]);
            H_LOS[1] = - SH_LOS[2]*SH_LOS[3]*(2*q1 + 2*a1*q0 - 2*a2*q3) - SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[10] + ve*SH_LOS[12] - vn*SH_LOS[8]);
            H_LOS[2] = SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[9] + ve*SH_LOS[8] + vn*SH_LOS[12]) - SH_LOS[2]*SH_LOS[3]*(2*q2 + 2*a2*q0 + 2*a1*q3);
            H_LOS[3] = SH_LOS[2]*SH_LOS[3]*(2*q3 - 2*a1*q2 + 2*a2*q1) + SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[8] - ve*SH_LOS[9] + vn*SH_LOS[10]);
            H_LOS[4] = -SH_LOS[0]*SH_LOS[3]*(a3*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) + 2*q0*q3 - 2*q1*q2 - a1*(2*q0*q2 + 2*q1*q3));
            H_LOS[5] = -SH_LOS[0]*SH_LOS[3]*(a1*(2*q0*q1 - 2*q2*q3) + a3*(2*q0*q3 + 2*q1*q2) - sq(q0) + sq(q1) - sq(q2) + sq(q3));
            H_LOS[6] = SH_LOS[0]*SH_LOS[3]*(a1*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + 2*q0*q1 + 2*q2*q3 + a3*(2*q0*q2 - 2*q1*q3));
            H_LOS[9] = SH_LOS[0]*SH_LOS[2]*SH_LOS[7];
            H_LOS[22] = -SH_LOS[0]*SH_LOS[2]*SH_LOS[7];

            // Calculate Kalman gain
            SKK_LOS[0] = a2*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + 2*q0*q2 - 2*q1*q3 - a3*(2*q0*q1 + 2*q2*q3);
            SKK_LOS[1] = a3*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + 2*q0*q3 + 2*q1*q2 + a2*(2*q0*q1 - 2*q2*q3);
            SKK_LOS[2] = a2*(2*q0*q2 + 2*q1*q3) + a3*(2*q0*q3 - 2*q1*q2) - sq(q0) - sq(q1) + sq(q2) + sq(q3);
            SKK_LOS[3] = a1*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + 2*q0*q1 + 2*q2*q3 + a3*(2*q0*q2 - 2*q1*q3);
            SKK_LOS[4] = a1*(2*q0*q1 - 2*q2*q3) + a3*(2*q0*q3 + 2*q1*q2) - sq(q0) + sq(q1) - sq(q2) + sq(q3);
            SKK_LOS[5] = a3*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) + 2*q0*q3 - 2*q1*q2 - a1*(2*q0*q2 + 2*q1*q3);
            SKK_LOS[6] = SH_LOS[2]*SH_LOS[3]*(2*q3 - 2*a1*q2 + 2*a2*q1) + SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[8] - ve*SH_LOS[9] + vn*SH_LOS[10]);
            SKK_LOS[7] = SH_LOS[2]*SH_LOS[3]*(2*q1 + 2*a1*q0 - 2*a2*q3) + SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[10] + ve*SH_LOS[12] - vn*SH_LOS[8]);
            SKK_LOS[8] = SH_LOS[2]*SH_LOS[3]*(2*a1*q1 - 2*q0 + 2*a2*q2) + SH_LOS[0]*SH_LOS[3]*(ve*SH_LOS[10] - vd*SH_LOS[12] + vn*SH_LOS[9]);
            SKK_LOS[9] = SH_LOS[2]*SH_LOS[3]*(2*q2 + 2*a2*q0 + 2*a1*q3) - SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[9] + ve*SH_LOS[8] + vn*SH_LOS[12]);
            SKK_LOS[10] = SH_LOS[1]*SH_LOS[3]*(2*a1*q1 - 2*q0 + 2*a2*q2) + SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[6] - ve*SH_LOS[11] + vn*SH_LOS[5]);
            SKK_LOS[11] = SH_LOS[1]*SH_LOS[3]*(2*q1 + 2*a1*q0 - 2*a2*q3) - SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[11] + ve*SH_LOS[6] + vn*SH_LOS[4]);
            SKK_LOS[12] = SH_LOS[1]*SH_LOS[3]*(2*q2 + 2*a2*q0 + 2*a1*q3) - SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[5] + ve*SH_LOS[4] - vn*SH_LOS[6]);
            SKK_LOS[13] = SH_LOS[1]*SH_LOS[3]*(2*q3 - 2*a1*q2 + 2*a2*q1) - SH_LOS[0]*SH_LOS[3]*(ve*SH_LOS[5] - vd*SH_LOS[4] + vn*SH_LOS[11]);
            SKK_LOS[14] = SH_LOS[0];

            SK_LOS[0] = 1/(R_LOS + SKK_LOS[8]*(P[0][0]*SKK_LOS[8] + P[1][0]*SKK_LOS[7] + P[2][0]*SKK_LOS[9] - P[3][0]*SKK_LOS[6] - P[9][0]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][0]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][0]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][0]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][0]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) + SKK_LOS[7]*(P[0][1]*SKK_LOS[8] + P[1][1]*SKK_LOS[7] + P[2][1]*SKK_LOS[9] - P[3][1]*SKK_LOS[6] - P[9][1]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][1]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][1]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][1]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][1]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) + SKK_LOS[9]*(P[0][2]*SKK_LOS[8] + P[1][2]*SKK_LOS[7] + P[2][2]*SKK_LOS[9] - P[3][2]*SKK_LOS[6] - P[9][2]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][2]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][2]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][2]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][2]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) - SKK_LOS[6]*(P[0][3]*SKK_LOS[8] + P[1][3]*SKK_LOS[7] + P[2][3]*SKK_LOS[9] - P[3][3]*SKK_LOS[6] - P[9][3]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][3]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][3]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][3]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][3]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) - SH_LOS[2]*SH_LOS[7]*SKK_LOS[14]*(P[0][9]*SKK_LOS[8] + P[1][9]*SKK_LOS[7] + P[2][9]*SKK_LOS[9] - P[3][9]*SKK_LOS[6] - P[9][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][9]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][9]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][9]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) + SH_LOS[2]*SH_LOS[7]*SKK_LOS[14]*(P[0][22]*SKK_LOS[8] + P[1][22]*SKK_LOS[7] + P[2][22]*SKK_LOS[9] - P[3][22]*SKK_LOS[6] - P[9][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][22]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][22]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][22]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) + SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14]*(P[0][4]*SKK_LOS[8] + P[1][4]*SKK_LOS[7] + P[2][4]*SKK_LOS[9] - P[3][4]*SKK_LOS[6] - P[9][4]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][4]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][4]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][4]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) + SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14]*(P[0][5]*SKK_LOS[8] + P[1][5]*SKK_LOS[7] + P[2][5]*SKK_LOS[9] - P[3][5]*SKK_LOS[6] - P[9][5]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][5]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][5]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][5]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]) - SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]*(P[0][6]*SKK_LOS[8] + P[1][6]*SKK_LOS[7] + P[2][6]*SKK_LOS[9] - P[3][6]*SKK_LOS[6] - P[9][6]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][6]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][6]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][6]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]));
            Kfusion[0] = -SK_LOS[0]*(P[0][0]*SKK_LOS[8] + P[0][1]*SKK_LOS[7] - P[0][3]*SKK_LOS[6] + P[0][2]*SKK_LOS[9] - P[0][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[0][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[0][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[0][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[0][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[1] = -SK_LOS[0]*(P[1][0]*SKK_LOS[8] + P[1][1]*SKK_LOS[7] - P[1][3]*SKK_LOS[6] + P[1][2]*SKK_LOS[9] - P[1][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[1][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[1][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[1][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[1][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[2] = -SK_LOS[0]*(P[2][0]*SKK_LOS[8] + P[2][1]*SKK_LOS[7] - P[2][3]*SKK_LOS[6] + P[2][2]*SKK_LOS[9] - P[2][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[2][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[2][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[2][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[2][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[3] = -SK_LOS[0]*(P[3][0]*SKK_LOS[8] + P[3][1]*SKK_LOS[7] - P[3][3]*SKK_LOS[6] + P[3][2]*SKK_LOS[9] - P[3][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[3][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[3][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[3][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[3][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[4] = -SK_LOS[0]*(P[4][0]*SKK_LOS[8] + P[4][1]*SKK_LOS[7] - P[4][3]*SKK_LOS[6] + P[4][2]*SKK_LOS[9] - P[4][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[4][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[4][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[4][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[5] = -SK_LOS[0]*(P[5][0]*SKK_LOS[8] + P[5][1]*SKK_LOS[7] - P[5][3]*SKK_LOS[6] + P[5][2]*SKK_LOS[9] - P[5][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[5][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[5][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[5][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[5][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[6] = -SK_LOS[0]*(P[6][0]*SKK_LOS[8] + P[6][1]*SKK_LOS[7] - P[6][3]*SKK_LOS[6] + P[6][2]*SKK_LOS[9] - P[6][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[6][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[6][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[6][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[6][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[7] = -SK_LOS[0]*(P[7][0]*SKK_LOS[8] + P[7][1]*SKK_LOS[7] - P[7][3]*SKK_LOS[6] + P[7][2]*SKK_LOS[9] - P[7][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[7][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[7][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[7][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[7][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[8] = -SK_LOS[0]*(P[8][0]*SKK_LOS[8] + P[8][1]*SKK_LOS[7] - P[8][3]*SKK_LOS[6] + P[8][2]*SKK_LOS[9] - P[8][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[8][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[8][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[8][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[8][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[9] = -SK_LOS[0]*(P[9][0]*SKK_LOS[8] + P[9][1]*SKK_LOS[7] - P[9][3]*SKK_LOS[6] + P[9][2]*SKK_LOS[9] - P[9][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[9][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[9][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[9][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[9][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[10] = -SK_LOS[0]*(P[10][0]*SKK_LOS[8] + P[10][1]*SKK_LOS[7] - P[10][3]*SKK_LOS[6] + P[10][2]*SKK_LOS[9] - P[10][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[10][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[10][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[10][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[10][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[11] = -SK_LOS[0]*(P[11][0]*SKK_LOS[8] + P[11][1]*SKK_LOS[7] - P[11][3]*SKK_LOS[6] + P[11][2]*SKK_LOS[9] - P[11][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[11][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[11][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[11][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[11][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[12] = -SK_LOS[0]*(P[12][0]*SKK_LOS[8] + P[12][1]*SKK_LOS[7] - P[12][3]*SKK_LOS[6] + P[12][2]*SKK_LOS[9] - P[12][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[12][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[12][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[12][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[12][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[13] = 0.0f;//-SK_LOS[0]*(P[13][0]*SKK_LOS[8] + P[13][1]*SKK_LOS[7] - P[13][3]*SKK_LOS[6] + P[13][2]*SKK_LOS[9] - P[13][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[13][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[13][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[13][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[13][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[14] = -SK_LOS[0]*(P[14][0]*SKK_LOS[8] + P[14][1]*SKK_LOS[7] - P[14][3]*SKK_LOS[6] + P[14][2]*SKK_LOS[9] - P[14][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[14][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[14][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[14][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[14][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[15] = -SK_LOS[0]*(P[15][0]*SKK_LOS[8] + P[15][1]*SKK_LOS[7] - P[15][3]*SKK_LOS[6] + P[15][2]*SKK_LOS[9] - P[15][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[15][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[15][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[15][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[15][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[16] = -SK_LOS[0]*(P[16][0]*SKK_LOS[8] + P[16][1]*SKK_LOS[7] - P[16][3]*SKK_LOS[6] + P[16][2]*SKK_LOS[9] - P[16][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[16][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[16][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[16][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[16][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[17] = -SK_LOS[0]*(P[17][0]*SKK_LOS[8] + P[17][1]*SKK_LOS[7] - P[17][3]*SKK_LOS[6] + P[17][2]*SKK_LOS[9] - P[17][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[17][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[17][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[17][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[17][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[18] = -SK_LOS[0]*(P[18][0]*SKK_LOS[8] + P[18][1]*SKK_LOS[7] - P[18][3]*SKK_LOS[6] + P[18][2]*SKK_LOS[9] - P[18][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[18][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[18][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[18][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[18][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[19] = -SK_LOS[0]*(P[19][0]*SKK_LOS[8] + P[19][1]*SKK_LOS[7] - P[19][3]*SKK_LOS[6] + P[19][2]*SKK_LOS[9] - P[19][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[19][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[19][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[19][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[19][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[20] = -SK_LOS[0]*(P[20][0]*SKK_LOS[8] + P[20][1]*SKK_LOS[7] - P[20][3]*SKK_LOS[6] + P[20][2]*SKK_LOS[9] - P[20][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[20][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[20][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[20][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[20][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[21] = -SK_LOS[0]*(P[21][0]*SKK_LOS[8] + P[21][1]*SKK_LOS[7] - P[21][3]*SKK_LOS[6] + P[21][2]*SKK_LOS[9] - P[21][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[21][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[21][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[21][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[21][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            Kfusion[22] = -SK_LOS[0]*(P[22][0]*SKK_LOS[8] + P[22][1]*SKK_LOS[7] - P[22][3]*SKK_LOS[6] + P[22][2]*SKK_LOS[9] - P[22][9]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][22]*SH_LOS[2]*SH_LOS[7]*SKK_LOS[14] + P[22][4]*SH_LOS[3]*SKK_LOS[5]*SKK_LOS[14] + P[22][5]*SH_LOS[3]*SKK_LOS[4]*SKK_LOS[14] - P[22][6]*SH_LOS[3]*SKK_LOS[3]*SKK_LOS[14]);
            varInnovOptFlow[0] = 1.0f/SK_LOS[0];
            innovOptFlow[0] = losPred[0] - losData[0];

            // reset the observation index to 0 (we start by fusing the X
            // measurement)
            obsIndex = 0;
            fuseOptFlowData = false;
        }
        else if (obsIndex == 1) // we are now fusing the Y measurement
        {
            // Calculate observation jacobians
            for (uint8_t i = 0; i < n_states; i++) H_LOS[i] = 0;
            H_LOS[0] = SH_LOS[1]*SH_LOS[3]*(2*a1*q1 - 2*q0 + 2*a2*q2) + SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[6] - ve*SH_LOS[11] + vn*SH_LOS[5]);
            H_LOS[1] = SH_LOS[1]*SH_LOS[3]*(2*q1 + 2*a1*q0 - 2*a2*q3) - SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[11] + ve*SH_LOS[6] + vn*SH_LOS[4]);
            H_LOS[2] = SH_LOS[1]*SH_LOS[3]*(2*q2 + 2*a2*q0 + 2*a1*q3) - SH_LOS[0]*SH_LOS[3]*(vd*SH_LOS[5] + ve*SH_LOS[4] - vn*SH_LOS[6]);
            H_LOS[3] = SH_LOS[0]*SH_LOS[3]*(ve*SH_LOS[5] - vd*SH_LOS[4] + vn*SH_LOS[11]) - SH_LOS[1]*SH_LOS[3]*(2*q3 - 2*a1*q2 + 2*a2*q1);
            H_LOS[4] = SH_LOS[0]*SH_LOS[3]*(a2*(2*q0*q2 + 2*q1*q3) + a3*(2*q0*q3 - 2*q1*q2) - sq(q0) - sq(q1) + sq(q2) + sq(q3));
            H_LOS[5] = -SH_LOS[0]*SH_LOS[3]*(a3*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + 2*q0*q3 + 2*q1*q2 + a2*(2*q0*q1 - 2*q2*q3));
            H_LOS[6] = SH_LOS[0]*SH_LOS[3]*(a2*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + 2*q0*q2 - 2*q1*q3 - a3*(2*q0*q1 + 2*q2*q3));
            H_LOS[9] = -SH_LOS[0]*SH_LOS[1]*SH_LOS[7];
            H_LOS[22] = SH_LOS[0]*SH_LOS[1]*SH_LOS[7];

            // Calculate Kalman gains
            SK_LOS[1] = 1/(R_LOS + SKK_LOS[12]*(P[0][2]*SKK_LOS[10] + P[1][2]*SKK_LOS[11] + P[2][2]*SKK_LOS[12] - P[3][2]*SKK_LOS[13] - P[9][2]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][2]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][2]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][2]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][2]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) - SKK_LOS[13]*(P[0][3]*SKK_LOS[10] + P[1][3]*SKK_LOS[11] + P[2][3]*SKK_LOS[12] - P[3][3]*SKK_LOS[13] - P[9][3]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][3]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][3]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][3]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][3]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) + SKK_LOS[10]*(P[0][0]*SKK_LOS[10] + P[1][0]*SKK_LOS[11] + P[2][0]*(SH_LOS[1]*SH_LOS[3]*(2*q2 + 2*a2*q0 + 2*a1*q3) - SH_LOS[3]*SKK_LOS[14]*(vd*SH_LOS[5] + ve*SH_LOS[4] - vn*SH_LOS[6])) - P[3][0]*(SH_LOS[1]*SH_LOS[3]*(2*q3 - 2*a1*q2 + 2*a2*q1) - SH_LOS[3]*SKK_LOS[14]*(ve*SH_LOS[5] - vd*SH_LOS[4] + vn*SH_LOS[11])) - P[9][0]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][0]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][0]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][0]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][0]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) + SKK_LOS[11]*(P[0][1]*SKK_LOS[10] + P[1][1]*SKK_LOS[11] + P[2][1]*(SH_LOS[1]*SH_LOS[3]*(2*q2 + 2*a2*q0 + 2*a1*q3) - SH_LOS[3]*SKK_LOS[14]*(vd*SH_LOS[5] + ve*SH_LOS[4] - vn*SH_LOS[6])) - P[3][1]*(SH_LOS[1]*SH_LOS[3]*(2*q3 - 2*a1*q2 + 2*a2*q1) - SH_LOS[3]*SKK_LOS[14]*(ve*SH_LOS[5] - vd*SH_LOS[4] + vn*SH_LOS[11])) - P[9][1]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][1]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][1]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][1]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][1]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) - SH_LOS[1]*SH_LOS[7]*SKK_LOS[14]*(P[0][9]*SKK_LOS[10] + P[1][9]*SKK_LOS[11] + P[2][9]*SKK_LOS[12] - P[3][9]*SKK_LOS[13] - P[9][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][9]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][9]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][9]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) + SH_LOS[1]*SH_LOS[7]*SKK_LOS[14]*(P[0][22]*SKK_LOS[10] + P[1][22]*SKK_LOS[11] + P[2][22]*SKK_LOS[12] - P[3][22]*SKK_LOS[13] - P[9][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][22]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][22]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][22]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) + SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14]*(P[0][4]*SKK_LOS[10] + P[1][4]*SKK_LOS[11] + P[2][4]*SKK_LOS[12] - P[3][4]*SKK_LOS[13] - P[9][4]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][4]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][4]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][4]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) - SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]*(P[0][5]*SKK_LOS[10] + P[1][5]*SKK_LOS[11] + P[2][5]*SKK_LOS[12] - P[3][5]*SKK_LOS[13] - P[9][5]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][5]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][5]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][5]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]) + SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]*(P[0][6]*SKK_LOS[10] + P[1][6]*SKK_LOS[11] + P[2][6]*SKK_LOS[12] - P[3][6]*SKK_LOS[13] - P[9][6]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][6]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][6]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][6]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14] + P[6][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14]));
            Kfusion[0] = SK_LOS[1]*(P[0][0]*SKK_LOS[10] - P[0][3]*SKK_LOS[13] + P[0][1]*SKK_LOS[11] + P[0][2]*SKK_LOS[12] - P[0][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[0][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[0][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[0][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[0][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[1] = SK_LOS[1]*(P[1][0]*SKK_LOS[10] - P[1][3]*SKK_LOS[13] + P[1][1]*SKK_LOS[11] + P[1][2]*SKK_LOS[12] - P[1][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[1][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[1][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[1][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[1][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[2] = SK_LOS[1]*(P[2][0]*SKK_LOS[10] - P[2][3]*SKK_LOS[13] + P[2][1]*SKK_LOS[11] + P[2][2]*SKK_LOS[12] - P[2][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[2][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[2][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[2][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[2][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[3] = SK_LOS[1]*(P[3][0]*SKK_LOS[10] - P[3][3]*SKK_LOS[13] + P[3][1]*SKK_LOS[11] + P[3][2]*SKK_LOS[12] - P[3][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[3][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[3][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[3][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[3][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[4] = SK_LOS[1]*(P[4][0]*SKK_LOS[10] - P[4][3]*SKK_LOS[13] + P[4][1]*SKK_LOS[11] + P[4][2]*SKK_LOS[12] - P[4][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[4][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[4][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[4][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[5] = SK_LOS[1]*(P[5][0]*SKK_LOS[10] - P[5][3]*SKK_LOS[13] + P[5][1]*SKK_LOS[11] + P[5][2]*SKK_LOS[12] - P[5][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[5][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[5][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[5][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[5][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[6] = SK_LOS[1]*(P[6][0]*SKK_LOS[10] - P[6][3]*SKK_LOS[13] + P[6][1]*SKK_LOS[11] + P[6][2]*SKK_LOS[12] - P[6][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[6][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[6][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[6][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[6][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[7] = SK_LOS[1]*(P[7][0]*SKK_LOS[10] - P[7][3]*SKK_LOS[13] + P[7][1]*SKK_LOS[11] + P[7][2]*SKK_LOS[12] - P[7][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[7][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[7][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[7][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[7][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[8] = SK_LOS[1]*(P[8][0]*SKK_LOS[10] - P[8][3]*SKK_LOS[13] + P[8][1]*SKK_LOS[11] + P[8][2]*SKK_LOS[12] - P[8][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[8][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[8][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[8][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[8][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[9] = SK_LOS[1]*(P[9][0]*SKK_LOS[10] - P[9][3]*SKK_LOS[13] + P[9][1]*SKK_LOS[11] + P[9][2]*SKK_LOS[12] - P[9][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[9][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[9][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[9][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[9][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[10] = SK_LOS[1]*(P[10][0]*SKK_LOS[10] - P[10][3]*SKK_LOS[13] + P[10][1]*SKK_LOS[11] + P[10][2]*SKK_LOS[12] - P[10][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[10][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[10][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[10][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[10][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[11] = SK_LOS[1]*(P[11][0]*SKK_LOS[10] - P[11][3]*SKK_LOS[13] + P[11][1]*SKK_LOS[11] + P[11][2]*SKK_LOS[12] - P[11][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[11][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[11][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[11][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[11][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[12] = SK_LOS[1]*(P[12][0]*SKK_LOS[10] - P[12][3]*SKK_LOS[13] + P[12][1]*SKK_LOS[11] + P[12][2]*SKK_LOS[12] - P[12][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[12][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[12][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[12][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[12][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[13] = 0.0f;//SK_LOS[1]*(P[13][0]*SKK_LOS[10] - P[13][3]*SKK_LOS[13] + P[13][1]*SKK_LOS[11] + P[13][2]*SKK_LOS[12] - P[13][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[13][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[13][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[13][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[13][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[14] = SK_LOS[1]*(P[14][0]*SKK_LOS[10] - P[14][3]*SKK_LOS[13] + P[14][1]*SKK_LOS[11] + P[14][2]*SKK_LOS[12] - P[14][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[14][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[14][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[14][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[14][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[15] = SK_LOS[1]*(P[15][0]*SKK_LOS[10] - P[15][3]*SKK_LOS[13] + P[15][1]*SKK_LOS[11] + P[15][2]*SKK_LOS[12] - P[15][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[15][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[15][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[15][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[15][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[16] = SK_LOS[1]*(P[16][0]*SKK_LOS[10] - P[16][3]*SKK_LOS[13] + P[16][1]*SKK_LOS[11] + P[16][2]*SKK_LOS[12] - P[16][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[16][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[16][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[16][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[16][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[17] = SK_LOS[1]*(P[17][0]*SKK_LOS[10] - P[17][3]*SKK_LOS[13] + P[17][1]*SKK_LOS[11] + P[17][2]*SKK_LOS[12] - P[17][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[17][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[17][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[17][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[17][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[18] = SK_LOS[1]*(P[18][0]*SKK_LOS[10] - P[18][3]*SKK_LOS[13] + P[18][1]*SKK_LOS[11] + P[18][2]*SKK_LOS[12] - P[18][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[18][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[18][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[18][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[18][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[19] = SK_LOS[1]*(P[19][0]*SKK_LOS[10] - P[19][3]*SKK_LOS[13] + P[19][1]*SKK_LOS[11] + P[19][2]*SKK_LOS[12] - P[19][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[19][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[19][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[19][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[19][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[20] = SK_LOS[1]*(P[20][0]*SKK_LOS[10] - P[20][3]*SKK_LOS[13] + P[20][1]*SKK_LOS[11] + P[20][2]*SKK_LOS[12] - P[20][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[20][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[20][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[20][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[20][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[21] = SK_LOS[1]*(P[21][0]*SKK_LOS[10] - P[21][3]*SKK_LOS[13] + P[21][1]*SKK_LOS[11] + P[21][2]*SKK_LOS[12] - P[21][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[21][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[21][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[21][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[21][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            Kfusion[22] = SK_LOS[1]*(P[22][0]*SKK_LOS[10] - P[22][3]*SKK_LOS[13] + P[22][1]*SKK_LOS[11] + P[22][2]*SKK_LOS[12] - P[22][9]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][22]*SH_LOS[1]*SH_LOS[7]*SKK_LOS[14] + P[22][6]*SH_LOS[3]*SKK_LOS[0]*SKK_LOS[14] + P[22][4]*SH_LOS[3]*SKK_LOS[2]*SKK_LOS[14] - P[22][5]*SH_LOS[3]*SKK_LOS[1]*SKK_LOS[14]);
            varInnovOptFlow[1] = 1.0f/SK_LOS[1];
            innovOptFlow[1] = losPred[1] - losData[1];
        }

        // Check the innovation for consistency and don't fuse if > 3Sigma
        if ((innovOptFlow[obsIndex]*innovOptFlow[obsIndex]/varInnovOptFlow[obsIndex]) < 9.0f)
        {
            // correct the state vector
            for (uint8_t j = 0; j < n_states; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovOptFlow[obsIndex];
            }
            // normalise the quaternion states
            float quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quatMag > 1e-12f)
            {
                for (uint8_t j= 0; j<=3; j++)
                {
                    float quatMagInv = 1.0f/quatMag;
                    states[j] = states[j] * quatMagInv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i < n_states; i++)
            {
                for (uint8_t j = 0; j <= 6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_LOS[j];
                }
                for (uint8_t j = 7; j <= 8; j++)
                {
                    KH[i][j] = 0.0f;
                }
                KH[i][9] = Kfusion[i] * H_LOS[9];
                for (uint8_t j = 10; j <= 21; j++)
                {
                    KH[i][j] = 0.0f;
                }
                KH[i][22] = Kfusion[i] * H_LOS[22];
            }
            for (uint8_t i = 0; i < n_states; i++)
            {
                for (uint8_t j = 0; j < n_states; j++)
                {
                    KHP[i][j] = 0.0f;
                    for (uint8_t k = 0; k <= 6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    KHP[i][j] = KHP[i][j] + KH[i][9] * P[9][j];
                    KHP[i][j] = KHP[i][j] + KH[i][22] * P[2][j];
                }
            }
        }
        for (uint8_t i = 0; i <  n_states; i++)
        {
            for (uint8_t j = 0; j <  n_states; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }
    obsIndex = obsIndex + 1;
    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::zeroCols(float (&covMat)[n_states][n_states], uint8_t first, uint8_t last)
{
    uint8_t row;
    uint8_t col;
    for (col=first; col<=last; col++)
    {
        for (row=0; row < n_states; row++)
        {
            covMat[row][col] = 0.0;
        }
    }
}

float AttPosEKF::sq(float valIn)
{
    return valIn*valIn;
}

// Store states in a history array along with time stamp
void AttPosEKF::StoreStates(uint64_t timestamp_ms)
{
    for (unsigned i=0; i<n_states; i++)
        storedStates[i][storeIndex] = states[i];
    statetimeStamp[storeIndex] = timestamp_ms;
    storeIndex++;
    if (storeIndex == data_buffer_size)
        storeIndex = 0;
}

void AttPosEKF::ResetStoredStates()
{
    // reset all stored states
    memset(&storedStates[0][0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));

    // reset store index to first
    storeIndex = 0;

    // overwrite all existing states
    for (unsigned i = 0; i < n_states; i++) {
        storedStates[i][storeIndex] = states[i];
    }

    statetimeStamp[storeIndex] = millis();

    // increment to next storage index
    storeIndex++;
}

// Output the state vector stored at the time that best matches that specified by msec
int AttPosEKF::RecallStates(float* statesForFusion, uint64_t msec)
{
    int ret = 0;

    int64_t bestTimeDelta = 200;
    unsigned bestStoreIndex = 0;
    for (unsigned storeIndexLocal = 0; storeIndexLocal < data_buffer_size; storeIndexLocal++)
    {
        // Work around a GCC compiler bug - we know 64bit support on ARM is
        // sketchy in GCC.
        uint64_t timeDelta;

        if (msec > statetimeStamp[storeIndexLocal]) {
            timeDelta = msec - statetimeStamp[storeIndexLocal];
        } else {
            timeDelta = statetimeStamp[storeIndexLocal] - msec;
        }

        if (timeDelta < (uint64_t)bestTimeDelta)
        {
            bestStoreIndex = storeIndexLocal;
            bestTimeDelta = timeDelta;
        }
    }
    if (bestTimeDelta < 200) // only output stored state if < 200 msec retrieval error
    {
        for (unsigned i=0; i < n_states; i++) {
            if (isfinite(storedStates[i][bestStoreIndex])) {
                statesForFusion[i] = storedStates[i][bestStoreIndex];
            } else if (isfinite(states[i])) {
                statesForFusion[i] = states[i];
            } else {
                // There is not much we can do here, except reporting the error we just
                // found.
                ret++;
            }
        }
    }
    else // otherwise output current state
    {
        for (unsigned i = 0; i < n_states; i++) {
            if (isfinite(states[i])) {
                statesForFusion[i] = states[i];
            } else {
                ret++;
            }
        }
    }

    return ret;
}

void AttPosEKF::quat2Tnb(Mat3f &Tnb, const float (&quat)[4])
{
    // Calculate the nav to body cosine matrix
    float q00 = sq(quat[0]);
    float q11 = sq(quat[1]);
    float q22 = sq(quat[2]);
    float q33 = sq(quat[3]);
    float q01 =  quat[0]*quat[1];
    float q02 =  quat[0]*quat[2];
    float q03 =  quat[0]*quat[3];
    float q12 =  quat[1]*quat[2];
    float q13 =  quat[1]*quat[3];
    float q23 =  quat[2]*quat[3];

    Tnb.x.x = q00 + q11 - q22 - q33;
    Tnb.y.y = q00 - q11 + q22 - q33;
    Tnb.z.z = q00 - q11 - q22 + q33;
    Tnb.y.x = 2*(q12 - q03);
    Tnb.z.x = 2*(q13 + q02);
    Tnb.x.y = 2*(q12 + q03);
    Tnb.z.y = 2*(q23 - q01);
    Tnb.x.z = 2*(q13 - q02);
    Tnb.y.z = 2*(q23 + q01);
}

void AttPosEKF::quat2Tbn(Mat3f &Tbn_ret, const float (&quat)[4])
{
    // Calculate the body to nav cosine matrix
    float q00 = sq(quat[0]);
    float q11 = sq(quat[1]);
    float q22 = sq(quat[2]);
    float q33 = sq(quat[3]);
    float q01 =  quat[0]*quat[1];
    float q02 =  quat[0]*quat[2];
    float q03 =  quat[0]*quat[3];
    float q12 =  quat[1]*quat[2];
    float q13 =  quat[1]*quat[3];
    float q23 =  quat[2]*quat[3];

    Tbn_ret.x.x = q00 + q11 - q22 - q33;
    Tbn_ret.y.y = q00 - q11 + q22 - q33;
    Tbn_ret.z.z = q00 - q11 - q22 + q33;
    Tbn_ret.x.y = 2*(q12 - q03);
    Tbn_ret.x.z = 2*(q13 + q02);
    Tbn_ret.y.x = 2*(q12 + q03);
    Tbn_ret.y.z = 2*(q23 - q01);
    Tbn_ret.z.x = 2*(q13 - q02);
    Tbn_ret.z.y = 2*(q23 + q01);
}

void AttPosEKF::eul2quat(float (&quat)[4], const float (&eul)[3])
{
    float u1 = cos(0.5f*eul[0]);
    float u2 = cos(0.5f*eul[1]);
    float u3 = cos(0.5f*eul[2]);
    float u4 = sin(0.5f*eul[0]);
    float u5 = sin(0.5f*eul[1]);
    float u6 = sin(0.5f*eul[2]);
    quat[0] = u1*u2*u3+u4*u5*u6;
    quat[1] = u4*u2*u3-u1*u5*u6;
    quat[2] = u1*u5*u3+u4*u2*u6;
    quat[3] = u1*u2*u6-u4*u5*u3;
}

void AttPosEKF::quat2eul(float (&y)[3], const float (&u)[4])
{
    y[0] = atan2f((2.0f*(u[2]*u[3]+u[0]*u[1])) , (u[0]*u[0]-u[1]*u[1]-u[2]*u[2]+u[3]*u[3]));
    y[1] = -asinf(2.0f*(u[1]*u[3]-u[0]*u[2]));
    y[2] = atan2f((2.0f*(u[1]*u[2]+u[0]*u[3])) , (u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3]));
}

void AttPosEKF::calcvelNED(float (&velNED)[3], float gpsCourse, float gpsGndSpd, float gpsVelD)
{
    velNED[0] = gpsGndSpd*cosf(gpsCourse);
    velNED[1] = gpsGndSpd*sinf(gpsCourse);
    velNED[2] = gpsVelD;
}

void AttPosEKF::calcposNED(float (&posNED)[3], double lat, double lon, float hgt, double latReference, double lonReference, float hgtReference)
{
    posNED[0] = earthRadius * (lat - latReference);
    posNED[1] = earthRadius * cos(latReference) * (lon - lonReference);
    posNED[2] = -(hgt - hgtReference);
}

void AttPosEKF::calcLLH(float posNED[3], double &lat, double &lon, float &hgt, double latRef, double lonRef, float hgtRef)
{
    lat = latRef + (double)posNED[0] * earthRadiusInv;
    lon = lonRef + (double)posNED[1] * earthRadiusInv / cos(latRef);
    hgt = hgtRef - posNED[2];
}

void AttPosEKF::OnGroundCheck()
{
    onGround = (((sq(velNED[0]) + sq(velNED[1]) + sq(velNED[2])) < 4.0f) && (VtasMeas < 8.0f));
    if (staticMode) {
        staticMode = (!refSet || (GPSstatus < GPS_FIX_3D));
    }
    // don't update wind states if there is no airspeed measurement
    if (onGround || !useAirspeed) {
        inhibitWindStates = true;
    } else {
        inhibitWindStates =false;
    }
    // don't update magnetic field states if on ground or not using compass
    if (onGround || !useCompass) {
        inhibitMagStates = true;
    } else {
        inhibitMagStates = false;
    }
    // don't update terrain offset state if on ground
    if (onGround) {
        inhibitGndHgtState = true;
    } else {
        inhibitGndHgtState = false;
    }
}

void AttPosEKF::calcEarthRateNED(Vector3f &omega, float latitude)
{
    //Define Earth rotation vector in the NED navigation frame
    omega.x  = earthRate*cosf(latitude);
    omega.y  = 0.0f;
    omega.z  = -earthRate*sinf(latitude);
}

void AttPosEKF::CovarianceInit()
{
    // Calculate the initial covariance matrix P
    P[0][0]   = 0.25f * sq(1.0f*deg2rad);
    P[1][1]   = 0.25f * sq(1.0f*deg2rad);
    P[2][2]   = 0.25f * sq(1.0f*deg2rad);
    P[3][3]   = 0.25f * sq(10.0f*deg2rad);
    P[4][4]   = sq(0.7f);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(0.7f);
    P[7][7]   = sq(15.0f);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(5.0f);
    P[10][10] = sq(0.1f*deg2rad*dtIMU);
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    P[13][13] = sq(0.2f*dtIMU);
    P[14][14] = sq(0.0f);
    P[15][15]  = P[14][14];
    P[16][16] = sq(0.02f);
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    P[19][19] = sq(0.02f);
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    P[22][22] = sq(0.5f);
}

float AttPosEKF::ConstrainFloat(float val, float min, float max)
{
    float ret;
    if (val > max) {
        ret = max;
        ekf_debug("> max: %8.4f, val: %8.4f", (double)max, (double)val);
    } else if (val < min) {
        ret = min;
        ekf_debug("< min: %8.4f, val: %8.4f", (double)min, (double)val);
    } else {
        ret = val;
    }

    if (!isfinite(val)) {
        //ekf_debug("constrain: non-finite!");
    }

    return ret;
}

void AttPosEKF::ConstrainVariances()
{
    if (!numericalProtection) {
        return;
    }

    // State vector:
    // 0-3: quaternions (q0, q1, q2, q3)
    // 4-6: Velocity - m/sec (North, East, Down)
    // 7-9: Position - m (North, East, Down)
    // 10-12: Delta Angle bias - rad (X,Y,Z)
    // 13: Delta Velocity bias - m/s (Z)
    // 14-15: Wind Vector  - m/sec (North,East)
    // 16-18: Earth Magnetic Field Vector - gauss (North, East, Down)
    // 19-21: Body Magnetic Field Vector - gauss (X,Y,Z)
    // 22: Terrain  offset - m

    // Constrain quaternion variances
    for (unsigned i = 0; i <= 3; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Constrain velocity variances
    for (unsigned i = 4; i <= 6; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e3f);
    }

    // Constrain position variances
    for (unsigned i = 7; i <= 9; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e6f);
    }

    // Constrain delta angle bias variances
    for (unsigned i = 10; i <= 12; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, sq(0.12f * dtIMU));
    }

    // Constrain delta velocity bias variance
    P[13][13] = ConstrainFloat(P[13][13], 0.0f, sq(1.0f * dtIMU));

    // Wind velocity variances
    for (unsigned i = 14; i <= 15; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e3f);
    }

    // Earth magnetic field variances
    for (unsigned i = 16; i <= 18; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Body magnetic field variances
    for (unsigned i = 19; i <= 21; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Constrain terrain offset variance
    P[22][22] = ConstrainFloat(P[22][22], 0.0f, 10000.0f);
}

void AttPosEKF::ConstrainStates()
{
    if (!numericalProtection) {
        return;
    }

    // State vector:
    // 0-3: quaternions (q0, q1, q2, q3)
    // 4-6: Velocity - m/sec (North, East, Down)
    // 7-9: Position - m (North, East, Down)
    // 10-12: Delta Angle bias - rad (X,Y,Z)
    // 13: Delta Velocity bias - m/s (Z)
    // 14-15: Wind Vector  - m/sec (North,East)
    // 16-18: Earth Magnetic Field Vector - gauss (North, East, Down)
    // 19-21: Body Magnetic Field Vector - gauss (X,Y,Z)
    // 22: Terrain  offset - m

    // Constrain quaternion
    for (unsigned i = 0; i <= 3; i++) {
        states[i] = ConstrainFloat(states[i], -1.0f, 1.0f);
    }

    // Constrain velocities to what GPS can do for us
    for (unsigned i = 4; i <= 6; i++) {
        states[i] = ConstrainFloat(states[i], -5.0e2f, 5.0e2f);
    }

    // Constrain position to a reasonable vehicle range (in meters)
    for (unsigned i = 7; i <= 8; i++) {
        states[i] = ConstrainFloat(states[i], -1.0e6f, 1.0e6f);
    }

    // Constrain altitude
    states[9] = ConstrainFloat(states[9], -4.0e4f, 1.0e4f);

    // Angle bias limit - set to 8 degrees / sec
    for (unsigned i = 10; i <= 12; i++) {
        states[i] = ConstrainFloat(states[i], -0.12f * dtIMU, 0.12f * dtIMU);
    }

    // Constrain delta velocity bias
    states[13] = ConstrainFloat(states[13], -1.0f * dtIMU, 1.0f * dtIMU);

    // Wind velocity limits - assume 120 m/s max velocity
    for (unsigned i = 14; i <= 15; i++) {
        states[i] = ConstrainFloat(states[i], -120.0f, 120.0f);
    }

    // Earth magnetic field limits (in Gauss)
    for (unsigned i = 16; i <= 18; i++) {
        states[i] = ConstrainFloat(states[i], -1.0f, 1.0f);
    }

    // Body magnetic field variances (in Gauss).
    // the max offset should be in this range.
    for (unsigned i = 19; i <= 21; i++) {
        states[i] = ConstrainFloat(states[i], -0.5f, 0.5f);
    }

    // Constrain terrain offset
    states[22] = ConstrainFloat(states[22], -1000.0f, 1000.0f);

}

void AttPosEKF::ForceSymmetry()
{
    if (!numericalProtection) {
        return;
    }

    // Force symmetry on the covariance matrix to prevent ill-conditioning
    // of the matrix which would cause the filter to blow-up
    for (unsigned i = 1; i < n_states; i++)
    {
        for (uint8_t j = 0; j < i; j++)
        {
            P[i][j] = 0.5f * (P[i][j] + P[j][i]);
            P[j][i] = P[i][j];

            if ((fabsf(P[i][j]) > EKF_COVARIANCE_DIVERGED) ||
                (fabsf(P[j][i]) > EKF_COVARIANCE_DIVERGED)) {
                current_ekf_state.covariancesExcessive = true;
                current_ekf_state.error |= true;
                InitializeDynamic(velNED, magDeclination);
                return;
            }

            float symmetric = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = symmetric;
            P[j][i] = symmetric;
        }
    }
}

bool AttPosEKF::GyroOffsetsDiverged()
{
    // Detect divergence by looking for rapid changes of the gyro offset
    Vector3f current_bias;
    current_bias.x = states[10];
    current_bias.y = states[11];
    current_bias.z = states[12];

    Vector3f delta = current_bias - lastGyroOffset;
    float delta_len = delta.length();
    float delta_len_scaled = 0.0f;

    // Protect against division by zero
    if (delta_len > 0.0f) {
        float cov_mag = ConstrainFloat((P[10][10] + P[11][11] + P[12][12]), 1e-12f, 1e-8f);
        delta_len_scaled = (5e-7 / (double)cov_mag) * (double)delta_len / (double)dtIMU;
    }

    bool diverged = (delta_len_scaled > 1.0f);
    lastGyroOffset = current_bias;
    current_ekf_state.error |= diverged;
    current_ekf_state.gyroOffsetsExcessive = diverged;

    return diverged;
}

bool AttPosEKF::VelNEDDiverged()
{
    Vector3f current_vel;
    current_vel.x = states[4];
    current_vel.y = states[5];
    current_vel.z = states[6];

    Vector3f gps_vel;
    gps_vel.x = velNED[0];
    gps_vel.y = velNED[1];
    gps_vel.z = velNED[2];

    Vector3f delta = current_vel - gps_vel;
    float delta_len = delta.length();

    bool excessive = (delta_len > 20.0f);

    current_ekf_state.error |= excessive;
    current_ekf_state.velOffsetExcessive = excessive;

    return excessive;
}

bool AttPosEKF::FilterHealthy()
{
    if (!statesInitialised) {
        return false;
    }

    // XXX Check state vector for NaNs and ill-conditioning

    // Check if any of the major inputs timed out
    if (current_ekf_state.posTimeout || current_ekf_state.velTimeout || current_ekf_state.hgtTimeout) {
        return false;
    }

    // Nothing fired, return ok.
    return true;
}

/**
 * Reset the filter position states
 *
 * This resets the position to the last GPS measurement
 * or to zero in case of static position.
 */
void AttPosEKF::ResetPosition(void)
{
    if (staticMode) {
        states[7] = 0;
        states[8] = 0;
    } else if (GPSstatus >= GPS_FIX_3D) {

        // reset the states from the GPS measurements
        states[7] = posNE[0];
        states[8] = posNE[1];
    }
}

/**
 * Reset the height state.
 *
 * This resets the height state with the last altitude measurements
 */
void AttPosEKF::ResetHeight(void)
{
    // write to the state vector
    states[9]   = -hgtMea;
}

/**
 * Reset the velocity state.
 */
void AttPosEKF::ResetVelocity(void)
{
    if (staticMode) {
        states[4] = 0.0f;
        states[5] = 0.0f;
        states[6] = 0.0f;
    } else if (GPSstatus >= GPS_FIX_3D) {

        states[4]  = velNED[0]; // north velocity from last reading
        states[5]  = velNED[1]; // east velocity from last reading
        states[6]  = velNED[2]; // down velocity from last reading
    }
}

bool AttPosEKF::StatesNaN() {
    bool err = false;

    // check all integrators
    if (!isfinite(summedDelAng.x) || !isfinite(summedDelAng.y) || !isfinite(summedDelAng.z)) {
        current_ekf_state.angNaN = true;
        ekf_debug("summedDelAng NaN: x: %f y: %f z: %f", (double)summedDelAng.x, (double)summedDelAng.y, (double)summedDelAng.z);
        err = true;
        goto out;
    } // delta angles

    if (!isfinite(correctedDelAng.x) || !isfinite(correctedDelAng.y) || !isfinite(correctedDelAng.z)) {
        current_ekf_state.angNaN = true;
        ekf_debug("correctedDelAng NaN: x: %f y: %f z: %f", (double)correctedDelAng.x, (double)correctedDelAng.y, (double)correctedDelAng.z);
        err = true;
        goto out;
    } // delta angles

    if (!isfinite(summedDelVel.x) || !isfinite(summedDelVel.y) || !isfinite(summedDelVel.z)) {
        current_ekf_state.summedDelVelNaN = true;
        ekf_debug("summedDelVel NaN: x: %f y: %f z: %f", (double)summedDelVel.x, (double)summedDelVel.y, (double)summedDelVel.z);
        err = true;
        goto out;
    } // delta velocities

    // check all states and covariance matrices
    for (unsigned i = 0; i < n_states; i++) {
        for (unsigned j = 0; j < n_states; j++) {
            if (!isfinite(KH[i][j])) {

                current_ekf_state.KHNaN = true;
                err = true;
                ekf_debug("KH NaN");
                goto out;
            } //  intermediate result used for covariance updates

            if (!isfinite(KHP[i][j])) {

                current_ekf_state.KHPNaN = true;
                err = true;
                ekf_debug("KHP NaN");
                goto out;
            } // intermediate result used for covariance updates

            if (!isfinite(P[i][j])) {

                current_ekf_state.covarianceNaN = true;
                err = true;
                ekf_debug("P NaN");
            } // covariance matrix
        }

        if (!isfinite(Kfusion[i])) {

            current_ekf_state.kalmanGainsNaN = true;
            ekf_debug("Kfusion NaN");
            err = true;
            goto out;
        } // Kalman gains

        if (!isfinite(states[i])) {

            current_ekf_state.statesNaN = true;
            ekf_debug("states NaN: i: %u val: %f", i, (double)states[i]);
            err = true;
            goto out;
        } // state matrix
    }

out:
    if (err) {
        current_ekf_state.error |= true;
    }

    return err;

}

/**
 * Check the filter inputs and bound its operational state
 *
 * This check will reset the filter states if required
 * due to a failure of consistency or timeout checks.
 * it should be run after the measurement data has been
 * updated, but before any of the fusion steps are
 * executed.
 */
int AttPosEKF::CheckAndBound(struct ekf_status_report *last_error)
{

    // Store the old filter state
    bool currStaticMode = staticMode;

    // Limit reset rate to 5 Hz to allow the filter
    // to settle
    if (millis() - lastReset < 200) {
        return 0;
    }

    if (ekfDiverged) {
        ekfDiverged = false;
    }

    int ret = 0;

    // Check if we're on ground - this also sets static mode.
    OnGroundCheck();

    // Reset the filter if the states went NaN
    if (StatesNaN()) {
        ekf_debug("re-initializing dynamic");

        // Reset and fill error report
	    InitializeDynamic(velNED, magDeclination);

        ret = 1;
    }

    // Reset the filter if the IMU data is too old
    if (dtIMU > 0.3f) {

        current_ekf_state.imuTimeout = true;

        // Fill error report
        GetFilterState(&last_ekf_error);

        ResetVelocity();
        ResetPosition();
        ResetHeight();
        ResetStoredStates();

        // Timeout cleared with this reset
        current_ekf_state.imuTimeout = false;

        // that's all we can do here, return
        ret = 2;
    }

    // Check if we switched between states
    if (currStaticMode != staticMode) {
        // Fill error report, but not setting error flag
        GetFilterState(&last_ekf_error);

        ResetVelocity();
        ResetPosition();
        ResetHeight();
        ResetStoredStates();

        ret = 0;
    }

    // Reset the filter if gyro offsets are excessive
    if (GyroOffsetsDiverged()) {

        // Reset and fill error report
        InitializeDynamic(velNED, magDeclination);

        // that's all we can do here, return
        ret = 4;
    }

    // Reset the filter if it diverges too far from GPS
    if (VelNEDDiverged()) {

        // Reset and fill error report
        InitializeDynamic(velNED, magDeclination);

        // that's all we can do here, return
        ret = 5;
    }

    // The excessive covariance detection already
    // reset the filter. Just need to report here.
    if (last_ekf_error.covariancesExcessive) {
        ret = 6;
    }

    if (ret) {
        ekfDiverged = true;
        lastReset = millis();

        // This reads the last error and clears it
        GetLastErrorState(last_error);
    }

    return ret;
}

void AttPosEKF::AttitudeInit(float ax, float ay, float az, float mx, float my, float mz, float declination, float *initQuat)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2f(-ay, -az);
    initialPitch = atan2f(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);
    /* true heading is the mag heading minus declination */
    initialHdg += declination;

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    initQuat[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    initQuat[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    initQuat[2] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    initQuat[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    /* normalize */
    float norm = sqrtf(initQuat[0]*initQuat[0] + initQuat[1]*initQuat[1] + initQuat[2]*initQuat[2] + initQuat[3]*initQuat[3]);

    initQuat[0] /= norm;
    initQuat[1] /= norm;
    initQuat[2] /= norm;
    initQuat[3] /= norm;
}

void AttPosEKF::InitializeDynamic(float (&initvelNED)[3], float declination)
{
    if (current_ekf_state.error) {
        GetFilterState(&last_ekf_error);
    }

    ZeroVariables();

    // Reset error states
    current_ekf_state.error = false;
    current_ekf_state.angNaN = false;
    current_ekf_state.summedDelVelNaN = false;
    current_ekf_state.KHNaN = false;
    current_ekf_state.KHPNaN = false;
    current_ekf_state.PNaN = false;
    current_ekf_state.covarianceNaN = false;
    current_ekf_state.kalmanGainsNaN = false;
    current_ekf_state.statesNaN = false;

    current_ekf_state.velHealth = true;
    current_ekf_state.posHealth = true;
    current_ekf_state.hgtHealth = true;
    
    current_ekf_state.velTimeout = false;
    current_ekf_state.posTimeout = false;
    current_ekf_state.hgtTimeout = false;

    fuseVelData = false;
    fusePosData = false;
    fuseHgtData = false;
    fuseMagData = false;
    fuseVtasData = false;

    // Fill variables with valid data
    velNED[0] = initvelNED[0];
    velNED[1] = initvelNED[1];
    velNED[2] = initvelNED[2];
    magDeclination = declination;

    // Calculate initial filter quaternion states from raw measurements
    float initQuat[4];
    Vector3f initMagXYZ;
    initMagXYZ   = magData - magBias;
    AttitudeInit(accel.x, accel.y, accel.z, initMagXYZ.x, initMagXYZ.y, initMagXYZ.z, declination, initQuat);

    // Calculate initial Tbn matrix and rotate Mag measurements into NED
    // to set initial NED magnetic field states
    quat2Tbn(Tbn, initQuat);
    Tnb = Tbn.transpose();
    Vector3f initMagNED;
    initMagNED.x = Tbn.x.x*initMagXYZ.x + Tbn.x.y*initMagXYZ.y + Tbn.x.z*initMagXYZ.z;
    initMagNED.y = Tbn.y.x*initMagXYZ.x + Tbn.y.y*initMagXYZ.y + Tbn.y.z*initMagXYZ.z;
    initMagNED.z = Tbn.z.x*initMagXYZ.x + Tbn.z.y*initMagXYZ.y + Tbn.z.z*initMagXYZ.z;

    magstate.q0 = initQuat[0];
    magstate.q1 = initQuat[1];
    magstate.q2 = initQuat[2];
    magstate.q3 = initQuat[3];
    magstate.magN = initMagNED.x;
    magstate.magE = initMagNED.y;
    magstate.magD = initMagNED.z;
    magstate.magXbias = magBias.x;
    magstate.magYbias = magBias.y;
    magstate.magZbias = magBias.z;
    magstate.R_MAG = sq(magMeasurementSigma);
    magstate.DCM = Tbn;

    // write to state vector
    for (uint8_t j=0; j<=3; j++) states[j] = initQuat[j]; // quaternions
    for (uint8_t j=4; j<=6; j++) states[j] = initvelNED[j-4]; // velocities
    // positions:
    states[7] = posNE[0];
    states[8] = posNE[1];
    states[9] = -hgtMea;
    for (uint8_t j=10; j<=15; j++) states[j] = 0.0f; // dAngBias, dVelBias, windVel
    states[16] = initMagNED.x; // Magnetic Field North
    states[17] = initMagNED.y; // Magnetic Field East
    states[18] = initMagNED.z; // Magnetic Field Down
    states[19] = magBias.x; // Magnetic Field Bias X
    states[20] = magBias.y; // Magnetic Field Bias Y
    states[21] = magBias.z; // Magnetic Field Bias Z
    states[22] = 0.0f; // terrain height

    ResetVelocity();
    ResetPosition();
    ResetHeight();

    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit();

    //Define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, latRef);

}

void AttPosEKF::InitialiseFilter(float (&initvelNED)[3], double referenceLat, double referenceLon, float referenceHgt, float declination)
{
    // store initial lat,long and height
    latRef = referenceLat;
    lonRef = referenceLon;
    hgtRef = referenceHgt;
    refSet = true;

    // we are at reference position, so measurement must be zero
    posNE[0] = 0.0f;
    posNE[1] = 0.0f;

    // we are at an unknown, possibly non-zero altitude - so altitude
    // is not reset (hgtMea)

    // the baro offset must be this difference now
    baroHgtOffset = baroHgt - referenceHgt;

    InitializeDynamic(initvelNED, declination);
}

void AttPosEKF::ZeroVariables()
{

    // Initialize on-init initialized variables

    storeIndex = 0;

    // Do the data structure init
    for (unsigned i = 0; i < n_states; i++) {
        for (unsigned j = 0; j < n_states; j++) {
            KH[i][j] = 0.0f; //  intermediate result used for covariance updates
            KHP[i][j] = 0.0f; // intermediate result used for covariance updates
            P[i][j] = 0.0f; // covariance matrix
        }

        Kfusion[i] = 0.0f; // Kalman gains
        states[i] = 0.0f; // state matrix
    }

    correctedDelAng.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    dAngIMU.zero();
    dVelIMU.zero();
    lastGyroOffset.zero();

    for (unsigned i = 0; i < data_buffer_size; i++) {

        for (unsigned j = 0; j < n_states; j++) {
            storedStates[j][i] = 0.0f;
        }

        statetimeStamp[i] = 0;
    }

    memset(&magstate, 0, sizeof(magstate));
    magstate.q0 = 1.0f;
    magstate.DCM.identity();

    memset(&current_ekf_state, 0, sizeof(current_ekf_state));

}

void AttPosEKF::GetFilterState(struct ekf_status_report *err)
{

    // Copy states
    for (unsigned i = 0; i < n_states; i++) {
        current_ekf_state.states[i] = states[i];
    }
    current_ekf_state.n_states = n_states;
    current_ekf_state.onGround = onGround;
    current_ekf_state.staticMode = staticMode;
    current_ekf_state.useCompass = useCompass;
    current_ekf_state.useAirspeed = useAirspeed;

    memcpy(err, &current_ekf_state, sizeof(*err));

    // err->velHealth = current_ekf_state.velHealth;
    // err->posHealth = current_ekf_state.posHealth;
    // err->hgtHealth = current_ekf_state.hgtHealth;
    // err->velTimeout = current_ekf_state.velTimeout;
    // err->posTimeout = current_ekf_state.posTimeout;
    // err->hgtTimeout = current_ekf_state.hgtTimeout;
}

void AttPosEKF::GetLastErrorState(struct ekf_status_report *last_error)
{
    memcpy(last_error, &last_ekf_error, sizeof(*last_error));
    memset(&last_ekf_error, 0, sizeof(last_ekf_error));
}
