#include <systemlib/param/param.h>

// 16 is max name length

/*================================================================*/
/* Gyroscope*/
/*================================================================*/

/**
 * Gyro noise density
 *
 * @group IEKF
 * @unit (rad / s) / sqrt(Hz)
 * @min 0
 * @max 1e-2
 * @decimal 7
 */
PARAM_DEFINE_FLOAT(IEKF_GYRO_ND, 9.8e-5f);

/**
 * Gyro random walk noise density
 *
 * @group IEKF
 * @unit (rad / s^2) / sqrt(Hz)
 * @min 0
 * @max 1e-2
 * @decimal 7
 */
PARAM_DEFINE_FLOAT(IEKF_GYRO_RW_ND, 1.2e-5f);

/**
 * Gyro random walk correlation time
 *
 * @group IEKF
 * @unit s
 * @min 0
 * @max 1e6
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(IEKF_GYRO_RW_CT, 1e3);

/*================================================================*/
/* Accelerometer*/
/*================================================================*/

/**
 * Accel noise density
 *
 * @group IEKF
 * @unit (m / s^2) / sqrt(Hz)
 * @min 0
 * @max 1e-2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_ACCEL_ND, 2.4e-3f);

/**
 * Accel random walk noise density
 *
 * @group IEKF
 * @unit (m / s^3) / sqrt(Hz)
 * @min 0
 * @max 1e-2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_ACCEL_RW_ND, 2e-3f);

/**
 * Accel random walk correlation time
 *
 * @group IEKF
 * @unit s
 * @min 0
 * @max 1e6
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(IEKF_ACCEL_RW_CT, 1e3);

/*================================================================*/
/* Barometric Altimeter*/
/*================================================================*/

/**
 * Barometric altitude noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_BARO_ND, 5.68e-2f);

/**
 * Barometric altitiude random walk noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_BARO_RW_ND, 3.8e-2f);

/**
 * Baro random walk correlation time
 *
 * @group IEKF
 * @unit s
 * @min 0
 * @max 1e6
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(IEKF_BARO_RW_CT, 1e3);

/**
 * Position estimate low pass filter cut freq
 *
 * @group IEKF
 * @unit Hz
 * @min 1
 * @max 250
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(IEKF_POS_LP, 10);

/*================================================================*/
/* Magnetometer */
/*================================================================*/

/**
 * Mag heading noise density
 *
 * @group IEKF
 * @unit (rad) / sqrt(Hz)
 * @min 0
 * @max 1
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_MAG_ND, 5.43e-2f);

/**
 * Mag heading random walk noise density
 *
 * @group IEKF
 * @unit (rad/s) / sqrt(Hz)

 * @max 1
 * @decimal 7
 */
PARAM_DEFINE_FLOAT(IEKF_MAG_RW_ND, 3.34e-5f);

/**
 * Mag heading random walk correlation time
 *
 * @group IEKF
 * @unit s
 * @min 0
 * @max 1e6
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(IEKF_MAG_RW_CT, 1e3);

/**
 * Magnetic declination
 *
 * @group IEKF
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IEKF_MAG_DECL, -4.7);

/*================================================================*/
/* GPS */
/*================================================================*/

/**
 * GPS xy position noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_GPS_XY_ND, 2e0f);

/**
 * GPS z position noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_GPS_Z_ND, 10.0f);

/**
 * GPS xy velocity noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_GPS_VXY_ND, 2e-1f);

/**
 * GPS z velocity noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_GPS_VZ_ND, 4e-1f);

/*================================================================*/
/* VISION */
/*================================================================*/

/**
 * Vision xy position noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_VIS_XY_ND, 2e0f);

/**
 * Vision z position noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_VIS_Z_ND, 10.0f);

/**
 * Vision xy velocity noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_VIS_VXY_ND, 2e-1f);

/**
 * Vision z velocity noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e2
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_VIS_VZ_ND, 4e-1f);



/*================================================================*/
/* Optical Flow */
/*================================================================*/

/**
 * Optical flow noise density
 *
 * @group IEKF
 * @unit (rad/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_FLOW_ND, 1e-2f);

/*================================================================*/
/* Lidar */
/*================================================================*/

/**
 * Lidar noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_LIDAR_ND, 1e-3f);


/*================================================================*/
/* Sonar */
/*================================================================*/

/**
 * Sonar noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_SONAR_ND, 1e-2f);

/*================================================================*/
/* Land */
/*================================================================*/

/**
 * Land velocity xy noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_LAND_VXY_ND, 1e-2f);

/**
 * Land velocity z noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_LAND_VZ_ND, 1e-2f);

/**
 * Land agl noise density
 *
 * @group IEKF
 * @unit (m) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_LAND_AGL_ND, 1e-2f);

/*================================================================*/
/* Process Noise */
/*================================================================*/

/**
 * Process noise xy position noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_XY_ND, 0);

/**
 * Process noise xy velocity noise density
 *
 * @group IEKF
 * @unit (m/s^2) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_VXY_ND, 0);

/**
 * Process noise z position noise density
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_Z_ND, 0);

/**
 * Process noise z velocity noise density
 *
 * @group IEKF
 * @unit (m/s^2) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_VZ_ND, 0);

/**
 * Process noise rotation noise density
 *
 * @group IEKF
 * @unit (rad/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_ROT_ND, 1e-2f);

/**
 * Process noise terrain asl, const term
 *
 * @group IEKF
 * @unit (m/s) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_T_ND, 1e-1f);

/**
 * Process noise terrain asl, speed term
 *
 * @group IEKF
 * @unit (m/s^2) / sqrt(Hz)
 * @min 0
 * @max 1e0
 * @decimal 5
 */
PARAM_DEFINE_FLOAT(IEKF_PN_TS_ND, 1e-1f);



/**
 * Accel max rate
 *
 * (for attitude correction/ integration
 *  uses full rate)
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_ACCEL, 250);

/**
 * Mag max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_MAG, 250);

/**
 * Baro max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_BARO, 250);

/**
 * GPS max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_GPS, 20);

/**
 * Airpseed max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_AIRSPD, 100);

/**
 * Flow max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_FLOW, 100);

/**
 * Sonar max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 250
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_SONAR, 100);

/**
 * Lidar max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_LIDAR, 20);

/**
 * Vision max rate
 *
 * @group IEKF
 * @unit Hz
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(IEKF_RATE_VISION, 20);

**
*Mocap max rate
*
*@group IEKF
*@unit Hz
*@min 0
*@max 50
* /
PARAM_DEFINE_FLOAT(IEKF_RATE_MOCAP, 20);

**
*Land max rate
*
*@group IEKF
*@unit Hz
*@min 0
*@max 50
* /
PARAM_DEFINE_FLOAT(IEKF_RATE_LAND, 20);
