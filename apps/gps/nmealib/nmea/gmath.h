/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: gmath.h 17 2008-03-11 11:56:11Z xtimor $
 *
 */

#ifndef __NMEA_GMATH_H__
#define __NMEA_GMATH_H__

#include "info.h"

#define NMEA_PI                     (3.141592653589793)             /**< PI value */
#define NMEA_PI180                  (NMEA_PI / 180)                 /**< PI division by 180 */
#define NMEA_EARTHRADIUS_KM         (6378)                          /**< Earth's mean radius in km */
#define NMEA_EARTHRADIUS_M          (NMEA_EARTHRADIUS_KM * 1000)    /**< Earth's mean radius in m */
#define NMEA_EARTH_SEMIMAJORAXIS_M  (6378137.0)                     /**< Earth's semi-major axis in m according WGS84 */
#define NMEA_EARTH_SEMIMAJORAXIS_KM (NMEA_EARTHMAJORAXIS_KM / 1000) /**< Earth's semi-major axis in km according WGS 84 */
#define NMEA_EARTH_FLATTENING       (1 / 298.257223563)             /**< Earth's flattening according WGS 84 */
#define NMEA_DOP_FACTOR             (5)                             /**< Factor for translating DOP to meters */

#ifdef  __cplusplus
extern "C" {
#endif

/*
 * degree VS radian
 */

float nmea_degree2radian(float val);
float nmea_radian2degree(float val);

/*
 * NDEG (NMEA degree)
 */

float nmea_ndeg2degree(float val);
float nmea_degree2ndeg(float val);

float nmea_ndeg2radian(float val);
float nmea_radian2ndeg(float val);

/*
 * DOP
 */

float nmea_calc_pdop(float hdop, float vdop);
float nmea_dop2meters(float dop);
float nmea_meters2dop(float meters);

/*
 * positions work
 */

void nmea_info2pos(const nmeaINFO *info, nmeaPOS *pos);
void nmea_pos2info(const nmeaPOS *pos, nmeaINFO *info);

float  nmea_distance(
        const nmeaPOS *from_pos,
        const nmeaPOS *to_pos
        );

float  nmea_distance_ellipsoid(
        const nmeaPOS *from_pos,
        const nmeaPOS *to_pos,
        float *from_azimuth,
        float *to_azimuth
        );

int     nmea_move_horz(
        const nmeaPOS *start_pos,
        nmeaPOS *end_pos,
        float azimuth,
        float distance
        );

int     nmea_move_horz_ellipsoid(
        const nmeaPOS *start_pos,
        nmeaPOS *end_pos,
        float azimuth,
        float distance,
        float *end_azimuth
        );

#ifdef  __cplusplus
}
#endif

#endif /* __NMEA_GMATH_H__ */
