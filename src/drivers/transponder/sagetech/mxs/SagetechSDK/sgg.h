/**
 * @copyright (c) 2020 Sagetech, Inc. All rights reserved.
 *
 * @file agg.h
 *
 * @date Mar 25, 2022
 *
 * GPS parsed sentence data structures
 */

#ifndef SGG_H_
#define SGG_H_

#include <stdbool.h>
#include <nmea.h>


/// The NMEA GPS RMC sentence data.
typedef struct
{
   char   time[8];   /// UTC time.
   bool   valid;     /// True if the RMC data is valid, false if invalid results.
   float  lat;       /// The position latitude, negative for west.
   float  lon;       /// The position longitude, negative for south.
   float  speed;     /// The speed in knots per hour.
   float  heading;   /// The heading in degrees.
   char   date[8];   /// The date ddmmyy format.
   float  magVar;    /// Magnetic variation.
} sgg_rmc_t;

typedef enum
{
   ggaFixQualNone,
   ggaFixQualGPS,
   ggaFixQualDGPS,
   ggaFixQualPPS,
   ggaFixQualRTK,
   ggaFixQualRTKF,
   ggaFixQualEst,
   ggaFixQualMan,
   ggaFixQualSim,
   ggaFixQualWAAS
} sgg_fixqual_t;

typedef struct
{
   char          time[8];   /// UTC time.
   float         lat;       /// The position latitude, negative for west.
   float         lon;       /// The position longitude, negative for south.
   sgg_fixqual_t fixQual;   /// The GGA fix quality indicator (none, GPS or DGPS).
   uint8_t       sats;      /// Satellites used for the fix.
   float         hdop;      /// Horizontal dilution of precision.
   float         alt;       /// Altitude above mean sea level.
   char          altUnits;  /// Altitude units (M for meters).
   float         gsep;      /// geoidal separation, the difference between WGS-84 earth ellipsoid and mean sea level.
   char          gsepUnits; /// Units of the above geoid separation (M for meters).
   uint16_t      diffAge;   /// Age of differential correction (ignored if inactive).
   char          diffId[6]; /// Differential station ID (ignored if inactive).
} sgg_gga_t;

#endif /* SGG_H_ */
