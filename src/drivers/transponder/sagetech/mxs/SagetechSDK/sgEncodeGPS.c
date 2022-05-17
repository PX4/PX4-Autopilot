/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file sgEncodeGPS.c
 * @author Jacob.Garrison
 *
 * @date Mar 1, 2021
 *
 * This file receives a populated GPS message struct and
 * converts it into a GPS message buffer.
 */


#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sg.h"
#include "sgUtil.h"

#define SG_PAYLOAD_LEN_GPS  SG_MSG_LEN_GPS - 5  /// the payload length.
#define _UNUSED(x) ((void)(x))

#define PBASE                    4   /// the payload offset.
#define OFFSET_LONGITUDE         0   /// the longitude offset in the payload.
#define OFFSET_LATITUDE         11   /// the latitude offset in the payload.
#define OFFSET_SPEED            21   /// the ground speed offset in the payload.
#define OFFSET_TRACK            27   /// the ground track offset in the payload.
#define OFFSET_STATUS           35   /// the hemisphere/data status offset in the payload.
#define OFFSET_TIME             36   /// the time of fix offset in the payload.
#define OFFSET_HEIGHT           46   /// the GNSS height offset in the payload.
#define OFFSET_HPL              50   /// the horizontal protection limit offset in the payload.
#define OFFSET_HFOM             54   /// the horizontal figure of merit offset in the payload.
#define OFFSET_VFOM             58   /// the vertical figure of merit offset in the payload.
#define OFFSET_NACV             62   /// the navigation accuracy for velocity offset in the payload.

#define LEN_LNG                 11   /// bytes in the longitude field
#define LEN_LAT                 10   /// bytes in the latitude field
#define LEN_SPD                  6   /// bytes in the speed over ground field
#define LEN_TRK                  8   /// bytes in the ground track field
#define LEN_TIME                10   /// bytes in the time of fix field

/**
 * Validate all input parameters prior to bufferizing data. Function is
 * used during development in debug mode, only.
 *
 * @param gps The populated gps message struct.
 *
 * no return value, an error code is thrown if any data is invalid
 */
static void checkGPSInputs(sg_gps_t *gps)
{
   // Validate longitude
   for (int i = 0; i < LEN_LNG; ++i)
   {
      if (i == 5)
      {
         assert(gps->longitude[i] == 0x2E &&
                "A period is expected to separate minutes from fractions of minutes.");
      }
      else
      {
         assert((0x30 <= gps->longitude[i] && gps->longitude[i] <= 0x39) &&
                "Longitude contains an invalid character");
      }
   }

   // Validate latitude
   for (int i = 0; i < LEN_LAT; ++i)
   {
      if (i == 4)
      {
         assert(gps->latitude[i] == 0x2E &&
                "A period is expected to separate minutes from fractions of minutes.");
      }
      else
      {
         assert((0x30 <= gps->latitude[i] && gps->latitude[i] <= 0x39) &&
                "Latitude contains an invalid character");
      }
   }

   // Validate speed over ground
   bool spdDecimal = false;
   for (int i = 0; i < LEN_SPD; ++i)
   {
      if (gps->grdSpeed[i] == 0x2E)
      {
         assert(spdDecimal == false &&
                "Only one period should be used in speed over ground.");

         spdDecimal = true;
      }
      else
      {
         assert((0x30 <= gps->grdSpeed[i] && gps->grdSpeed[i] <= 0x39) &&
                "Ground speed contains an invalid character");
      }
   }
   assert(spdDecimal == true &&
          "Use a period in ground speed to signify the start of fractional knots.");
   _UNUSED(spdDecimal);

   // Validate ground track
   for (int i = 0; i < LEN_TRK; ++i)
   {
      if (i == 3)
      {
         assert(gps->grdTrack[i] == 0x2E &&
                "A period is expected to signify the start of fractional degrees.");
      }
      else
      {
         assert((0x30 <= gps->grdTrack[i] && gps->grdTrack[i] <= 0x39) &&
                "Ground track contains an invalid character");
      }
   }

   // Validate time of fix
   bool tofSpaces = false;
   for (int i = 0; i < LEN_TIME; ++i)
   {
      if (i == 6)
      {
         assert(gps->timeOfFix[i] == 0x2E &&
                "A period is expected to signify the start of fractional seconds.");
      }
      else if (i == 0 && gps->timeOfFix[i] == 0x20)
      {
         tofSpaces = true;
      }
      else
      {
         if (tofSpaces)
         {
            assert((gps->timeOfFix[i] == 0x20) &&
                   "All characters must be filled with spaces.");
         }
         else
         {
            assert((0x30 <= gps->timeOfFix[i] && gps->timeOfFix[i] <= 0x39) &&
                   "Time of Fix contains an invalid character");
         }
      }
   }

   // Validate height
   assert((-1200.0 <= gps->height && gps->height <= 160000.0) &&
          "GPS height is not within the troposphere");

   // Validate hpl
   assert(0 <= gps->hpl &&
          "HPL cannot be negative");

   // Validate hfom
      assert(0 <= gps->hfom &&
             "HFOM cannot be negative");

   // Validate vfom
      assert(0 <= gps->vfom &&
             "VFOM cannot be negative");

   // Validate status
   assert(nacvUnknown <= gps->nacv && gps->nacv <= nacv0dot3 &&
          "NACv is not an enumerated value");
}

/*
 * Documented in the header file.
 */
bool sgEncodeGPS(uint8_t *buffer, sg_gps_t *gps, uint8_t msgId)
{
   // Validate all data inputs (debug mode, only)
   checkGPSInputs(gps);

   // populate header
   buffer[0]       = SG_MSG_START_BYTE;
   buffer[1]       = SG_MSG_TYPE_HOST_GPS;
   buffer[2]       = msgId;
   buffer[3]       = SG_PAYLOAD_LEN_GPS;

   // populate longitude
   charArray2Buf(&buffer[PBASE + OFFSET_LONGITUDE], gps->longitude, LEN_LNG);

   // populate latitude
   charArray2Buf(&buffer[PBASE + OFFSET_LATITUDE], gps->latitude, LEN_LAT);

   // populate ground speed
   charArray2Buf(&buffer[PBASE + OFFSET_SPEED], gps->grdSpeed, LEN_SPD);

   // populate ground track
   charArray2Buf(&buffer[PBASE + OFFSET_TRACK], gps->grdTrack, LEN_TRK);

   // populate hemisphere/data status
   buffer[PBASE + OFFSET_STATUS] = !gps->gpsValid  << 7 |
                                   gps->fdeFail    << 6 |
                                   gps->lngEast    << 1 |
                                   gps->latNorth;

   // populate time of fix
   charArray2Buf(&buffer[PBASE + OFFSET_TIME], gps->timeOfFix, LEN_TIME);

   // populate gnss height
   float2Buf(&buffer[PBASE + OFFSET_HEIGHT], gps->height);

   // populate HPL
   float2Buf(&buffer[PBASE + OFFSET_HPL], gps->hpl);

   // populate HFOM
   float2Buf(&buffer[PBASE + OFFSET_HFOM], gps->hfom);

   // populate VFOM
   float2Buf(&buffer[PBASE + OFFSET_VFOM], gps->vfom);

   // populate NACv
   buffer[PBASE + OFFSET_NACV] = gps->nacv << 4;

   // populate checksum
   appendChecksum(buffer, SG_MSG_LEN_GPS);

   return true;
}

