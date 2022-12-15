#ifndef VN_TYPES_H_INCLUDED
#define VN_TYPES_H_INCLUDED

/** \brief Standard types used through out the library. */
#include "vn/int.h"

#if !defined(__cplusplus)

#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L) || defined(__GNUC__)

    #include <stddef.h>

#else

	/* Must not have C99. */

	/** Backup definition of size_t. */
	typedef unsigned int size_t;

#endif

#else
extern "C" {
#endif


/* Adding VectorNav data types */
typedef struct
{
  int8_t year;	  /** \brief Year field. */
  uint8_t month;  /** \brief Month field. */
  uint8_t day;	  /** \brief Day field. */
  uint8_t hour;	  /** \brief Hour field. */
  uint8_t min;	  /** \brief Min field. */
  uint8_t sec;	  /** \brief Sec field. */
  uint16_t ms;	  /** \brief Ms field. */
} TimeUtc;

typedef struct
{
  float gDOP;	/** \brief Gdop field. */
  float pDOP;	/** \brief Pdop field. */
  float tDOP;	/** \brief Tdop field. */
  float vDOP;	/** \brief Vdop field. */
  float hDOP;	/** \brief Hdop field. */
  float nDOP;	/** \brief Ndop field. */
  float eDOP;	/** \brief Edop field. */
} GpsDop;

typedef struct
{
  uint8_t timeStatus;	/** \brief timeStatus field. */
  int8_t  leapSeconds;	/** \brief leapSeconds field. */
} TimeInfo;

#ifdef __cplusplus
}
#endif

#endif
