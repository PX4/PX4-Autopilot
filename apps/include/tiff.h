/************************************************************************************
 * apps/include/tiff.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "TIFF, Revision 6.0, Final," June 3, 1992, Adobe Developers Association.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __APPS_INCLUDE_TIFF_H
#define __APPS_INCLUDE_TIFF_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/nx/nxglib.h>

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* TIFF File Format Definitions *****************************************************/
/* Values for the IFD field type */

#define IFD_FIELD_BYTE              1 /* 8-bit unsigned integer */
#define IFD_FIELD_ASCII             2 /* 8-bit byte that contains a 7-bit ASCII code.
                                       * The last byte must be NUL */
#define IFD_FIELD_SHORT             3 /* 16-bit (2-byte) unsigned integer */
#define IFD_FIELD_LONG              4 /* 32-bit (4-byte) unsigned integer */
#define IFD_FIELD_RATIONAL          5 /* Two LONGs: the first represents the
                                       * numerator of a fraction, the second the
                                       * denominator */
#define IFD_FIELD_SBYTE             6 /* An 8-bit signed (twos-complement) integer */
#define IFD_FIELD_UNDEFINED         7 /* An 8-bit byte that may contain anything,
                                       * depending on the definition of the field */
#define IFD_FIELD_SSHORT            8 /* A 16-bit (2-byte) signed (twos-complement)
                                       * integer */
#define IFD_FIELD_SLONG             9 /* A 32-bit (4-byte) signed (twos-complement)
                                       * integer */
#define IFD_FIELD_SRATIONAL        10 /* Two SLONG’s: the first represents the
                                       * numerator of a fraction, the second the
                                       * denominator */
#define IFD_FIELD_FLOAT            11 /* Single precision (4-byte) IEEE format */
#define IFD_FIELD_DOUBLE           12 /* Double precision (8-byte) IEEE format */

/* Values for the IFD tag type */

#define IFD_TAG_NEWSUBFILETYPE    254 /* NewSubfileType, LONG */
#  define TAG_NEWSUBFILETYPE_REDUCED (1 << 0) /* Bit 0: Reduced resolution verson of image */
#  define TAG_NEWSUBFILETYPE_SINGLE  (1 << 1) /* Bit 1: Single page of a multi-page image */
#  define TAG_NEWSUBFILETYPE_TRANSP  (1 << 2) /* Bit 2: Defines a transparency mask for image */
#define IFD_TAG_SUBFILETYPE       255 /* SubfileType, SHORT */
#  define TAG_SUBFILETYPE_FULL      1 /*   Full-resolution image data */
#  define TAG_SUBFILETYPE_REDUCED   2 /*   Reduced-resolution image data */
#  define TAG_SUBFILETYPE_SINGLE    3 /*   Single page of a multi-page image */
#define IFD_TAG_IMAGEWIDTH        256 /* ImageLength, SHORT or LONG (Required) */
#define IFD_TAG_IMAGELENGTH       257 /* ImageWidth, SHORT or LONG (Required) */
#define IFD_TAG_BITSPERSAMPLE     258 /* BitsPerSample, SHORT (Required
                                       * in greyscale and pallette-color image files) */ 
#define IFD_TAG_COMPRESSION       259 /* Compression, SHORT (Required) */
#  define TAG_COMP_NONE             1 /*   No compression */
#  define TAG_COMP_CCITT            2 /*   CCITT Group 3 1-Dimensional Modified Huffman
                                       *   run length encoding */
#  define TAG_COMP_T4               3 /*   CCITT T.4 bi-level encoding */
#  define TAG_COMP_T6               4 /*   CCITT T.6 bi-level encoding */
#  define TAG_COMP_LZW              5 /*   LZW */
#  define TAG_COMP_JPEG             6 /*   LZW */
#  define TAG_COMP_PACKBITS     32773 /*   PackBits compression */
#define IFD_TAG_PMI               262 /* PhotometricInterpretation, SHORT (Required) */
#  define TAG_PMI_WHITE             0 /*   WhiteIsZero */
#  define TAG_PMI_BLACK             1 /*   BlackIsZero */
#  define TAG_PMI_RGB               2 /*   RGB */
#  define TAG_PMI_PALETTE           3 /*   Palette color */
#  define TAG_PMI_TRANSP            4 /*   Transparency mask */
#  define TAG_PMI_CMYK              5 /*   CMYK */
#  define TAG_PMI_YCbCr             6 /*   YCbCr */
#  define TAG_PMI_CIELAB            8 /*   1976 CIE L*a*b* */
#define IFD_TAG_THRESHHOLDING     263 /* Threshholding, SHORT */
#  define TAG_THRESHHOLD_NONE       1 /*   No dithering or halftoning has been applied */
#  define TAG_THRESHHOLD_ORDERED    2 /*   Ordered dither or halftone technique has been applied */
#  define TAG_THRESHHOLD_RANDOM     3 /*   Randomized process has been applied */
#define IFD_TAG_CELLWIDTH         264 /* CellWidth, SHORT */
#define IFD_TAG_CELLLENGTH        265 /* CellLength, SHORT */
#define IFD_TAG_FILLORDER         266 /* FillOrder, SHORT */
#  define TAG_FILLORDER_HIGH        1 /*   Lower column values are stored in the
                                       *   higher-order bits */
#  define TAG_FILLORDER_LOW         2 /*   Lower column values are stored in the
                                       *   lower-order bits */
#define IFD_TAG_DOCUMENTNAME      269 /* DocumentName, ASCII */
#define IFD_TAG_IMAGEDESCRIPTION  270 /* ImageDescription, ASCII */
#define IFD_TAG_MAKE              271 /* Make, ASCII */
#define IFD_TAG_MODEL             272 /* Model, ASCII */
#define IFD_TAG_STRIPOFFSETS      273 /* StripOffsets, SHORT or LONG (Required) */
#define IFD_TAG_ORIENTATION       274 /* Orientation, SHORT */
#  define TAG_ORIENTATION_TL        1 /*   (0,0)=top left */
#  define TAG_ORIENTATION_TR        2 /*   (0,0)=top right */
#  define TAG_ORIENTATION_BR        3 /*   (0,0)=bottom right */
#  define TAG_ORIENTATION_BL        4 /*   (0,0)=bottom left */
#  define TAG_ORIENTATION_LT        5 /*   (0,0)=left top */
#  define TAG_ORIENTATION_RT        6 /*   (0,0)=right top */
#  define TAG_ORIENTATION_RB        7 /*   (0,0)=right bottom */
#  define TAG_ORIENTATION_LB        8 /*   (0,0)=left bottom */
#define IFD_TAG_SAMPLESPERPIXEL   277 /* SamplesPerPixel, SHORT (Required in
                                       * RGB full color files) */
#define IFD_TAG_ROWSPERSTRIP      278 /* RowsPerStrip, SHORT or LONG (Required) */
#define IFD_TAG_STRIPCOUNTS       279 /* StripByteCounts, SHORT or LONG (Required) */
#define IFD_TAG_MINSAMPLEVALUE    280 /* MinSampleValue, SHORT */
#define IFD_TAG_MAXSAMPLEVALUE    281 /* MaxSampleValue, SHORT */
#define IFD_TAG_XRESOLUTION       282 /* XResolution, RATIONAL (Required) */
#define IFD_TAG_YRESOLUTION       283 /* YResolution, RATIONAL (Required) */
#define IFD_TAG_PLANARCONFIG      284 /* PlanarConfiguration, SHORT */
#  define TAG_PLCONFIG_CHUNKY       1 /*   Chunky format */
#  define TAG_PLCONFIG_PLANAR       2 /*   Planar format */
#define IFD_TAG_PAGENAME          285 /* PageName, ASCII */
#define IFD_TAG_XPOSITION         286 /* XPosition, RATIONAL */
#define IFD_TAG_YPOSITION         287 /* YPosition, RATIONAL */
#define IFD_TAG_FREEOFFSETS       288 /* FreeOffsets, LONG */
#define IFD_TAG_FREEBYTECOUNTS    289 /* FreeByteCounts, LONG */
#define IFD_TAG_GRAYRESPONSEUNIT  290 /* GrayResponseUnit, SHORT */
#  define TAG_GRAYRESPUNIT_10THS    1 /*   Number represents tenths of a unit */
#  define TAG_GRAYRESPUNIT_100THS   2 /*   Number represents hundredths of a unit */
#  define TAG_GRAYRESPUNIT_1KTHS    3 /*   Number represents thousandths of a unit */
#  define TAG_GRAYRESPUNIT_10KTHS   4 /*   Number represents ten-thousandths of a unit */
#  define TAG_GRAYRESPUNIT_100KTHS  5 /*   Number represents hundred-thousandths of a unit */
#define IFD_TAG_GRAYRESPONSECURVE 291 /* GrayResponseCurve, SHORT */
#define IFD_TAG_T4OPTIONS         292 /* T4Options, LONG */
#  define TAG_T4OPTIONS_2D        (1 << 0) /*   2-dimensional coding */
#  define TAG_T4OPTIONS_NONE      (1 << 1) /*   Uncompressed mode */
#  define TAG_T4OPTIONS_FILL      (1 << 2) /*   Fill bits have been added */
#define IFD_TAG_T6OPTIONS         293 /* T6Options, LONG */
#  define TAG_T6OPTIONS_NONE      (1 << 1) /*   Uncompressed mode allowed */
#define IFD_TAG_RESUNIT           296 /* ResolutionUnit, SHORT (Required) */
#  define TAG_RESUNIT_NONE          1 /* No absolute unit of measurement */
#  define TAG_RESUNIT_INCH          2 /* Inch (default) */
#  define TAG_RESUNIT_CENTIMETER    3 /* Centimeter */
#define IFD_TAG_PAGENUMBER        297 /* PageNumber, SHORT */
#define IFD_TAG_TRANSFERFUNCTION  301 /* TransferFunction, SHORT */
#define IFD_TAG_SOFTWARE          305 /* Software, ASCII */
#define IFD_TAG_DATETIME          306 /* DateTime, ASCII */
#define IFD_TAG_ARTIST            315 /* Artist, ASCII */
#define IFD_TAG_HOSTCOMPUTER      316 /* HostComputer, ASCII */
#define IFD_TAG_PREDICTOR         317 /* Predictor SHORT */
#  define TAG_PREDICTOR_NONE        1 /*   No prediction scheme used before coding */
#  define TAG_PREDICTOR_HORIZ       2 /*   Horizontal differencing */
#define IFD_TAG_WHITEPOINT        318 /* WhitePoint, RATIONAL */
#define IFD_TAG_PRIMARYCHROMA     319 /* PrimaryChromaticities, RATIONAL */
#define IFD_TAG_COLORMAP          320 /* ColorMap, SHORT (Required in palette
                                       * color image files) */
#define IFD_TAG_HALFTONEHINTS     321 /* HalftoneHints, SHORT */
#define IFD_TAG_TILEWIDTH         322 /* TileWidth, SHORT or LONG */
#define IFD_TAG_TILELENGTH        323 /* TileLength, SHORT or LONG */
#define IFD_TAG_TILEOFFSETS       324 /* TileOffsets, LONG */
#define IFD_TAG_TILEBYTECOUNTS    325 /* TileByteCounts, SHORT or LONG */
#define IFD_TAG_INKSET            332 /* InkSet, SHORT */
#  define TAG_INKSET_CMYK           1 /*   CMYK */
#  define TAG_INKSET_OTHER          2 /*   Not CMYK */
#define IFD_TAG_INKNAMES          333 /* InkNames, ASCII */
#define IFD_TAG_NUMBEROFINKS      334 /* NumberOfInks, SHORT */
#define IFD_TAG_DOTRANGE          336 /* DotRange, BYTE or SHORT */
#define IFD_TAG_TARGETPRINTER     337 /* TargetPrinter, ASCII */
#define IFD_TAG_EXTRASAMPLES      338 /* ExtraSamples, SHORT */
#  define TAG_EXTSAMP_UNSPEC        0 /*   Unspecified */
#  define TAG_EXTSAMP_ASSOCALPHA    1 /*   Associated alpha data */
#  define TAG_EXTSAMP_UNASSALPHA    2 /*   Unassociated alpha data */
#define IFD_TAG_SAMPLEFORMAT      339 /* SampleFormat, SHORT */
#  define TAG_SAMPLEFMT_UNSIGED     1 /*   Unsigned integer data */
#  define TAG_SAMPLEFMT_SIGNED      2 /*   Two’s complement signed integer data */
#  define TAG_SAMPLEFMT_FLOAT       3 /*   IEEE floating point data */
#  define TAG_SAMPLEFMT_UNDEFINED   4 /*   Undefined data format */
#define IFD_TAG_SMINSAMPLEVALUE   340 /* SMinSampleValue, type matches sample data */
#define IFD_TAG_SMAXSAMPLEVALUE   341 /* SMaxSampleValue, type matches sample data */
#define IFD_TAG_TRANSFERRANGE     342 /* TransferRange, SHORT */
#define IFD_TAG_JPEGPROC          512 /* JPEGProc, SHORT */
#define IFD_TAG_JPEGFMT           513 /* JPEGInterchangeFormat, LONG */
#define IFD_TAG_JPEGLENGTH        514 /* JPEGInterchangeFormatLength, LONG */
#define IFD_TAG_JPEGRESTART       515 /* JPEGRestartInterval, SHORT */
#define IFD_TAG_JPEGLLPREDICTORS  517 /* JPEGLosslessPredictors, SHORT */
#define IFD_TAG_JPEGPOINTXFORMS   518 /* JPEGPointTransforms, SHORT */
#define IFD_TAG_JPEGQTABLES       519 /* JPEGQTables, LONG */
#define IFD_TAG_JPEGDCTABLES      520 /* JPEGDCTables, LONG */
#define IFD_TAG_JPEGACTABLES      521 /* JPEGACTables, LONG */
#define IFD_TAG_YCbCrCOEFFS       529 /* YCbCrCoefficients, RATIONAL */
#define IFD_TAG_YCbCrSUBSAMPLING  530 /* YCbCrSubSampling, SHORT */
#define IFD_TAG_YCbCrPOSITIONING  531 /* YCbCrPositioning, SHORT */
#define IFD_TAG_REFERENCEBW       532 /* ReferenceBlackWhite, RATIONAL */
#define IFD_TAG_COPYRIGHT       33432 /* Copyright, ASCII */

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* TIFF File Format Structure *******************************************************/
/* "A TIFF file begins with an 8-byte image file header that points to an
 *  image file directory (IFD). An image file directory contains information
 *  about the image, as well as pointers to the actual image data."
 */

struct tiff_header_s
{
  uint8_t order[2];  /* 0-1: Byte order:  "II"=little endian, "MM"=big endian */
  uint8_t magic[2];  /* 2-3: 42 in appropriate byte order */
  uint8_t offset[4]; /* 4-7: Offset to the first IFD */
};
#define SIZEOF_TIFF_HEADER 8

/* "An Image File Directory (IFD) consists of a 2-byte count of the number
 *  of directory entries (i.e., the number of fields), followed by a sequence
 *  of 12-byte field entries, followed by a 4-byte offset of the next IFD (or
 *  0 if none).
 *
 * Each 12-byte IFD entry has the following format:
 */

struct tiff_ifdentry_s
{
  uint8_t tag[2];    /* 0-1: The Tag that identifies the field */
  uint8_t type[2];   /* 2-3 The field Type */
  uint8_t count[4];  /* 4-7: The number of values of the indicated type */
  uint8_t offset[4]; /* 8-11: The Value Offset (or the value itself) */
};
#define SIZEOF_IFD_ENTRY 12

/************************************************************************************/
/* Structures needed to interface with the TIFF file creation library )and also 
 * structures used only internally by the TIFF file creation library).
 */

/* This structure describes on strip in tmpfile2 */

struct tiff_strip_s
{
  uint32_t offset; /* Offset to the strip data in tmpfile1 */
  uint32_t count;  /* Count of pixels in the strip */
};

/* This structure is used only internally by the TIFF file creation library to
 * manage file offsets.
 */

struct tiff_filefmt_s
{
  uint16_t nifdentries;    /* Number of IFD entries */
  uint16_t soifdoffset;    /* Offset to StripOffset IFD entry */
  uint16_t sbcifdoffset;   /* Offset to StripByteCount IFD entry */
  uint16_t valoffset;      /* Offset to first values */
  uint16_t xresoffset;     /* Offset to XResolution values */
  uint16_t yresoffset;     /* Offset to yResolution values */
  uint16_t swoffset;       /* Offset to Software string */
  uint16_t dateoffset;     /* Offset to DateTime string */
  uint16_t sbcoffset;      /* Offset to StripByteCount values */
};

/* These type is used to hold information about the TIFF file under
 * construction
 */

struct tiff_info_s
{
  /* The first fields are used to pass information to the TIFF file creation
   * logic via tiff_initialize().
   *
   * Filenames.  Three file names are required.  (1) path to the final
   * output file and (2) two paths to temporary files.  One temporary file
   * (tmpfile1) will be used to hold the strip image data and the other
   * (tmpfile2) will be used to hold strip offset and count information.
   *
   * colorfmt  - Specifies the form of the color data that will be provided
   *             in the strip data.  These are the FB_FMT_* definitions
   *             provided in include/nuttx/fb.h.  Only the following values
   *             are supported:
   *
   *             FB_FMT_Y1               BPP=1, monochrome, 0=black
   *             FB_FMT_Y4               BPP=4, 4-bit greyscale, 0=black
   *             FB_FMT_Y8               BPP=8, 8-bit greyscale, 0=black
   *             FB_FMT_RGB16_565        BPP=16 R=6, G=6, B=5
   *             FB_FMT_RGB24            BPP=24 R=8, G=8, B=8
   *
   * rps       - TIFF RowsPerStrip
   * imgwidth  - TIFF ImageWidth, Number of columns in the image
   * imgheight - TIFF ImageLength, Number of rows in the image
   */

  FAR const char *outfile;  /* Full path to the final output file name */
  FAR const char *tmpfile1; /* Full path to first temporary file */
  FAR const char *tmpfile2; /* Full path to second temporary file */

  uint8_t      colorfmt;    /* See FB_FMT_* definitions in include/nuttx/fb.h */
  nxgl_coord_t rps;         /* TIFF RowsPerStrip */
  nxgl_coord_t imgwidth;    /* TIFF ImageWidth, Number of columns in the image */
  nxgl_coord_t imgheight;   /* TIFF ImageLength, Number of rows in the image */

  /* The caller must provide an I/O buffer as well.  This I/O buffer will
   * used for color conversions and as the intermediate buffer for copying
   * files.  The larger the buffer, the better the performance.
   */

  FAR uint8_t *iobuffer;    /* IO buffer allocated by the caller */
  unsigned int iosize;      /* The size of the I/O buffer in bytes */

  /* The second set of fields are used only internally by the TIFF file
   * creation logic.  These fields must be set to zero initially by the
   * caller of tiff_initialize().  User logic should not depend upon any
   * definitions in the following -- they are subject to change without
   * notice.  They are only exposed here so that the caller can allocate
   * memory for their storage.
   */

  uint8_t      imgflags;    /* Bit-encoded image flags */
  nxgl_coord_t nstrips;     /* Number of strips in tmpfile3 */
  size_t       pps;         /* Pixels per strip */
  size_t       bps;         /* Bytes per strip */
  int          outfd;       /* outfile file descriptor */
  int          tmp1fd;      /* tmpfile1 file descriptor */
  int          tmp2fd;      /* tmpfile2 file descriptor */
  off_t        outsize;     /* Current size of outfile */
  off_t        tmp1size;    /* Current size of tmpfile1 */
  off_t        tmp2size;    /* Current size of tmpfile2 */

  /* Points to an internal constant structure of file offsets */
  
  FAR const struct tiff_filefmt_s *filefmt;
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: tiff_initialize
 *
 * Description:
 *   Setup to create a new TIFF file.  The overall steps to creating a TIFF file are
 *   as follows:
 *
 *   1) Create an initialize a struct tiff_info_s instance
 *   2) Call tiff_initialize() to setup the file creation
 *   3) Call tiff_addstrip() repeatedly to add strips to the graphic image
 *   4) Call tiff_finalize() to complete the file creation.
 *
 * Input Parameters:
 *   info - A pointer to the caller allocated parameter passing/TIFF state instance.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ************************************************************************************/

EXTERN int tiff_initialize(FAR struct tiff_info_s *info);

/************************************************************************************
 * Name: tiff_addstrip
 *
 * Description:
 *   Add an image data strip.  The size of the strip in pixels must be equal to
 *   the RowsPerStrip x ImageWidth values that were provided to tiff_initialize().
 *
 * Input Parameters:
 *   info    - A pointer to the caller allocated parameter passing/TIFF state instance.
 *   buffer  - A buffer containing a single row of data.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ************************************************************************************/

EXTERN int tiff_addstrip(FAR struct tiff_info_s *info, FAR const uint8_t *strip);

/************************************************************************************
 * Name: tiff_finalize
 *
 * Description:
 *   Finalize the TIFF output file, completing the TIFF file creation steps.
 *
 * Input Parameters:
 *   info - A pointer to the caller allocated parameter passing/TIFF state instance.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ************************************************************************************/

EXTERN int tiff_finalize(FAR struct tiff_info_s *info);

/************************************************************************************
 * Name: tiff_abort
 *
 * Description:
 *   Abort the TIFF file creation and create-up resources.
 *
 * Input Parameters:
 *   info - A pointer to the caller allocated parameter passing/TIFF state instance.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

EXTERN void tiff_abort(FAR struct tiff_info_s *info);

/************************************************************************************
 * Name: tiff_put/get16/32
 *
 * Description:
 *   Put and get 16 and 32 values in the correct byte order at the specified position.
 *
 * Input Parameters:
 *   dest - The location to store the multi-byte data (put only)
 *   src - The location to get the multi-byte data (get only)
 *
 * Returned Value:
 *   None (put)
 *   The extracted value (get)
 *
 ************************************************************************************/

EXTERN void tiff_put16(FAR uint8_t *dest, uint16_t value);
EXTERN void tiff_put32(FAR uint8_t *dest, uint32_t value);
EXTERN uint16_t tiff_get16(FAR uint8_t *dest);
EXTERN uint32_t tiff_get32(FAR uint8_t *dest);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_TIFF_H */
