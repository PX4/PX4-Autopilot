/****************************************************************************
 * apps/graphics/tiff/tiff_initialize.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <apps/tiff.h>

#include "tiff_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Bi-level Images
 *
 *          Offset Description                 Contents/Notes
 * Header:    0    Byte Order                  "II" or "MM"
 *            2    Magic Number                42     
 *            4    1st IFD offset              10
 *            8    [2 bytes padding]
 * IFD:      10    Number of Directory Entries 13
 *           12    NewSubfileType
 *           24    ImageWidth                  Number of columns is a user parameter
 *           36    ImageLength                 Number of rows is a user parameter
 *           48    Compression                 Hard-coded no compression (for now)
 *           60    PhotometricInterpretation   Value is a user parameter
 *           72    StripOffsets                Offset and count determined as strips added
 *           84    RowsPerStrip                Value is a user parameter
 *           96    StripByteCounts             Offset and count determined as strips added
 *          108    XResolution                 Value is a user parameter
 *          120    YResolution                 Value is a user parameter
 *          132    Resolution Unit             Hard-coded to "inches"
 *          144    Software
 *          156    DateTime
 *          168    Next IFD offset             0
 *          170    [2 bytes padding]
 * Values:
 *          172    XResolution                 Hard-coded to 300/1
 *          180    YResolution                 Hard-coded to 300/1
 *          188    "NuttX"                     Length = 6 (including NUL terminator)
 *          194    "YYYY:MM:DD HH:MM:SS"       Length = 20 (ncluding NUL terminator)
 *          214    [2 bytes padding]
 *          216    StripByteCounts             Beginning of strip byte counts
 *          xxx    StripOffsets                Beginning of strip offsets
 *          xxx    [Probably padding]
 *          xxx    Data for strips             Beginning of strip data
 */

#define TIFF_IFD_OFFSET           (SIZEOF_TIFF_HEADER+2)

#define TIFF_BILEV_NIFDENTRIES    13
#define TIFF_BILEV_STRIPIFDOFFS   72
#define TIFF_BILEV_STRIPBCIFDOFFS 96
#define TIFF_BILEV_VALOFFSET      172
#define TIFF_BILEV_XRESOFFSET     172
#define TIFF_BILEV_YRESOFFSET     180
#define TIFF_BILEV_SWOFFSET       188
#define TIFF_BILEV_DATEOFFSET     194
#define TIFF_BILEV_STRIPBCOFFSET  216

#define TIFF_SOFTWARE_STRING      "NuttX"
#define TIFF_SOFTWARE_STRLEN      6

#define TIFF_DATETIME_FORMAT      "%Y:%m:%d %H:%M:%S"
#define TIFF_DATETIME_STRLEN      20

/* Greyscale Images have one additional IFD entry: BitsPerSample (4 or 8)
 *
 * Header:    0    Byte Order                  "II" or "MM"
 *            2    Magic Number                42     
 *            4    1st IFD offset              10
 *            8    [2 bytes padding]
 * IFD:      10    Number of Directory Entries 14
 *           12    NewSubfileType
 *           24    ImageWidth                  Number of columns is a user parameter
 *           36    ImageLength                 Number of rows is a user parameter
 *           48    BitsPerSample
 *           60    Compression                 Hard-coded no compression (for now)
 *           72    PhotometricInterpretation   Value is a user parameter
 *           84    StripOffsets                Offset and count determined as strips added
 *           96    RowsPerStrip                Value is a user parameter
 *          108    StripByteCounts             Offset and count determined as strips added
 *          120    XResolution                 Value is a user parameter
 *          132    YResolution                 Value is a user parameter
 *          144    Resolution Unit             Hard-coded to "inches"
 *          156    Software
 *          168    DateTime
 *          180    Next IFD offset             0
 *          182    [2 bytes padding]
 * Values:
 *          184    XResolution                 Hard-coded to 300/1
 *          192    YResolution                 Hard-coded to 300/1
 *          200    "NuttX"                     Length = 6 (including NUL terminator)
 *          206    "YYYY:MM:DD HH:MM:SS"       Length = 20 (ncluding NUL terminator)
 *          226    [2 bytes padding]
 *          228    StripByteCounts             Beginning of strip byte counts
 *          xxx    StripOffsets                Beginning of strip offsets
 *          xxx    [Probably padding]
 *          xxx    Data for strips             Beginning of strip data
 */

#define TIFF_GREY_NIFDENTRIES    14
#define TIFF_GREY_STRIPIFDOFFS   84
#define TIFF_GREY_STRIPBCIFDOFFS 108
#define TIFF_GREY_VALOFFSET      184
#define TIFF_GREY_XRESOFFSET     184
#define TIFF_GREY_YRESOFFSET     192
#define TIFF_GREY_SWOFFSET       200
#define TIFF_GREY_DATEOFFSET     206
#define TIFF_GREY_STRIPBCOFFSET  228

/* RGB Images have two additional IFD entries: BitsPerSample (8,8,8) and
 * SamplesPerPixel (3):
 *
 * Header:    0    Byte Order                  "II" or "MM"
 *            2    Magic Number                42     
 *            4    1st IFD offset              10
 *            8    [2 bytes padding]
 * IFD:      10    Number of Directory Entries 15
 *           12    NewSubfileType
 *           24    ImageWidth                  Number of columns is a user parameter
 *           36    ImageLength                 Number of rows is a user parameter
 *           48    BitsPerSample               8, 8, 8
 *           60    Compression                 Hard-coded no compression (for now)
 *           72    PhotometricInterpretation   Value is a user parameter
 *           84    StripOffsets                Offset and count determined as strips added
 *           96    SamplesPerPixel             Hard-coded to 3
 *          108    RowsPerStrip                Value is a user parameter
 *          120    StripByteCounts             Offset and count determined as strips added
 *          132    XResolution                 Value is a user parameter
 *          144    YResolution                 Value is a user parameter
 *          156    Resolution Unit              Hard-coded to "inches"
 *          168    Software
 *          180    DateTime
 *          192    Next IFD offset             0
 *          194    [2 bytes padding]
 * Values:
 *          196    XResolution                 Hard-coded to 300/1
 *          204    YResolution                 Hard-coded to 300/1
 *          212    BitsPerSample               8, 8, 8
 *          218    [2 bytes padding]
 *          220    "NuttX"                     Length = 6 (including NUL terminator)
 *          226    "YYYY:MM:DD HH:MM:SS"       Length = 20 (ncluding NUL terminator)
 *          246    [2 bytes padding]
 *          248    StripByteCounts             Beginning of strip byte counts
 *          xxx    StripOffsets                Beginning of strip offsets
 *          xxx    [Probably padding]
 *          xxx    Data for strips             Beginning of strip data
 */

#define TIFF_RGB_NIFDENTRIES    15
#define TIFF_RGB_STRIPIFDOFFS   84
#define TIFF_RGB_STRIPBCIFDOFFS 120
#define TIFF_RGB_VALOFFSET      196
#define TIFF_RGB_XRESOFFSET     196
#define TIFF_RGB_YRESOFFSET     204
#define TIFF_RGB_BPSOFFSET      212
#define TIFF_RGB_SWOFFSET       220
#define TIFF_RGB_DATEOFFSET     226
#define TIFF_RGB_STRIPBCOFFSET  248

/* Debug *******************************************************************/
/* CONFIG_DEBUG_TIFFOFFSETS may be defined (along with CONFIG_DEBUG and
 * CONFIG_DEBUG_GRAPHICS) in order to verify the pre-determined TIFF file
 * offsets.
 */

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_GRAPHICS)
#  undef CONFIG_DEBUG_TIFFOFFSETS
#endif

#ifdef CONFIG_DEBUG_TIFFOFFSETS
#  define tiff_offset(o,l)      (o) += (l)
#  define tiff_checkoffs(o,x)   ASSERT((o) == (x))
#else
#  define tiff_offset(o,l)
#  define tiff_checkoffs(o,x)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct tiff_filefmt_s g_bilevinfo =
{
  TIFF_BILEV_NIFDENTRIES,    /* nifdentries, Number of IFD entries */
  TIFF_BILEV_STRIPIFDOFFS,   /* soifdoffset, Offset to StripOffset IFD entry */
  TIFF_BILEV_STRIPBCIFDOFFS, /* sbcifdoffset, Offset to StripByteCount IFD entry */
  TIFF_BILEV_VALOFFSET,      /* valoffset, Offset to first values */
  TIFF_BILEV_XRESOFFSET,     /* xresoffset, Offset to XResolution values */
  TIFF_BILEV_YRESOFFSET,     /* yresoffset, Offset to yResolution values */
  TIFF_BILEV_SWOFFSET,       /* swoffset, Offset to Software string */
  TIFF_BILEV_DATEOFFSET,     /* dateoffset, Offset to DateTime string */
  TIFF_BILEV_STRIPBCOFFSET   /* sbcoffset,  Offset to StripByteCount values */
};

static const struct tiff_filefmt_s g_greyinfo =
{
  TIFF_GREY_NIFDENTRIES,     /* nifdentries, Number of IFD entries */
  TIFF_GREY_STRIPIFDOFFS,    /* soifdoffset, Offset to StripOffset IFD entry */
  TIFF_GREY_STRIPBCIFDOFFS,  /* sbcifdoffset, Offset to StripByteCount IFD entry */
  TIFF_GREY_VALOFFSET,       /* valoffset, Offset to first values */
  TIFF_GREY_XRESOFFSET,      /* xresoffset, Offset to XResolution values */
  TIFF_GREY_YRESOFFSET,      /* yresoffset, Offset to yResolution values */
  TIFF_GREY_SWOFFSET,        /* swoffset, Offset to Software string */
  TIFF_GREY_DATEOFFSET,      /* dateoffset, Offset to DateTime string */
  TIFF_GREY_STRIPBCOFFSET    /* sbcoffset,  Offset to StripByteCount values */
};

static const struct tiff_filefmt_s g_rgbinfo =
{
  TIFF_RGB_NIFDENTRIES,      /* nifdentries, Number of IFD entries */
  TIFF_RGB_STRIPIFDOFFS,     /* soifdoffset, Offset to StripOffset IFD entry */
  TIFF_RGB_STRIPBCIFDOFFS,   /* sbcifdoffset, Offset to StripByteCount IFD entry */
  TIFF_RGB_VALOFFSET,        /* valoffset, Offset to first values */
  TIFF_RGB_XRESOFFSET,       /* xresoffset, Offset to XResolution values */
  TIFF_RGB_YRESOFFSET,       /* yresoffset, Offset to yResolution values */
  TIFF_RGB_SWOFFSET,         /* swoffset, Offset to Software string */
  TIFF_RGB_DATEOFFSET,       /* dateoffset, Offset to DateTime string */
  TIFF_RGB_STRIPBCOFFSET     /* sbcoffset,  Offset to StripByteCount values */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiff_putheader
 *
 * Description:
 *   Setup to create a new TIFF file.
 *
 * Input Parameters:
 *   info - A pointer to the caller allocated parameter passing/TIFF state
 *          instance.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ****************************************************************************/

static inline int tiff_putheader(FAR struct tiff_info_s *info)
{
  struct tiff_header_s hdr;
  int ret;

  /* 0-1: Byte order */

#ifdef CONFIG_ENDIAN_BIG
  hdr.order[0] = 'M';  /* "MM"=big endian */
  hdr.order[1] = 'M';
#else
  hdr.order[0] = 'I';  /* "II"=little endian */
  hdr.order[1] = 'I';
#endif

  /* 2-3: 42 in appropriate byte order */

  tiff_put16(hdr.magic, 42);

  /* 4-7: Offset to the first IFD */

  tiff_put32(hdr.offset, TIFF_IFD_OFFSET);

  /* Write the header to the output file */

  ret = tiff_write(info->outfd, &hdr, SIZEOF_TIFF_HEADER);
  if (ret != OK)
    {
      return ret;
    }

 /* Two pad bytes following the header */

 ret = tiff_putint16(info->outfd, 0);
 return ret;
}

/****************************************************************************
 * Name: tiff_putifdentry
 *
 * Description:
 *   Write an IFD entry to outfile
 *
 * Input Parameters:
 *   info   - A pointer to the caller allocated parameter passing/TIFF state
 *            instance.
 *   tag    - The value for the IFD tag field
 *   type   - The value for the IFD type field
 *   count  - The value for the IFD count field
 *   offset - The value for the IFD offset field
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ****************************************************************************/

static int tiff_putifdentry(FAR struct tiff_info_s *info, uint16_t tag,
                           uint16_t type, uint32_t count, uint32_t offset)
{
  struct tiff_ifdentry_s ifd;
  tiff_put16(ifd.tag, tag);
  tiff_put16(ifd.type, type);
  tiff_put32(ifd.count, count);
  tiff_put32(ifd.offset, offset);
  return tiff_write(info->outfd, &ifd, SIZEOF_IFD_ENTRY);
}

/****************************************************************************
 * Name: tiff_putifdentry
 *
 * Description:
 *   Write an IFD with a 16-bit immediate value
 *
 * Input Parameters:
 *   info - A pointer to the caller allocated parameter passing/TIFF state
 *          instance.
 *   tag    - The value for the IFD tag field
 *   type   - The value for the IFD type field
 *   count  - The value for the IFD count field
 *   value  - The 16-bit immediate value
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ****************************************************************************/

static int tiff_putifdentry16(FAR struct tiff_info_s *info, uint16_t tag,
                             uint16_t type, uint32_t count, uint16_t value)
{
  union
  {
    uint8_t  b[4];
    uint32_t w;
  } u;

  u.w = 0;
  tiff_put16(u.b, value);
  return tiff_putifdentry(info, tag, type, count, u.w);
}

/****************************************************************************
 * Name: tiff_datetime
 *
 * Description:
 *   Get the DateTime string
 *
 * Input Parameters:
 *   
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ****************************************************************************/

static int tiff_datetime(FAR char *timbuf, unsigned int buflen)
{
  struct timespec ts;
  struct tm tm;
  int ret;

  /* Get the current time */

  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if (ret < 0)
    {
      gdbg("clock_gettime failed: %d\n", errno);
      return ERROR;
    }

  /* Break the current time up into the format needed by strftime */

  (void)gmtime_r((FAR const time_t*)&ts.tv_sec, &tm);

  /* Comvert the current time in the TIFF format */

  (void)strftime(timbuf, buflen, TIFF_DATETIME_FORMAT, &tm);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiff_initialize
 *
 * Description:
 *   Setup to create a new TIFF file.
 *
 * Input Parameters:
 *   info - A pointer to the caller allocated parameter passing/TIFF state instance.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value on failure.
 *
 ****************************************************************************/

int tiff_initialize(FAR struct tiff_info_s *info)
{
  uint16_t val16;
#if CONFIG_DEBUG_TIFFOFFSETS
  off_t offset = 0;
#endif
  char timbuf[TIFF_DATETIME_STRLEN + 8];
  int ret = -EINVAL;

  DEBUGASSERT(info && info->outfile && info->tmpfile1 && info->tmpfile2);

  /* Open all output files */

  info->outfd = open(info->outfile, O_RDWR|O_CREAT|O_TRUNC, 0666);
  if (info->outfd < 0)
    {
      gdbg("Failed to open %s for reading/writing: %d\n", info->outfile, errno);
      goto errout;
    }

  info->tmp1fd = open(info->tmpfile1, O_RDWR|O_CREAT|O_TRUNC, 0666);
  if (info->tmp1fd < 0)
    {
      gdbg("Failed to open %s for reading/writing: %d\n", info->tmpfile1, errno);
      goto errout;
    }

  info->tmp2fd = open(info->tmpfile2, O_RDWR|O_CREAT|O_TRUNC, 0666);
  if (info->tmp2fd < 0)
    {
      gdbg("Failed to open %s for reading/writing: %d\n", info->tmpfile2, errno);
      goto errout;
    }

  /* Make some decisions using the color format.  Only the following are
   * supported:
   */

  info->pps = info->imgwidth * info->rps;           /* Pixels per strip */
  switch (info->colorfmt)
    {
      case FB_FMT_Y1:                               /* BPP=1, monochrome, 0=black */
        info->filefmt  = &g_bilevinfo;              /* Bi-level file image file info */
        info->imgflags = IMGFLAGS_FMT_Y1;           /* Bit encoded image characteristics */
        info->bps      = (info->pps + 7) >> 3;      /* Bytes per strip */
        break;

      case FB_FMT_Y4:                               /* BPP=4, 4-bit greyscale, 0=black */
        info->filefmt  = &g_greyinfo;               /* Greyscale file image file info */
        info->imgflags = IMGFLAGS_FMT_Y4;           /* Bit encoded image characteristics */
        info->bps      = (info->pps + 1) >> 1;      /* Bytes per strip */
        break;

      case FB_FMT_Y8:                               /* BPP=8, 8-bit greyscale, 0=black */
        info->filefmt  = &g_greyinfo;               /* Greyscale file image file info */
        info->imgflags = IMGFLAGS_FMT_Y8;           /* Bit encoded image characteristics */
        info->bps      = info->pps;                 /* Bytes per strip */
        break;

      case FB_FMT_RGB16_565:                        /* BPP=16 R=6, G=6, B=5 */
        info->filefmt  = &g_rgbinfo;                /* RGB file image file info */
        info->imgflags = IMGFLAGS_FMT_RGB16_565;    /* Bit encoded image characteristics */
        info->bps      = 3 * info->pps;             /* Bytes per strip */
        break;

      case FB_FMT_RGB24:                            /* BPP=24 R=8, G=8, B=8 */
        info->filefmt  = &g_rgbinfo;                /* RGB file image file info */
        info->imgflags = IMGFLAGS_FMT_RGB24;        /* Bit encoded image characteristics */
        info->bps      = 3 *info->pps;              /* Bytes per strip */
        break;

      default:
        gdbg("Unsupported color format: %d\n", info->colorfmt);
        return -EINVAL;
    }

  /* Write the TIFF header data to the outfile:
   *
   * Header:    0    Byte Order                  "II" or "MM"
   *            2    Magic Number                42     
   *            4    1st IFD offset              10
   *            8    [2 bytes padding]
   */

  ret = tiff_putheader(info);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, TIFF_IFD_OFFSET);

  /* Write the Number of directory entries
   *
   * All formats: Offset 10 Number of Directory Entries 12
   */

  ret = tiff_putint16(info->outfd, info->filefmt->nifdentries);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 2);

  /* Write the NewSubfileType IFD entry
   *
   * All formats: Offset 12 NewSubfileType
   */

  ret = tiff_putifdentry16(info, IFD_TAG_NEWSUBFILETYPE, IFD_FIELD_LONG, 1, 0);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write ImageWidth and ImageLength
   *
   * All formats: Offset 24 ImageWidth  Number of columns is a user parameter
   *                     36 ImageLength Number of rows is a user parameter
   */

  ret = tiff_putifdentry16(info, IFD_TAG_IMAGEWIDTH, IFD_FIELD_SHORT, 1, info->imgwidth);
  if (ret == OK)
    {
      ret= tiff_putifdentry16(info, IFD_TAG_IMAGELENGTH, IFD_FIELD_SHORT, 1, info->imgheight);
    }

  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 2*SIZEOF_IFD_ENTRY);

  /* Write BitsPerSample
   *
   * Bi-level Images: None
   * Greyscale:       Offset 48 BitsPerSample (4 or 8)
   * RGB:             Offset 48 BitsPerSample (8,8,8)
   */

  tiff_checkoffs(offset, 48);
  if (IMGFLAGS_ISGREY(info->imgflags))
    {
      if (IMGFLAGS_ISGREY8(info->imgflags))
        {
          val16 = 8;
        }
      else
        {
          val16 = 4;
        }

      ret = tiff_putifdentry16(info, IFD_TAG_BITSPERSAMPLE, IFD_FIELD_SHORT, 1, val16);
      if (ret < 0)
        {
          goto errout;
        }
      tiff_offset(offset, SIZEOF_IFD_ENTRY);
    }
  else if (IMGFLAGS_ISRGB(info->imgflags))
    {
      ret = tiff_putifdentry(info, IFD_TAG_BITSPERSAMPLE, IFD_FIELD_SHORT, 3, TIFF_RGB_BPSOFFSET);
      if (ret < 0)
        {
          goto errout;
        }
      tiff_offset(offset, SIZEOF_IFD_ENTRY);
    }

  /* Write Compression:
   *
   * Bi-level Images: Offset 48 Hard-coded no compression (for now)
   * Greyscale:       Offset 60  "  " "   " "" "          " " " " "
   * RGB:             Offset 60 "  " "   " "" "          " " " " "
   */

  ret = tiff_putifdentry16(info, IFD_TAG_COMPRESSION, IFD_FIELD_SHORT, 1, TAG_COMP_NONE);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write PhotometricInterpretation:
   *
   * Bi-level Images: Offset 48 Hard-coded BlackIsZero
   * Greyscale:       Offset 72 Hard-coded BlackIsZero
   * RGB:             Offset 72 Hard-coded RGB
   */

  if (IMGFLAGS_ISRGB(info->imgflags))
    {
      val16 = TAG_PMI_RGB;
    }
  else
    {
      val16 = TAG_PMI_BLACK;
    }

  ret = tiff_putifdentry16(info, IFD_TAG_PMI, IFD_FIELD_SHORT, 1, val16);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write StripOffsets:
   *
   * Bi-level Images: Offset 72 Value determined by switch statement above
   * Greyscale:       Offset 84 Value determined by switch statement above
   * RGB:             Offset 84 Value determined by switch statement above
   */

  tiff_checkoffs(offset, info->filefmt->soifdoffset);
  ret = tiff_putifdentry(info, IFD_TAG_STRIPOFFSETS, IFD_FIELD_LONG, 0, 0);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write SamplesPerPixel
   *
   * Bi-level Images: N/A
   * Greyscale:       N/A
   * RGB:             Offset 96 Hard-coded to 3
   */

  if (IMGFLAGS_ISRGB(info->imgflags))
    {
      ret = tiff_putifdentry16(info, IFD_TAG_SAMPLESPERPIXEL, IFD_FIELD_SHORT, 1, 3);
      if (ret < 0)
        {
          goto errout;
        }
      tiff_offset(offset, SIZEOF_IFD_ENTRY);
    }

  /* Write RowsPerStrip:
   *
   * Bi-level Images: Offset  84 Value is a user parameter
   * Greyscale:       Offset  96 Value is a user parameter
   * RGB:             Offset 108 Value is a user parameter
   */

  ret = tiff_putifdentry16(info, IFD_TAG_ROWSPERSTRIP, IFD_FIELD_SHORT, 1, info->rps);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write StripByteCounts:
   *
   * Bi-level Images: Offset  96 Count determined as strips added, Value offset = 216
   * Greyscale:       Offset 108 Count determined as strips added, Value offset = 228
   * RGB:             Offset 120 Count determined as strips added, Value offset = 248
   */

  tiff_checkoffs(offset, info->filefmt->sbcifdoffset);
  ret = tiff_putifdentry(info, IFD_TAG_STRIPCOUNTS, IFD_FIELD_LONG, 0, info->filefmt->sbcoffset);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write XResolution and YResolution:
   *
   * Bi-level Images: Offset 108 and 120, Values are a user parameters
   * Greyscale:       Offset 120 and 132, Values are a user parameters
   * RGB:             Offset 132 and 144, Values are a user parameters
   */

  ret = tiff_putifdentry(info, IFD_TAG_XRESOLUTION, IFD_FIELD_RATIONAL, 1, info->filefmt->xresoffset);
  if (ret == OK)
    {
      ret = tiff_putifdentry(info, IFD_TAG_YRESOLUTION, IFD_FIELD_RATIONAL, 1, info->filefmt->yresoffset);
    }

  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 2*SIZEOF_IFD_ENTRY);

  /* Write ResolutionUnit:
   *
   * Bi-level Images: Offset 132, Hard-coded to "inches"
   * Greyscale:       Offset 144, Hard-coded to "inches"
   * RGB:             Offset 156, Hard-coded to "inches"
   */

  ret = tiff_putifdentry16(info, IFD_TAG_RESUNIT, IFD_FIELD_SHORT, 1, TAG_RESUNIT_INCH);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write Software:
   *
   * Bi-level Images: Offset 144 Count, Hard-coded "NuttX"
   * Greyscale:       Offset 156 Count, Hard-coded "NuttX"
   * RGB:             Offset 168 Count, Hard-coded "NuttX"
   */

  ret = tiff_putifdentry(info, IFD_TAG_SOFTWARE, IFD_FIELD_ASCII, TIFF_SOFTWARE_STRLEN, info->filefmt->swoffset);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write DateTime:
   *
   * Bi-level Images: Offset 156 Count, Format "YYYY:MM:DD HH:MM:SS"
   * Greyscale:       Offset 168 Count, Format "YYYY:MM:DD HH:MM:SS"
   * RGB:             Offset 180 Count, Format "YYYY:MM:DD HH:MM:SS"
   */

  ret = tiff_putifdentry(info, IFD_TAG_DATETIME, IFD_FIELD_ASCII, TIFF_DATETIME_STRLEN, info->filefmt->dateoffset);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, SIZEOF_IFD_ENTRY);

  /* Write Next IFD Offset and 2 bytes of padding:
   *
   * Bi-level Images: Offset 168, Next IFD offset
   *                  Offset 170, [2 bytes padding]
   * Greyscale:       Offset 180, Next IFD offset
   *                  Offset 182, [2 bytes padding]
   * RGB:             Offset 192, Next IFD offset
   *                  Offset 194, [2 bytes padding]
   */

  ret = tiff_putint32(info->outfd, 0);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 4);

  /* Now we begin the value section of the file */
  
  tiff_checkoffs(offset, info->filefmt->valoffset);

  /* Write the XResolution and YResolution data:
   * 
   * Bi-level Images: Offset 172 Count, Hard-coded to 300/1
   *                  Offset 180 Count, Hard-coded to 300/1
   * Greyscale:       Offset 184 Count, Hard-coded to 300/1
   *                  Offset 192 Count, Hard-coded to 300/1
   * RGB:             Offset 196 Count, Hard-coded to 300/1
   *                  Offset 204 Count, Hard-coded to 300/1
   */

  tiff_checkoffs(offset, info->filefmt->xresoffset);
  ret = tiff_putint32(info->outfd, 300);
  if (ret == OK)
    {
      ret = tiff_putint32(info->outfd, 1);
    }

  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 8);

  tiff_checkoffs(offset, info->filefmt->yresoffset);
  ret = tiff_putint32(info->outfd, 300);
  if (ret == OK)
    {
      ret = tiff_putint32(info->outfd, 1);
    }

  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 8);

  /* Write RGB BitsPerSample Data:
   *
   * Bi-level Images: N/A
   * Greyscale:       N/A
   * RGB:             Offset 212 BitsPerSample (8,8,8)
   *                  Offset 218  [2 bytes padding]
   */

  if (IMGFLAGS_ISRGB(info->imgflags))
    {
      tiff_checkoffs(offset, TIFF_RGB_BPSOFFSET);
      tiff_putint16(info->outfd, 8);
      tiff_putint16(info->outfd, 8);
      tiff_putint16(info->outfd, 8);
      tiff_putint16(info->outfd, 0);
      tiff_offset(offset, 8);
    }

  /* Write the Software string:
   *
   *
   * Bi-level Images: Offset 188, Hard-coded "NuttX"
   * Greyscale:       Offset 200, Hard-coded "NuttX"
   * RGB:             Offset 220, Hard-coded "NuttX"
   */
   
  tiff_checkoffs(offset, info->filefmt->swoffset);
  ret = tiff_putstring(info->outfd, TIFF_SOFTWARE_STRING, TIFF_SOFTWARE_STRLEN);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, TIFF_SOFTWARE_STRLEN);

  /* Write the DateTime string:
   *
   *
   * Bi-level Images: Offset 188, Format "YYYY:MM:DD HH:MM:SSS"
   * Greyscale:       Offset 200, Hard-coded "NuttX"
   * RGB:             Offset 220, Hard-coded "NuttX"
   */

  tiff_checkoffs(offset, info->filefmt->dateoffset);
  ret = tiff_datetime(timbuf, TIFF_DATETIME_STRLEN + 8);
  if (ret < 0)
    {
      goto errout;
    }

  ret = tiff_putstring(info->outfd, timbuf, TIFF_DATETIME_STRLEN);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, TIFF_DATETIME_STRLEN);

  /* Add two bytes of padding */

  ret = tiff_putint16(info->outfd, 0);
  if (ret < 0)
    {
      goto errout;
    }
  tiff_offset(offset, 2);

  /* And that should do it! */

  tiff_checkoffs(offset, info->filefmt->sbcoffset);
  info->outsize = info->filefmt->sbcoffset;
  return OK;

errout:
  tiff_abort(info);
  return ret;
}

