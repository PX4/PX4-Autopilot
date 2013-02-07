/****************************************************************************
 * fs/fat/fs_configfat.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/mkfatfs.h>

#include "fs_internal.h"
#include "fs_fat32.h"
#include "fs_mkfatfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NDX12 0
#define NDX16 1
#define NDX32 2

#define fatconfig12 fatconfig[NDX12]
#define fatconfig16 fatconfig[NDX16]
#define fatconfig32 fatconfig[NDX32]

/* JMP rel8 and NOP opcodes */

#define OPCODE_JMP_REL8    0xeb
#define OPCODE_NOP         0x90

#define BOOTCODE_MSGOFFSET 29

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fat_config_s
{
  uint32_t fc_navailsects;  /* The number of available sectors */
  uint32_t fc_nfatsects;    /* The number of sectors in one FAT */
  uint32_t fc_nclusters;    /* The number of clusters in the filesystem */
  uint32_t fc_rsvdseccount; /* The number of reserved sectors */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Reverse engineered, generic boot message logic for non-bootable disk.
 * Message begins at offset 29; Sector relative offset must be poked into
 * offset 3.
 */
 
static uint8_t g_bootcodeblob[] =
{
  0x0e, 0x1f, 0xbe, 0x00, 0x7c, 0xac, 0x22, 0xc0, 0x74, 0x0b, 0x56,
  0xb4, 0x0e, 0xbb, 0x07, 0x00, 0xcd, 0x10, 0x5e, 0xeb, 0xf0, 0x32,
  0xe4, 0xcd, 0x16, 0xcd, 0x19, 0xeb, 0xfe, 0x54, 0x68, 0x69, 0x73,
  0x20, 0x69, 0x73, 0x20, 0x6e, 0x6f, 0x74, 0x20, 0x61, 0x20, 0x62,
  0x6f, 0x6f, 0x74, 0x61, 0x62, 0x6c, 0x65, 0x20, 0x64, 0x69, 0x73,
  0x6b, 0x2e, 0x20, 0x20, 0x50, 0x6c, 0x65, 0x61, 0x73, 0x65, 0x20,
  0x69, 0x6e, 0x73, 0x65, 0x72, 0x74, 0x20, 0x61, 0x20, 0x62, 0x6f,
  0x6f, 0x74, 0x61, 0x62, 0x6c, 0x65, 0x20, 0x66, 0x6c, 0x6f, 0x70,
  0x70, 0x79, 0x20, 0x61, 0x6e, 0x64, 0x0d, 0x0a, 0x70, 0x72, 0x65,
  0x73, 0x73, 0x20, 0x61, 0x6e, 0x79, 0x20, 0x6b, 0x65, 0x79, 0x20,
  0x74, 0x6f, 0x20, 0x74, 0x72, 0x79, 0x20, 0x61, 0x67, 0x61, 0x69,
  0x6e, 0x20, 0x2e, 0x2e, 0x2e, 0x0d, 0x0a, 0x00
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mkfatfs_nfatsect12
 *
 * Description:
 *   Calculate the number of sectors need for one fat in a FAT12 file system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   navailsects - The number of sectors available for both FAT and data.
 *     This is a precalculated value equal to the total number of sectors
 *     minus the number of root directory sectors and minus the number of
 *     reserved sectors.
 *
 * Return:
 *   0: That calculation would have overflowed
 *  >0: The size of one FAT in sectors.
 *
 ****************************************************************************/
static inline uint32_t
mkfatfs_nfatsect12(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var,
                   uint32_t navailsects)
{
#ifdef CONFIG_HAVE_LONG_LONG
   uint64_t denom;
   uint64_t numer;
#else
   uint32_t denom;
   uint32_t numer;
#endif

  /* For FAT12, the cluster number is held in a 12-bit number or 1.5 bytes per
   * cluster reference.  So each FAT sector will hold sectorsize/1.5 cluster
   * references (except for the first sector of each FAT which has two reserved
   * 12-bit values).  And the total number of FAT sectors needed is:
   *
   *   nfatsects = (1.5 * (ndataclust + 2) / sectorsize)
   *
   * where:
   *
   *   ndataclust = ndatasect / clustsize
   *   nvailsects = nfatsects + ndatasect
   *
   * The solution to this set of linear equations is:
   *
   *   nfatsects  = (3 * navailsects + 6 * clustersize) / 
   *                (3 * nfats + 2 * sectorsize * clustersize)
   *
   * The numerator would overflow uint32_t if:
   *
   *   3 * navailsects + 6 * clustersize > 0xffffffff
   *
   * Or 
   *
   *   navailsects > 0x55555555 - 2 * clustersize
   */

#ifndef CONFIG_HAVE_LONG_LONG
  if (navailsects <= (0x55555555 - (1 << (fmt->ff_clustshift + 1))))
    {
#endif

      denom = (fmt->ff_nfats << 1) + fmt->ff_nfats
            + (var->fv_sectorsize << (fmt->ff_clustshift + 1));
      numer = (navailsects << 1) + navailsects
            + (1 << (fmt->ff_clustshift + 2)) + (1 << (fmt->ff_clustshift + 1));
      return (uint32_t)((numer + denom - 1) / denom);

#ifndef CONFIG_HAVE_LONG_LONG
    }
  else
    {
      return 0;
    }
#endif
}

/****************************************************************************
 * Name:  mkfatfs_nfatsect16
 *
 * Description:
 *   Calculate the number of sectors need for one fat in a FAT16 file system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   navailsects - The number of sectors available for both FAT and data.
 *     This is a precalculated value equal to the total number of sectors
 *     minus the number of root directory sectors and minus the number of
 *     reserved sectors.
 *
 * Return:
 *    The size of one FAT in sectors.
 *
 ****************************************************************************/
static inline uint32_t
mkfatfs_nfatsect16(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var,
                   uint32_t navailsects)
{
#ifdef CONFIG_HAVE_LONG_LONG
   uint64_t denom;
   uint64_t numer;
#else
   uint32_t denom;
   uint32_t numer;
#endif

  /* For FAT16, the cluster number is held in a 16-bit number or 2 bytes per
   * cluster reference.  So each FAT sector will hold sectorsize/2 cluster
   * references (except for the first sector of each FAT which has two reserved
   * 16-bit values).  And the total number of FAT sectors needed is:
   *
   *   nfatsects = (2 * (ndataclust + 2) / sectorsize)
   *
   * where:
   *
   *   ndataclust = ndatasect / clustsize
   *   nvailsects = nfatsects + ndatasect
   *
   * The solution to this set of linear equations is:
   *
   *   nfatsects  = (navailsects + 2 * clustersize) / 
   *                (nfats + sectorsize * clustersize / 2)
   *
   * Overflow in the calculation of the numerator could occur if:
   *
   *   navailsects > 0xffffffff - 2 * clustersize
   */

  if (fmt->ff_clustshift == 0)
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize >> 1);
      numer = navailsects + 2;
    }
  else
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize << (fmt->ff_clustshift - 1));
      numer = navailsects + (1 << (fmt->ff_clustshift + 1));
    }
   return (uint32_t)((numer + denom - 1) / denom);
}

/****************************************************************************
 * Name:  mkfatfs_nfatsect32
 *
 * Description:
 *   Calculate the number of sectors need for one fat in a FAT32 file system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   navailsects - The number of sectors available for both FAT and data.
 *     This is a precalculated value equal to the total number of sectors
 *     minus the number of root directory sectors and minus the number of
 *     reserved sectors.
 *
 * Return:
 *   The size of one FAT in sectors.
 *
 ****************************************************************************/
static inline uint32_t
mkfatfs_nfatsect32(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var,
                   uint32_t navailsects)
{
#ifdef CONFIG_HAVE_LONG_LONG
   uint64_t denom;
   uint64_t numer;
#else
   uint32_t denom;
   uint32_t numer;
#endif

  /* For FAT32, the cluster number is held in a 32-bit number or 4 bytes per
   * cluster reference.  So each FAT sector will hold sectorsize/4 cluster
   * references (except for the first sector of each FAT which has three reserved
   * 32-bit values).  And the total number of FAT sectors needed is:
   *
   *   nfatsects = (4 * (ndataclust + 3) / sectorsize)
   *
   * where:
   *
   *   ndataclust = ndatasect / clustsize
   *   nvailsects = nfatsects + ndatasect
   *
   * The solution to this set of linear equations is:
   *
   *   nfatsects  = (navailsects + 3 * clustersize) / 
   *                (nfats + sectorsize * clustersize / 4)
   *
   * Overflow in the 32-bit calculation of the numerator could occur if:
   *
   *   navailsects > 0xffffffff - 3 * clustersize
   */

  if (fmt->ff_clustshift == 0)
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize >> 2);
      numer = navailsects + 3;
    }
  else if (fmt->ff_clustshift == 1)
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize >> 1);
      numer = navailsects + 6;
    }
  else
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize << (fmt->ff_clustshift - 2));
      numer = navailsects + (1 << (fmt->ff_clustshift + 1)) + (1 << fmt->ff_clustshift);
    }
   return (uint32_t)((numer + denom - 1) / denom);
}

/****************************************************************************
 * Name:  mkfatfs_clustersearchlimits
 *
 * Description:
 *   Pick the starting and ending cluster size to use in the search for the
 *   the optimal cluster size.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *
 * Return:
 *   Starting cluster size is set in fmt->ff_clustshift; Final cluster
 *   size is the returned value.
 *
 ****************************************************************************/
static inline uint8_t
mkfatfs_clustersearchlimits(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var)
{
  uint8_t mxclustshift;
  
  /* Did the caller already pick the cluster size?  If not, the clustshift value
   * will be 0xff
   */

  if (fmt->ff_clustshift == 0xff)
    {
      /* Pick a starting size based on the number of sectors on the device */

      if (fmt->ff_nsectors < 2048)
        {
          /* 2k sectors, start wit 1 sector/cluster. */
          fmt->ff_clustshift = 0;
        }
      else if (fmt->ff_nsectors  < 4096)
        {
          /* 4k sectors, start with 2 sector/cluster. */
          fmt->ff_clustshift = 1;
        }
      else if (fmt->ff_nsectors  < 8192)
        {
          /* 8k sectors, start with 4 sector/cluster. */
          fmt->ff_clustshift = 2;
        }
      else if (fmt->ff_nsectors  < 16384)
        {
          /* 16k sectors, start with 8 sector/cluster. */
          fmt->ff_clustshift = 3;
        }
      else if (fmt->ff_nsectors  < 32768)
        {
          /* 32k sectors, start with 16 sector/cluster. */
          fmt->ff_clustshift = 4;
        }   
      else
        {
          /* Otherwise, 32 sector/cluster. */
          fmt->ff_clustshift = 5;
        }

      /* Wherever the search starts, it will end with the maximum of
       * 128 sectors per cluster
       */

      mxclustshift = 7;
    }
  else
    {
      /* The caller has selected a cluster size.  There will be no search!
       * Just set the maximum to the caller specificed value.
       */
 
      mxclustshift = fmt->ff_clustshift;
    }
  return mxclustshift;
}

/****************************************************************************
 * Name:  mkfatfs_tryfat12
 *
 * Description:
 *   Try to define a FAT12 filesystem on the device using the candidate
 *   sectors per cluster
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   fatconfig - FAT12-specific configuration
 *
 * Return:
 *    Zero on success configuration of a FAT12 file system; negated errno
 *    on failure
 *
 ****************************************************************************/
static inline int
mkfatfs_tryfat12(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var,
                 FAR struct fat_config_s *config)
{
  uint32_t maxnclusters;

  /* Calculate the number sectors in one FAT required to access all of the
   * available sectors.
   */

  config->fc_nfatsects = mkfatfs_nfatsect12(fmt, var, config->fc_navailsects);
  if (config->fc_nfatsects > 0)
    {
      /* Calculate the number of clusters available given the number of available
       * sectors and the number of those that will be used for FAT:
       */

      config->fc_nclusters =
        (config->fc_navailsects -
          fmt->ff_nfats * config->fc_nfatsects) >> fmt->ff_clustshift;

      /* Calculate the maximum number of clusters that could be supported by a
       * FAT of this size.
       *
       *   maxnclusters = nfatsects * sectorsize / 1.5 - 2
       */

      maxnclusters = (config->fc_nfatsects >> (var->fv_sectshift + 1)) / 3;
      if (maxnclusters > FAT_MAXCLUST12)
        {
          maxnclusters = FAT_MAXCLUST12;
        }
      fvdbg("nfatsects=%u nclusters=%u (max=%u)\n",
            config->fc_nfatsects, config->fc_nclusters, maxnclusters);

      /* Check if this number of clusters would overflow the maximum for
       * FAT12 (remembering that two FAT cluster slots are reserved).
       */

      if (config->fc_nclusters + 2 > maxnclusters)
        {
          fvdbg("Too many clusters for FAT12\n");
          return -ENFILE;
        }
    }
  return 0;
}
 
/****************************************************************************
 * Name:  mkfatfs_tryfat16
 *
 * Description:
 *   Try to define a FAT16 filesystem on the device using the candidate
 *   sectors per cluster
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   fatconfig - FAT16-specific configuration
 *
 * Return:
 *    Zero on success configuration of a FAT16 file system; negated errno
 *    on failure
 *
 ****************************************************************************/
static inline int
mkfatfs_tryfat16(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var,
                 FAR struct fat_config_s *config)
{
  uint32_t maxnclusters;

  /* Calculate the number sectors in one FAT required to access all of the
   * available sectors.
   */

  config->fc_nfatsects = mkfatfs_nfatsect16(fmt, var, config->fc_navailsects);
  if (config->fc_nfatsects > 0)
    {
      /* Calculate the number of clusters available given the number of available
       * sectors and the number of those that will be used for FAT:
       */

      config->fc_nclusters =
        (config->fc_navailsects -
          fmt->ff_nfats * config->fc_nfatsects) >> fmt->ff_clustshift;

      /* Calculate the maximum number of clusters that could be supported by a
       * FAT of this size.
       *
       *   maxnclusters = nfatsects * sectorsize / 2 - 2
       */

      maxnclusters = config->fc_nfatsects << (var->fv_sectorsize - 1);
      if (maxnclusters > FAT_MAXCLUST16)
        {
          maxnclusters = FAT_MAXCLUST16;
        }
      fvdbg("nfatsects=%u nclusters=%u (min=%u max=%u)\n",
            config->fc_nfatsects, config->fc_nclusters, FAT_MINCLUST16, maxnclusters);

      /* Check if this number of clusters would overflow the maximum for
       * FAT16 (remembering that two FAT cluster slots are reserved).
       * Check the lower limit as well.  The FAT12 is distinguished from FAT16
       * by comparing the number of clusters on the device agains a known
       * threshold.  If a small FAT16 file system were created, then it would
       * be confused as a FAT12 at mount time.
       */

      if ((config->fc_nclusters + 2 > maxnclusters) ||
          (config->fc_nclusters < FAT_MINCLUST16))
        {
          fvdbg("Too few or too many clusters for FAT16\n");
          return -ENFILE;
        }
    }
  return 0;
}

/****************************************************************************
 * Name:  mkfatfs_tryfat32
 *
 * Description:
 *   Try to define a FAT32 filesystem on the device using the candidate
 *   sectors per cluster
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   fatconfig - FAT32-specific configuration
 *
 * Return:
 *    Zero on success configuration of a FAT32 file system; negated errno
 *    on failure
 *
 ****************************************************************************/
static inline int
mkfatfs_tryfat32(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var,
                 FAR struct fat_config_s *config)
{
  uint32_t maxnclusters;

  /* Calculate the number sectors in one FAT required to access all of the
   * available sectors.
   */

  config->fc_nfatsects = mkfatfs_nfatsect32(fmt, var, config->fc_navailsects);
  if (config->fc_nfatsects > 0)
    {
      /* Calculate the number of clusters available given the number of available
       * sectors and the number of those that will be used for FAT:
       */

      config->fc_nclusters =
        (config->fc_navailsects -
          fmt->ff_nfats * config->fc_nfatsects) >> fmt->ff_clustshift;

      /* Calculate the maximum number of clusters that could be supported by a
       * FAT of this size.
       *
       *   maxnclusters = nfatsects * sectorsize / 4 - 2
       */

      maxnclusters = (config->fc_nfatsects << (var->fv_sectshift - 2));
      if (maxnclusters > FAT_MAXCLUST32)
        {
          maxnclusters = FAT_MAXCLUST32;
        }
      fvdbg("nfatsects=%u nclusters=%u (max=%u)\n",
            config->fc_nfatsects, config->fc_nclusters, maxnclusters);

      /* Check if this number of clusters would overflow the maximum for
       * FAT32 (remembering that two FAT cluster slots are reserved).
       */

      if ((config->fc_nclusters + 3 > maxnclusters) ||
          (config->fc_nclusters < FAT_MINCLUST32 && fmt->ff_fattype != 32))
        {
          fvdbg("Too few or too many clusters for FAT32\n");
          return -ENFILE;
        }
    }
  return 0;
}

/****************************************************************************
 * Name:  mkfatfs_selectfat
 *
 * Description:
 *   The cluster search has succeeded, select the specified FAT FS
 *
 * Input:
 *   fattype - The FAT size selected
 *   fmt     - Caller specified format parameters
 *   var     - Format parameters that are not caller specifiable.
 *
 * Return:
 *    None
 *
 ****************************************************************************/

static inline void
mkfatfs_selectfat(int fattype, FAR struct fat_format_s *fmt,
                  FAR struct fat_var_s *var, FAR struct fat_config_s *config)
{
  /* Return the appropriate information about the selected file system. */

  fdbg("Selected FAT%d\n", fattype);
  var->fv_fattype      = fattype;
  var->fv_nclusters    = config->fc_nclusters;
  var->fv_nfatsects    = config->fc_nfatsects;
  fmt->ff_rsvdseccount = config->fc_rsvdseccount;
}

/****************************************************************************
 * Name:  mkfatfs_clustersearch
 *
 * Description:
 *   Search to find the smallest (reasonable) cluster size for the FAT file
 *   system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/

static inline int
mkfatfs_clustersearch(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var)
{
  struct fat_config_s fatconfig[3];
  uint8_t  mxclustshift;

  memset(fatconfig, 0, 3*sizeof(struct fat_config_s));

  /* Select the reserved sector count for each FAT size */

  if (fmt->ff_rsvdseccount)
    {
      fatconfig12.fc_rsvdseccount = fmt->ff_rsvdseccount;
      fatconfig16.fc_rsvdseccount = fmt->ff_rsvdseccount;

      if (fmt->ff_rsvdseccount < 2)
        {
          fvdbg("At least 2 reserved sectors needed by FAT32\n");
          fatconfig32.fc_rsvdseccount = 2;
        }
      else
        {
          fatconfig32.fc_rsvdseccount = fmt->ff_rsvdseccount;
        }
    }
  else
    {
      fatconfig12.fc_rsvdseccount = 1;
      fatconfig16.fc_rsvdseccount = 1;
      fatconfig32.fc_rsvdseccount = 32;
    }

  /* Determine the number of sectors needed by the root directory.
   * This is a constant value, independent of cluster size for FAT12/16
   */

  if (var->fv_fattype != 32)
    {
      /* Calculate the number of sectors reqired to contain the selected
       * number of root directory entries.  This value is save in the var
       * structure but will be overwritten if FAT32 is selected.  FAT32 uses
       * a cluster chain for the root directory, so the concept of the number
       * of root directory entries does not apply to FAT32
       */

      var->fv_nrootdirsects =
        ((fmt->ff_rootdirentries << DIR_SHIFT) + var->fv_sectorsize - 1) >> var->fv_sectshift;
 
      /* The number of data sectors available (includes the fat itself)
       * This value is a constant for FAT12/16, but not FAT32 because the
       * size of the root directory cluster changes
       */

      fatconfig12.fc_navailsects =
        fatconfig16.fc_navailsects =
          fmt->ff_nsectors - var->fv_nrootdirsects - fatconfig12.fc_rsvdseccount;
   }

  /* Select an initial and terminal clustersize to use in the search (if these
   * values were not provided by the caller)
   */

  mxclustshift = mkfatfs_clustersearchlimits(fmt, var);

  do
    {
      fvdbg("Configuring with %d sectors/cluster...\n", 1 << fmt->ff_clustshift);
 
      /* Check if FAT12 has not been excluded */

      if (var->fv_fattype == 0 || var->fv_fattype == 12)
        {
          /* Try to configure a FAT12 filesystem with this cluster size */

          if (mkfatfs_tryfat12(fmt, var, &fatconfig12) != 0)
            {
                {
                  fvdbg("Cannot format FAT12 at %u sectors/cluster\n", 1 << fmt->ff_clustshift);
                  fatconfig12.fc_nfatsects = 0;
                  fatconfig12.fc_nclusters = 0;
                }
            }
        }

      /* Check if FAT16 has not been excluded */

      if (var->fv_fattype == 0 || var->fv_fattype == 16)
        {
          /* Try to configure a FAT16 filesystem with this cluster size */

          if (mkfatfs_tryfat16(fmt, var, &fatconfig16) != 0)
            {
                {
                  fvdbg("Cannot format FAT16 at %u sectors/cluster\n", 1 << fmt->ff_clustshift);
                  fatconfig16.fc_nfatsects = 0;
                  fatconfig16.fc_nclusters = 0;
                }
            }
        }

      /* If either FAT12 or 16 was configured at this sector/cluster setting,
       * then finish the configuration and break out now
       */

      if (fatconfig12.fc_nclusters || fatconfig16.fc_nclusters)
        {
          if ((!var->fv_fattype && fatconfig16.fc_nclusters > fatconfig12.fc_nclusters) ||
              (var ->fv_fattype == 16))
            {
              /* The caller has selected FAT16 -OR- no FAT type has been selected, but
               * the FAT16 selection has more clusters. Select FAT16.
               */

              mkfatfs_selectfat(16, fmt, var, &fatconfig16);
            }
          else
            {
              /* The caller has selected FAT12 -OR- no FAT type has been selected, but
               * the FAT12 selected has more clusters.  Selected FAT12
               */

              mkfatfs_selectfat(12, fmt, var, &fatconfig12);
            }
         return OK;
        }

      /* Check if FAT32 has not been excluded */

      if (var->fv_fattype == 0 || var->fv_fattype == 32)
        {
          /* The number of data sectors available (includes the fat itself)
           * This value is a constant with respect to cluster sizefor FAT12/16, but not FAT32
           * because the size of the root directory cluster changes with cluster size.
           */

          fatconfig32.fc_navailsects = fmt->ff_nsectors - (1 << fmt->ff_clustshift) - fatconfig32.fc_rsvdseccount;

          /* Try to configure a FAT32 filesystem with this cluster size */

          if (mkfatfs_tryfat32(fmt, var, &fatconfig32) != 0)
            {
                {
                  fvdbg("Cannot format FAT32 at %u sectors/cluster\n", 1 << fmt->ff_clustshift);
                  fatconfig32.fc_nfatsects = 0;
                  fatconfig32.fc_nclusters = 0;
                }
            }
          else
            {
              /* Select FAT32 if we have not already done so */

              mkfatfs_selectfat(32, fmt, var, &fatconfig32);
              return OK;
            }
        }

      /* Otherwise, bump up the sectors/cluster for the next time around the loop. */

      fmt->ff_clustshift++;
    }
  while (fmt->ff_clustshift <= mxclustshift);
  return -ENFILE;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs_configfatfs
 *
 * Description:
 *   Based on the geometry of the block device and upon the caller-selected
 *   values, configure the FAT filesystem for the device.
 *
 * Input:
 *    fmt  - Caller specified format parameters
 *    var  - Holds disk geomtry data.  Also, the location to return FAT
 *           configuration data
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/
int mkfatfs_configfatfs(FAR struct fat_format_s *fmt,
                        FAR struct fat_var_s *var)
{
  int ret;

  /* Select the number of root directory entries (FAT12/16 only).  If FAT32 is selected,
   * this value will be cleared later
   */

  if (!fmt->ff_rootdirentries)
    {
      /* The caller did not specify the number of root directory entries; use a default of 512. */

      fmt->ff_rootdirentries = 512;
    }

  /* Search to determine the smallest (reasonable) cluster size.  A by-product
   * of this search will be the selection of the FAT size (12/16/32) if the
   * caller has not specified the FAT size
   */

  ret = mkfatfs_clustersearch(fmt, var);
  if (ret < 0)
    {
       fdbg("Failed to set cluster size\n");
       return ret;
    }

  /* Perform FAT specific initialization */

  /* Set up boot jump assuming FAT 12/16 offset to bootcode */

  var->fv_jump[0]      = OPCODE_JMP_REL8;
  var->fv_jump[2]      = OPCODE_NOP;
  var->fv_bootcode     = g_bootcodeblob;
  var->fv_bootcodesize = sizeof(g_bootcodeblob);

  if (var->fv_fattype != 32)
    {
      /* Set up additional, non-zero FAT12/16 fields */

      /* Patch in the correct offset to the boot code */
 
      var->fv_jump[1]   = BS16_BOOTCODE - 2;
      g_bootcodeblob[3] = BS16_BOOTCODE + BOOTCODE_MSGOFFSET;
    }
  else
    {
      /* Patch in the correct offset to the boot code */

      var->fv_jump[1]   = BS32_BOOTCODE - 2;
      g_bootcodeblob[3] = BS32_BOOTCODE + BOOTCODE_MSGOFFSET;

      /* The root directory is a cluster chain... its is initialize size is one cluster */
  
      var->fv_nrootdirsects = 1 << fmt->ff_clustshift;

      /* The number of reported root directory entries should should be zero for
       * FAT32 because the root directory is a cluster chain.
       */

      fmt->ff_rootdirentries = 0;

      /* Verify the caller's backupboot selection */

      if (fmt->ff_backupboot <= 1 || fmt->ff_backupboot >= fmt->ff_rsvdseccount)
        {
          fdbg("Invalid backup boot sector: %d\n", fmt->ff_backupboot);
          fmt->ff_backupboot = 0;
        }

     /* Check if the caller has selected a location for the backup boot record */

      if (!fmt->ff_backupboot)
        {
          /* There must be reserved sectors in order to have a backup boot sector */

          if (fmt->ff_rsvdseccount > 0 && fmt->ff_rsvdseccount >= 2)
            {
              /* Sector 0 is the MBR; 1... ff_rsvdseccount are reserved.  Try the next
               * the last reserved sector.
               */

              fmt->ff_backupboot = fmt->ff_rsvdseccount - 1;
              if (fmt->ff_backupboot > 6)
                {
                  /* Limit the location to within the first 7 */

                  fmt->ff_backupboot = 6;
                }
            }
        }
    }

  /* Report the selected fat type */

  fmt->ff_fattype = var->fv_fattype;

  /* Describe the configured filesystem */

#ifdef CONFIG_DEBUG
  fdbg("Sector size:          %d bytes\n", var->fv_sectorsize);
  fdbg("Number of sectors:    %d sectors\n", fmt->ff_nsectors);
  fdbg("FAT size:             %d bits\n", var->fv_fattype);
  fdbg("Number FATs:          %d\n", fmt->ff_nfats);
  fdbg("Sectors per cluster:  %d sectors\n", 1 << fmt->ff_clustshift);
  fdbg("FS size:              %d sectors\n", var->fv_nfatsects);
  fdbg("                      %d clusters\n", var->fv_nclusters);
  if (var->fv_fattype != 32)
    {
       fdbg("Root directory slots: %d\n", fmt->ff_rootdirentries);
    }
  fdbg("Volume ID:            %08x\n", fmt->ff_volumeid);
  fdbg("Volume Label:         \"%c%c%c%c%c%c%c%c%c%c%c\"\n",
    fmt->ff_volumelabel[0], fmt->ff_volumelabel[1], fmt->ff_volumelabel[2], 
    fmt->ff_volumelabel[3], fmt->ff_volumelabel[4], fmt->ff_volumelabel[5], 
    fmt->ff_volumelabel[6], fmt->ff_volumelabel[7], fmt->ff_volumelabel[8], 
    fmt->ff_volumelabel[9], fmt->ff_volumelabel[10]);
#endif
  return OK;
}

