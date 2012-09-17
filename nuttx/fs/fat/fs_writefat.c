/****************************************************************************
 * fs/fat/fs_writefat.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/mkfatfs.h>

#include "fs_internal.h"
#include "fs_fat32.h"
#include "fs_mkfatfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs_initmbr
 *
 * Description:
 *   Initialize the sector image of a masterbood record
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *
 * Return:
 *    None; caller is responsible for providing valid parameters.
 *
 ****************************************************************************/
static inline void mkfatfs_initmbr(FAR struct fat_format_s *fmt,
                                   FAR struct fat_var_s *var)
{
  memset(var->fv_sect, 0, var->fv_sectorsize);

  /* 3@0: Jump instruction to boot code */

  memcpy(&var->fv_sect[BS_JUMP], var->fv_jump, 3);

  /* 8@3: Usually "MSWIN4.1" */

  strcpy((char*)&var->fv_sect[BS_OEMNAME], "NUTTX   ");

  /* 2@11: Bytes per sector: 512, 1024, 2048, 4096  */

  MBR_PUTBYTESPERSEC(var->fv_sect, var->fv_sectorsize);

  /* 1@13: Sectors per allocation unit: 2**n, n=0..7 */

  MBR_PUTSECPERCLUS(var->fv_sect, (1 << fmt->ff_clustshift));

  /* 2@14: Reserved sector count: Usually 32 */

  MBR_PUTRESVDSECCOUNT(var->fv_sect, fmt->ff_rsvdseccount);

  /* 1@16: Number of FAT data structures: always 2 */

  MBR_PUTNUMFATS(var->fv_sect, fmt->ff_nfats);

  /* 2@17: FAT12/16: Must be 0 for FAT32 */

  MBR_PUTROOTENTCNT(var->fv_sect, fmt->ff_rootdirentries);

  /* 2@19: FAT12/16: Must be 0, see BS_TOTSEC32.
   * Handled with 4@32: Total count of sectors on the volume */

  if (fmt->ff_nsectors >= 65536)
    {
      MBR_PUTTOTSEC32(var->fv_sect, fmt->ff_nsectors);
    }
  else
    {
      MBR_PUTTOTSEC16(var->fv_sect, (uint16_t)fmt->ff_nsectors);
    }

  /* 1@21: Media code: f0, f8, f9-fa, fc-ff */ 

  MBR_PUTMEDIA(var->fv_sect, FAT_DEFAULT_MEDIA_TYPE); /* Only "hard drive" supported */
  
  /* 2@22: FAT12/16: Must be 0, see BS32_FATSZ32  -- handled in FAT specific logic */
 
  /* 2@24: Sectors per track geometry value and 2@26: Number of heads geometry value */

  MBR_PUTSECPERTRK(var->fv_sect, FAT_DEFAULT_SECPERTRK);
  MBR_PUTNUMHEADS(var->fv_sect, FAT_DEFAULT_NUMHEADS);

  /* 4@28: Count of hidden sectors preceding FAT */

  MBR_PUTHIDSEC(var->fv_sect, fmt->ff_hidsec);

  /* 4@32: Total count of sectors on the volume -- handled above */

  /* Most of the rest of the sector depends on the FAT size */

  if (fmt->ff_fattype != 32)
    {
      /* 2@22: FAT12/16: Must be 0, see BS32_FATSZ32 */

      MBR_PUTFATSZ16(var->fv_sect, (uint16_t)var->fv_nfatsects);

      /* The following fields are only valid for FAT12/16 */
      /*  1@36: Drive number for MSDOS bootstrap -- left zero */
      /*  1@37: Reserved (zero) */
      /*  1@38: Extended boot signature: 0x29 if following valid */

      MBR_PUTBOOTSIG16(var->fv_sect, EXTBOOT_SIGNATURE);

      /* 4@39: Volume serial number */

      MBR_PUTVOLID16(var->fv_sect, fmt->ff_volumeid);

      /* 11@43: Volume label */

      memcpy(&var->fv_sect[BS16_VOLLAB], fmt->ff_volumelabel, 11);

      /* 8@54: "FAT12  ", "FAT16  ", or "FAT    " */

      if (fmt->ff_fattype == 12)
        {
          memcpy(&var->fv_sect[BS16_FILESYSTYPE], "FAT12   ", 8);
        }
      else /* if (fmt->ff_fattype == 16) */
        {
          memcpy(&var->fv_sect[BS16_FILESYSTYPE], "FAT16   ", 8);
        }

      /* Boot code may be placed in the remainder of the sector */

      memcpy(&var->fv_sect[BS16_BOOTCODE], var->fv_bootcode, var->fv_bootcodesize);
    }
  else
    {
      /* The following fields are only valid for FAT32 */
      /*  4@36: Count of sectors occupied by one FAT */

      MBR_PUTFATSZ32(var->fv_sect, var->fv_nfatsects);

      /* 2@40: 0-3:Active FAT, 7=0 both FATS, 7=1 one FAT -- left zero*/ 
      /* 2@42: MSB:Major LSB:Minor revision number (0.0) -- left zero */
      /* 4@44: Cluster no. of 1st cluster of root dir */

      MBR_PUTROOTCLUS(var->fv_sect, FAT32_DEFAULT_ROOT_CLUSTER);

      /* 2@48: Sector number of fsinfo structure. Usually 1. */

      MBR_PUTFSINFO(var->fv_sect, FAT_DEFAULT_FSINFO_SECTOR);

      /* 2@50: Sector number of boot record. Usually 6  */

      MBR_PUTBKBOOTSEC(var->fv_sect, fmt->ff_backupboot);

      /* 12@52: Reserved (zero) */
      /*  1@64: Drive number for MSDOS bootstrap -- left zero */
      /*  1@65: Reserved (zero) */
      /*  1@66: Extended boot signature: 0x29 if following valid */
 
      MBR_PUTBOOTSIG32(var->fv_sect, EXTBOOT_SIGNATURE);

      /* 4@67: Volume serial number */

      MBR_PUTVOLID32(var->fv_sect, fmt->ff_volumeid);

      /* 11@71: Volume label */

      memcpy(&var->fv_sect[BS32_VOLLAB], fmt->ff_volumelabel, 11);

      /* 8@82: "FAT12  ", "FAT16  ", or "FAT    " */

      memcpy(&var->fv_sect[BS32_FILESYSTYPE], "FAT32   ", 8);

      /* Boot code may be placed in the remainder of the sector */

      memcpy(&var->fv_sect[BS16_BOOTCODE], var->fv_bootcode, var->fv_bootcodesize);
    }

  /* The magic bytes at the end of the MBR are common to FAT12/16/32 */
  /*  2@510: Valid MBRs have 0x55aa here */

  MBR_PUTSIGNATURE(var->fv_sect, BOOT_SIGNATURE16);
}

/****************************************************************************
 * Name: mkfatfs_initfsinfo
 *
 * Description:
 *   Initialize the FAT32 FSINFO sector image
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *
 * Return:
 *    None; caller is responsible for providing valid parameters.
 *
 ****************************************************************************/
static inline void mkfatfs_initfsinfo(FAR struct fat_format_s *fmt,
                                      FAR struct fat_var_s *var)
{
  memset(var->fv_sect, 0, var->fv_sectorsize);

  /* 4@0: 0x41615252 = "RRaA" */

  FSI_PUTLEADSIG(var->fv_sect, 0x41615252);

  /* 480@4: Reserved (zero) */
  /* 4@484: 0x61417272 = "rrAa" */

  FSI_PUTSTRUCTSIG(var->fv_sect, 0x61417272);

  /* 4@488: Last free cluster count on volume */

  FSI_PUTFREECOUNT(var->fv_sect, var->fv_nclusters - 1);

  /* 4@492: Cluster number of 1st free cluster */

  FSI_PUTNXTFREE(var->fv_sect, FAT32_DEFAULT_ROOT_CLUSTER);

  /* 12@496: Reserved (zero) */
  /* 4@508:  0xaa550000 */

  FSI_PUTTRAILSIG(var->fv_sect, BOOT_SIGNATURE32);
}

/****************************************************************************
 * Name: mkfatfs_initrootdir
 *
 * Description:
 *   Initialize one root directory sector image
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *    sectno - On FAT32, the root directory is a cluster chain.
 *        This value indicates which sector of the cluster should be produced.
 *
 * Return:
 *    None; caller is responsible for providing valid parameters.
 *
 ****************************************************************************/
static inline void mkfatfs_initrootdir(FAR struct fat_format_s *fmt,
                                       FAR struct fat_var_s *var, int sectno)
{
  memset(var->fv_sect, 0, var->fv_sectorsize);
  if (sectno == 0)
    {
      /* It is only necessary to set data in the first sector of the directory */
      
      if (memcmp(fmt->ff_volumelabel, "           ", 11))
        {
          memcpy(&var->fv_sect[DIR_NAME], fmt->ff_volumelabel, 11);
        }

      DIR_PUTATTRIBUTES(var->fv_sect, FATATTR_VOLUMEID);
      DIR_PUTCRTIME(var->fv_sect, var->fv_createtime & 0xffff);
      DIR_PUTWRTTIME(var->fv_sect, var->fv_createtime & 0xffff);
      DIR_PUTCRDATE(var->fv_sect, var->fv_createtime >> 16);
      DIR_PUTWRTDATE(var->fv_sect, var->fv_createtime >> 16);
    }
}

/****************************************************************************
 * Name: mkfatfs_writembr
 *
 * Description:
 *   Write the master boot record and, for FAT32, the backup boot record and
 *   the fsinfo sector.
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/

static inline int mkfatfs_writembr(FAR struct fat_format_s *fmt,
                                   FAR struct fat_var_s *var)
{
  int sectno;
  int ret;

  /* Create an image of the configured master boot record */

  mkfatfs_initmbr(fmt, var);

  /* Write the master boot record as sector zero */

  ret = DEV_WRITE(var->fv_sect, 0, 1);

  /* Write all of the reserved sectors */

  memset(var->fv_sect, 0, var->fv_sectorsize);
 for (sectno = 1; sectno < fmt->ff_rsvdseccount && ret >= 0; sectno++)
    {
      ret = DEV_WRITE(var->fv_sect, sectno, 1);
    }
 
  /* Write FAT32-specific sectors */

  if (ret >= 0 && fmt->ff_fattype == 32)
    {
      /* Write the backup master boot record */

      if (fmt->ff_backupboot != 0)
        {
          /* Create another copy of the configured master boot record */

          mkfatfs_initmbr(fmt, var);

          /* Write it to the backup location */

          ret = DEV_WRITE(var->fv_sect, fmt->ff_backupboot, 1);
        }

      if (ret >= 0)
        {
          /* Create an image of the fsinfo sector*/

          mkfatfs_initfsinfo(fmt, var);

          /* Write the fsinfo sector */

          ret = DEV_WRITE(var->fv_sect, FAT_DEFAULT_FSINFO_SECTOR, 1);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mkfatfs_writefat
 *
 * Description:
 *   Write the FAT sectors
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/

static inline int mkfatfs_writefat(FAR struct fat_format_s *fmt,
                                   FAR struct fat_var_s *var)
{
  off_t offset = fmt->ff_rsvdseccount;
  int fatno;
  int sectno;
  int ret;

  /* Loop for each FAT copy */

  for (fatno = 0; fatno < fmt->ff_nfats; fatno++)
    {
      /* Loop for each sector in the FAT */

       for (sectno = 0; sectno < var->fv_nfatsects; sectno++)
         {
           memset(var->fv_sect, 0, var->fv_sectorsize);

          /* Mark cluster allocations in sector one of each FAT */

           if (sectno == 0)
             {
               memset(var->fv_sect, 0, var->fv_sectorsize);
               switch(fmt->ff_fattype)
                 {
                   case 12:
                     /* Mark the first two full FAT entries -- 24 bits, 3 bytes total */

                     memset(var->fv_sect, 0xff, 3);
                     break;

                   case 16:
                     /* Mark the first two full FAT entries -- 32 bits, 4 bytes total */

                     memset(var->fv_sect, 0xff, 4);
                     break;

                   case 32:
                   default: /* Shouldn't happen */
                     /* Mark the first two full FAT entries -- 64 bits, 8 bytes total */

                     memset(var->fv_sect, 0xff, 8);

                     /* Cluster 2 is used as the root directory.  Mark as EOF */

                     var->fv_sect[8] =  0xf8;
                     memset(&var->fv_sect[9], 0xff, 3);
                     break;
                 }

               /* Save the media type in the first byte of the FAT */

               var->fv_sect[0] = FAT_DEFAULT_MEDIA_TYPE;
            }

           /* Write the FAT sector */

           ret = DEV_WRITE(var->fv_sect, offset, 1);
           if (ret < 0)
             {
               return ret;
             }
           offset++;
         }
     }
   return OK;
}

/****************************************************************************
 * Name: mkfatfs_writerootdir
 *
 * Description:
 *   Write the root directory sectors
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/

static inline int mkfatfs_writerootdir(FAR struct fat_format_s *fmt,
                                       FAR struct fat_var_s *var)
{
  off_t offset = fmt->ff_rsvdseccount + fmt->ff_nfats * var->fv_nfatsects;
  int ret;
  int i;

  /* Write the root directory after the last FAT. This is the root directory
   * area for FAT12/16, and the first cluster on FAT32.
   */

  for (i = 0; i < var->fv_nrootdirsects; i++)
    {
      /* Format the next sector of the root directory */

      mkfatfs_initrootdir(fmt, var, i);

      /* Write the next sector of the root directory */

      ret = DEV_WRITE(var->fv_sect, offset, 1);
      if (ret < 0)
        {
          return ret;
        }
      offset++;
    }
  return 0;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs_writefat
 *
 * Description:
 *   Write the configured fat filesystem to the block device
 *
 * Input:
 *    fmt  - Caller specified format parameters
 *    var  - Other format parameters that are not caller specifiable. (Most
 *           set by mkfatfs_configfatfs()).
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/

int mkfatfs_writefatfs(FAR struct fat_format_s *fmt,
                       FAR struct fat_var_s *var)
{
  int ret;

  /* Write the master boot record (also the backup and fsinfo sectors) */

  ret = mkfatfs_writembr(fmt, var);

  /* Write FATs */

  if (ret >= 0)
    {
      ret = mkfatfs_writefat(fmt, var);
    }

  /* Write the root directory after the last FAT. */

  if (ret >= 0)
    {
      ret = mkfatfs_writerootdir(fmt, var);
    }
  return ret;
}

