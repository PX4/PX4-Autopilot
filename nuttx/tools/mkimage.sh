#!/bin/bash
#
# File: mkimage.sh
#
# Copyright (C) 2002 RidgeRun, Inc.
# Author: RidgeRun, Inc  <skranz@@ridgerun.com>
#  - Adapted for the Cadenux environment, 9-6-02, Gregory Nutt
#  - Added --EAddr option, 6-18-03, Gregory Nutt
#
# This program is free software; you can redistribute  it and/or modify it
# under  the terms of  the GNU General  Public License as published by the
# Free Software Foundation;  either version 2 of the  License, or (at your
# option) any later version.
#
# THIS  SOFTWARE  IS  PROVIDED  ``AS  IS''  AND   ANY  EXPRESS  OR IMPLIED
# WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
# NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT,  INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
# USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# You should have received a copy of the  GNU General Public License along
# with this program; if not, write  to the Free Software Foundation, Inc.,
# 675 Mass Ave, Cambridge, MA 02139, USA.
#
########################################################
# Description:
#   -----------
#   Scenario #1
#   -----------
#   This utility was developed by RidgeRun for the
#   purpose of converting a standard binary executable
#   image (such as ELF) into a special format (RR
#   format) suitable for quick downloads to the target
#   TI925 RidgeRun Bootloader (rrload). The image is
#   produced by constructing a special header which is
#   then tacked onto the front of the supplied binary
#   image.  The resulting binary image is smaller than
#   what would normally be encountered with traditional
#   download formats (such as SREC or uuencoded; both
#   ascii based). The special header at the front of the
#   image is used to guide the target's rrload (a
#   booloader developed by RidgeRun Inc). The header
#   data contains a field representing the total byte
#   count of the binary data following the header as
#   well as a field that indicates the load address of
#   run image. Additionally, a field exists in the
#   header which indicates the image's entry point which
#   could be called by the bootloader to invoked the
#   just downloaded program.
#   -----------
#   Scenario #2
#   -----------
#   If the supplied image is not a standard binary
#   executagle image then that is ok too, a header is 
#   constructed and tacked onto the front of the supplied
#   binary data forming the new binary image (in rr format).
#   In this case the EntryAddr is set to 0xFFFFFFFF by 
#   default and the LoadAddr is set to   0x00000000 by
#   default unless otherwise indicated by command line 
#   arguments -LEntry and -LAddr which if used are assumed
#   to be in hexidecimal units.
#   
#   -----------
#   Scenario #3
#   -----------
#
#   Read/Write file system (like JFFS) that will not
#   work if rrload stores a 20 byte header at the beginning
#   of the flashed component image
#
#   mkimage [--NoHeader ] <input-bin> <out-RR>
#
# Usage: 
#   mkimage [--LAddr h] [--EAddr h] [--NoHeader] <input-bin> <out-RR> 
#  
# Examples: 
#   $ mkimage linux linux.rr
#     ..or..
#   $ mkimage -LAddr 10008000 -EAddr 10008000 vmlinux vmlinux.rr
#     ..or..
#   $ mkimage --NoHeader fileSys.gz fileSys.gz.rr
#     ..or..
#   $ mkimage --LAddr A00 fileSys.gz fileSys.gz.rr
#     ..or..
#   $ mkimage --LAddr A00 fileSys.gz fileSys.gz.rr
#                     ^
#                     |
#                 Assumed hex units. 
#                 Please omit the 
#                 leading "0x".
########################################################

if [ $# -lt 2 ] ; then
  echo "Error: missing argument"
  echo "Usage: mkimage [--Prefix prefix] [--LAddr n] [--EAddr n] [--NoHeader] <input-Bin> <out-RR>"
  exit 1
fi

# Pleae Note the following formatting inconsistency.
# (Sorry, for now this is necessary)
LoadAddr="00000000"     # Note: hex val *without* proceeding "0x"
EntryAddr="0xFFFFFFFF"  # Note: hex val *with* procedding  "0x"

unset prefix
Header="y"
LAddrSupplied="n"
EAddrSupplied="n"
compress="n"

while [ $# -gt 0 ] ; do
  case "$1" in
  --Prefix)
     shift;
     prefix="$1"
     shift
     ;;
  --LAddr )
     shift
     LoadAddr="$1"
     # Next, make the supplied LAddr exactly 8 hex chars long.
     LoadAddr="0000000${LoadAddr}"
     LoadAddr=$(echo $LoadAddr | sed -e "s/^.*\(........\)$/\1/g")
     LAddrSupplied="y"
     shift
     ;;
  --EAddr )
     shift
     EntryAddr="$1"
     # Next, make the supplied LEntry exactly 8 hex chars long.
     EntryAddr="0000000${EntryAddr}"
     EntryAddr=$(echo $EntryAddr | sed -e "s/^.*\(........\)$/\1/g")
     EntryAddr=0x$EntryAddr
     EAddrSupplied="y"
     shift
     ;;
  --NoHeader )
     Header="n"
     shift
     ;;
  --compress )
     compress="y"
     shift
     ;;
  *)
    break
  ;;
  esac
done

if [ ! $# -eq 2 ] ; then
  echo "Error: invalid argument set."
  echo "Usage: mkimage [--LAddr h] <input-Bin> <out-RR>"
  exit 1
fi

binary=$1.stripped
outbin=$2

cp $1 $binary
FileTypeExec=$(${prefix}objdump -f $binary 2>/dev/null | egrep "EXEC_P")

if [ ! -z "$FileTypeExec" ] ; then

  # -----------
  # Scenario #1
  # -----------
  # We have an executable style binary (like ELF, etc).
  # So...
  # ---------------------------------
  # Next  | Create the binary image data.
  # ---------------------------------
    ${prefix}strip ${binary}
    ${prefix}objcopy -S -O binary $binary ${binary}.binary
  # ---------------------------------
  # Next  | Create compress image if requested
  # ---------------------------------
    image_file=${binary}.binary
    if [  "$compress" = "y" ] ; then
	gzip -f -9 -c ${binary}.binary        > ${binary}.binary.gz
	image_file=${binary}.binary.gz
    fi
  # ---------------------------------
  # Next  | Create the header information (ascii) needed 
  #       | by the TI925 bootloader. This includes the
  #       | load address, entry address and byte count of
  #       | the binary executable data which will follow it.
  # ---------------------------------
    if [ "$LAddrSupplied" = "n" ] ; then
      # Next, Since LoadAddr not already supplied by user we'll
      # derive it by consulting the binary executable file.
      LoadAddr=$(${prefix}objdump -h ${binary} | grep "  0 \.")
      LoadAddr=$(echo $LoadAddr | cut -d' ' -f4) # eight hex chars
    fi
    if [ "$EAddrSupplied" = "n" ] ; then
      # Next, Since EntryAddr not already supplied by user we'll
      # derive it by consulting the binary executable file.
      EntryAddr=$(${prefix}objdump -f ${binary} | grep -i "start")
      EntryAddr=$(echo $EntryAddr | cut -d' ' -f3) # eight hex chars
    fi
    # Next, Compute byte length of binary portion.
    numBytes=$(wc --bytes $image_file)
    numBytes=$(echo $numBytes | cut -d' ' -f1)
    numBytes=$(echo 16o $numBytes p | dc) # converts to hex.
    # Next, make the numBytes string exactly 8 hex chars long.
    numBytes="0000000${numBytes}"
    numBytes=$(echo $numBytes | sed -e "s/^.*\(........\)$/\1/g")
  # ---------------------------------
  # Next  | Combine the ascii header information
  #       | with the binary image to make the 
  #       | final downloadable *mostly* binary 
  #       | image.
  # ---------------------------------
    rm -f ${outbin}
    echo ">LoadAddr :0x${LoadAddr}"     >> ${outbin}
    if [ "${Header}" = "y" ]; then
	echo ">EntryAddr:${EntryAddr}"  >> ${outbin}
    else
	echo ">NoHeader"  >> ${outbin}
    fi
    echo ">NumBytes :0x${numBytes}"     >> ${outbin}
    cat $image_file            >> ${outbin}
  # ---------------------------------
  # Cleanup and exit
  # ---------------------------------
    rm -f ${binary}.binary $image_file
    exit 0

else

  # -----------
  # Scenario #2
  # -----------
  # Just a binary image but not a standard executable 
  # style binary (like ELF, etc). Might be a compressed
  # filesystem image, etc.
  # So...
  # ---------------------------------
  # Next  | Create the header information (ascii) needed 
  #       | by the TI925 bootloader. This includes the
  #       | load address, entry address and byte count of
  #       | the binary file which will follow it.
  # ---------------------------------
  #       | Create compress image if requested
  # ---------------------------------
  #
    image_file=${binary}
    if [  "$compress" = "y" ] ; then
	gzip -f -9 -c ${image_file}        > ${image_file}.gz
	image_file=${image_file}.gz
    fi
  #
  # Note: The LoadAddr and EntryAddr are already established
  # for us at this point, but we will need to compute the 
  # byte length of binary portion next.
  #
    numBytes=$(wc --bytes ${image_file})
    numBytes=$(echo $numBytes | cut -d' ' -f1)
    numBytes=$(echo 16o $numBytes p | dc) # converts to hex.
    # Next, make the numBytes string exactly 8 hex chars long.
    numBytes="0000000${numBytes}"
    numBytes=$(echo $numBytes | sed -e "s/^.*\(........\)$/\1/g")
  #
  # ---------------------------------
  # Next  | Combine the ascii header information
  #       | with the binary image to make the 
  #       | final downloadable *mostly* binary 
  #       | image.
  # ---------------------------------
  #
    rm -f ${outbin}
    echo ">LoadAddr :0x${LoadAddr}" >> ${outbin}
    if [ ${Header} = "y" ]; then
	echo ">EntryAddr:${EntryAddr}"  >> ${outbin}
    else
	echo ">NoHeader"  >> ${outbin}
    fi
    echo ">NumBytes :0x${numBytes}" >> ${outbin}
    cat ${image_file}                   >> ${outbin}
  # ---------------------------------
  # Cleanup and exit
  # ---------------------------------
    rm -f ${image_file}.gz
    exit 0
fi
