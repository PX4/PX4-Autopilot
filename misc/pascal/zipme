#!/bin/sh

#set -x

DATECODE=$1

TAR="tar cvf"
ZIP=gzip

# Move up one directory

cd ..
HOME=`pwd`
DIR=${HOME}/pascal
SUBDIR=pascal-${DATECODE}

# Make sure we know what is going on

if [ -z ${DATECODE} ] ; then
   echo "You must supply a date code like a.b.c as a parameter"
   exit 1;
fi

if [ ! -d ${SUBDIR} ] ; then
   echo "Directory ${SUBDIR} does not exist."
   exit 1;
fi

if [ ! -x ${SUBDIR}/$0 ] ; then
   echo "You must cd to the directory containing this script."
   exit 1;
fi

# Define the ZIP file pathes

TAR_NAME=${SUBDIR}.tar
ZIP_NAME=${TAR_NAME}.gz

# Prepare the directory

make -C ${SUBDIR} deep-clean

find ${SUBDIR} -name \*~ -exec rm -f {} \; || \
	{ echo "Removal of emacs garbage failed!" ; exit 1 ; }

find ${SUBDIR} -name \#\* -exec rm -f {} \; || \
	{ echo "Removal of emacs garbage failed!" ; exit 1 ; }

find ${SUBDIR} -name .\*swp\* -exec rm -f {} \; || \
	{ echo "Removal of vi garbage failed!" ; exit 1 ; }

# Remove any previous tarballs

if [ -f ${TAR_NAME} ] ; then
    echo "Removing ${HOME}/${TAR_NAME}"
    rm -f ${TAR_NAME} || \
	{ echo "rm ${TAR_NAME} failed!" ; exit 1 ; }
fi

if [ -f ${ZIP_NAME} ] ; then
    echo "Removing ${HOME}/${ZIP_NAME}"
    rm -f ${ZIP_NAME} || \
	{ echo "rm ${ZIP_NAME} failed!" ; exit 1 ; }
fi

# Then zip it

${TAR} ${TAR_NAME} ${SUBDIR} || \
    { echo "tar of ${DIR} failed!" ; exit 1 ; }
${ZIP} ${TAR_NAME} || \
    { echo "zip of ${TAR_NAME} failed!" ; exit 1 ; }
