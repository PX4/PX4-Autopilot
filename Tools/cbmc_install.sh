#!/bin/bash

INSTALL_DIR=/usr/local/bin

# CBMC
CBMC_SITE=http://www.cprover.org/cbmc/download
CBMC_TARBALL=cbmc-5-1-linux-32.tgz 
CBMC_TMP_DIR=/tmp/cbmc
CBMC_FILES="cbmc hw-cbmc goto-cc goto-instrument"

# SATABS
SATABS_SITE=http://www.cprover.org/satabs/download
SATABS_TARBALL=satabs-3-2-linux-32.tgz 
SATABS_TMP_DIR=/tmp/satabs
SATABS_FILES="satabs boom"
SATABS_GOTO_FILES="goto-cc goto-instrument"

# CBMC install
if [ -f ${CBMC_TMP_DIR}/${CBMC_TARBALL} ]
then
	echo "cbmc tarball already downloaded"
else
	wget -P $CBMC_TMP_DIR ${CBMC_SITE}/${CBMC_TARBALL}
fi
mkdir -p ${CBMC_TMP_DIR}
tar xzf ${CBMC_TMP_DIR}/${CBMC_TARBALL} -C ${CBMC_TMP_DIR}
mkdir -p ${INSTALL_DIR}
for file in $CBMC_FILES
do
	echo installing $file
	sudo install ${CBMC_TMP_DIR}/$file ${INSTALL_DIR}
done

# SATABS install
if [ -f ${SATABS_TMP_DIR}/${SATABS_TARBALL} ]
then
	echo "satabs tarball already downloaded"
else
	wget -P $SATABS_TMP_DIR ${SATABS_SITE}/${SATABS_TARBALL}
fi
mkdir -p ${SATABS_TMP_DIR}
tar xzf ${SATABS_TMP_DIR}/${SATABS_TARBALL} -C ${SATABS_TMP_DIR}
mkdir -p ${INSTALL_DIR}
for file in $SATABS_FILES
do
	echo installing $file
	sudo install ${SATABS_TMP_DIR}/$file ${INSTALL_DIR}
done

# need to rename goto for satabs since difference from cbmc ver.
for file in $SATABS_GOTO_FILES
do
	sfile=satabs-$file
	echo installing $sfile
	sudo install ${SATABS_TMP_DIR}/$file ${INSTALL_DIR}/$sfile
done
