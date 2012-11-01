#!/bin/bash

usage="USAGE: $0 <full path to the NuttX directory>"
special="include/features.h"

# Get the single, required command line argument

nuttx_path=$1
if [ -z "${nuttx_path}" ]; then
  echo "ERROR: Missing path to the NuttX directory"
  echo $usage
  exit 1
fi

# Lots of sanity checking so that we do not do anything too stupid

if [ ! -d libxx ]; then
  echo "ERROR: Directory libxx does not exist in this directory"
  echo "       Please CD into the misc/uClibc++ directory and try again"
  echo $usage
  exit 1
fi

if [ ! -d include ]; then
  echo "ERROR: Directory include does not exist in this directory"
  echo "       Please CD into the misc/uClibc++ directory and try again"
  echo $usage
  exit 1
fi

if [ ! -d "${nuttx_path}" ]; then
  echo "ERROR: Directory ${nuttx_path} does not exist"
  echo $usage
  exit 1
fi

if [ ! -f "${nuttx_path}/Makefile" ]; then
  echo "ERROR: No Makefile in directory ${nuttx_path}"
  echo $usage
  exit 1
fi

libxx_srcdir=${nuttx_path}/libxx

if [ ! -d "${libxx_srcdir}" ]; then
  echo "ERROR: Directory ${libxx_srcdir} does not exist"
  echo $usage
  exit 1
fi

if [ ! -f "${libxx_srcdir}/Makefile" ]; then
  echo "ERROR: No Makefile in directory ${libxx_srcdir}"
  echo $usage
  exit 1
fi

uclibc_srcdir=${libxx_srcdir}/uClibc++

if [ ! -d "${uclibc_srcdir}" ]; then
  echo "ERROR: Directory ${uclibc_srcdir} does not exiss"
  echo "       uClibc++ has not been installed"
  echo $usage
  exit 1
fi

nuttx_incdir=${nuttx_path}/include

if [ ! -d "${nuttx_incdir}" ]; then
  echo "ERROR: Directory ${nuttx_incdir} does not exist"
  echo $usage
  exit 1
fi

nuttxcxx_incdir=${nuttx_incdir}/cxx

if [ ! -d "${nuttxcxx_incdir}" ]; then
  echo "ERROR: Directory ${nuttxcxx_incdir} does not exist"
  echo $usage
  exit 1
fi

uclibc_incdir=${nuttx_incdir}/uClibc++

if [ ! -d "${uclibc_incdir}" ]; then
  echo "ERROR: Directory ${uclibc_incdir} does not exist"
  echo "       uClibc++ has not been installed"
  echo $usage
  exit 1
fi

echo "##### Comparing libxx"

filelist=`find libxx -type f | fgrep -v '.svn'`

for file in $filelist; do
  diff -u $file ${nuttx_path}/${file}
done

echo ""
echo "##### Comparing include"

filelist=`find include -type f | fgrep -v '.svn'`

for file in $filelist; do
  diff -u $file ${nuttx_path}/${file}
done

echo ""
echo "##### Comparing other files"

for file in $speical; do
  diff -u $file ${nuttx_path}/${file}
done

