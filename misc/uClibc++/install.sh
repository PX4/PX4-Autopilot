#!/bin/bash

usage="USAGE: $0 <full path to the NuttX directory>"

lgpl="\n
GNU LESSER GENERAL PUBLIC LICENSE\n
\n
Version 3, 29 June 2007\n
\n
Copyright © 2007 Free Software Foundation, Inc. <http://fsf.org/>\n
\n
Everyone is permitted to copy and distribute verbatim copies of this \n
license document, but changing it is not allowed.\n
\n
This version of the GNU Lesser General Public License incorporates the\n
terms and conditions of version 3 of the GNU General Public License,\n
supplemented by the additional permissions listed below.\n
\n
0. Additional Definitions.\n
\n
As used herein, “this License” refers to version 3 of the GNU Lesser\n
General Public License, and the “GNU GPL” refers to version 3 of the\n
GNU General Public License.\n
\n
“The Library” refers to a covered work governed by this License, other\n
than an Application or a Combined Work as defined below.\n
\n
An “Application” is any work that makes use of an interface provided\n
by the Library, but which is not otherwise based on the Library. Defining\n
a subclass of a class defined by the Library is deemed a mode of using an\n
interface provided by the Library.\n
\n
A “Combined Work” is a work produced by combining or linking an\n
Application with the Library. The particular version of the Library with\n
which the Combined Work was made is also called the “Linked Version”.\n
\n
The “Minimal Corresponding Source” for a Combined Work means the\n
Corresponding Source for the Combined Work, excluding any source code\n
for portions of the Combined Work that, considered in isolation, are based\n
on the Application, and not on the Linked Version.\n
\n
The “Corresponding Application Code” for a Combined Work means the object\n
code and/or source code for the Application, including any data and utility\n
programs needed for reproducing the Combined Work from the Application,\n
but excluding the System Libraries of the Combined Work.\n
\n
1. Exception to Section 3 of the GNU GPL.\n
\n
You may convey a covered work under sections 3 and 4 of this License\n
without being bound by section 3 of the GNU GPL.\n
\n
2. Conveying Modified Versions.\n
\n
If you modify a copy of the Library, and, in your modifications, a\n
facility refers to a function or data to be supplied by an Application\n
that uses the facility (other than as an argument passed when the facility\n
is invoked), then you may convey a copy of the modified version:\n
\n
    a) under this License, provided that you make a good faith effort to\n
       ensure that, in the event an Application does not supply the function\n
       or data, the facility still operates, and performs whatever part of\n
       its purpose remains meaningful, or\n
    b) under the GNU GPL, with none of the additional permissions of this\n
       License applicable to that copy.\n
\n
3. Object Code Incorporating Material from Library Header Files.\n
\n
The object code form of an Application may incorporate material from a\n
header file that is part of the Library. You may convey such object code\n
under terms of your choice, provided that, if the incorporated material is\n
not limited to numerical parameters, data structure layouts and accessors, or\n
small macros, inline functions and templates (ten or fewer lines in length),\n
you do both of the following:\n
\n
    a) Give prominent notice with each copy of the object code that the\n
       Library is used in it and that the Library and its use are covered\n
       by this License.\n
    b) Accompany the object code with a copy of the GNU GPL and this license\n
       document.\n
\n
4. Combined Works.\n
\n
You may convey a Combined Work under terms of your choice that, taken\n
together, effectively do not restrict modification of the portions of the\n
Library contained in the Combined Work and reverse engineering for debugging\n
such modifications, if you also do each of the following:\n
\n
    a) Give prominent notice with each copy of the Combined Work that the\n
      Library is used in it and that the Library and its use are covered by\n
      this License.\n
    b) Accompany the Combined Work with a copy of the GNU GPL and this\n
       license document.\n
    c) For a Combined Work that displays copyright notices during execution,\n
       include the copyright notice for the Library among these notices,\n
       as well as a reference directing the user to the copies of the\n
       GNU GPL and this license document.\n
    d) Do one of the following:\n
\n
        0) Convey the Minimal Corresponding Source under the terms of\n
           this License, and the Corresponding Application Code in a form\n
           suitable for, and under terms that permit, the user to recombine\n
           or relink the Application with a modified version of the Linked\n
           Version to produce a modified Combined Work, in the manner\n
           specified by section 6 of the GNU GPL for conveying Corresponding\n
           Source.\n
        1) Use a suitable shared library mechanism for linking with the Library.\n
           A suitable mechanism is one that (a) uses at run time a copy of\n
           the Library already present on the user's computer system, and\n
           (b) will operate properly with a modified version of the Library\n
           that is interface-compatible with the Linked Version.\n
\n
    e) Provide Installation Information, but only if you would otherwise\n
       be required to provide such information under section 6 of the\n
       GNU GPL, and only to the extent that such information is necessary\n
       to install and execute a modified version of the Combined Work\n
       produced by recombining or relinking the Application with a\n
       modified version of the Linked Version. (If you use option 4d0, the\n
       Installation Information must accompany the Minimal Corresponding\n
       Source and Corresponding Application Code. If you use option 4d1,\n
       you must provide the Installation Information in the manner\n
       specified by section 6 of the GNU GPL for conveying Corresponding\n
       Source.)\n
\n
5. Combined Libraries.\n
\n
You may place library facilities that are a work based on the Library\n
side by side in a single library together with other library facilities\n
that are not Applications and are not covered by this License, and convey\n
such a combined library under terms of your choice, if you do both of the\n
following:\n
\n
    a) Accompany the combined library with a copy of the same work based\n
       on the Library, uncombined with any other library facilities,\n
       conveyed under the terms of this License.\n
    b) Give prominent notice with the combined library that part of it\n
       is a work based on the Library, and explaining where to find the\n
       accompanying uncombined form of the same work.\n
\n
6. Revised Versions of the GNU Lesser General Public License.\n
\n
The Free Software Foundation may publish revised and/or new versions of\n
the GNU Lesser General Public License from time to time. Such new\n
versions will be similar in spirit to the present version, but may\n
differ in detail to address new problems or concerns.\n
\n
Each version is given a distinguishing version number. If the Library\n
as you received it specifies that a certain numbered version of the\n
GNU Lesser General Public License “or any later version” applies to\n
it, you have the option of following the terms and conditions either\n
of that published version or of any later version published by the Free\n
Software Foundation. If the Library as you received it does not specify\n
a version number of the GNU Lesser General Public License, you may\n
choose any version of the GNU Lesser General Public License ever\n
published by the Free Software Foundation.\n
\n
If the Library as you received it specifies that a proxy can decide\n
whether future versions of the GNU Lesser General Public License shall\n
apply, that proxy's public statement of acceptance of any version is\n
permanent authorization for you to choose that version for the Library.\n"

# readans prompt default

function readans () {
    echo -n "$1 ($2): "
    IFS='@' read ans || exit 1
    [ -z "$ans" ] && ans=$2
}

# readyn prompt default

function readyn () {
    while :; do
        readans "$1 [Y/N]" $2
        case "$ans" in
        [yY] | [yY]es )
            ans=y
            break ;;
        [nN] | [nN]o )
            ans=n
            break ;;
        * )
            echo "Please answer Y or N"
            ;;
        esac
    done
}

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

if [ -d "${uclibc_srcdir}" ]; then
  echo "ERROR: Directory ${uclibc_srcdir} already exists"
  echo "       Please remove the  ${uclibc_srcdir} directory and try again"
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

if [ -d "${uclibc_incdir}" ]; then
  echo "ERROR: Directory ${uclibc_incdir} already exists"
  echo "       Please remove the  ${uclibc_incdir} directory and try again"
  echo $usage
  exit 1
fi

# Licensing

echo "You are about to install the uClibc++ library into the NuttX source"
echo "tree.  NuttX is licensed under the permissive, modified BSD License; uClibc"
echo "is licensed under the GNU LGPL Version 3 license.  When you install"
echo "uClibc++ into the NuttX source tree, you must then comply with uClibc's"
echo "stricter GNU LGPL Version 3 license."
echo ""

readyn "Continue" "N"
echo ""

if [ "X${ans}" != "Xy" -a "X${ans}" != "XY" ]; then
  echo "Good bye"
  exit 0
fi

echo "You must read and agree to the following license"
echo ""

readyn "Continue" "N"
echo ""

if [ "X${ans}" != "Xy" -a "X${ans}" != "XY" ]; then
  echo "Good bye"
  exit 0
fi

echo -e ${lgpl} | more
echo ""

readyn "I agree to the termso the GNU LGPL Version 3 license" "N"
echo ""

if [ "X${ans}" != "Xy" -a "X${ans}" != "XY" ]; then
  echo "Good bye"
  exit 0
fi

echo "Installing uClibc++ in the NuttX source tree"

filelist=`find libxx -type f | fgrep -v '.svn'`

for file in $filelist; do
  install -D $file ${nuttx_path}/${file}
done

filelist=`find include -type f | fgrep -v '.svn'`

for file in $filelist; do
  install -D $file ${nuttx_path}/${file}
done

echo "Installation suceeded"
echo ""
