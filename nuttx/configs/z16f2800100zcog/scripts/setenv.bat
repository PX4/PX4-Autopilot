@echo off

rem configs/z16f2800100zcog/scripts/setenv.bat
rem
rem   Copyright (C) 2012 Gregory Nutt. All rights reserved.
rem   Author: Gregory Nutt <gnutt@nuttx.org>
rem
rem Redistribution and use in source and binary forms, with or without
rem modification, are permitted provided that the following conditions
rem are met:
rem
rem 1. Redistributions of source code must retain the above copyright
rem    notice, this list of conditions and the following disclaimer.
rem 2. Redistributions in binary form must reproduce the above copyright
rem    notice, this list of conditions and the following disclaimer in
rem    the documentation and/or other materials provided with the
rem    distribution.
rem 3. Neither the name NuttX nor the names of its contributors may be
rem    used to endorse or promote products derived from this software
rem    without specific prior written permission.
rem
rem THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
rem "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
rem LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
rem FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
rem COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
rem INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
rem BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
rem OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
rem AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
rem LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
rem ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
rem POSSIBILITY OF SUCH DAMAGE.

rem This is the location where I installed in the MinGW compiler. With
rem this configuration, it is recommended that you do NOT install the
rem MSYS tools; they conflict with the GNUWin32 tools.  See
rem http://www.mingw.org/ for further info.

set PATH=C:\MinGW\bin;%PATH%

rem This is the location where I installed the ZDS-II toolchain.

set PATH=C:\Program Files (x86)\ZiLOG\ZDSII_ZNEO_5.0.1\bin;%PATH%

rem This is the location where I installed the GNUWin32 tools.  See
rem http://gnuwin32.sourceforge.net/.

set PATH=C:\gnuwin32\bin;%PATH%
echo %PATH%
