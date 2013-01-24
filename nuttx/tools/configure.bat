@echo off

rem tools/configure.bat
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
rem

rem Parse command line arguments

set debug=
set fmt=-w
set posix=
set help=
set appdir=
set config=

:ArgLoop
if "%1"=="" goto :NoConfig
if "%1"=="-h" goto :ShowUsage
if "%1"=="-d" goto :SetDebug
if "%1"=="-w" goto :SetWindows
if "%1"=="-l" goto :SetPosix
if "%1"=="-a" goto :SetAppDir

set config=%1
goto EndOfLoop

:SetDebug
set debug=-d
goto :NextArg

:SetWindows
set fmt=-w
goto :NextArg

:SetWindows
set fmt=-l
goto :NextArg

:SetAppDir
shift
set appdir=-a %1

:NextArg
shift
goto :ArgLoop

:EndOfLoop

rem Check if we have to build configure.exe

if exist configure.exe goto :HaveConfigureExe

set cc=mingw32-gcc.exe
set cflags=-Wall -Wstrict-prototypes -Wshadow -g -pipe -I. -DCONFIG_WINDOWS_NATIVE=y
%cc% %cflags% -o configure.exe configure.c cfgparser.c
if errorlevel 1 (
  echo ERROR: %cc% failed
  echo Is ming32-gcc.exe installed?  Is it in the PATH variable?
  goto End
)

:HaveConfigureExe
configure.exe %debug% %fmt% %appdir% %config%
if errorlevel 1 echo configure.exe failed
goto End

:NoConfig
echo Missing ^<board-name^>/^<config-name^> argument

:ShowUsage
echo USAGE: %0  [-d] [-w] [-l] [-h] [-a ^<app-dir^>] ^<board-name^>\^<config-name^>
echo\nWhere:
echo  ^<board-name^>:
echo    Identifies the board.  This must correspond to a board directory
echo    under nuttx/configs/.
echo  ^<config-name^>:
echo    Identifies the specific configuratin for the selected ^<board-name^>.
echo    This must correspond to a sub-directory under the board directory at
echo    under nuttx/configs/^<board-name^>/.
echo  ^<-d^>:
echo    Enables debug output
echo  ^<-w^>:
echo    Informs the tool that it should use Windows style paths like C:\\Program Files
echo    instead of POSIX style paths are used like /usr/local/bin.  Windows
echo    style paths are used by default.
echo  ^<-l^>:
echo    Informs the tool that it should use POSIX style paths like /usr/local/bin.
echo    By default, Windows style paths like C:\\Program Files are used.
echo  -a ^<app-dir^>:
echo    Informs the configuration tool where the application build
echo    directory.  This is a relative path from the top-level NuttX
echo    build directory.  But default, this tool will look in the usual
echo    places to try to locate the application directory:  ../apps or
echo    ../apps-xx.yy where xx.yy is the NuttX version number.
echo  ^<-h^>:
echo    Prints this message and exits.

:End
