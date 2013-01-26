@echo off

rem tools/kconfig.bat
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

rem Remember the state of the PATH variable on entry

set oldpath=%PATH%

rem Handle command line options

set action=%1
shift
if "%action%"=="" goto :MissingArgument

set appsdir=..\apps
set cygwindir=C:\Cygwin

:ArgLoop

if "%1"=="" goto :CheckArguments

if "%1"=="-a" (
  shift
  set appsdir=%1
  goto :NextArg
)

if "%1"=="-c" (
  shift
  set cygwindir=%1
  goto :NextArg
)

echo ERROR: Unrecognized option: %1
goto :ShowUsage

:NextArg
shift
goto :ArgLoop

rem Verify that all of the paths are valid

:CheckArguments
if exist "%appsdir%" goto :CheckCygwinDir

echo ERROR: %appsdir% does not exist
goto :ShowUsage

:CheckCygwinDir

if exist "%cygwindir%" goto :SetPath

echo ERROR: %cygwindir% does not exist
goto :ShowUsage

rem Setup some required environment variables and PATH settings

:SetPath
set PATH=%cygwindir%\usr\local\bin;%cygwindir%\usr\bin;%cygwindir%\bin;%PATH%
set APPSDIR=%appsdir%

rem Execute the requested action

if "%action%"=="config" goto :DoConfig
if "%action%"=="oldconfig" goto :DoOldConfig
if "%action%"=="menuconfig" goto :DoMenuConfig

echo ERROR: Unrecognized action: %action%
goto :ShowUsage

:DoConfig
kconfig-conf Kconfig
goto End

:DoOldConfig
kconfig-conf --oldconfig Kconfig
goto End

:DoMenuConfig
kconfig-mconf Kconfig
goto End

:MissingArgument

echo ERROR: Missing required argument

:ShowUsage
echo USAGE: %0 ^<action^> [-a ^<appsdir^>] [-c ^<cygwindir^>]
echo Where:
echo   ^<action^> is one of config, oldconf, or menuconfig
echo   ^<appsdir^> is the relative path to the apps\ directory.
echo     This defaults to ..\apps
echo   ^<cygwindir^> is the relative path to the Cygwin installation
echo     directory.  This defaults to C:\Cygwin

rem Restore the original PATH settings

:End
set PATH=%oldpath%

