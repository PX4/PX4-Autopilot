@echo off

rem tools/copydir.bat
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

rem
rem NuttX uses symbolic links to configure platform-specific directories into
rem the build system.  This works great except for when a Windows native
rem toolchain is used in a Cygwin environment.  In that case, symbolic
rem links do not work correctly when accessed from the Windows native toolchain;
rem rather, just look link files with the extension .lnk
rem
rem In this environment, the build system will work around this using this script
rem as a replacement for the 'ln' command.  This scrpt will simply copy the
rem directory into the expected positiion.
rem

set src=%1
set dest=%2

rem Verify that arguments were provided

if "%src%"=="" goto :MissingSrc
if "%dest%"=="" goto :MissingDest
goto CheckSrc

:MissingSrc

echo Missing ^<src^> and ^<dest^> arguments
goto :ShowUsage

:MissingDest

echo Missing ^<dest^> arguments
goto :ShowUsage

rem Verify that a directory exists at the source path

:CheckSrc

if exist %src% goto :CheckDest

echo No directory at %src%
goto :ShowUsage

:CheckDest

rem If something already exists at the destination path, remove it

if not exist %dest% goto :CopyDir

rmdir /q /s %dest%
if errorlevel 1 (
  echo Failed to remove existing object at %dest%
  goto :ShowUsage
)

rem Copy the directory

:CopyDir

xcopy %src% %dest% /c /q /s /e /y /i
echo FAKELNK >  %dest%\.fakelnk
goto :End

:ShowUsage
echo USAGE: %0 ^<src^> ^<dest^>
echo Where:
echo   ^<src^> is the source directory to be copied
echo   ^<dest^> is the destination directory to be created

:End
