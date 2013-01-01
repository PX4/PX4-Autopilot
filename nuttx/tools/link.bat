@echo off

rem tools/link.bat
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

set src=%1
set link=%2

rem Verify that arguments were provided

if "%src%"=="" goto :MissingSrc
if "%link%"=="" goto :MissingLink
goto CheckSrc

:MissingSrc

echo Missing ^<src^> and ^<link^> arguments
goto :ShowUsage

:MissingLink

echo Missing ^<link^> arguments
goto :ShowUsage

rem Verify that a directory exists at the source path

:CheckSrc

if exist %src% goto :CheckLink

echo No directory at %src%
goto :ShowUsage

:CheckLink

rem If something already exists at the destination path, remove it

if not exist %link% goto :MkLink

rmdir /q /s %link%
if errorlevel 1 (
  echo Failed to remove existing object at %link%
  goto :ShowUsage
)

rem Copy the directory

:MkLink

/user:administrator mklink /d %src% %link%
goto :End

:ShowUsage
echo USAGE: %0 ^<src^> ^<link^>
echo Where:
echo   ^<src^> is the source directory to be linked
echo   ^<link^> is the link to be created

:End
