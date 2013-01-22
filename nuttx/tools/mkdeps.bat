@echo off

rem tools/mkdeps.sh
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

rem Accumulate CFLAGS up to "--"

set cc=
set cflags=
set altpath=
set files=
set args=
set debug=n

:Loop
if "%1"=="" goto Continue

if "%1"=="--" (
  set cc=%cflags%
  set cflags=%args%
  set args=
  goto NextParm
)

if "%1"=="--dep-path" (
  if "%args%"=="" (
    set altpath=%altpath% %2
  ) else (
    set args=%args% %2
  )
  shift
  goto NextParm
)

if "%1"=="--dep-debug" (
rem  @echo on
  set debug=y
  goto NextParm
)

if "%1"=="--help" goto Usage

if "%args%"=="" (
  set args=%1
) else (
  set args=%args% %1
)

:NextParm
shift
goto Loop
:Continue

set files=%args%

if "%debug%"=="y" (
  echo cc=%cc%
  echo cflags=%cflags%
  echo files=%files%
  echo altpath=%altpath%
)

rem Now check if we have everything

if "%cc%"=="" (
  echo ERROR: No compiler specified
  goto Usage
)

if "%files%"=="" (
  rem Don't report an error -- this happens normally in some configurations
  echo # No files specified for dependency generataion
  goto End
)

rem Then get the dependencies for each file

if "%altpath%"=="" goto NoPaths
for %%G in (%files%) do (
  set fullpath=
  set file=%%G
  call :Checkpaths
  if "%debug%"=="y" echo %file%: fullpath=%fullpath%
  if "%fullpath%"=="" goto :NoFile
  if "%debug%"=="y" echo CMD: %cc% -M %cflags% %fullpath%
  %cc% -M %cflags% %fullpath% || goto DepFail
)
goto :End

:NoPaths
for %%G in (%files%) do (
  set fullpath=
  set file=%%G
  call :CheckFile %%G
)
goto :End

:CheckFile
if "%debug%"=="y" echo Checkfile: Checking %file%
if not exist %file% goto :NoFile
set fullpath=%file%
  if "%debug%"=="y" echo CMD: %cc% -M %cflags% %fullpath%
%cc% -M %cflags% %fullpath% || goto DepFail
goto :EOF

:CheckPaths
for %%H in (%altpath%) do (
  set tmppath=%%H\%file%
  if "%debug%"=="y" echo Checkfile: Checking %tmppath% 
  if exist %tmppath% (
    set fullpath=%tmppath%
    goto :EOF
  )
)
goto :EOF

:NoFile
echo ERROR: No readable file at %file%
goto Usage

:DepFail
echo ERROR: Failed to created dependencies for %file%

:Usage
echo Usage: mkdeps  [OPTIONS] CC -- CFLAGS -- file [file [file...]]
echo Where:
echo   CC
echo     A variable number of arguments that define how to execute the compiler
echo   CFLAGS
echo     The compiler compilation flags
echo   file
echo     One or more C files whose dependencies will be checked.  Each file is expected
echo     to reside in the current directory unless --dep-path is provided on the command line
echo And [OPTIONS] include:
echo   --dep-debug
echo     Enable script debug
echo   --dep-path ^<path^>
echo     Do not look in the current directory for the file.  Instead, look in <path> to see
echo     if the file resides there.  --dep-path may be used multiple times to specify
echo     multiple alternative location
echo   --help
echo     Shows this message and exits

:End
