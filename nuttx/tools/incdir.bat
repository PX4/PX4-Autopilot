@echo off

rem tools/incdir.sh
rem
rem   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

rem Handle command line options

set progname=%0
set pathtype=user

:ArgLoop

rem [-d] [-w] [-s] [-h]. [-w] and [-d] Ignored for compatibility with incdir.sh

if "%1"=="-d" goto :NextArg
if "%1"=="-w" goto :NextArg
if "%1"=="-h" goto :Usage

if "%1"=="-s" (
  set pathtype=system
  goto :NextArg
)

goto :CheckCompiler

:NextArg
shift
goto :ArgLoop

:CheckCompiler
if "%1"=="" (
  echo ERROR: Missing compiler name
  goto :Usage
)

set ccpath=%1
shift

set compiler=
for /F %%i in ("%ccpath%") do set compiler=%%~ni

if "%1"=="" (
  echo ERROR: Missing directory paths
  goto :Usage
)

rem Check for some well known, non-GCC Windows native tools that require
rem a special output format as well as special paths

:GetFormat
set fmt=std
if "%compiler%"=="ez8cc" goto :SetZdsFormt
if "%compiler%"=="zneocc" goto :SetZdsFormt
if "%compiler%"=="ez80cc" goto :SetZdsFormt
goto :GeneratePaths

:SetZdsFormt
set fmt=zds

rem Generate the compiler include path directives.

:GeneratePaths
set response=

:DirLoop
if "%1" == "" (
  echo %response%
  goto :End
)

if not exist %1 (
  echo ERROR: Path %1 does not exist
  goto :Usage
)

if "%fmt%"=="zds" goto :GenerateZdsPath
if "%response%"=="" goto :FirstStdPath
if "%pathtype%"=="system" goto :NextStdSystemPath

set response=%response% -I "%1"
goto :EndOfDirLoop

:NextStdSystemPath

set response=%response% -isystem "%1"
goto :EndOfDirLoop

:FirstStdPath

if "%pathtype%"=="system" goto :FirstStdSystemPath
set response=-I "%1"
goto :EndOfDirLoop

:FirstStdSystemPath

set response=-isystem "%1"
goto :EndOfDirLoop

:GenerateZdsPath

if "%response%"=="" goto :FirstZdsPath
set response=%response%;%1
goto :EndOfDirLoop

:FirstZdsPath

if "%pathtype%"=="system" goto :FirstZdsSystemPath
set response=-usrinc:%1
goto :EndOfDirLoop

:FirstZdsSystemPath

set response=-stdinc:%1

:EndOfDirLoop
shift
goto :DirLoop

:Usage
echo %progname% is a tool for flexible generation of include path arguments for a
echo variety of different compilers in a variety of compilation environments
echo USAGE: %progname% [-w] [-d] [-s] [-h] ^<compiler-path^> ^<dir1^> [^<dir2^> [^<dir3^> ...]]
echo Where:
echo   ^<compiler-path^>
echo       The full path to your compiler
echo   ^<dir1^> [^<dir2^> [^<dir3^> ...]]
echo       A list of include directories
echo   -w, -d
echo       For compatibility with incdir.sh (ignored)
echo   -s
echo       Generate standard, system header file paths instead of normal user
echo       header file paths.
echo   -h
echo       Shows this help text and exits.
:End
