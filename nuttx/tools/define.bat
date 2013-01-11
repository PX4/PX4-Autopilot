@echo off

rem tools/define.bat
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

rem Handle command line options
rem [-h] <compiler-path> <def1> [-val <val1>] [<def2> [-val <val2>] [<def3> [-val <val3>] ...]]
rem [-w] [-d] ignored for compatibility with define.sh

set progname=%0

:ArgLoop
if "%1"=="-d" goto :NextArg
if "%1"=="-w" goto :NextArg
if "%1"=="-h" goto :ShowUsage

goto :CheckCompilerPath

:NextArg
shift
goto :ArgLoop

:CheckCompilerPath

if "%1"=="" (
  echo Missing compiler path
  goto :ShowUsage
)

set ccpath=%1
shift

set compiler=
for /F %%i in ("%ccpath%") do set compiler=%%~ni

if "%1"=="" (
  echo Missing definition list
  goto :ShowUsage
)

rem Check for some well known, non-GCC Windows native tools that require
rem a special output format as well as special paths

:GetFormat
set fmt=std
if "%compiler%"=="ez8cc" goto :SetZdsFormt
if "%compiler%"=="zneocc" goto :SetZdsFormt
if "%compiler%"=="ez80cc" goto :SetZdsFormt
goto :ProcessDefinitions

:SetZdsFormt
set fmt=zds

rem Now process each directory in the directory list

:ProcessDefinitions
set response=

:DefinitionLoop
if "%1"=="" goto :Done

set varname=%1
shift

rem Handle the output depending on if there is a value for the variable or not

if "%1"=="-val" goto :GetValue

rem Handle the output using the selected format

:NoValue
if "%fmt%"=="zds" goto :NoValueZDS

:NoValueStandard
rem Treat the first definition differently

if "%response%"=="" (
  set response=-D%varname%
  goto :DefinitionLoop
)

set response=%response% -D%varname%
goto :DefinitionLoop

:NoValueZDS
rem Treat the first definition differently

if "%response%"=="" (
  set response=-define:%varname%
  goto :DefinitionLoop
)

set response=%response% -define:%varname%
goto :DefinitionLoop

rem Get value following the variable name

:GetValue
shift
set varvalue=%1
shift

rem Handle the output using the selected format

if "%fmt%"=="zds" goto :ValueZDS

:ValueStandard
rem Treat the first definition differently

if "%response%"=="" (
  set response=-D%varname%=%varvalue%
  goto :DefinitionLoop
)

set response=%response% -D%varname%=%varvalue%
goto :DefinitionLoop

:ValueZds
rem Treat the first definition differently

if "%response%"=="" (
  set response=-define:%varname%=%varvalue%
  goto :DefinitionLoop
)

set response=%response% -define:%varname%=%varvalue%
goto :DefinitionLoop

:Done
echo %response%
goto :End

:ShowUsage
echo %progname% is a tool for flexible generation of command line pre-processor
echo definitions arguments for a variety of diffent ccpaths in a variety of
echo compilation environments"
echo USAGE:%progname% [-h] ^<compiler-path^> [-val ^<^val1^>] [^<def2^> [-val ^<val2^>] [^<def3^> [-val ^<val3^>] ...]]
echo Where:"
echo 	^<compiler-path^>
echo 		The full path to your ccpath
echo 	^<def1^> ^<def2^> ^<def3^> ...
echo 		A list of pre-preprocesser variable names to be defined.
echo 	[-val ^<val1^>] [-val ^<val2^>] [-val ^<val3^>] ...
echo 		optional values to be assigned to each pre-processor variable.
echo 		If not supplied, the variable will be defined with no explicit value.
echo 	-h
echo 		Show this text and exit

:End
