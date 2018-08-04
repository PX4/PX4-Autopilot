#! /bin/sh

JLinkGDBServerCLExe -startserver -USB -device Cortex-M4 -if SWD -speed auto > jlink.log &
