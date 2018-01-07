#!/usr/bin/env bash

make posix_sitl_default
cp build/posix_sitl_default/parameters.xml ../qgroundcontrol/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml
#cp build/posix_sitl_default/airframes.xml ../qgroundcontrol/src/AutoPilotPlugins/PX4/AirframeFactMetaData.xml
