#!/usr/bin/env bash

make px4fmu-v4_default
cp build_px4fmu-v4_default/parameters.xml ../qgroundcontrol/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml
cp build_px4fmu-v4_default/airframes.xml ../qgroundcontrol/src/AutoPilotPlugins/PX4/AirframeFactMetaData.xml
