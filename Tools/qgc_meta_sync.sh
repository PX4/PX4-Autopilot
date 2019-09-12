#!/usr/bin/env bash

make parameters_metadata
cp parameters.xml ../qgroundcontrol/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml
make airframe_metadata
cp airframes.xml ../qgroundcontrol/src/AutoPilotPlugins/PX4/AirframeFactMetaData.xml
