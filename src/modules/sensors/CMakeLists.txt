############################################################################
#
#   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(data_validator)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	add_subdirectory(vehicle_acceleration)
endif()

add_subdirectory(vehicle_imu)

if(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	add_subdirectory(vehicle_air_data)
endif()

if(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	add_subdirectory(vehicle_angular_velocity)
endif()

if(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	add_subdirectory(vehicle_gps_position)
endif()

if(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	add_subdirectory(vehicle_magnetometer)
endif()

if(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
	add_subdirectory(vehicle_optical_flow)
endif()

px4_add_module(
	MODULE modules__sensors
	MAIN sensors
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
	SRCS
		sensors.cpp
		voted_sensors_update.cpp
		voted_sensors_update.h
		Integrator.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		conversion
		data_validator
		mathlib
		sensor_calibration
		vehicle_imu
	)

if(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	target_link_libraries(modules__sensors PRIVATE vehicle_acceleration)
endif()

if(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	target_link_libraries(modules__sensors PRIVATE vehicle_air_data)
endif()

if(CONFIG_SENSORS_VEHICLE_AIRSPEED)
	target_link_libraries(modules__sensors PRIVATE airspeed)
endif()

if(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	target_link_libraries(modules__sensors PRIVATE vehicle_angular_velocity)
endif()

if(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	target_link_libraries(modules__sensors PRIVATE vehicle_gps_position)
endif()

if(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	target_link_libraries(modules__sensors PRIVATE vehicle_magnetometer)
endif()

if(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
	target_link_libraries(modules__sensors PRIVATE vehicle_optical_flow)
endif()
