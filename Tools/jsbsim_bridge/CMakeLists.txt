cmake_minimum_required (VERSION 2.8.11)
project(jsbsim_bridge)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(MAVLink)
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(TinyXML REQUIRED)

message(STATUS "${MAVLINK_INCLUDE_DIRS}")

if (NOT JSBSIM_FOUND)
	if(DEFINED ENV{JSBSIM_ROOT_DIR} )
		set(JSBSIM_ROOT_DIR "$ENV{JSBSIM_ROOT_DIR}" )
	endif()

	find_path(JSBSIM_INCLUDE_DIR
		NAMES
			FGFDMExec.h
		PATHS
			${JSBSIM_ROOT_DIR}/include/JSBSim
			/usr/include/JSBSim
			/usr/local/include/JSBSim
	)

	find_library(JSBSIM_LIBRARY
			NAMES JSBSim
			PATHS
			${JSBSIM_ROOT_DIR}/lib
			/usr/lib/
			/usr/local/lib
	)

if (JSBSIM_INCLUDE_DIR AND JSBSIM_LIBRARY)
	set(JSBSIM_FOUND TRUE)
endif()

if (NOT JSBSIM_FOUND)
	message(FATAL_ERROR "Could not find JSBSIM")
endif()
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_definitions(-DJSBSIM_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(
	-Wno-address-of-packed-member   # MAVLink annoyances
	--warn-uninitialized
)

include_directories(include)

add_executable(jsbsim_bridge
			   src/main.cpp
			   src/configuration_parser.cpp
			   src/jsbsim_bridge.cpp
			   src/geo_mag_declination.cpp
			   src/mavlink_interface.cpp
			   src/actuator_plugin.cpp
			   src/sensor_plugin.cpp
			   src/sensor_airspeed_plugin.cpp
			   src/sensor_baro_plugin.cpp
			   src/sensor_imu_plugin.cpp
			   src/sensor_gps_plugin.cpp
			   src/sensor_mag_plugin.cpp
			   )

target_include_directories(jsbsim_bridge
                BEFORE
				PUBLIC ${MAVLINK_INCLUDE_DIRS} ${JSBSIM_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

target_link_libraries(jsbsim_bridge ${JSBSIM_LIBRARY} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_TIMER_LIBRARY_RELEASE} ${EIGEN_LIBRARIES} ${TinyXML_LIBRARIES})

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
	message(STATUS "catkin ENABLED")
	find_package(catkin REQUIRED)
	if (catkin_FOUND)
		catkin_package()
	else()
		message(FATAL_ERROR "catkin not found")
	endif()

	find_package(catkin REQUIRED COMPONENTS
			roscpp
			rospy
			mavlink
		)

	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES geometric_controller
		CATKIN_DEPENDS roscpp rospy std_msgs
	)

  add_executable(jsbsim_bridge_node
	src/jsbsim_bridge_node.cpp
	src/jsbsim_bridge_ros.cpp
	src/configuration_parser.cpp
	src/jsbsim_bridge.cpp
	src/geo_mag_declination.cpp
	src/mavlink_interface.cpp
	src/actuator_plugin.cpp
	src/sensor_plugin.cpp
	src/sensor_airspeed_plugin.cpp
	src/sensor_baro_plugin.cpp
	src/sensor_imu_plugin.cpp
	src/sensor_gps_plugin.cpp
	src/sensor_mag_plugin.cpp
	)

  target_include_directories(jsbsim_bridge_node
    BEFORE
	PUBLIC ${catkin_INCLUDE_DIRS} ${MAVLINK_INCLUDE_DIRS} ${JSBSIM_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS}
	)

	target_link_libraries(jsbsim_bridge_node ${catkin_LIBRARIES} ${JSBSIM_LIBRARY} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_TIMER_LIBRARY_RELEASE} ${EIGEN_LIBRARIES} ${TinyXML_LIBRARIES}
	)

  else()
  message(STATUS "catkin DISABLED")
endif()
