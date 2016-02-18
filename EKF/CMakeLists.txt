cmake_minimum_required(VERSION 2.8)

project (ECL CXX)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CURRENT_SOURCE_DIR EKF)
set(CMAKE_CXX_FLAGS "-DPOSIX_SHARED")
set (EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3/")

IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

include_directories(
	./
	../
	../matrix
	EIGEN3_INCLUDE_DIR
)
set(SRCS 
	estimator_interface.cpp
	ekf.cpp
	ekf_helper.cpp
	covariance.cpp
	vel_pos_fusion.cpp
	mag_fusion.cpp
	gps_checks.cpp
	control.cpp
	geo.cpp
	mathlib.cpp
)
add_definitions(-std=c++11)
add_library(ecl SHARED ${SRCS})