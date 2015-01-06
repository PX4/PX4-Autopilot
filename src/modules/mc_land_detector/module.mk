#
# Multirotor land detector
#

MODULE_COMMAND	= mc_land_detector

SRCS		= mc_land_detector_main.cpp \
			  MulticopterLandDetector.cpp

EXTRACXXFLAGS   = -Weffc++
