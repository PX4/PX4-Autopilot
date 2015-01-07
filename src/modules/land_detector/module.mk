#
# Land detector
#

MODULE_COMMAND	= land_detector

SRCS		= land_detector_main.cpp \
              LandDetector.cpp \
			  MulticopterLandDetector.cpp \
              FixedwingLandDetector.cpp

EXTRACXXFLAGS   = -Weffc++ -Os
