
# uorb_test
add_executable(uorb_test uORBCommunicator_gtests.cpp
						 uORBCommunicatorMock.cpp
						 uORBCommunicatorMockLoopback.cpp
						 
						 ${PX4_SRC}/modules/uORB/uORB.cpp
						 ${PX4_SRC}/modules/uORB/uORBManager.cpp
						 ${PX4_SRC}/modules/uORB/uORBDevices_posix.cpp
						 
						 ${PX4_SRC}/modules/uORB/uORBUtils.cpp
						 
						 )
add_gtest(uorb_test)