
# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message(STATUS "Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)

	message(STATUS "Building without lockstep for replay")
	set(ENABLE_LOCKSTEP_SCHEDULER no)
elseif(CMAKE_BUILD_TYPE STREQUAL FuzzTesting)
	set(ENABLE_LOCKSTEP_SCHEDULER no)
else()
	set(ENABLE_LOCKSTEP_SCHEDULER yes)
endif()
