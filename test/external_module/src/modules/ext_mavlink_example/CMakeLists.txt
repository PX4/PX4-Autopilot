px4_add_external_mavlink_dialect(XML ${CMAKE_CURRENT_SOURCE_DIR}/../../../mavlink/ext_example.xml)

px4_add_module(
	MODULE modules__ext_mavlink_example
	MAIN ext_mavlink_example
	SRCS
		ExtMavlinkExample.cpp
		ExtMavlinkExample.hpp
	EXTERNAL
)

# In-tree libraries are configured after external modules: link by name, not via DEPENDS
target_link_libraries(modules__ext_mavlink_example PRIVATE px4_work_queue)
px4_target_use_external_mavlink_dialect(modules__ext_mavlink_example)
