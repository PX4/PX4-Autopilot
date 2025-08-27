px4_add_library(vehicle_gnss_heading
	VehicleGNSSHeading.cpp
	VehicleGNSSHeading.hpp
)
target_link_libraries(vehicle_gnss_heading
	PRIVATE
		px4_work_queue
)
