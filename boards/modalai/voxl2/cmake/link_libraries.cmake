
# libfc_sensor.so is provided in the Docker build environment
target_link_libraries(px4 PRIVATE
		/home/libfc_sensor.so
		px4_layer
		${module_libraries}
)
