
# Link against the public stub version of the proprietary fc sensor library
target_link_libraries(px4 PRIVATE
		${PX4_BOARD_DIR}/libfc-sensor-api/build/libfc_sensor.so
		px4_layer
		${module_libraries}
)
