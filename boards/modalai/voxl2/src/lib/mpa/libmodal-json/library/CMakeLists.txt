
# Build lib from all source files
file(GLOB all_src_files src/*.c)
add_library(${LIBNAME} SHARED ${all_src_files})
target_link_libraries(${LIBNAME} LINK_PUBLIC m)
target_include_directories(${LIBNAME} PUBLIC include )

# make the include directory public for install
file(GLOB LIB_HEADERS include/*.h)
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "${LIB_HEADERS}")

# make sure everything is installed where we want
# LIB_INSTALL_DIR comes from the parent cmake file
install(
	TARGETS			${LIBNAME}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
