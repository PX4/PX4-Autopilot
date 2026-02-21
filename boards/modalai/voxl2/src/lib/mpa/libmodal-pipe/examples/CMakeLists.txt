

# for every C file make an executable with its own name
file(GLOB SRCS *.c)
foreach(file ${SRCS})
	get_filename_component(name ${file} NAME_WE)
	add_executable(${name} ${file})
	target_link_libraries(${name} LINK_PUBLIC ${LIBNAME})
	## add this target to our global running list of targets
	list(APPEND ALL_TARGETS ${name} )
endforeach(file ${SRCS})


# make sure everything is installed where we want
# LIB_INSTALL_DIR comes from the parent cmake file
install(
	TARGETS			${ALL_TARGETS}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
