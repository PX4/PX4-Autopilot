
if (${OS} MATCHES "qurt")

	add_subdirectory(adsp)
else()

	include(fastrpc)

	FASTRPC_STUB_GEN(px4muorb.idl)

	add_library(muorb px4muorb_stub.c)
	add_dependencies(muorb generate_px4muorb_stubs)
	target_compile_options(muorb PRIVATE -Wno-missing-declarations -Wno-missing-prototypes)
	target_include_directories(muorb
		PRIVATE ${FASTRPC_DSP_INCLUDES}
		PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
	)

	target_link_libraries(muorb PRIVATE ${FASTRPC_ARM_LIBS})

	add_subdirectory(krait)
endif()
