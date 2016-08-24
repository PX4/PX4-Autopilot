# compiler tools
foreach(tool nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo patch grep rm mkdir nm genromfs cp touch make unzip zip sh)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()
