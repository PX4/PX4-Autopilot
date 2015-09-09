#=============================================================================
#
#	px4_create_git_hash_header
#
#	Create a header file containing the git hash of the current tree
#
#	Usage:
#		px4_create_git_hash_header(HEADER ${CMAKE_BUILD_DIR}/git_hash.h)
#
#	Input:
#		HEADER 		: path of the header file to generate
#
#	Example:
#		px4_create_git_hash_header(HEADER ${CMAKE_BUILD_DIR}/git_hash.h)
#
function(px4_create_git_hash_header)
	px4_parse_function_args(
		NAME px4_create_git_hash_header
		ONE_VALUE HEADER 
		REQUIRED HEADER 
		ARGN ${ARGN})
	execute_process(
		COMMAND git log -n 1 --pretty=format:"%H"
		OUTPUT_VARIABLE git_desc
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
	message(STATUS "GIT_DESC = ${git_desc}")
	set(git_desc_short)
	string(SUBSTRING ${git_desc} 1 16 git_desc_short)
	configure_file(${CMAKE_SOURCE_DIR}/cmake/build_git_version.h.in ${HEADER} @ONLY)
endfunction()
