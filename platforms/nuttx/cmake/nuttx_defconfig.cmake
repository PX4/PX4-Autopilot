include_guard(GLOBAL)

# Resolve the NuttX config spec for this .px4board.
#
# A spec is `base` or `base+fragment[+fragment]`. base names a full
# defconfig; each fragment is overlaid last-wins.
#
# CONFIG_BOARD_NUTTX selects the spec. If it is empty, fall back to the
# historical directory-name match so unmigrated boards keep working.

function(px4_nuttx_infer_spec out_var)
	set(_dir ${PX4_BOARD_DIR}/nuttx-config)
	set(_label ${PX4_BOARD_LABEL})

	if(CONFIG_BOARD_NUTTX)
		set(${out_var} "${CONFIG_BOARD_NUTTX}" PARENT_SCOPE)
		return()
	endif()

	if(EXISTS ${_dir}/${_label}.defconfig OR EXISTS ${_dir}/${_label}/defconfig)
		set(${out_var} "${_label}" PARENT_SCOPE)
		return()
	endif()

	if(EXISTS ${_dir}/${_label}.fragment OR EXISTS ${_dir}/${_label}/defconfig.fragment)
		if(EXISTS ${_dir}/default.defconfig OR EXISTS ${_dir}/default/defconfig)
			set(${out_var} "default+${_label}" PARENT_SCOPE)
		else()
			set(${out_var} "nsh+${_label}" PARENT_SCOPE)
		endif()
		return()
	endif()

	if("${_label}" MATCHES "^bootloader" AND (EXISTS ${_dir}/bootloader.defconfig OR EXISTS ${_dir}/bootloader/defconfig))
		set(${out_var} "bootloader" PARENT_SCOPE)
		return()
	endif()

	if(EXISTS ${_dir}/default.defconfig OR EXISTS ${_dir}/default/defconfig)
		set(${out_var} "default" PARENT_SCOPE)
		return()
	endif()

	set(${out_var} "nsh" PARENT_SCOPE)
endfunction()

function(px4_nuttx_find_defconfig name out_var)
	set(_dir ${NUTTX_CONFIG_DIR})
	set(_candidates
		${_dir}/${name}.defconfig
		${_dir}/${name}/defconfig
	)

	if("${name}" STREQUAL "default")
		list(APPEND _candidates ${_dir}/nsh/defconfig)
	endif()

	foreach(_c ${_candidates})
		if(EXISTS ${_c})
			set(${out_var} ${_c} PARENT_SCOPE)
			return()
		endif()
	endforeach()

	message(FATAL_ERROR "NuttX defconfig '${name}' not found under ${_dir} (tried ${name}.defconfig, ${name}/defconfig)")
endfunction()

function(px4_nuttx_find_fragment name out_var)
	set(_dir ${NUTTX_CONFIG_DIR})
	set(_candidates
		${_dir}/${name}.fragment
		${_dir}/${name}/defconfig.fragment
	)

	foreach(_c ${_candidates})
		if(EXISTS ${_c})
			set(${out_var} ${_c} PARENT_SCOPE)
			return()
		endif()
	endforeach()

	message(FATAL_ERROR "NuttX fragment '${name}' not found under ${_dir} (tried ${name}.fragment, ${name}/defconfig.fragment)")
endfunction()

# Parse NUTTX_CONFIG spec, set NUTTX_DEFCONFIG / NUTTX_DEFCONFIG_BASE /
# NUTTX_DEFCONFIG_FRAGMENTS / NUTTX_DEFCONFIG_SAVE_FRAGMENT.
function(px4_nuttx_resolve_defconfig)
	string(REPLACE "+" ";" _parts "${NUTTX_CONFIG}")
	list(LENGTH _parts _n)

	if(_n LESS 1)
		message(FATAL_ERROR "NUTTX_CONFIG spec is empty")
	endif()

	list(GET _parts 0 _base_name)
	list(REMOVE_AT _parts 0)

	px4_nuttx_find_defconfig(${_base_name} _base)
	set(NUTTX_DEFCONFIG_BASE ${_base} CACHE FILEPATH "path to NuttX base defconfig" FORCE)

	set(_frags)
	foreach(_f ${_parts})
		px4_nuttx_find_fragment(${_f} _fp)
		list(APPEND _frags ${_fp})
	endforeach()
	set(NUTTX_DEFCONFIG_FRAGMENTS "${_frags}" CACHE INTERNAL "NuttX defconfig fragments" FORCE)

	set(_depends ${_base})
	if(_frags)
		list(APPEND _depends ${_frags})
	endif()
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_depends})

	if(_frags)
		file(MAKE_DIRECTORY ${PX4_BINARY_DIR}/NuttX)
		file(READ ${_base} _merged)
		foreach(_fp ${_frags})
			file(READ ${_fp} _overlay)
			string(REGEX MATCH "\n$" _nl "${_merged}")
			if(NOT _nl)
				set(_merged "${_merged}\n")
			endif()
			set(_merged "${_merged}${_overlay}")
		endforeach()
		file(WRITE ${PX4_BINARY_DIR}/NuttX/merged_defconfig.tmp "${_merged}")
		execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
			${PX4_BINARY_DIR}/NuttX/merged_defconfig.tmp ${PX4_BINARY_DIR}/NuttX/merged_defconfig)
		execute_process(COMMAND ${CMAKE_COMMAND} -E remove -f ${PX4_BINARY_DIR}/NuttX/merged_defconfig.tmp)
		set(NUTTX_DEFCONFIG ${PX4_BINARY_DIR}/NuttX/merged_defconfig CACHE FILEPATH "path to defconfig" FORCE)
	else()
		set(NUTTX_DEFCONFIG ${_base} CACHE FILEPATH "path to defconfig" FORCE)
	endif()

	list(LENGTH _frags _nfrags)
	if(_nfrags EQUAL 1)
		list(GET _frags 0 _save)
		set(NUTTX_DEFCONFIG_SAVE_FRAGMENT ${_save} CACHE FILEPATH "fragment to rewrite on savedefconfig" FORCE)
	else()
		set(NUTTX_DEFCONFIG_SAVE_FRAGMENT "" CACHE FILEPATH "fragment to rewrite on savedefconfig" FORCE)
	endif()
endfunction()

function(px4_nuttx_check_fragments inflated_path)
	if(NOT NUTTX_DEFCONFIG_FRAGMENTS)
		return()
	endif()

	file(READ ${inflated_path} _inflated)
	set(_padded "\n${_inflated}\n")

	foreach(_fp ${NUTTX_DEFCONFIG_FRAGMENTS})
		file(STRINGS ${_fp} _lines)
		foreach(_line ${_lines})
			if(_line MATCHES "^CONFIG_")
				string(FIND "${_padded}" "\n${_line}\n" _found)
				if(_found EQUAL -1)
					message(FATAL_ERROR "NuttX fragment option '${_line}' from ${_fp} did not survive olddefconfig for ${NUTTX_CONFIG}")
				endif()
			elseif(_line MATCHES "^# (CONFIG_[^ ]+) is not set$")
				set(_sym ${CMAKE_MATCH_1})
				string(FIND "${_padded}" "\n${_sym}=" _found)
				if(NOT _found EQUAL -1)
					message(FATAL_ERROR "NuttX fragment unset '${_line}' from ${_fp} did not survive olddefconfig for ${NUTTX_CONFIG}")
				endif()
			endif()
		endforeach()
	endforeach()
endfunction()
