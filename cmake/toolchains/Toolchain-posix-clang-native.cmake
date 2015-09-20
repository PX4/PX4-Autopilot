
set(WARNINGS
    -Wall
    -Wextra
    -Wdouble-promotion
    -Wshadow
    -Wfloat-equal
    -Wframe-larger-than=1024
    -Wpointer-arith
    -Wlogical-op
    -Wmissing-declarations
    -Wno-unused-parameter
    -Werror=format-security
    -Werror=array-bounds
    -Wfatal-errors
    -Wformat=1
    -Werror=unused-but-set-variable
    -Werror=unused-variable
    -Werror=double-promotion
    -Werror=reorder
    -Werror=uninitialized
    -Werror=init-self
    -Werror=return-type
    -Werror=deprecated
    -Werror=unused-private-field
    -Wno-packed
    -Wno-frame-larger-than=
    #-Wcast-qual  - generates spurious noreturn attribute warnings,
    #               try again later
    #-Wconversion - would be nice, but too many "risky-but-safe"
    #               conversions in the code
    #-Wcast-align - would help catch bad casts in some cases,
    #               but generates too many false positives
    )

set(OPT_FLAGS
	-Os -g3
	)

#=============================================================================
#		c flags
#
set(C_WARNINGS
	-Wbad-function-cast
	-Wstrict-prototypes
	-Wold-style-declaration
	-Wmissing-parameter-type
	-Wmissing-prototypes
	-Wnested-externs
	)
set(C_FLAGS
	-std=gnu99
	-fno-common
	)

#=============================================================================
#		cxx flags
#
set(CXX_WARNINGS
	-Wno-missing-field-initializers
	)
set(CXX_FLAGS
	-fno-exceptions
	-fno-rtti
	-std=gnu++0x
	-fno-threadsafe-statics
	-DCONFIG_WCHAR_BUILTIN
	-D__CUSTOM_FILE_IO__
	)

#=============================================================================
#		ld flags
#
set(LD_FLAGS
	-Wl,--warn-common
	-Wl,--gc-sections
	)

#=============================================================================
#		misc flags
#
set(VISIBILITY_FLAGS
	-fvisibility=hidden
	"-include ${CMAKE_SOURCE_DIR}/src/include/visibility.h"
	)
set(EXE_LINK_FLAGS)

