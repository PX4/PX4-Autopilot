set(directories
	./external_lgpl
	./geo
	./launchdetection
	./mathlib/math/filter
	./mathlib
	./rc
	./geo_lookup
	./ecl
	./conversion
	)

foreach(directory ${directories})
	add_subdirectory(${directory} EXCLUDE_FROM_ALL)
endforeach()
