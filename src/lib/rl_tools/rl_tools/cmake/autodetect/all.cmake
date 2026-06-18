# tiers of optional dependencies
# tier 0: none
# tier 1: backend (MKL, OpenBLAS, Accelerate)
# tier 2: json, hdf5
# tier 3: tensorboard
# tier 4: cuda
# tier 5: cli11
include(cmake/autodetect/tier1-blas.cmake)
include(cmake/autodetect/tier2-json-hdf5-zlib.cmake)
include(cmake/autodetect/tier3-tensorboard.cmake)
include(cmake/autodetect/tier4-cuda.cmake)
include(cmake/autodetect/tier5-cli11.cmake)
include(cmake/autodetect/git-hash.cmake)
include(cmake/autodetect/git-diff.cmake)
