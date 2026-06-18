#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_HDF5_HDF5)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_HDF5_HDF5

#include <highfive/H5File.hpp>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::persist::backends::hdf5{
    template <typename T=void>
    struct GroupSpecification{
    };
    template <typename SPEC = GroupSpecification<>>
    struct Group{
        HighFive::Group group;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif