#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_TAR_TAR_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_TAR_TAR_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::persist::backends::tar {
    template <typename TI>
    static constexpr TI BLOCK_SIZE = 512;
    // UStar tar header structure
    struct header {
        char name[100];
        char mode[8];
        char uid[8];
        char gid[8];
        char size[12];
        char mtime[12];
        char chksum[8];
        char typeflag;
        char linkname[100];
        char magic[6];
        char version[2];
        char uname[32];
        char gname[32];
        char devmajor[8];
        char devminor[8];
        char prefix[155];
        char padding[12];
    };
    static_assert(sizeof(header) == 512);
    template <typename T_TI, typename T_WRITER>
    struct WriterGroupSpecification{
        using TI = T_TI;
        using WRITER = T_WRITER;
        static constexpr TI MAX_PATH_LENGTH = 100;
        static constexpr TI META_SIZE = 2000;
    };
    template <typename T_SPEC>
    struct WriterGroup{
        using SPEC = T_SPEC;
        using TI = typename SPEC::TI;
        using WRITER = typename SPEC::WRITER;
        char path[SPEC::MAX_PATH_LENGTH] = "";
        WRITER* writer = nullptr;
        char meta[SPEC::META_SIZE] = "";
        TI meta_position = 0;
        bool success = true;
    };
    template <typename TI>
    struct BufferData{
        const char* data = nullptr;
        TI size = 0;
    };
    template <typename T_TI, typename T_DATA_BACKEND=BufferData<T_TI>>
    struct ReaderGroupSpecification{
        using TI = T_TI;
        using DATA_BACKEND = T_DATA_BACKEND;
        static constexpr TI MAX_PATH_LENGTH = 100;
    };
    template <typename T_SPEC>
    struct ReaderGroup{
        using SPEC = T_SPEC;
        using TI = typename SPEC::TI;
        char path[SPEC::MAX_PATH_LENGTH] = "";
        typename SPEC::DATA_BACKEND data;
        bool success = true;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




