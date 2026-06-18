
#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_POSIX_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_POSIX_H

#include <stdio.h>
#include "tar.h"
#include "../../../utils/string/operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::persist::backends::tar {
    template <typename TI>
    struct PosixFileData{
        FILE *f;
        TI size = 0;
    };
}
namespace rl_tools{
    namespace persist::backends::tar{
        template <typename DEVICE, typename BD_TI, typename READ_OFFSET_TI, typename READ_SIZE_TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool seek(DEVICE& device, PosixFileData<BD_TI>& data_backend, const char* entry_name, READ_OFFSET_TI& entry_offset, READ_SIZE_TI& entry_size){
            using TI = typename DEVICE::index_t;
            if(!utils::assert_exit(device, fseek(data_backend.f, 0, SEEK_SET) == 0, "persist::backends::tar: Failed to seek to start of file")){return false;}
            header h;
            while(fread(&h, sizeof(header), 1, data_backend.f) == 1){
                if(h.name[0] == '\0'){
                    break;
                }
                if (!utils::assert_exit(device, utils::string::compare(h.magic, "ustar", 5), "Warning: Not a UStar format archive or header is corrupted.")){return false;};

                entry_size = utils::string::parse_octal<TI>(h.size, 12);
                if (utils::string::compare(h.name, entry_name, 100)){
                    entry_offset = ftell(data_backend.f);
                    return true;
                }

                size_t padding_size = (BLOCK_SIZE<TI> - (entry_size % BLOCK_SIZE<TI>)) % BLOCK_SIZE<TI>;
                if(fseek(data_backend.f, entry_size + padding_size, SEEK_CUR) != 0){
                    return false;
                }
            }
            return false;
        }
        template <typename DEVICE, typename BD_TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool get(DEVICE& device, PosixFileData<BD_TI>& data_backend, const char* entry_name, char* output_data, typename DEVICE::index_t output_size, typename DEVICE::index_t& read_size){
            using TI = typename DEVICE::index_t;
            if(!utils::assert_exit(device, fseek(data_backend.f, 0, SEEK_SET) == 0, "persist::backends::tar: Failed to seek to start of file")){return false;}
            TI entry_offset;
            if (seek(device, data_backend, entry_name, entry_offset, read_size)){
                if (!utils::assert_exit(device, read_size <= output_size, "persist::backends::tar: Output buffer is too small for the requested entry")){return false;};
                if(fread(output_data, 1, read_size, data_backend.f) != read_size){
                    return false;
                }
                return true;
            }
            return false;
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"
#endif
