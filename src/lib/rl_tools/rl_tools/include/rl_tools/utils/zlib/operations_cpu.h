#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_ZLIB_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_ZLIB_OPERATIONS_CPU_H

#ifdef RL_TOOLS_ENABLE_ZLIB
#include <zlib.h>
#include <cstring>
#endif


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
#ifdef RL_TOOLS_ENABLE_ZLIB
    bool compress_zlib(const std::string& input, std::vector<uint8_t>& compressed_output) {
        z_stream zs;
        memset(&zs, 0, sizeof(zs));

        if (deflateInit2(&zs, Z_DEFAULT_COMPRESSION, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY) != Z_OK) {
            return false;
        }
        zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(input.data()));
        zs.avail_in = input.size();
        int ret;
        std::vector<uint8_t> outbuffer(32768);
        do {
            zs.next_out = reinterpret_cast<Bytef*>(outbuffer.data());
            zs.avail_out = outbuffer.size();
            ret = deflate(&zs, Z_FINISH);
            if (ret != Z_STREAM_ERROR) {
                size_t have = outbuffer.size() - zs.avail_out;
                compressed_output.insert(compressed_output.end(), outbuffer.begin(), outbuffer.begin() + have);
            }
        } while (zs.avail_out == 0);
        deflateEnd(&zs);
        if (ret != Z_STREAM_END) {
            return false;
        }
        return true;
    }
#endif
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

