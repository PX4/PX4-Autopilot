#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_GENERIC)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_TAR_OPERATIONS_GENERIC

#include "tar.h"
#include "../../../utils/string/operations_generic.h"


/*
import tarfile;
import numpy as np;
f = tarfile.open("test_persist_backends_tar_dense_layer.tar", "r");
meta = dict([l.split(": ") for l in f.extractfile("weights/parameters/meta").read().decode("utf-8").split("\n")][:-1]);
data = np.frombuffer(f.extractfile("weights/parameters/data").read(), dtype=np.float32 if meta["dtype"] == "float32" else np.float64);
data = data.reshape([int(meta[f"dim_{i}"]) for i in range(int(meta["num_dims"]))]) if meta["type"] == "tensor" else data.reshape((int(meta["rows"]), int(meta["cols"])));
print(data)
 */

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace persist::backends::tar{
        template <typename TI>
        TI calculate_checksum(const header& header) {
            const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&header);
            // Sum all bytes, treating the checksum field as spaces
            TI sum = 0;
            // Sum bytes before checksum field (0 to 147)
            for (TI i = 0; i < 148; i++) {
                sum += bytes[i];
            }
            // Add 8 spaces for the checksum field (148 to 155)
            sum += (' ' * 8);
            // Sum bytes after checksum field (156 to BLOCK_SIZE)
            for (TI i = 156; i < BLOCK_SIZE<TI>; i++) {
                sum += bytes[i];
            }
            return sum;
        }
        template <typename T>
        T min(T a, T b) {
            return a < b ? a : b;
        }
        template <typename DEVICE, typename TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool seek_in_metadata(DEVICE& device, const char* metadata, TI metadata_size, const char* key, TI& position, TI& value_len) {
            // this assumes that key is well formed (is terminated)
            constexpr TI MAX_KEY_LENGTH = 100;
            TI key_len = utils::string::length(key, MAX_KEY_LENGTH);
            if (!utils::assert_exit(device, key_len < MAX_KEY_LENGTH, "persist::backends::tar: Key is too long")){return false;};
            if (key_len >= MAX_KEY_LENGTH){
                return false;
            }
            if (!utils::assert_exit(device, key_len + 2 < metadata_size, "persist::backends::tar: Key is longer than metadata size")){return false;};
            if (key_len + 2 >= metadata_size){
                return false;
            }
            bool previous_was_newline = true;
            if (!utils::assert_exit(device, metadata_size >= 2, "persist::backends::tar::seek_in_metadata: metadata_size too small")){return false;};
            for (position = 0; position < metadata_size-key_len-2; position++){
                if (previous_was_newline && utils::string::compare<TI>(metadata + position, key, key_len) && metadata[position + key_len] == ':'){
                    position += key_len + 2;
                    value_len = 0;
                    while (position + value_len < metadata_size && metadata[position+value_len] != '\n'){
                        value_len++;
                    }
                    return true;
                }
                previous_was_newline = (metadata[position] == '\n');
            }
            return false; // Key not found
        }
        template <typename DEVICE, typename WRITER, typename TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool write_entry(DEVICE& device, WRITER& writer, const char* entry_name, const char* data, TI data_size) {
            constexpr TI MAX_KEY_LENGTH = 100;
            TI key_length = utils::string::length<TI>(entry_name, MAX_KEY_LENGTH-1);
            utils::assert_exit(device, key_length < MAX_KEY_LENGTH, "persist::backends::tar: Entry name is too long");
            if (key_length >= MAX_KEY_LENGTH){
                return false;
            }

            header header{};

            utils::string::copy(header.name, entry_name, 99);
            utils::string::format_octal<unsigned int, TI>(header.mode, sizeof(header.mode), 0644); // Octal permissions
            utils::string::format_octal<unsigned int, TI>(header.uid, sizeof(header.uid), 1000);
            utils::string::format_octal<unsigned int, TI>(header.gid, sizeof(header.gid), 1000);
            utils::string::format_octal<unsigned long long, TI>(header.size, sizeof(header.size), (unsigned long long)data_size);
            utils::string::format_octal<unsigned long, TI>(header.mtime, sizeof(header.mtime), 0); // Using 0 for timestamp (epoch)
            header.typeflag = '0'; // Regular file
            utils::string::memcpy<TI>(header.magic, "ustar", 5);
            utils::string::memcpy<TI>(header.version, "00", 2);
            utils::string::copy(header.uname, "user", 31);
            utils::string::copy(header.gname, "group", 31);

            unsigned int chksum = calculate_checksum<TI>(header);
            utils::string::format_octal<unsigned int, TI>(header.chksum, sizeof(header.chksum), chksum);

            write(device, writer, reinterpret_cast<const char*>(&header), BLOCK_SIZE<TI>);
            write(device, writer, data, data_size);

            TI padding_size = (BLOCK_SIZE<TI> - (data_size % BLOCK_SIZE<TI>)) % BLOCK_SIZE<TI>;
            if (padding_size > 0) {
                const char padding[1] = {0};
                for (TI i = 0; i < padding_size; i++){
                    write(device, writer, padding, 1);
                }
            }
            return true;
        }

        template <typename DEVICE, typename WRITER>
        RL_TOOLS_FUNCTION_PLACEMENT void finalize(DEVICE& device, WRITER& writer) {
            // Write two empty blocks to signify the end of the archive
            using TI = typename DEVICE::index_t;
            const char padding[1] = {0};
            for (TI i = 0; i < BLOCK_SIZE<TI>*2; i++){
                write(device, writer, padding, 1);
            }
        }

        template <typename DEVICE, typename BD_TI, typename READ_OFFSET_TI, typename READ_SIZE_TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool seek(DEVICE& device, BufferData<BD_TI>& data_backend, const char* entry_name, READ_OFFSET_TI& entry_offset, READ_SIZE_TI& entry_size){
            // assumptions: entry_name is null-terminated or at least 100 characters long
            // assumption: length of tar_data is correct
            using TI = typename DEVICE::index_t;
            char* ptr = const_cast<char*>(data_backend.data);
            entry_offset = 0;
            while (ptr <= data_backend.data + data_backend.size - BLOCK_SIZE<TI>) {
                header* h = reinterpret_cast<header*>(ptr);
                ptr += BLOCK_SIZE<TI>;
                entry_offset += BLOCK_SIZE<TI>;

                // An all-zero block marks the end of the archive
                if (h->name[0] == '\0') {
                    break;
                }

                if (!utils::assert_exit(device, utils::string::compare(h->magic, "ustar", 5), "Warning: Not a UStar format archive or header is corrupted.")){return false;};

                entry_size = utils::string::parse_octal<TI>(h->size, 12);
                if (utils::string::compare(h->name, entry_name, 100)){
                    if (!utils::assert_exit(device, ptr + entry_size <= data_backend.data + data_backend.size, "persist::backends::tar: entry size goes beyond tar data size")){return false;};
                    return true;
                }
                ptr += entry_size;
                entry_offset += entry_size;

                size_t padding_size = (BLOCK_SIZE<TI> - (entry_size % BLOCK_SIZE<TI>)) % BLOCK_SIZE<TI>;
                if (padding_size > 0) {
                    ptr += padding_size;
                    entry_offset += padding_size;
                }
            }
            return false;
        }

        template <typename DEVICE, typename BD_TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool get(DEVICE& device, BufferData<BD_TI>& data_backend, const char* entry_name, char* output_data, typename DEVICE::index_t output_size, typename DEVICE::index_t& read_size){
            // assumptions: entry_name is null-terminated or at least 100 characters long
            // assumption: length of tar_data is correct
            using TI = typename DEVICE::index_t;
            TI entry_offset;
            if (seek(device, data_backend, entry_name, entry_offset, read_size)){
                if (!utils::assert_exit(device, read_size <= output_size, "persist::backends::tar: Output buffer is too small for the requested entry")){return false;};
                char* ptr = const_cast<char*>(data_backend.data) + entry_offset;
                utils::string::memcpy<TI>(output_data, ptr, read_size);
                return true;
            }
            return false;
        }
        namespace containers::tensor{
            template <typename SPEC, typename TI = typename SPEC::TI, TI METADATA_SIZE, TI DIM = 0>
            RL_TOOLS_FUNCTION_PLACEMENT void dim_helper(char* metadata, TI& metadata_position){
                if constexpr(DIM < SPEC::SHAPE::LENGTH){
                    constexpr TI DIM_KEY_LENGTH = 64;
                    constexpr TI DIM_VALUE_LENGTH = 16;
                    char dim_key[DIM_KEY_LENGTH];
                    char dim_value[DIM_VALUE_LENGTH];
                    TI pos = 0;
                    pos += utils::string::copy(dim_key, "dim_", DIM_KEY_LENGTH);
                    pos += utils::string::int_to_string<TI, TI>(dim_key + pos, DIM_KEY_LENGTH - pos - 1, DIM);
                    pos += utils::string::copy(dim_key + pos, ": ", DIM_KEY_LENGTH - pos - 1);
                    metadata_position += utils::string::copy(metadata + metadata_position, dim_key, METADATA_SIZE - metadata_position - 1);
                    utils::string::int_to_string<TI, TI>(dim_value, DIM_VALUE_LENGTH-1, SPEC::SHAPE::template GET<DIM>);
                    metadata_position += utils::string::copy<TI>(metadata + metadata_position, dim_value, METADATA_SIZE - metadata_position - 1);
                    metadata_position += utils::string::copy(metadata + metadata_position, "\n", METADATA_SIZE - metadata_position - 1);
                    dim_helper<SPEC, TI, METADATA_SIZE, DIM + 1>(metadata, metadata_position);
                }
            }
            template <typename DEVICE, typename SPEC, typename TI = typename SPEC::TI, TI METADATA_SIZE, TI DIM = 0>
            RL_TOOLS_FUNCTION_PLACEMENT bool dim_helper_read(DEVICE& device, char* metadata){
                static_assert(SPEC::SHAPE::LENGTH <= 9, "Only tensors with up to 9 dimensions are supported for now");
                char key[] = "dim_0";
                key[4] = '0' + DIM;
                if constexpr(DIM < SPEC::SHAPE::LENGTH){
                    TI type_position;
                    TI type_value_length;
                    utils::assert_exit(device, persist::backends::tar::seek_in_metadata(device, metadata, METADATA_SIZE, key, type_position, type_value_length), "persist::backends::tar: 'type' not found in metadata");
                    TI value = utils::string::string_to_int<TI>(metadata + type_position, type_value_length);
                    if (!utils::assert_exit(device, value == SPEC::SHAPE::template GET<DIM>, "persist::backends::tar: Dimension mismatch in metadata")){return false;};
                    return dim_helper_read<DEVICE, SPEC, TI, METADATA_SIZE, DIM + 1>(device, metadata);
                }
                else{
                    return true;
                }
            }

        }
        namespace containers::matrix{
            template <typename SPEC, typename TI = typename SPEC::TI, TI METADATA_SIZE>
            RL_TOOLS_FUNCTION_PLACEMENT void write_metadata(char* metadata, TI& metadata_position){
                metadata_position += utils::string::copy(metadata + metadata_position, "rows: ", METADATA_SIZE - metadata_position);
                char rows_str[16];
                utils::string::int_to_string<TI, TI>(rows_str, 16, SPEC::ROWS);
                metadata_position += utils::string::copy(metadata + metadata_position, rows_str, METADATA_SIZE - metadata_position);
                metadata_position += utils::string::copy(metadata + metadata_position, "\n", METADATA_SIZE - metadata_position);

                metadata_position += utils::string::copy(metadata + metadata_position, "cols: ", METADATA_SIZE - metadata_position);
                char cols_str[16];
                utils::string::int_to_string<TI, TI>(cols_str, 16, SPEC::COLS);
                metadata_position += utils::string::copy(metadata + metadata_position, cols_str, METADATA_SIZE - metadata_position);
                metadata_position += utils::string::copy(metadata + metadata_position, "\n", METADATA_SIZE - metadata_position);
            }
            template <typename DEVICE, typename SPEC, typename TI = typename SPEC::TI, TI METADATA_SIZE>
            RL_TOOLS_FUNCTION_PLACEMENT void read_metadata(DEVICE& device, char* metadata){
                TI rows_position;
                TI rows_value_length;
                utils::assert_exit(device, persist::backends::tar::seek_in_metadata(device, metadata, METADATA_SIZE, "rows", rows_position, rows_value_length), "persist::backends::tar: 'rows' not found in metadata");
                TI rows_value = utils::string::string_to_int<TI>(metadata + rows_position, rows_value_length);
                utils::assert_exit(device, rows_value == SPEC::ROWS, "persist::backends::tar: Rows mismatch in metadata");

                TI cols_position;
                TI cols_value_length;
                utils::assert_exit(device, persist::backends::tar::seek_in_metadata(device, metadata, METADATA_SIZE, "cols", cols_position, cols_value_length), "persist::backends::tar: 'cols' not found in metadata");
                TI cols_value = utils::string::string_to_int<TI>(metadata + cols_position, cols_value_length);
                utils::assert_exit(device, cols_value == SPEC::COLS, "persist::backends::tar: Cols mismatch in metadata");
            }

        }
    }

    namespace persist::backends::tar{
        template <typename T>
        struct Optional{
            T value;
            bool set;
        };
        template<typename DEVICE, typename GROUP>
        GROUP create_group(DEVICE& device, GROUP& group, const char* name) {
            // assumes name to be 100 characters or null terminated
            using TI = typename DEVICE::index_t;
            using GROUP_SPEC = typename GROUP::SPEC;
            GROUP new_group = group;
            new_group.success = true;
            utils::string::copy(new_group.path, group.path, GROUP_SPEC::MAX_PATH_LENGTH);
            TI group_path_length = utils::string::length(new_group.path, GROUP_SPEC::MAX_PATH_LENGTH);
            TI name_length = utils::string::length(name, GROUP_SPEC::MAX_PATH_LENGTH);
            if (!(new_group.success = utils::assert_exit(device, group_path_length + 1 + name_length <= GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Group path and name exceed maximum length"))){return new_group;};
            TI current_position = group_path_length;
            if (group_path_length > 0){
                new_group.path[group_path_length] = '/';
                current_position += 1;
            }
            utils::string::copy(new_group.path + current_position, name, GROUP_SPEC::MAX_PATH_LENGTH - group_path_length - 1);
            return new_group;
        }
    }

    template<typename DEVICE, typename GROUP_SPEC>
    persist::backends::tar::WriterGroup<GROUP_SPEC> create_group(DEVICE& device, persist::backends::tar::WriterGroup<GROUP_SPEC>& group, const char* name){
        auto new_group = persist::backends::tar::create_group(device, group, name);
        new_group.meta[0] = '\0';
        new_group.meta_position = 0;
        return new_group;
    }
    template<typename DEVICE, typename GROUP_SPEC>
    persist::backends::tar::ReaderGroup<GROUP_SPEC> create_group(DEVICE& device, persist::backends::tar::ReaderGroup<GROUP_SPEC>& group, const char* name){
        return persist::backends::tar::create_group(device, group, name);
    }

    template<typename DEVICE, typename GROUP_SPEC>
    persist::backends::tar::WriterGroup<GROUP_SPEC> get_group(DEVICE& device, persist::backends::tar::WriterGroup<GROUP_SPEC>& group, const char* name){
        return persist::backends::tar::create_group(device, group, name);
    }
    template<typename DEVICE, typename GROUP_SPEC>
    persist::backends::tar::ReaderGroup<GROUP_SPEC> get_group(DEVICE& device, persist::backends::tar::ReaderGroup<GROUP_SPEC>& group, const char* name){
        return persist::backends::tar::create_group(device, group, name);
    }
    namespace persist::backends::tar{
        template<typename DEVICE, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void finish_set_attribute(DEVICE& device, persist::backends::tar::WriterGroup<SPEC>& group, const char* name, const char* value){
            using TI = typename DEVICE::index_t;
            utils::assert_exit(device, group.meta_position + utils::string::length(value, SPEC::META_SIZE-1) + 1 < SPEC::META_SIZE, "persist::backends::tar: Metadata size exceeded");
            group.meta_position += utils::string::copy(group.meta + group.meta_position, value, SPEC::META_SIZE - group.meta_position);
            group.meta_position += utils::string::copy(group.meta + group.meta_position, "\n", SPEC::META_SIZE - group.meta_position);
        }
        template<typename DEVICE, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void finish_set_attribute(DEVICE& device, persist::backends::tar::WriterGroup<SPEC>& group, const char* name, long int value){
            using TI = typename DEVICE::index_t;
            constexpr TI BUFFER_SIZE = 32;
            char value_str[BUFFER_SIZE];
            utils::string::int_to_string<long int, TI>(value_str, BUFFER_SIZE, value);
            utils::assert_exit(device, group.meta_position + utils::string::length(value_str, SPEC::META_SIZE-1) + 1 < SPEC::META_SIZE, "persist::backends::tar: Metadata size exceeded");
            group.meta_position += utils::string::copy(group.meta + group.meta_position, value_str, SPEC::META_SIZE - group.meta_position);
            group.meta_position += utils::string::copy(group.meta + group.meta_position, "\n", SPEC::META_SIZE - group.meta_position);
        }
    }

    template<typename TYPE, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void set_attribute(DEVICE& device, persist::backends::tar::WriterGroup<SPEC>& group, const char* name, TYPE value) {
        using TI = typename DEVICE::index_t;
        utils::assert_exit(device, group.meta_position + utils::string::length(name, SPEC::META_SIZE-1) + 2 < SPEC::META_SIZE, "persist::backends::tar: Metadata size exceeded");
        group.meta_position += utils::string::copy(group.meta + group.meta_position, name, SPEC::META_SIZE - group.meta_position);
        group.meta_position += utils::string::copy(group.meta + group.meta_position, ": ", SPEC::META_SIZE - group.meta_position);

        persist::backends::tar::finish_set_attribute(device, group, name, value);

    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void write_attributes(DEVICE& device, persist::backends::tar::WriterGroup<SPEC>& group){
        using TI = typename DEVICE::index_t;
        char group_path[SPEC::MAX_PATH_LENGTH];
        utils::string::copy(group_path, group.path, SPEC::MAX_PATH_LENGTH-1);
        TI group_path_length = utils::string::length(group_path, SPEC::MAX_PATH_LENGTH-1);
        utils::assert_exit(device, group_path_length + sizeof("meta") < SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Group path and name exceed maximum length");
        TI current_position = group_path_length;
        if (group_path_length > 0){
            group_path[group_path_length] = '/';
            current_position += 1;
        }
        utils::string::copy(group_path + current_position, "meta", SPEC::MAX_PATH_LENGTH - group_path_length - 1);
        write_entry(device, *group.writer, group_path, group.meta, group.meta_position);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT bool group_exists(DEVICE& device, persist::backends::tar::ReaderGroup<SPEC>& group, const char* name) {
        using TI = typename DEVICE::index_t;
        char* output_data;
        TI read_size;
        return get(device, group.data, group.size, name, output_data, read_size);
    }
    template<typename TYPE, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void get_attribute(DEVICE& device, persist::backends::tar::ReaderGroup<SPEC>& group, const char* name, char* output, typename DEVICE::index_t output_size){
        using TI = typename DEVICE::index_t;
        char group_path[SPEC::MAX_PATH_LENGTH];
        utils::string::copy(group_path, group.path, SPEC::MAX_PATH_LENGTH);
        TI group_path_length = utils::string::length(group_path, SPEC::MAX_PATH_LENGTH+1);
        utils::assert_exit(device, group_path_length + sizeof("meta") + 2 < SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Group path and name exceed maximum length");
        TI current_position = group_path_length;
        if (group_path_length > 0 && group_path_length + sizeof("meta") + 2 < SPEC::MAX_PATH_LENGTH){
            group_path[group_path_length] = '/';
            current_position += 1;
        }
        utils::string::copy(group_path + current_position, "meta", SPEC::MAX_PATH_LENGTH - group_path_length - 1);
        constexpr TI METADATA_SIZE = 100;
        char metadata[METADATA_SIZE];
        TI read_size = 0;
        utils::assert_exit(device, persist::backends::tar::get(device, group.data, group_path, metadata, METADATA_SIZE, read_size), "persist::backends::tar: Failed to read metadata entry from tar archive");
        using TI = typename DEVICE::index_t;
        TI position;
        TI value_length = 0;
        utils::assert_exit(device, persist::backends::tar::seek_in_metadata(device, metadata, METADATA_SIZE, name, position, value_length), "persist::backends::tar: key not found in metadata");
        utils::string::memcpy(output, metadata + position, value_length < output_size ? value_length : output_size);
        output[output_size-1] = '\0';
    }
    template<typename TYPE, typename DEVICE, typename SPEC>
    TYPE get_attribute_int(DEVICE& device, persist::backends::tar::ReaderGroup<SPEC>& group, const char* name){
        constexpr typename DEVICE::index_t BUFFER_SIZE = 32;
        char string_value[BUFFER_SIZE];
        get_attribute<char*>(device, group, name, string_value, BUFFER_SIZE);
        TYPE cols_value = utils::string::string_to_int<TYPE>(string_value, BUFFER_SIZE);
        return TYPE(cols_value);
    }
    // template<typename DEVICE, typename SPEC>
    // persist::backends::tar::WriterGroup<SPEC> get_group(DEVICE& device, persist::backends::tar::WriterGroup<SPEC>& group, std::string name) {
    //
    // }
    // template<typename TYPE, typename DEVICE, typename SPEC>
    // TYPE get_attribute(DEVICE& device, persist::backends::tar::WriterGroup<SPEC>& group, std::string name) {
    //     return group.group.getAttribute(name).template read<TYPE>();
    // }


    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT bool load(DEVICE& device, Tensor<SPEC>& tensor, persist::backends::tar::ReaderGroup<GROUP_SPEC>& group, const char* name) {
        // assumes name to be GROUP_SPEC::MAX_PATH_LENGTH = 100 characters or null terminated
        using TI = typename DEVICE::index_t;
        auto tensor_group = get_group(device, group, name);
        if (!utils::assert_exit(device, tensor_group.success, "persist::backends::tar: Failed to get tensor group")){return false;};
        char current_path[GROUP_SPEC::MAX_PATH_LENGTH];
        utils::string::copy<TI>(current_path, tensor_group.path, GROUP_SPEC::MAX_PATH_LENGTH);
        TI current_path_length = utils::string::length(current_path, GROUP_SPEC::MAX_PATH_LENGTH);
        TI meta_current_position = current_path_length;
        if (!utils::assert_exit(device, current_path_length + 1 < GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Current path length exceeds maximum length")){return false;};
        if (current_path_length > 0 && current_path_length + 1 < GROUP_SPEC::MAX_PATH_LENGTH){
            current_path[current_path_length] = '/';
            if (current_path_length < GROUP_SPEC::MAX_PATH_LENGTH-1){
                current_path[current_path_length + 1] = '\0';
            }
            meta_current_position += 1;
        }
        if (!utils::assert_exit(device, current_path_length + 1 + (sizeof("meta")-1) <= GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Meta path and name exceed maximum length")){return false;};
        utils::string::copy(current_path + meta_current_position, "meta", GROUP_SPEC::MAX_PATH_LENGTH - current_path_length - 1);
        constexpr TI METADATA_SIZE = 100;
        char metadata[METADATA_SIZE];
        TI read_size = 0;
        if (!utils::assert_exit(device, persist::backends::tar::get(device, group.data, current_path, metadata, METADATA_SIZE, read_size), "persist::backends::tar: Failed to read metadata entry from tar archive")){return false;};
        metadata[read_size] = '\0';
        TI type_position = 0;
        TI type_value_length = 0;
        if (!utils::assert_exit(device, persist::backends::tar::seek_in_metadata(device, metadata, METADATA_SIZE, "type", type_position, type_value_length), "persist::backends::tar: 'type' not found in metadata")){return false;};
        if (!utils::assert_exit(device, utils::string::compare(metadata + type_position, "tensor", sizeof("tensor")-1), "persist::backends::tar: 'type' is not 'tensor' in metadata")){return false;};

        // constexpr TI MAX_VALUE_LENGTH = 20;
        // char value[MAX_VALUE_LENGTH];
        // utils::string::copy(value, metadata + type_position, type_value_length + 1 < MAX_VALUE_LENGTH ? type_value_length + 1 : MAX_VALUE_LENGTH);
        // std::cout << "type: " << value << std::endl;
        if (!utils::assert_exit(device, persist::backends::tar::containers::tensor::dim_helper_read<DEVICE, SPEC, TI, METADATA_SIZE>(device, metadata), "persist::backends::tar::load(Tensor) dimension mismatch")){return false;};

        if (!utils::assert_exit(device, current_path_length + 1 + (sizeof("data")-1) < GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Meta path and name exceed maximum length")){return false;};
        utils::string::copy(current_path + meta_current_position, "data", GROUP_SPEC::MAX_PATH_LENGTH - current_path_length - 1);
        if (!utils::assert_exit(device, persist::backends::tar::get(device, group.data, current_path, (char*)tensor._data, SPEC::SIZE_BYTES, read_size), "persist::backends::tar: 'data' not found in metadata")){return false;};
        if (!utils::assert_exit(device, read_size == SPEC::SIZE_BYTES, "persist::backends::tar: Data size mismatch")){return false;};
        return true;
    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void save(DEVICE& device, Tensor<SPEC>& tensor, persist::backends::tar::WriterGroup<GROUP_SPEC>& group, const char* name) {
        using TI = typename DEVICE::index_t;
        char group_path[GROUP_SPEC::MAX_PATH_LENGTH];
        utils::string::copy(group_path, group.path, GROUP_SPEC::MAX_PATH_LENGTH-1);
        TI group_path_length = utils::string::length(group_path, GROUP_SPEC::MAX_PATH_LENGTH-1);
        TI name_length = utils::string::length(name, GROUP_SPEC::MAX_PATH_LENGTH-1);
        utils::assert_exit(device, group_path_length + 1 + name_length < GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Group path and name exceed maximum length");
        TI current_position = group_path_length;
        if (group_path_length > 0){
            group_path[group_path_length] = '/';
            current_position += 1;
        }
        utils::string::copy(group_path + current_position, name, GROUP_SPEC::MAX_PATH_LENGTH - group_path_length - 1);
        char current_path[GROUP_SPEC::MAX_PATH_LENGTH];
        utils::string::copy(current_path, group_path, GROUP_SPEC::MAX_PATH_LENGTH-1);
        TI current_path_length = utils::string::length(current_path, GROUP_SPEC::MAX_PATH_LENGTH-1);
        TI meta_current_position = current_path_length;
        if (current_path_length > 0){
            current_path[current_path_length] = '/';
            meta_current_position += 1;
        }
        constexpr TI METADATA_SIZE = 100;
        char metadata[METADATA_SIZE];
        TI metadata_position = 0;
        metadata_position += utils::string::copy(metadata, "type: tensor\n", METADATA_SIZE - metadata_position-1);
        static_assert(utils::typing::is_same_v<typename SPEC::T, float> || utils::typing::is_same_v<typename SPEC::T, double>, "Only float32 and float64 are supported for now");
        if constexpr(utils::typing::is_same_v<typename SPEC::T, float>){
            metadata_position += utils::string::copy(metadata+metadata_position, "dtype: float32\n", METADATA_SIZE - metadata_position-1);
        }
        else if constexpr(utils::typing::is_same_v<typename SPEC::T, double>){
            metadata_position += utils::string::copy(metadata+metadata_position, "dtype: float64\n", METADATA_SIZE - metadata_position-1);
        }
        metadata_position += utils::string::copy(metadata + metadata_position, "num_dims: ", METADATA_SIZE - metadata_position-1);
        char num_dims_str[16];
       utils::string::int_to_string<TI, TI>(num_dims_str, 16, SPEC::SHAPE::LENGTH);
        metadata_position += utils::string::copy(metadata + metadata_position, num_dims_str, METADATA_SIZE - metadata_position-1);
        metadata_position += utils::string::copy(metadata + metadata_position, "\n", METADATA_SIZE - metadata_position-1);
        persist::backends::tar::containers::tensor::dim_helper<SPEC, TI, METADATA_SIZE>(metadata, metadata_position);
        utils::assert_exit(device, current_path_length + 1 + sizeof("meta") - 1 < GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Meta path and name exceed maximum length");
        utils::string::copy(current_path + meta_current_position, "meta", GROUP_SPEC::MAX_PATH_LENGTH - current_path_length - 1);
        write_entry(device, *group.writer, current_path, metadata, metadata_position);
        Tensor<tensor::Specification<typename SPEC::T, typename SPEC::TI, typename SPEC::SHAPE>> tensor_dense;
        malloc(device, tensor_dense);
        copy(device, device, tensor, tensor_dense);
        utils::assert_exit(device, current_path_length + 1 + sizeof("data") - 1 < GROUP_SPEC::MAX_PATH_LENGTH, "persist::backends::tar: Meta path and name exceed maximum length");
        utils::string::copy(current_path + meta_current_position, "data", GROUP_SPEC::MAX_PATH_LENGTH - current_path_length - 1);
        write_entry(device, *group.writer, current_path, reinterpret_cast<const char*>(data(tensor_dense)), SPEC::SIZE_BYTES);
        free(device, tensor_dense);
    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT bool load(DEVICE& device, Matrix<SPEC>& matrix, persist::backends::tar::ReaderGroup<GROUP_SPEC>& group, const char* name) {
        auto tensor = to_tensor(device, matrix);
        return load(device, tensor, group, name);
    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void save(DEVICE& device, Matrix<SPEC>& matrix, persist::backends::tar::WriterGroup<GROUP_SPEC>& group, const char* name) {
        auto tensor = to_tensor(device, matrix);
        save(device, tensor, group, name);
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif

