#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_PERSIST_BACKENDS_HDF5_OPERATIONS_GENERIC)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_PERSIST_BACKENDS_HDF5_OPERATIONS_GENERIC

#include "hdf5.h"
#include <highfive/H5File.hpp>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{

    namespace tensor{
        template<typename DEVICE, typename SPEC, typename TI>
        bool check_dimensions(DEVICE& device, Tensor<SPEC>& tensor, const std::vector<TI>& dims, typename DEVICE::index_t current_dim=0){
            if constexpr(length(typename SPEC::SHAPE{}) == 1){
                return dims[current_dim] == get<0>(typename SPEC::SHAPE{});
            }
            else{
                auto next_tensor = view(device, tensor, current_dim);
                return dims[current_dim] == get<0>(typename SPEC::SHAPE{}) && check_dimensions(device, next_tensor, dims, current_dim+1);
            }
        }
    }
    template<typename DEVICE, typename VECTOR, typename SPEC>
    void from_vector(DEVICE& device, const VECTOR& vector, Tensor<SPEC>& tensor) {
        using TI = typename DEVICE::index_t;
        if constexpr(length(typename SPEC::SHAPE{}) == 1){
            utils::assert_exit(device, vector.size() == get<0>(typename SPEC::SHAPE{}), "Vector size mismatch");
            for (TI i = 0; i < get<0>(typename SPEC::SHAPE{}); i++) {
                set(device, tensor, vector[i], i);
            }
        }
        else{
            utils::assert_exit(device, vector.size() == get<0>(typename SPEC::SHAPE{}), "Vector size mismatch");
            for (TI i = 0; i < get<0>(typename SPEC::SHAPE{}); i++) {
                auto next_tensor = view(device, tensor, i);
                from_vector(device, vector[i], next_tensor);
            }
        }
    }
    template<typename DEVICE, typename VT, typename SPEC>
    void from_flat_vector(DEVICE& device, const std::vector<VT>& vector, Tensor<SPEC>& tensor) {
        using T = typename SPEC::T;
        if constexpr(utils::typing::is_same_v<VT, T>){
            utils::assert_exit(device, vector.size() == SPEC::SIZE, "Vector size mismatch");
            std::memcpy(data(tensor), vector.data(), SPEC::SIZE * sizeof(T));
        }
        else{
            using TI = typename DEVICE::index_t;
            utils::assert_exit(device, vector.size() == SPEC::SIZE, "Vector size mismatch");
            std::vector<T> buffer(SPEC::SIZE);
            for (TI i = 0; i < SPEC::SIZE; i++) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
                buffer[i] = static_cast<VT>(vector[i]); // this is usually not called but mightt still make issues because the datatype is compile-time dispatched on the hdf5 side
#pragma GCC diagnostic pop
            }
            std::memcpy(data(tensor), buffer.data(), SPEC::SIZE * sizeof(T));
        }
    }

    template<typename DEVICE, typename SPEC>
    auto to_vector(DEVICE& device, Tensor<SPEC>& tensor) {
        using TI = typename DEVICE::index_t;
        if constexpr(length(typename SPEC::SHAPE{}) == 1){
            std::vector<typename SPEC::T> data(get<0>(typename SPEC::SHAPE{}));
            for (TI i = 0; i < get<0>(typename SPEC::SHAPE{}); i++) {
                data[i] = get(device, tensor, i);
            }
            return data;
        }
        else{
            auto next_tensor_shape = view(device, tensor, 0);
            std::vector<decltype(to_vector(device, next_tensor_shape))> result(get<0>(typename SPEC::SHAPE{}));
            for (TI i = 0; i < get<0>(typename SPEC::SHAPE{}); i++) {
                auto next_tensor = view(device, tensor, i);
                result[i] = to_vector(device, next_tensor);
            }
            return result;
        }
    }

    namespace containers::tensor{
        template <typename SPEC, auto DIM = 0>
        void dim_helper(HighFive::DataSet& dataset){
            if constexpr(DIM < SPEC::SHAPE::LENGTH){
                std::string key = "dim_" + std::to_string(DIM);
                dataset.template createAttribute<std::string>(key, std::to_string(SPEC::SHAPE::template GET<DIM>));
                dim_helper<SPEC, DIM + 1>(dataset);
            }
        }

    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    bool load(DEVICE& device, Tensor<SPEC>& tensor, const persist::backends::hdf5::Group<GROUP_SPEC>& group, std::string dataset_name, bool fallback_to_zero = false) {
        using T = typename SPEC::T;
        if(fallback_to_zero && !group.group.exist(dataset_name)){
            set_all(device, tensor, 0);
        }
        else{
            auto dataset = group.group.getDataSet(dataset_name);
            auto dims = dataset.getDimensions();
            static_assert(tensor::dense_row_major_layout<SPEC>(), "Load only supports dense tensors for now");
            if (!utils::assert_exit(device, dims.size() == length(typename SPEC::SHAPE{}), "Rank mismatch")){return false;};
            if (!utils::assert_exit(device, tensor::check_dimensions(device, tensor, dims), "Dimension mismatch")){return false;};
            typename SPEC::T* data_ptr = data(tensor);
            if (!utils::assert_exit(device, data_ptr != nullptr, "Data pointer is null")){return false;};
            auto data_type = dataset.getDataType();
            auto data_type_class = data_type.getClass();
            auto data_type_size = data_type.getSize();
            if (!utils::assert_exit(device, data_type_class == HighFive::DataTypeClass::Float || data_type_class == HighFive::DataTypeClass::Integer, "Only Float and Int are currently supported")){return false;};
            if (!utils::assert_exit(device, data_type_size == 4 || data_type_size == 8, "Only Float32 and Float64 are currently supported")){return false;};
            if (!utils::assert_exit(device, dataset.getStorageSize() == data_type_size * SPEC::SIZE, "Storage size mismatch")){return false;};
            if (data_type_class == HighFive::DataTypeClass::Float){
                if(data_type_size == 4){
                    std::vector<float> buffer(SPEC::SIZE);
                    dataset.read(buffer.data());
                    from_flat_vector(device, buffer, tensor);
                }
                else{
                    if(data_type_size == 8){
                        std::vector<double> buffer(SPEC::SIZE);
                        dataset.read(buffer.data());
                        from_flat_vector(device, buffer, tensor);
                    }
                    else{
                        if (!utils::assert_exit(device, false, "Unsupported data type size")){return false;};
                    }
                }
            }
            else{
                if(data_type_class == HighFive::DataTypeClass::Integer){
                    if(data_type_size == 4){
                        std::vector<int32_t> buffer(SPEC::SIZE);
                        dataset.read(buffer.data());
                        from_flat_vector(device, buffer, tensor);
                    }
                    else{
                        if(data_type_size == 8){
                            std::vector<int64_t> buffer(SPEC::SIZE);
                            dataset.read(buffer.data());
                            from_flat_vector(device, buffer, tensor);
                        }
                        else{
                            if (!utils::assert_exit(device, false, "Unsupported data type size")){return false;};
                        }
                    }
                }
            }
        }
        return true;
    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    bool load(DEVICE& device, Matrix<SPEC>& m, persist::backends::hdf5::Group<GROUP_SPEC>& group, std::string dataset_name, bool fallback_to_zero = false) {
        if(fallback_to_zero && !group.group.exist(dataset_name)){
            set_all(device, m, 0);
        }
        else{
            auto dataset = group.group.getDataSet(dataset_name);
            auto dims = dataset.getDimensions();
            if (!utils::assert_exit(device, dims.size() == 2, "Matrix persist::load from hdf5 expects dim=2")){return false;};
            if (!utils::assert_exit(device, dims[0] == SPEC::ROWS, "Matrix persist::load row dimension deviates from expected")){return false;};
            if (!utils::assert_exit(device, dims[1] == SPEC::COLS, "Matrix persist::load col dimension deviates from expected")){return false;};
            std::vector<std::vector<typename SPEC::T>> data;
            dataset.read(data);
            for(typename DEVICE::index_t i=0; i < SPEC::ROWS; i++){
                for(typename DEVICE::index_t j=0; j < SPEC::COLS; j++){
                    set(m, i, j, data[i][j]);
                }
            }
        }
        return true;
    }
    template<typename DEVICE, typename SPEC, typename T>
    bool load(DEVICE& device, Matrix<SPEC>& m, std::vector<std::vector<T>> data) {
        if (!utils::assert_exit(device, data.size() == SPEC::ROWS, "Matrix persist::load (from vector) row dimension deviates from expected")){return false;};
        if (!utils::assert_exit(device, data[0].size() == SPEC::COLS, "Matrix persist::load (from vector) col dimension deviates from expected")){return false;};
        for(typename DEVICE::index_t i=0; i < SPEC::ROWS; i++){
            for(typename DEVICE::index_t j=0; j < SPEC::COLS; j++){
                set(m, i, j, data[i][j]);
            }
        }
        return true;
    }

    template<typename DEVICE, typename SPEC>
    void save(DEVICE& device, Tensor<SPEC>& tensor, HighFive::Group group, std::string dataset_name) {
        auto data = to_vector(device, tensor);
        auto dataset = group.createDataSet(dataset_name, data);
        dataset.template createAttribute<std::string>("type", "tensor");
        dataset.template createAttribute<std::string>("num_dims", std::to_string(SPEC::SHAPE::LENGTH));
        containers::tensor::dim_helper<SPEC>(dataset);
    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    void save(DEVICE& device, Matrix<SPEC>& m, persist::backends::hdf5::Group<GROUP_SPEC>& group, std::string dataset_name) {
        using T = typename SPEC::T;
        std::vector<std::vector<T>> data(SPEC::ROWS);
        for(typename DEVICE::index_t i=0; i < SPEC::ROWS; i++){
            data[i] = std::vector<T>(SPEC::COLS);
            for(typename DEVICE::index_t j=0; j < SPEC::COLS; j++){
                data[i][j] = get(m, i, j);
            }
        }
        auto dataset = group.group.createDataSet(dataset_name, data);
        dataset.template createAttribute<std::string>("type", "matrix");
        dataset.template createAttribute<std::string>("rows", std::to_string(SPEC::ROWS));
        dataset.template createAttribute<std::string>("cols", std::to_string(SPEC::COLS));
    }

    template<typename DEVICE, typename SPEC, typename GROUP_SPEC>
    void save(DEVICE& device, Tensor<SPEC>& tensor, persist::backends::hdf5::Group<GROUP_SPEC>& group, std::string dataset_name) {
        save(device, tensor, group.group, dataset_name);
    }
    template<typename DEVICE, typename SPEC>
    persist::backends::hdf5::Group<SPEC> create_group(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group, std::string name) {
        return {group.group.createGroup(name)};
    }
    template<typename DEVICE>
    persist::backends::hdf5::Group<persist::backends::hdf5::GroupSpecification<>> create_group(DEVICE& device, HighFive::File& file, std::string name) {
        return {file.createGroup(name)};
    }
    template<typename DEVICE, typename SPEC>
    void set_attribute(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group, const char* name, const char* value) {
        group.group.template createAttribute<std::string>(name, value);
    }
    template<typename DEVICE, typename SPEC>
    void write_attributes(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group){
    }
    template<typename DEVICE, typename SPEC>
    persist::backends::hdf5::Group<SPEC> get_group(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group, std::string name) {
        return {group.group.getGroup(name)};
    }
    template<typename DEVICE>
    persist::backends::hdf5::Group<persist::backends::hdf5::GroupSpecification<>> get_group(DEVICE& device, HighFive::File& file, std::string name) {
        return {file.getGroup(name)};
    }
    template<typename DEVICE>
    persist::backends::hdf5::Group<persist::backends::hdf5::GroupSpecification<>> get_group(DEVICE& device, HighFive::Group& group, std::string name) {
        return {group.getGroup(name)};
    }
    template<typename DEVICE, typename SPEC>
    bool group_exists(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group, std::string name) {
        return group.group.exist(name);
    }
    template<typename DEVICE, typename SPEC>
    std::string get_attribute(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group, std::string name) {
        return group.group.getAttribute(name).template read<std::string>();
    }
    template<typename TYPE, typename DEVICE, typename SPEC>
    TYPE get_attribute_int(DEVICE& device, persist::backends::hdf5::Group<SPEC>& group, std::string name) {
        return std::stoi(group.group.getAttribute(name).template read<std::string>());
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif