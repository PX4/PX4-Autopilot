#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_TENSOR_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_TENSOR_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace tensor{
        struct FinalElement{
            static constexpr auto LENGTH = 0;
            static constexpr auto FIRST = 0;
            static constexpr auto LAST = 0;
            template <auto N>
            struct GET {
                static_assert(N == 0, "Index out of bounds in FinalElement");
            };
        };
        template <typename TI, typename ELEMENT, TI N>
        struct GET_IMPL {
            static constexpr TI VALUE() {
                if constexpr (N == 0) {
                    return ELEMENT::VALUE;
                } else {
                    static_assert(!utils::typing::is_same_v<ELEMENT, FinalElement>, "Index out of bounds in GET_IMPL");
                    return GET_IMPL<TI, typename ELEMENT::NEXT_ELEMENT, N - 1>::VALUE();
                }
            }
        };
        
        template <typename TI, TI N>
        struct GET_IMPL<TI, FinalElement, N> {
            static constexpr TI VALUE() {
                static_assert(N == 0, "Index out of bounds accessing FinalElement");
                return 0;
            }
        };

        template <typename T_TI, T_TI T_VALUE, typename T_NEXT_ELEMENT>
        struct Element{
            using TI = T_TI;
            static constexpr TI VALUE = T_VALUE;
    //            static constexpr bool FINAL_ELEMENT = utils::typing::is_same_v<T_NEXT_ELEMENT, FinalElement>;
            using NEXT_ELEMENT = T_NEXT_ELEMENT;

            static constexpr bool NEXT_IS_FINAL = utils::typing::is_same_v<T_NEXT_ELEMENT, FinalElement>;
            static constexpr TI LENGTH = (NEXT_IS_FINAL ? 0 : 1) + NEXT_ELEMENT::LENGTH;


            template <TI N>
            static constexpr TI GET = GET_IMPL<TI, Element<T_TI, T_VALUE, T_NEXT_ELEMENT>, N>::VALUE();

            static constexpr TI FIRST = VALUE;
            static constexpr TI _compute_last(){
                if constexpr (LENGTH == 0){
                    return 0;
                }
                else{
                    return GET<LENGTH-1>;
                }
            }
            static constexpr TI LAST = _compute_last();
        };


        template <typename T_TI, T_TI... T_VALUES>
        struct Tuple: Element<T_TI, 0, FinalElement>{
        };

        template <typename T_TI, T_TI T_VALUE, T_TI... T_VALUES>
        struct Tuple<T_TI, T_VALUE, T_VALUES...>: Element<T_TI, T_VALUE, Tuple<T_TI, T_VALUES...>>{
            using TI = T_TI;
            static constexpr TI VALUE = T_VALUE;
        };

        template <typename TI, TI... T_DIMS>
        struct Shape: Tuple<TI, T_DIMS...> {
        };

        template <typename TI, TI... T_DIMS>
        struct Stride: Tuple<TI, T_DIMS...> {
        };

    }
    template <typename TI, TI VALUE, typename NEXT_ELEMENT>
    RL_TOOLS_FUNCTION_PLACEMENT TI constexpr length(tensor::Element<TI, VALUE, NEXT_ELEMENT>, TI current_length=0){
        if constexpr(utils::typing::is_same_v<NEXT_ELEMENT, tensor::FinalElement>){
            return current_length;
        }
        else{
            return length(NEXT_ELEMENT{}, current_length+1);
        }
    }
    template <typename TI, TI VALUE, typename NEXT_ELEMENT>
    RL_TOOLS_FUNCTION_PLACEMENT TI constexpr product(tensor::Element<TI, VALUE, NEXT_ELEMENT>){
        if constexpr(utils::typing::is_same_v<NEXT_ELEMENT, tensor::FinalElement>){
            return 1;
        }
        else{
            return VALUE * product(NEXT_ELEMENT{});
        }
    }
    template <auto TARGET_INDEX_INPUT, typename TI, TI VALUE, typename NEXT_ELEMENT>
    RL_TOOLS_FUNCTION_PLACEMENT TI constexpr get(tensor::Element<TI, VALUE, NEXT_ELEMENT>){
        constexpr TI TARGET_INDEX = TARGET_INDEX_INPUT;
    //        constexpr bool LAST_ELEMENT = utils::typing::is_same_v<NEXT_ELEMENT, tensor::FinalElement>;
        static_assert(TARGET_INDEX <= length(NEXT_ELEMENT{}), "Index out of bounds");
        if constexpr(TARGET_INDEX == 0){
            return VALUE;
        }
        else{
            return get<TARGET_INDEX_INPUT-1>(NEXT_ELEMENT{});
        }
    }
    template <typename DEVICE, typename TI, TI VALUE, typename NEXT_ELEMENT>
    RL_TOOLS_FUNCTION_PLACEMENT TI get(DEVICE& device, const tensor::Element<TI, VALUE, NEXT_ELEMENT>, TI index){
        utils::assert_exit(device, index < length(tensor::Element<TI, VALUE, NEXT_ELEMENT>{}), "Index out of bounds");
        if constexpr (utils::typing::is_same_v<NEXT_ELEMENT, tensor::FinalElement>){
            return VALUE;
        }
        else{
            if(index == 0){
                return VALUE;
            }
            else{
                return get(device, NEXT_ELEMENT{}, index-1);
            }
        }
    }
    template <typename TI, TI VALUE, typename NEXT_ELEMENT>
    RL_TOOLS_FUNCTION_PLACEMENT TI constexpr get_last(tensor::Element<TI, VALUE, NEXT_ELEMENT>){
        constexpr TI TARGET_INDEX = length(tensor::Element<TI, VALUE, NEXT_ELEMENT>{}) - 1;
        if constexpr(TARGET_INDEX == 0){
            return VALUE;
        }
        else{
            return get<TARGET_INDEX-1>(NEXT_ELEMENT{});
        }
    }
    namespace tensor {
        template<typename ELEMENT, auto NEW_ELEMENT> // since the last Element is a dummy element containing 0, we need to insert the new element once the NEXT_ELEMENT is the FinalElement
        struct Append: Element<
                typename ELEMENT::TI,
                !utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT, FinalElement> ? ELEMENT::VALUE : NEW_ELEMENT,
        utils::typing::conditional_t<!utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT, FinalElement>,
        Append<typename ELEMENT::NEXT_ELEMENT, NEW_ELEMENT>,
        Element<typename ELEMENT::TI, 0, FinalElement>
        >>{};
        template<typename ELEMENT, auto NEW_ELEMENT> // since the last Element is a dummy element containing 0, we need to insert the new element once the NEXT_ELEMENT is the FinalElement
        struct Prepend: Element<typename ELEMENT::TI, NEW_ELEMENT, ELEMENT>{};

        template<typename ELEMENT>
        struct PopFront: ELEMENT::NEXT_ELEMENT{
            static_assert(length(ELEMENT{}) > 0);
        };

        template<typename ELEMENT>
        struct PopBack: utils::typing::conditional_t<
                utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT::NEXT_ELEMENT, FinalElement>,
                typename ELEMENT::NEXT_ELEMENT,
                Element<typename ELEMENT::TI, ELEMENT::VALUE, PopBack<typename ELEMENT::NEXT_ELEMENT>>
        >{
            static_assert(length(ELEMENT{}) > 0);
        };

        template <typename ELEMENT>
        struct CumulativeProduct: Element< // e.g. (2, 3, 4) -> (24, 12, 4)
                typename ELEMENT::TI,
                product(ELEMENT{}),
                utils::typing::conditional_t<!utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT, FinalElement>,
                        CumulativeProduct<typename ELEMENT::NEXT_ELEMENT>,
                        FinalElement
        >>{};
        template <typename ELEMENT, auto NEW_ELEMENT, auto NEW_ELEMENT_OFFSET>
        struct Replace: Element<
                typename ELEMENT::TI,
                NEW_ELEMENT_OFFSET == 0 ? NEW_ELEMENT : ELEMENT::VALUE,
                utils::typing::conditional_t<!utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT, FinalElement>,
                        Replace<typename ELEMENT::NEXT_ELEMENT, NEW_ELEMENT, NEW_ELEMENT_OFFSET-1>,
                        FinalElement
                >>{
        };

        template <typename ELEMENT, auto NEW_ELEMENT, auto NEW_ELEMENT_OFFSET>
        struct Insert: utils::typing::conditional_t<NEW_ELEMENT_OFFSET == 0,
                Element<typename ELEMENT::TI, NEW_ELEMENT, ELEMENT>,//Element<typename ELEMENT::TI, ELEMENT::VALUE, typename ELEMENT::NEXT_ELEMENT>>,
                Element<typename ELEMENT::TI, ELEMENT::VALUE,
                utils::typing::conditional_t<!utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT, FinalElement>,
                Insert<typename ELEMENT::NEXT_ELEMENT, NEW_ELEMENT, NEW_ELEMENT_OFFSET-1>,
                FinalElement
                >>>{
        };

        template <typename SHAPE, auto COMPARISON>
        constexpr bool RANK_LARGER_THAN = length(SHAPE{}) > COMPARISON;

        template <typename ELEMENT, auto ELEMENT_OFFSET> //, typename utils::typing::enable_if_t<tensor::RANK_LARGER_THAN<ELEMENT, ELEMENT_OFFSET>, void>* = nullptr>
        struct Remove: utils::typing::conditional_t<ELEMENT_OFFSET == 0,
                typename ELEMENT::NEXT_ELEMENT,
                utils::typing::conditional_t<!utils::typing::is_same_v<typename ELEMENT::NEXT_ELEMENT, FinalElement>,
                        Element<typename ELEMENT::TI, ELEMENT::VALUE, Remove<typename ELEMENT::NEXT_ELEMENT, ELEMENT_OFFSET-1>>,
                        FinalElement
                >>{
            static_assert(length(ELEMENT{}) > ELEMENT_OFFSET);
        };

        template <typename SHAPE>
        using RowMajorStride = Append<PopFront<CumulativeProduct<SHAPE>>, 1>;

        template <typename SHAPE, typename STRIDE>
        constexpr typename SHAPE::TI max_span(){
            static_assert(length(SHAPE{}) == length(STRIDE{}));
            if constexpr(length(SHAPE{}) == 1){
                return get<0>(SHAPE{}) * get<0>(STRIDE{});
            }
            else{
                using NEXT_SHAPE = PopFront<SHAPE>;
                using NEXT_STRIDE = PopFront<STRIDE>;
                auto previous = max_span<NEXT_SHAPE, NEXT_STRIDE>();
                auto current = get<0>(SHAPE{}) * get<0>(STRIDE{});
                return previous > current ? previous : current;
            }
        }

        template <typename T_T, typename T_TI, typename T_SHAPE, bool T_DYNAMIC_ALLOCATION=true, typename T_STRIDE = RowMajorStride<T_SHAPE>, bool T_CONST=false>
        struct Specification{
            using T = T_T;
            using TI = T_TI;
            using SHAPE = T_SHAPE;
            using STRIDE = T_STRIDE;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            static constexpr bool CONST = T_CONST;
            static constexpr TI SIZE = max_span<SHAPE, STRIDE>();
            static constexpr TI SIZE_BYTES = SIZE * sizeof(T);

        };
        template<auto T_DIM, auto T_SIZE=0>
        struct ViewSpec{
            static constexpr auto DIM = T_DIM;
            static constexpr auto SIZE = T_SIZE;
        };
        template <typename SHAPE, typename STRIDE>
        RL_TOOLS_FUNCTION_PLACEMENT bool constexpr generalized_row_major(){
            static_assert(length(SHAPE{}) == length(STRIDE{}));
            if constexpr(length(SHAPE{}) == 1){
                return true;
            }
            else{
                constexpr auto back_value_shape = get<length(SHAPE{})-1>(SHAPE{});
                constexpr auto back_value_stride = get<length(STRIDE{})-1>(STRIDE{});
                using NEXT_SHAPE = PopBack<SHAPE>;
                using NEXT_STRIDE = PopBack<STRIDE>;
                return back_value_shape * back_value_stride <= get<length(NEXT_STRIDE{})-1>(NEXT_STRIDE{}) && generalized_row_major<NEXT_SHAPE, NEXT_STRIDE>();
            }
        }
        template <typename A, typename B>
       RL_TOOLS_FUNCTION_PLACEMENT  bool constexpr same_dimensions_shape(){
            if constexpr(length(A{}) != length(B{})){
                return false;
            }
            if constexpr(length(A{}) == 0){
                return true;
            }
            else{
                using NEXT_A = PopFront<A>;
                using NEXT_B = PopFront<B>;
                return (A::VALUE == B::VALUE) && same_dimensions_shape<NEXT_A, NEXT_B>();
            }
        }
        template <typename SPEC_A, typename SPEC_B>
        RL_TOOLS_FUNCTION_PLACEMENT bool constexpr same_dimensions(){
            return same_dimensions_shape<typename SPEC_A::SHAPE, typename SPEC_B::SHAPE>();
        }


        template <typename SHAPE, typename STRIDE, bool RELAX_MAJOR=false>
        RL_TOOLS_FUNCTION_PLACEMENT bool constexpr _dense_row_major_layout_shape(){
            static_assert(length(SHAPE{}) > 0);
            if(length(STRIDE{}) != length(SHAPE{})){
                return false;
            }
            if constexpr(length(STRIDE{}) == 1){
                return RELAX_MAJOR || STRIDE::FIRST == 1;
            }
            else{
                if constexpr(RELAX_MAJOR && STRIDE::LENGTH == 2){
                    return STRIDE::FIRST >= STRIDE::template GET<1> * SHAPE::template GET<1>;
                }
                else{
                    using NEXT_SHAPE = PopFront<SHAPE>;
                    using NEXT_STRIDE = PopFront<STRIDE>;
                    return (STRIDE::VALUE == NEXT_STRIDE::FIRST * NEXT_SHAPE::FIRST || ((SHAPE::FIRST == 1) && (STRIDE::VALUE >= NEXT_STRIDE::FIRST * NEXT_SHAPE::FIRST))) && _dense_row_major_layout_shape<NEXT_SHAPE, NEXT_STRIDE, RELAX_MAJOR>();
                }
            }
        }
        template <typename SPEC, bool RELAX_MAJOR=false>
        RL_TOOLS_FUNCTION_PLACEMENT bool constexpr dense_row_major_layout(){
            return _dense_row_major_layout_shape<typename SPEC::SHAPE, typename SPEC::STRIDE, RELAX_MAJOR>();
        }
        namespace spec::view{
            namespace range{
                template <typename SHAPE, typename VIEW_SPEC>
                using Shape = tensor::Replace<SHAPE, VIEW_SPEC::SIZE, VIEW_SPEC::DIM>;
                template <typename STRIDE, typename VIEW_SPEC>
                using Stride = STRIDE;
                template <typename SPEC, typename VIEW_SPEC, bool T_CONST>
                using Specification = tensor::Specification<typename SPEC::T, typename SPEC::TI, Shape<typename SPEC::SHAPE, VIEW_SPEC>, true, Stride<typename SPEC::STRIDE, VIEW_SPEC>, T_CONST>;
            }
            namespace point{
                template <typename SHAPE, typename VIEW_SPEC>
                using Shape = tensor::Remove<SHAPE, VIEW_SPEC::DIM>;
                template <typename STRIDE, typename VIEW_SPEC>
                using Stride = tensor::Remove<STRIDE, VIEW_SPEC::DIM>;
                template <typename SPEC, typename VIEW_SPEC, bool T_CONST>
                using Specification = tensor::Specification<typename SPEC::T, typename SPEC::TI, Shape<typename SPEC::SHAPE, VIEW_SPEC>, true, Stride<typename SPEC::STRIDE, VIEW_SPEC>, T_CONST>;
            }
        }
    }

    namespace tensor{
        template <typename T, typename TI, TI SIZE>
        struct TensorStatic{
            static constexpr bool DYNAMIC_ALLOCATION = false;
            static_assert(SIZE > 0, "MSVC does not allow SIZE=0");
            T _data[SIZE];
        };
        template <typename T>
        struct TensorStaticEmpty{
            static constexpr bool DYNAMIC_ALLOCATION = false;
            T* _data = nullptr;
        };
        template <typename T, typename TI, TI SIZE, bool CONST = false>
        struct TensorDynamic{
            static constexpr bool DYNAMIC_ALLOCATION = true;
            T* _data = nullptr;
        };
        template <typename T, typename TI, TI SIZE>
        struct TensorDynamic<T, TI, SIZE, true>{
            static constexpr bool DYNAMIC_ALLOCATION = true;
            const T* _data;
        };
    }

    template <typename T_SPEC>
    struct Tensor: utils::typing::conditional_t<T_SPEC::DYNAMIC_ALLOCATION, tensor::TensorDynamic<typename T_SPEC::T, typename T_SPEC::TI, T_SPEC::SIZE, T_SPEC::CONST>, utils::typing::conditional_t<(T_SPEC::SIZE > 0), tensor::TensorStatic<typename T_SPEC::T, typename T_SPEC::TI, T_SPEC::SIZE>, tensor::TensorStaticEmpty<typename T_SPEC::T>>>{
        using SPEC = T_SPEC;
        using SHAPE = typename SPEC::SHAPE;
        using T = typename SPEC::T;
        template <typename VIEW_SPEC>
        using VIEW_POINT = Tensor<tensor::spec::view::point::Specification<SPEC, VIEW_SPEC, SPEC::CONST>>;
        template <typename VIEW_SPEC>
        using VIEW_RANGE = Tensor<tensor::spec::view::range::Specification<SPEC, VIEW_SPEC, SPEC::CONST>>;
        // Tensor() = default;
//        Tensor(DATA_TYPE data): _data(data){};
    };

    template <typename T, typename TI, TI SIZE>
    RL_TOOLS_FUNCTION_PLACEMENT auto data(tensor::TensorStatic<T, TI, SIZE>& tensor){
        return tensor._data;
    }
    template <typename T, typename TI, TI SIZE>
    RL_TOOLS_FUNCTION_PLACEMENT auto data(const tensor::TensorStatic<T, TI, SIZE>& tensor){
        return &tensor._data[0];
    }

    template <typename T, typename TI, TI SIZE, bool CONST>
    RL_TOOLS_FUNCTION_PLACEMENT auto data(tensor::TensorDynamic<T, TI, SIZE, CONST>& tensor){
        return tensor._data;
    }
    template <typename T, typename TI, TI SIZE, bool CONST>
    RL_TOOLS_FUNCTION_PLACEMENT auto data(const tensor::TensorDynamic<T, TI, SIZE, CONST>& tensor){
        return &tensor._data[0];
    }

    template <typename T, typename TI, TI SIZE, bool CONST>
    RL_TOOLS_FUNCTION_PLACEMENT T** data_pointer(tensor::TensorDynamic<T, TI, SIZE, CONST>& tensor){
        return &tensor._data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif