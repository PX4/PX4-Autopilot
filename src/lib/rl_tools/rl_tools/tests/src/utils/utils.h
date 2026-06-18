#ifndef TEST_UTILS_H
#define TEST_UTILS_H
#include <vector>

#define RL_TOOLS_STRINGIZE(x) #x
#define RL_TOOLS_MACRO_TO_STR(macro) RL_TOOLS_STRINGIZE(macro)

template <typename T, typename TI, TI DIM>
void standardise(const T input[DIM], const T mean[DIM], const T std[DIM], T output[DIM]){
    for (TI i = 0; i < DIM; i++){
        output[i] = (input[i] - mean[i]) / std[i];
    }
}
template <typename T, typename SPEC>
T abs_diff_matrix(const RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::Matrix<SPEC> A, const std::vector<std::vector<T>>& B) {
    using TI = typename SPEC::TI;
    T acc = 0;
    for (TI i = 0; i < SPEC::ROWS; i++){
        for (TI j = 0; j < SPEC::COLS; j++){
            acc += std::abs(get(A, i, j) - B[i][j]);
        }
    }
    return acc;
}

template <typename T, typename SPEC>
T abs_diff_matrix(RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::Matrix<SPEC> A, const T* B) {
    T acc = 0;
    using TI = typename SPEC::TI;
    for (TI i = 0; i < SPEC::ROWS; i++){
        for (TI j  = 0; j < SPEC::COLS; j++){
            acc += std::abs(get(A, i, j) - B[i * SPEC::COLS + j]);
        }
    }
    return acc;
}

template <typename T, typename TI, TI N_ROWS>
T abs_diff_vector(const T A[N_ROWS], const T B[N_ROWS]) {
    T acc = 0;
    for (TI i = 0; i < N_ROWS; i++){
        acc += std::abs(A[i] - B[i]);
    }
    return acc;
}

template <typename T, typename SPEC>
void assign(const std::vector<std::vector<T>>& source, RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::Matrix<SPEC> target) {
    using TI = typename SPEC::TI;
    for (TI i = 0; i < SPEC::ROWS; i++){
        for (TI j = 0; j < SPEC::COLS; j++){
            set(target, i, j, source[i][j]);
        }
    }
}

#endif