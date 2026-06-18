#include <rl_tools/operations/cpu.h>
#include <rl_tools/utils/generic/typing.h>

struct A{};
struct B: A{};
struct C: B{};
struct D{};

#include <gtest/gtest.h>
TEST(RL_TOOLS_UTILS, TYPING){
    static_assert(rl_tools::utils::typing::is_base_of_v<A, B>);
    static_assert(rl_tools::utils::typing::is_base_of_v<A, C>);
    static_assert(!rl_tools::utils::typing::is_base_of_v<B, A>);
    static_assert(!rl_tools::utils::typing::is_base_of_v<C, B>);
    static_assert(!rl_tools::utils::typing::is_base_of_v<D, B>);
    static_assert(!rl_tools::utils::typing::is_base_of_v<B, D>);
    static_assert(rl_tools::utils::typing::is_base_of_v<D, D>);
    static_assert(rl_tools::utils::typing::is_base_of_v<A, A>);
}
