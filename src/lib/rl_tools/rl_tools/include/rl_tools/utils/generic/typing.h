#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_GENERIC_TYPING_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_GENERIC_TYPING_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils::typing {
    template<class T, T v>
    struct integral_constant {
        static constexpr T value = v;
        using value_type = T;
        using type = integral_constant; // using injected-class-name
        constexpr operator value_type() const noexcept { return value; }
        constexpr value_type operator()() const noexcept { return value; } // since c++14
    };
    using false_type = integral_constant<bool, false>;
    using true_type = integral_constant<bool, true>;

    template<class T, class U>
    struct is_same : false_type {};

    template<class T>
    struct is_same<T, T> : true_type {};

    template< class T, class U >
    inline constexpr bool is_same_v = is_same<T, U>::value;


    template<class T> struct is_reference : false_type {};
    template<class T> struct is_reference<T&> : true_type {};
    template<class T> struct is_reference<T&&> : true_type {};
    template <class T>
    constexpr bool is_reference_v = is_reference<T>::value;

    template<class T>
    struct remove_reference{
        typedef T type;
    };
    template<class T>
    struct remove_reference<T&>{
        typedef T type;
    };

    template<class T>
    using remove_reference_t = typename remove_reference<T>::type;

    template< class T > struct remove_pointer                    {typedef T type;};
    template< class T > struct remove_pointer<T*>                {typedef T type;};
    template< class T > struct remove_pointer<T* const>          {typedef T type;};
    template< class T > struct remove_pointer<T* volatile>       {typedef T type;};
    template< class T > struct remove_pointer<T* const volatile> {typedef T type;};

    template<bool B, class T = void>
    struct enable_if {};

    template<class T>
    struct enable_if<true, T> { typedef T type; };

    template< bool B, class T = void >
    using enable_if_t = typename enable_if<B,T>::type;

    template< class T > struct remove_cv                   { typedef T type; };
    template< class T > struct remove_cv<const T>          { typedef T type; };
    template< class T > struct remove_cv<volatile T>       { typedef T type; };
    template< class T > struct remove_cv<const volatile T> { typedef T type; };

    template< class T > struct remove_const                { typedef T type; };
    template< class T > struct remove_const<const T>       { typedef T type; };

    template< class T > struct remove_volatile             { typedef T type; };
    template< class T > struct remove_volatile<volatile T> { typedef T type; };

    template< class T >
    using remove_cv_t = typename remove_cv<T>::type;

    template<bool condition>
    struct warn_if{};

    template<> struct [[deprecated]] warn_if<false>{constexpr warn_if() = default;};

    template<bool B, class T, class F>
    struct conditional { using type = T; };

    template<class T, class F>
    struct conditional<false, T, F> { using type = F; };

    template< bool B, class T, class F >
    using conditional_t = typename conditional<B,T,F>::type;

    template <bool value, typename... Args>
    inline constexpr bool dependent_bool_value = value;
    template <typename... Args>
    inline constexpr bool dependent_false = dependent_bool_value<false, Args...>;

    template <typename Base, typename Derived>
    class IsSubclass {
    private:
        typedef char Yes[1];
        typedef char No[2];

        static Yes& test(Base*);
        static No& test(...);

    public:
        enum { value = sizeof(test(static_cast<Derived*>(0))) == sizeof(Yes) };
    };

    template <typename Base, typename Derived>
    constexpr bool is_base_of_v = IsSubclass<Base, Derived>::value;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#define rl_tools_static_warn(x, ...) ((void) warn_if<x>())

#endif