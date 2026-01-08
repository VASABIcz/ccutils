#pragma once
#include "BetterOption.h"
#include "utils.h"

template<typename T, typename Ret, typename Class, typename... Args>
BetterOption<Ret> tryDispatch(Class obj, Ret (Class::*lambda)(Args...) const, T* value) {
    auto cst = dynamic_cast<typename First<Args...>::TYPE>(value);
    if (cst != nullptr) {
        if constexpr (std::is_void_v<Ret>) {
            (obj.*lambda)(cst);
            return BetterOption<void>::FULL();
        } else {
            return BetterOption<Ret>((obj.*lambda)(cst));
        }
    } else {
        if constexpr (std::is_void_v<Ret>) {
            return BetterOption<void>::EMPTY();
        } else {
            return BetterOption<Ret>();
        }
    }
}

template<typename T, typename FN>
auto tryDispatch(T* value, FN fn) {
    return tryDispatch(fn, &FN::operator(), value);
}

template<typename T, typename FN, typename... FNs>
auto tryDispatch(T* value, FN fn, FNs... fns) {
    auto res = tryDispatch(value, fn);
    if (res.has_value()) return res;

    return tryDispatch(value, fns...);
}

template<typename T, typename... Args>
auto dispatch(T* value, Args... args) {
    return tryDispatch(value, args...);
}