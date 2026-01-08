#pragma once

#include <span>
#include <vector>

template<typename T>
struct BetterSpan {
    std::span<T> inner;

    BetterSpan(std::span<T> s) {
        inner = std::span<T>(s.begin(), s.end());
    }

    BetterSpan(std::initializer_list<T> s) {
        inner = std::span<T>((T*)s.begin(), (T*)(s.end()));
    }

    BetterSpan(std::vector<T>& s) {
        inner = std::span<T>(s.begin(), s.end());
    }

    auto begin() {
        return inner.begin();
    }

    auto end() {
        return inner.end();
    }
};