#pragma once
#include <optional>

// option that supports void value

template<typename T>
struct BetterOption: std::optional<T> {
    T unwrap() {
        return UNWRAP(*this);
    }
};

template<>
struct BetterOption<void> {
    bool hasValue = false;

    static BetterOption EMPTY() { return BetterOption{false}; }

    static BetterOption FULL() { return BetterOption{true}; }

    bool has_value() { return hasValue; }
};
