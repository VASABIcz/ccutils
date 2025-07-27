#include <cstdio>

// array that fails at comp time when initialized with wrong number of arguments
template<typename T, size_t N>
struct BetterArray {
    T items[N];

    [[nodiscard]] constexpr size_t size() const {
        return N;
    }

    constexpr T* data() {
        return items;
    }

    constexpr T* begin() {
        return items;
    }

    constexpr T* end() {
        return items+N;
    }

    template<typename... Args>
    constexpr BetterArray(Args... args): items(args...) {
        static_assert(sizeof...(Args) == N, "array must be initialized with the same number of values as its size");
    }
};