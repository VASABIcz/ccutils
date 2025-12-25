#pragma once

#include <functional>

struct Logger {
    virtual void DEBUG(std::function<void()> funk) = 0;

    template<typename... Args>
    constexpr auto DEBUG(StringChecker<type_identity_t<Args>...> strArg, Args&&... argz) {
        this->DEBUG([&] {
            println(strArg, std::forward<Args>(argz)...);
        });
    }

    void DEBUG() {}

    virtual ~Logger() = default;
};