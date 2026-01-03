#pragma once

#include <functional>

class NOPLogger;

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

    static std::shared_ptr<Logger> NOP();
};

class NOPLogger: public Logger {
    static shared_ptr<NOPLogger> _the;
public:
    static std::shared_ptr<NOPLogger> getInstance() {
        if (_the == nullptr) _the = std::make_shared<NOPLogger>();

        return _the;
    }

    void DEBUG(std::function<void ()> funk) override {}
};

shared_ptr<NOPLogger> NOPLogger::_the{};

std::shared_ptr<Logger> Logger::NOP() {
    return NOPLogger::getInstance();
}