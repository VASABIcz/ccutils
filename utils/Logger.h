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

    static Logger* NOP();
};

class NOPLogger: Logger {
    static NOPLogger* _the;
public:
    static NOPLogger* getInstance() {
        if (_the == nullptr) _the = new NOPLogger();

        return _the;
    }

    void DEBUG(std::function<void ()> funk) override {}
};

NOPLogger* NOPLogger::_the = nullptr;

Logger* Logger::NOP() {
    return NOPLogger::getInstance();
}