#pragma once
#include "gen64/Register.h"

struct BaseRegister {
    virtual ~BaseRegister() = default;

    template<typename T>
    T* as() {
        return dynamic_cast<T*>(this);
    }

    template<typename T>
    bool is() {
        return as<T>() != nullptr;
    }

    virtual std::string toString() const = 0;
};

struct Physical: BaseRegister {
    x86::X64Register id;
    Physical(x86::X64Register id) : id(id) {}

    std::string toString() const override { return id.toString(); }
};

struct Virtual: BaseRegister {
    size_t id;
    Virtual(size_t id) : id(id) {}

    std::string toString() const override { return stringify("v{}", id); }
};

struct StackSlot {
    size_t id = 999'999;
};