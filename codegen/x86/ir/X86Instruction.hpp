#pragma once
#include "forward.hpp"
#include "utils/Debuggable.h"
#include "gen64/Register.h"
#include "Registers.hpp"
#include <vector>

struct X86Instruction: Debuggable {
    std::vector<BaseRegister*> uses;
    std::vector<BaseRegister*> defs;
    std::vector<BaseRegister*> kills;
    X86Instruction* prev;
    X86Instruction* next;
    long orderId = 0;

    bool hasDefVirt(size_t id) {
        for (auto def: defs) {
            auto virt = def->as<Virtual>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    void rewriteDef(Virtual* old, Virtual* newReg) {
        for (auto& def: defs) {
            if (def->is<Virtual>() && def->as<Virtual>()->id == old->id) def = newReg;
        }
    }

    void rewriteUse(Virtual* old, Virtual* newReg) {
        for (auto& def: uses) {
            if (def->is<Virtual>() && def->as<Virtual>()->id == old->id) def = newReg;
        }
    }

    bool hasUseVirt(size_t id) {
        for (auto use: uses) {
            auto virt = use->as<Virtual>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    bool hasDefPhy(x86::X64Register id) {
        for (auto def: defs) {
            auto virt = def->as<Physical>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    template<typename T>
    bool is() {
        return dynamic_cast<T*>(this) != nullptr;
    }
};
