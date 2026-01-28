#pragma once
#include "gen64/Register.h"
#include "forward.hpp"

class RegBackPtr {
    X86Instruction* instruction;
    BaseRegister* reg;

  public:
    RegBackPtr(X86Instruction* inst, BaseRegister* reg): instruction(inst), reg(reg) {

    }

    BaseRegister* getReg() {
        return reg;
    }

    X86Instruction* getInst() {
        return instruction;
    }
};

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

    VHAND getId() {
        return this;
    }

    std::set<RegBackPtr*> useList;

    RegBackPtr* add(X86Instruction* inst) {
        auto ptr = new RegBackPtr{inst, this};

        useList.insert(ptr);

        return ptr;
    }

    void remove(RegBackPtr* back) {
        useList.erase(back);
    }

    void removeAll(X86Instruction* inst) {
        erase_if(useList, [&](RegBackPtr* ptr) {
           return ptr->getInst() == inst;
        });
    }

    bool canSpill() {
        return not hintReg.has_value();
    }

    std::optional<x86::X64Register> hintReg;
};

struct Virtual: BaseRegister {
    friend Graph;
    size_t id;

    std::string toString() const override {
        if (hintReg.has_value()) return hintReg->toString();

        return stringify("v{}", id);
    }
  private:
    Virtual(size_t id) : id(id) {}
};

struct StackSlot {
    size_t id = 999'999;
};