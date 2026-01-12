#pragma once
#include "Registers.hpp"
#include "forward.hpp"
#include "gen64/Register.h"
#include "utils/Debuggable.h"
#include <vector>

struct Range;
class X86Instruction: public Debuggable {
    friend Block;
    friend Range;
    friend Graph;

  private:
    std::vector<RegBackPtr*> uses;
    std::vector<RegBackPtr*> defs;
    std::vector<RegBackPtr*> kills;
    X86Instruction* prev;
    X86Instruction* next;
    long orderId = 0;
    Block* block;

  public:
    void setDef(std::vector<BaseRegister*> regs) {
        assert(defs.empty());
        for (auto reg: regs)
            addDef(reg);
    }

    void setUse(std::vector<BaseRegister*> regs) {
        assert(uses.empty());
        for (auto reg: regs)
            addUse(reg);
    }

    void setKill(std::vector<BaseRegister*> regs) {
        assert(kills.empty());
        for (auto reg: regs)
            addKill(reg);
    }

    size_t defCount() { return defs.size(); }

    size_t useCount() { return uses.size(); }

    auto forEachDef(auto fn) {
        for (auto def: defs) {
            fn(def->getReg());
        }
    }

    Graph* getGraph();

    BaseRegister* PHY(x86::X64Register reg);

    void addUse(BaseRegister* use) {
        uses.push_back(use->add(this));
    }

    void addDef(BaseRegister* def) { defs.push_back(def->add(this)); }

    void addKill(BaseRegister* kill) { kills.push_back(kill->add(this)); }

    BaseRegister* getUse(size_t i) { return uses[i]->getReg(); }

    BaseRegister* getDef(size_t i) { return defs[i]->getReg(); }

    X86Instruction* getNext() { return next; }

    X86Instruction* getPrev() { return prev; }

    void forEachUse(auto fn) {
        for (auto item: uses) {
            fn(item->getReg());
        }
    }

    void forEachKill(auto fn) {
        for (auto item: kills) {
            fn(item->getReg());
        }
    }

    bool hasDefVirt(VHAND id) {
        for (auto i = 0u; i < defCount(); i++) {
            assert((getDef(i) == id) == (getDef(i)->toString() == id->toString()));
            if (getDef(i) == id) return true;
        }
        return false;
    }

    void rewriteDef(VHAND old, VHAND newReg) {
        for (auto& def : defs) {
            if (def->getReg() == old) {
                old->remove(def);
                def = newReg->add(this);
            }
        }
        assert(!hasDefVirt(old));
        assert(hasDefVirt(newReg));
    }

    void rewriteUse(VHAND old, VHAND newReg) {
        for (auto& use : uses) {
            if (use->getReg() == old) {
                old->remove(use);
                use = newReg->add(this);
            }
        }
    }

    bool hasUseVirt(VHAND id) {
        for (auto i = 0u; i < useCount(); i++) {
            if (getUse(i) == id) return true;
        }
        return false;
    }

    template<typename T>
    bool is() {
        return dynamic_cast<T*>(this) != nullptr;
    }
};
