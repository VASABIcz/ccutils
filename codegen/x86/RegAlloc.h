#pragma once

#include <bitset>
#include <functional>
#include <map>
#include <set>

#include "../StackAllocator.h"
#include "../../gen64/definitions.h"

class  RegAlloc {
    template<typename CTX>
    friend struct BetterAllocator;
    template<typename CTX>
    friend struct NewAllocator;
    constexpr static array<x86::X64Register, 14> regs64{
        x86::R10,
        x86::R11,
        x86::R12,
        x86::R13,
        x86::R14,
        x86::R15,
        x86::Rbx,
        x86::Rax,
        x86::R8,
        x86::R9,
        x86::Rcx,
        x86::Rdx,
        x86::Rsi,
        x86::Rdi,
    };
    std::array<bool, regs64.size()> regs{};
    // handle - size
    set<x86::X64Register> mClobberedRegs;
    StackAllocator stack;
    
    static constexpr size_t MOST_SIGNIFICANT_BIT = ~(~static_cast<size_t>(0u) >> 1);
    static constexpr size_t IS_INT_BIT = ~(~static_cast<size_t>(0u) >> 1) >> 1;
    static constexpr size_t IS_SHORT_BIT = ~(~static_cast<size_t>(0u) >> 1) >> 2;
    static constexpr size_t IS_BYTE_BIT = ~(~static_cast<size_t>(0u) >> 1) >> 3;
    static constexpr size_t REG_SIZE = 8;
    static constexpr size_t GENERAL_PURPOUSE_REG_COUNT = 14;

    void freeStack(size_t handle) {
        stack.freeStack(handle);
    }

    void freeReg(size_t handle);
public:
    void dumpStack() {
        stack.dumpStack();
    }

    void clear() {
        for (auto i = 0UL; i < regs64.size(); i++) {
            regs[i] = false;
        }
    }

    string debugString(size_t handle) const;

    bool isAcquired(const x86::X64Register& reg) const;

    bool hasFreeReg() const;

    void setReg(size_t idex, bool value) {
        // println("setting reg {} to {}", idex, value);
        if (value) mClobberedRegs.insert(getReg(idex));

        regs[idex] = value;
    }

    bool isAvailable(size_t n) {
        return not isAllocated(n);
    }

    size_t numRealRegs() const {
        size_t acum = 0;
        for (auto i = 0ul; i < regs.size(); i++) {
            if (isAllocated(i)) acum += 1;
        }
        return acum;
    }

    size_t numRegs() const {
        return stack.numAllocs() + numRealRegs();
    }

    void forceReserveStack(size_t nBytes) {
        stack.forceReserve(nBytes);
    }

    vector<x86::X64Register> availableRegs();

    size_t allocateReg(size_t size);

    size_t allocateAny(size_t sizeBytes);

    pair<size_t, x86::X64Register> allocReg();

    x86::X64Register acquireAny();

    pair<x86::X64Register, bool> allocateEvenClobered(set<x86::X64Register>& ignoreStackSpill);

    void freeReg(const x86::X64Register& reg);

    size_t acquireSpecific(const x86::X64Register& reg);

    size_t allocateStack(size_t amountBytes) {
        return stack.allocateStack(amountBytes);
    }

    bool isStack(size_t handle) {
        return stack.isStack(handle);
    }

    x86::X64Register getReg(size_t handle) const;

    bool isAllocated(size_t idex) const {
        return regs[idex];
    }

    size_t getStackOffset(size_t handle) const {
        return stack.getStackOffset(handle);
    }

    map<x86::X64Register, size_t> saveCallRegs(span<const x86::X64Register> exclude, const std::function<x86::X64Register::SaveType(const x86::X64Register&)>& convention);

    void restoreCallRegs(const map<x86::X64Register, size_t>& state);

    void freeHandle(size_t handle);

    size_t stackSize(size_t handle) {
        return stack.stackSize(handle);
    }

    size_t regSize(size_t handle) {
        if ((handle & IS_BYTE_BIT) == IS_BYTE_BIT) {
            return 1;
        }
        if ((handle & IS_SHORT_BIT) == IS_SHORT_BIT) {
            return 2;
        }
        if ((handle & IS_INT_BIT) == IS_INT_BIT) {
            return 4;
        }
        return 8;
    }

    size_t sizeOf(size_t handle) {
        if (isStack(handle)) {
            return stackSize(handle);
        }
        else {
            return regSize(handle);
        }
    }

    size_t stackSize() const {
        return stack.stackSize();
    }

    [[nodiscard]] set<x86::X64Register> clobberedRegs() const;

    size_t regToHandle(const x86::X64Register& reg) {
        for (auto i = 0UL; i < this->regs64.size(); i++) {
            if (regs64[i] == reg) {
                return i;
            }
        }

        PANIC()
    }

    size_t toHandleStupid(const x86::X64Register& reg) {
        assert(isAcquired(reg));

        return regToIdex(reg);
    }

    size_t regToIdex(const x86::X64Register& reg) const {
        for (auto i = 0UL; i < regs64.size(); i++) {
            if (regs64[i] == reg) return i;
        }
        PANIC()
    }
};