#include "RegAlloc.h"
#include "utils/utils.h"

void RegAlloc::freeReg(size_t handle) {
    // println("[REG] deallocated {}", regs64[handle]);
    assert(!isStack(handle));
    auto stripedBits = (handle << 4) >> 4;
    assert(stripedBits < regs64.size());
    setReg(stripedBits, false);
}

bool RegAlloc::isAcquired(const x86::X64Register& reg) const {
    return isAllocated(regToIdex(reg));
}

bool RegAlloc::hasFreeReg() const {
    for (auto i : views::iota(0u, GENERAL_PURPOUSE_REG_COUNT)) {
        if (not isAllocated(i)) return true;
    }

    return false;
}

size_t RegAlloc::allocateReg(size_t size) {
    for (auto i : views::iota(0u, GENERAL_PURPOUSE_REG_COUNT)) {
        if (not isAllocated(i)) {
            setReg(i, true);
            // println("[REG] allocated: {}", regs64[i]);

            if (size == 4) {
                return i | IS_INT_BIT;
            }
            if (size == 2) {
                return i | IS_SHORT_BIT;
            }
            if (size == 1) {
                return i | IS_BYTE_BIT;
            }
            // FIXME
            // assert(size == 8);
            return i;
        }
    }

    // stack alloc fallback
    // return allocateStack(8);
    TODO();
}

pair<size_t, x86::X64Register> RegAlloc::allocReg() {
    auto regHandle = allocateReg(8);

    return {regHandle, regs64[regHandle]};
}

x86::X64Register RegAlloc::acquireAny() {
    auto index = allocateReg(8);

    x86::X64Register reg = regs64[index];

    return reg;
}

void RegAlloc::freeReg(const x86::X64Register& reg) {
    assert(isAcquired(reg));
    setReg(regToIdex(reg), false);
}

size_t RegAlloc::acquireSpecific(const x86::X64Register& reg) {
    assert(!isAcquired(reg));
    auto res = regToIdex(reg);
    setReg(res, true);

    return res;
}

x86::X64Register RegAlloc::getReg(size_t handle) const {
    if (stack.isStack(handle)) PANIC();
    auto stripedBits = (handle << 4) >> 4;
    if (stripedBits >= regs64.size()) PANIC();
    return regs64[stripedBits];
}

map<x86::X64Register, size_t> RegAlloc::saveCallRegs(span<const x86::X64Register> exclude, const std::function<x86::X64Register::SaveType(const x86::X64Register&)>& convention) {
    map<x86::X64Register, size_t> state;

    for (auto i : views::iota(0u, regs.size())) {
        const auto& reg = regs64[i];
        if (std::ranges::find(exclude, reg) != exclude.end()) continue;
        auto saveType = convention(reg);
        if (isAllocated(i) && saveType == x86::X64Register::SaveType::Caller) {
            auto stackHandle = allocateStack(REG_SIZE);
            state.emplace(reg, stackHandle);
        }
    }

    return state;
}

void RegAlloc::restoreCallRegs(const map<x86::X64Register, size_t >& state) {
    for (const auto& item : state) {
        freeHandle(item.second);
    }
}

void RegAlloc::freeHandle(size_t handle) {
    if (isStack(handle)) {
        freeStack(handle);
    }
    else {
        freeReg(handle);
    }
}

set<x86::X64Register> RegAlloc::clobberedRegs() const {
    return mClobberedRegs;
}

size_t RegAlloc::allocateAny(size_t sizeBytes) {
    if (hasFreeReg() && sizeBytes <= REG_SIZE)
        return allocateReg(sizeBytes);
    else
        return allocateStack(sizeBytes);
}

pair<x86::X64Register, bool> RegAlloc::allocateEvenClobered(set<x86::X64Register>& ignoreStackSpill) {
    auto available = availableRegs();

    for (auto av : available) {
        if (ignoreStackSpill.contains(av)) continue;
        acquireSpecific(av);
        return {available.front(), false};
    }

    for (auto reg : regs64) {
        if (ignoreStackSpill.contains(reg)) continue;

        return {reg, true};
    }

    std::terminate();
}

vector<x86::X64Register> RegAlloc::availableRegs() {
    vector<x86::X64Register> buf;

    for (auto i : views::iota(0u, GENERAL_PURPOUSE_REG_COUNT)) {
        if (isAvailable(i)) {
            buf.push_back(regs64[i]);
        }
    }

    return buf;
}

string RegAlloc::debugString(size_t handle) const {
    if (stack.isStack(handle)) {
        return stringify("rsp+{}", getStackOffset(handle));
    }
    return getReg(handle).toString();
}