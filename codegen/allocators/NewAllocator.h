#pragma once
#include "Allocator.h"
#include "codegen/x86/RegAlloc.h"
#include "codegen/x86/X86Assembler.h"

template<typename CTX>
struct NewAllocator : Allocator<CTX> {
    typedef size_t RegisterHandle;
    using IRInstruction = IRInstruction<CTX>;

    CTX::ASSEMBLER* assembler;
    CTX::IRGEN* irGen;
    RegAlloc& regAlloc;
    
    std::map<SSARegisterHandle, std::pair<size_t, size_t>> regs;
    size_t currentInstructionCounter = 0;
    LiveRanges currentLiveRanges;
    
    explicit NewAllocator(RegAlloc& regAlloc): regAlloc(regAlloc) {
        
    }

    RegisterHandle allocateTmp(size_t size) override {
        return assembler->allocateRegister(size);
    }

    void freeTmp(RegisterHandle handle) override {
        assembler->freeRegister(handle);
    }

    RegisterHandle getReg(SSARegisterHandle reg) override {
        assert(regs.contains(reg));
        return regs[reg].first;
    }

    void beforeInst(IRInstruction& instruction, size_t time) override {
        currentInstructionCounter = time;
        applyRegs(regAlloc, currentInstructionCounter, regs, currentLiveRanges);
    }

    void afterInst(IRInstruction& instruction, size_t time) override {
        currentInstructionCounter = time;
    }

    void setup(CTX::ASSEMBLER& assembler, LiveRanges currentLiveRanges, CTX::IRGEN& irGen) override {
        this->assembler = &assembler;
        this->irGen = &irGen;
        this->currentLiveRanges = currentLiveRanges;
        regs = allocateRegisters();
    }

    size_t sizeBytes(SSARegisterHandle handle) const {
        return irGen->getRecord(handle).sizeBytes();
    }

    std::map<SSARegisterHandle, std::pair<RegisterHandle , size_t>> allocateRegisters() {
        std::map<SSARegisterHandle, std::pair<RegisterHandle, size_t>> simulated;
        std::set<SSARegisterHandle> aliveSet;

        auto argMap = getArgMap();

        auto doAllocate = [&](SSARegisterHandle reg) -> std::pair<RegisterHandle, size_t> {
            if (argMap.contains(reg)) {
                return {argMap[reg], sizeBytes(reg)};
            }
            return {assembler->allocateRegister(sizeBytes(reg)), sizeBytes(reg)};
        };

        auto regz = currentLiveRanges.regs();
        for (auto i = 0UL; i < currentLiveRanges.length(); i++) {
            for (auto reg : regz) {
                auto isAlive = currentLiveRanges.isAlive(reg, i);
                // no change ignore
                if (isAlive == aliveSet.contains(reg)) continue;
                // becoming alive + was already alive aka resurrected
                if (isAlive && simulated.contains(reg)) {
                    TODO()
                }
                // just alive
                if (isAlive) {
                    auto allocated = doAllocate(reg);
                    simulated.insert({reg, allocated});
                    aliveSet.insert(reg);
                } else {
                    assert(simulated.contains(reg));
                    assembler->freeRegister(simulated.at(reg).first);
                    aliveSet.erase(reg);
                }
            }
        }

        return simulated;
    }

    void applyRegs(RegAlloc& alloc, size_t time, std::map<SSARegisterHandle, std::pair<size_t, size_t>> allocation, LiveRanges& ranges) {
        for (auto reg : ranges.regs()) {
            auto isAlive = ranges.isAlive(reg, time);
            auto wasAlive = ranges.isAliveSafe(reg, (long)time-1);
            auto [item, itemSize] = allocation[reg];

            // alloc
            if (not wasAlive && isAlive) {
                if (alloc.isStack(item)) {
                    alloc.stack.allocateStackRange(alloc.stack.getStackOffset(item), itemSize);
                    alloc.stack.stackAllocations[item] = itemSize;
                } else {
                    alloc.setReg(item, true);
                }
            }
            // dealloc
            else if (wasAlive && not isAlive) {
                if (alloc.isStack(item)) {
                    alloc.stack.deAllocateStackRange(alloc.stack.getStackOffset(item), itemSize);
                    alloc.stack.stackAllocations.erase(item);
                } else {
                    alloc.setReg(item, false);
                }
            }
        }
    }

    std::map<SSARegisterHandle, RegisterHandle> getArgMap() {
        std::map<SSARegisterHandle, RegisterHandle> mapa;

        auto asmRegs = assembler->getArgHandles();
        auto ssaRegs = irGen->root().getArgRegisters();

        assert(asmRegs.size() == ssaRegs.size());

        for (const auto& [asmReg, ssaReg]: views::zip(asmRegs, ssaRegs)) {
            mapa[ssaReg] = asmReg;
        }

        return mapa;
    }
};
