#pragma once
#include "Allocator.h"
#include "codegen/x86/RegAlloc.h"
#include "codegen/x86/X86Assembler.h"

template<typename CTX>
struct BetterAllocator : Allocator<CTX> {
    typedef size_t RegisterHandle;
    using IRInstruction = IRInstruction<CTX>;

    CTX::ASSEMBLER* assembler;
    CTX::IRGEN* irGen;
    RegAlloc& regAlloc;

    std::map<SSARegisterHandle, std::pair<size_t, size_t>> regs;
    size_t currentInstructionCounter = 0;
    LiveRanges currentLiveRanges;

    explicit BetterAllocator(RegAlloc& regAlloc): regAlloc(regAlloc) {

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
        freeRegs(regAlloc, currentInstructionCounter, regs, currentLiveRanges);
        allocRegs(regAlloc, currentInstructionCounter, regs, currentLiveRanges);
    }

    void afterInst(IRInstruction& instruction, size_t time) override {
        currentInstructionCounter = time;
    }

    void setup(CTX::ASSEMBLER& assembler, LiveRanges currentLiveRanges, CTX::IRGEN& irGen) override {
        this->assembler = &assembler;
        this->irGen = &irGen;
        this->currentLiveRanges = currentLiveRanges;
        idk();
    }

    void allocRegs(RegAlloc& alloc, size_t time, std::map<SSARegisterHandle, std::pair<size_t, size_t>> allocation, LiveRanges& ranges) {
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
        }
    }

    void freeRegs(RegAlloc& alloc, size_t time, std::map<SSARegisterHandle, std::pair<size_t, size_t>> allocation, LiveRanges& ranges) {
        for (auto reg : ranges.regs()) {
            auto isAlive = ranges.isAlive(reg, time);
            auto wasAlive = ranges.isAliveSafe(reg, (long)time-1);
            auto [item, itemSize] = allocation[reg];

            // dealloc
            if (wasAlive && not isAlive) {
                if (alloc.isStack(item)) {
                    alloc.stack.deAllocateStackRange(alloc.stack.getStackOffset(item), itemSize);
                    alloc.stack.stackAllocations.erase(item);
                } else {
                    alloc.setReg(item, false);
                }
            }
        }
    }


    void idk() {
        auto a = currentLiveRanges.calcLength();
        auto b = irGen->graph.calcUseCount();
        auto c = currentLiveRanges.calcStarts();
        std::vector<SSARegisterHandle> sorted;
        for (auto pair : a) {
            sorted.push_back(pair.first);
        }

        const auto calcPrio = [&](SSARegisterHandle reg) {
            auto length = a.contains(reg) ? a[reg] : 1;
            auto useCount = b.contains(reg) ? b[reg] : 1;
            auto v = (double)useCount/(double)(length*length);
            return v;
        };

        double min = 3000;
        SSARegisterHandle minR;
        for (auto [reg, length] : a) {
            auto useCount = b[reg];
            auto v = (double)useCount/(double)(length*length);
            if (v < min) {
                min = v;
                minR = reg;
            }
            // println("[PRIORITY] {} - {} - {} - {}", reg, v, length, useCount);
        }
        // println("[PRIORITY] MIN {} - {}", minR, min);

        std::sort(sorted.begin(), sorted.end(), [&](SSARegisterHandle a, SSARegisterHandle b) {
            auto a1 = c.contains(a) ? c[a] : 1;
            auto b1 = c.contains(b) ? c[b] : 1;

            if (calcPrio(a) > calcPrio(b)) return true;
            if (calcPrio(a) == calcPrio(b)) return a1 < b1;

            return false;
        });

        std::map<SSARegisterHandle, std::pair<size_t, size_t>> allocated;

        auto argMap = getArgMap();
        for (auto [k, v] : argMap) {
            allocated[k] = {v, sizeBytes(k)};
        }

        // println("=== XD ===");
        // currentLiveRanges.doPrintLiveRanges();

        for (auto reg : sorted) {
            if (allocated.contains(reg)) continue;
            assert(currentLiveRanges.currentLiveRanges[SSARegisterHandle::valid(0,0)][0]);
            auto intersections = currentLiveRanges.intersections(currentLiveRanges.firstLiveRange(reg));

            assert(currentLiveRanges.currentLiveRanges[SSARegisterHandle::valid(0,0)][0]);

            regAlloc.clear();
            regAlloc.stack.clear();

            for (auto inter : intersections) {
                if (not allocated.contains(inter)) continue;
                auto [item, itemSize] = allocated[inter];

                assert(currentLiveRanges.currentLiveRanges[SSARegisterHandle::valid(0,0)][0]);

                if (regAlloc.isStack(item)) {
                    regAlloc.stack.allocateStackRange(regAlloc.stack.getStackOffset(item), itemSize);
                    regAlloc.stack.stackAllocations[item] = itemSize;
                } else {
                    regAlloc.setReg(item, true);
                }
            }

            allocated[reg] = {regAlloc.allocateAny(sizeBytes(reg)), sizeBytes(reg)};
            assert(currentLiveRanges.currentLiveRanges[SSARegisterHandle::valid(0,0)][0]);
        }

        regs = allocated;

        println("=== AAAAAA ===");
        for (auto reg : sorted) {
            LiveRanges::printRange(reg, currentLiveRanges.currentLiveRanges[reg]);
            println("{}", regAlloc.debugString(regs[reg].first));
        }

        regAlloc.clear();
        regAlloc.stack.clear();
    }

    size_t sizeBytes(SSARegisterHandle handle) const {
        return irGen->getRecord(handle).sizeBytes();
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
