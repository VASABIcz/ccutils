#pragma once
#include "Allocator.h"
#include "codegen/x86/RegAlloc.h"
#include "codegen/x86/X86Assembler.h"
#include <deque>

template<typename CTX>
struct OtherAllocator : Allocator<CTX> {
    typedef size_t RegisterHandle;
    using IRInstruction = IRInstruction<CTX>;

    CTX::ASSEMBLER* assembler;
    CTX::IRGEN* irGen;
    RegAlloc& regAlloc;

    std::map<size_t, std::pair<size_t, size_t>> regs; // intervalId : regId : size
    std::map<SSARegisterHandle, std::vector<size_t>> regIntervals;
    std::map<SSARegisterHandle, std::vector<size_t>> regTimes;
    size_t currentInstructionCounter = 0;
    LiveRanges currentLiveRanges;
    std::map<size_t, std::vector<std::pair<size_t, size_t>>> splitMap; // time -> original : new

    explicit OtherAllocator(RegAlloc& regAlloc): regAlloc(regAlloc) {

    }

    RegisterHandle allocateTmp(size_t size) override {
        return assembler->allocateRegister(size);
    }

    void freeTmp(RegisterHandle handle) override {
        assembler->freeRegister(handle);
    }

    RegisterHandle getReg(SSARegisterHandle reg) override {
        // first interval <= currentInstructionCounter
        auto found = std::upper_bound(regTimes[reg].begin(), regTimes[reg].end(), currentInstructionCounter);
        if (found == regTimes[reg].end()) found = regTimes[reg].end();
        found--;
        // iterator to idex
        auto idex = distance(regTimes[reg].begin(), found);
        // idex to intervalId
        auto inter = regIntervals[reg][idex];

        // intervalId to register
        return regs[inter].first;
    }

    void beforeInst(IRInstruction& instruction, size_t time) override {
        currentInstructionCounter = time;
        freeRegs(regAlloc, currentInstructionCounter, regs, currentLiveRanges);
        allocRegs(regAlloc, currentInstructionCounter, regs, currentLiveRanges);

        std::map<size_t, size_t> asdasd;
        asdasd.upper_bound(3);

        auto rr = splitMap[currentInstructionCounter];
        for (auto r : rr) {
            println("WHAHSDH {} - {}", regAlloc.debugString(regs[r.first].first), regAlloc.debugString(regs[r.second].first));
            assembler->movReg(regs[r.first].first, regs[r.second].first);
        }
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

    void allocRegs(RegAlloc& alloc, size_t time, std::map<size_t, std::pair<size_t, size_t>> allocation, LiveRanges& ranges) {
        for (auto reg : ranges.intervals()) {
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

    void freeRegs(RegAlloc& alloc, size_t time, std::map<size_t, std::pair<size_t, size_t>> allocation, LiveRanges& ranges) {
        for (auto reg : ranges.intervals()) {
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
        auto starts = currentLiveRanges.calcStarts2();

        std::deque<size_t> sorted;
        for (auto pair : starts) {
            sorted.push_back(pair.first);
        }

        std::sort(sorted.begin(), sorted.end(), [&](size_t a, size_t b) {
            return starts[a] < starts[b];
        });

        auto argMap = getArgMap();
        for (auto [k, v] : argMap) {
            currentLiveRanges.collorReg(k, v);
        }

        while (not sorted.empty()) {
            auto currentInt = sorted.front();
            sorted.pop_front();

            auto intersections = currentLiveRanges.intersections2(currentLiveRanges.firstLiveRange(currentInt));

            regAlloc.clear();
            regAlloc.stack.clear();

            // color current allocator
            std::map<size_t, size_t> reg2int;
            for (auto inter : intersections) {
                if (not regs.contains(inter)) continue;

                auto [regIntId, itemSize] = regs[inter];
                reg2int[regIntId] = inter;

                if (regAlloc.isStack(regIntId)) {
                    regAlloc.stack.allocateStackRange(regAlloc.stack.getStackOffset(regIntId), itemSize);
                    regAlloc.stack.stackAllocations[regIntId] = itemSize;
                } else {
                    regAlloc.setReg(regIntId, true);
                }
            }

            auto required = currentLiveRanges.getColoredOne(currentInt);
            if (required.has_value()) { // precolored reg
                if (regAlloc.isAllocated(*required)) {
                    auto badInt = reg2int[*required];
                    // println("GONA SPLIT {} because {}, requiring {}", badInt, currentInt, regAlloc.debugString(*required));
                    auto newInt = currentLiveRanges.split(badInt, starts[currentInt]);
                    starts[newInt] = starts[currentInt];
                    sorted.insert(sorted.begin(), newInt);
                    splitMap[starts[currentInt]].emplace_back(newInt, badInt);
                }

                regs[currentInt] = {*required, 8};
            } else { // uncolored
                auto rr = regAlloc.allocateAny(sizeBytes(currentLiveRanges.getSSA(currentInt)));
                regs[currentInt] = {rr, sizeBytes(currentLiveRanges.getSSA(currentInt))};
            }

            // bind allocated range to ssa
            auto ssa = currentLiveRanges.getSSA(currentInt);
            if (ssa.isValid()) {
                regIntervals[ssa].push_back(currentInt);
                regTimes[ssa].push_back(starts[currentInt]);
            }
        }

        if (false) {
            println("=== AAAAAA ===");
            for (auto in : currentLiveRanges.intervals()) {
                currentLiveRanges.printRange(stringify(currentLiveRanges.getSSA(in).toString()), currentLiveRanges.rangeForReg(in));
                println("{}", regAlloc.debugString(regs[in].first));
            }
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