#pragma once
#include "Allocator.h"

template<typename CTX>
struct SimpleAlloc: Allocator<CTX> {
    typedef size_t RegisterHandle;
    using IRInstruction = IRInstruction<CTX>;

    CTX::ASSEMBLER* assembler = nullptr;
    CTX::IRGEN* irGen = nullptr;
    LiveRanges currentLiveRanges;

    map<SSARegisterHandle, size_t> regs;
    map<SSARegisterHandle, size_t> alreadyAllocated;
    size_t currentInstructionCounter = 0;

    explicit SimpleAlloc() {}

    RegisterHandle allocateTmp(size_t size) override {
        return assembler->allocateRegister(size);
    }

    void freeTmp(RegisterHandle handle) override {
        assembler->freeRegister(handle);
    }

    RegisterHandle getReg(SSARegisterHandle reg) override {
        assert(reg.isValid());

        if (regs.contains(reg)) {
            return regs.at(reg);
        }

        PANIC("trying to get allocated reg {}", reg)
    }

    void beforeInst(IRInstruction& instruction, size_t time) override {
        currentInstructionCounter = time;
        assignRegisters(instruction);
    }

    void afterInst(IRInstruction& instruction, size_t time) override {
        currentInstructionCounter = time;
        vector<SSARegisterHandle> toFree;

        auto canBeDeallocated = [&](SSARegisterHandle reg) {
            return not currentLiveRanges.isAnyAlive(reg, {currentInstructionCounter, currentLiveRanges.length()-1});
        };

        for (const auto& reg : regs) {
            if (canBeDeallocated(reg.first)) {
                toFree.push_back(reg.first);
            }
        }

        // here bcs if it would be done inline it would case sigsegv
        for (auto reg : toFree) {
            // println("DEALOCATING! {}", reg);
            freeRegister(reg);
        }
    }

    void setup(CTX::ASSEMBLER& assembler, LiveRanges currentLiveRanges, CTX::IRGEN& irGen) override {
        this->assembler = &assembler;
        this->currentLiveRanges = currentLiveRanges;
        this->irGen = &irGen;

        initializeArgs();
    }

    void initializeArgs() {
        auto argMap = getArgMap();
        for (auto [ssa, asmReg] : argMap) {
            internalAllocateRegister(ssa, asmReg);
        }
    }

    RegisterHandle internalAllocateRegister(SSARegisterHandle tgt, const RegisterHandle registerHandle, bool check = true) {
        if (alreadyAllocated.contains(tgt) && check) {
            println("trying to double allocate: {} at {}", tgt.toString(), currentInstructionCounter);
            assert(false);
        }

        assert(tgt.isValid());

        assert(!regs.contains(tgt));
        regs.emplace(tgt, registerHandle);
        alreadyAllocated.insert({tgt, registerHandle});
        // println("[GEN] allocating {} -> {}", tgt, assembler->toString(registerHandle));

        return registerHandle;
    }

    void freeRegister(SSARegisterHandle reg) {
        if (!regs.contains(reg)) std::terminate();
        auto r = regs.at(reg);
        // println("[GEN] freeing {} -> {}", reg, assembler->toString(r));
        regs.erase(reg);
        assembler->freeRegister(r);
    }

    void assignRegisters(IRInstruction& instruction) {
        for (auto& reg : currentLiveRanges.regs()) {
            if (regs.contains(reg) || !currentLiveRanges.isAlive(reg, currentInstructionCounter)) continue;

            // handle allocating of alloca instructions
            auto alloca = instruction.template cst<instructions::Alloca>();
            if (alloca != nullptr && alloca->target == reg) {
                doAlloca(alloca->target, alloca->size);
                continue;
            }
            // arg is phony
            auto arg = instruction.template cst<instructions::Arg>();;
            if (arg != nullptr && arg->target == reg) continue;

            auto phi = instruction.template cst<instructions::PhiFunction>();
            if (phi != nullptr && phi->target == reg) {
                if (not regs.contains(reg)) allocateRegister(reg);
                continue;
            }

            allocateRegister(reg);
        }
    }

    RegisterHandle doAlloca(const SSARegisterHandle& reg, size_t size) {
        return internalAllocateRegister(reg, assembler->allocateRegister(size));
    }

    RegisterHandle allocateRegister(const SSARegisterHandle& tgt, bool check = true) {
        if (!tgt.isValid()) PANIC();
        size_t registerHandle;
        if (this->irGen->getRecord(tgt).isForceStack()) {
            registerHandle = assembler->allocateStack(sizeBytes(tgt));
        }
        else {
            registerHandle = assembler->allocateRegister(sizeBytes(tgt));
        }
        auto handle = internalAllocateRegister(tgt, registerHandle, check);
        return handle;
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

    size_t getRegTotallyUnsafeDontUseThis(SSARegisterHandle handle) {
        if (regs.contains(handle)) {
            return regs.at(handle);
        }
        if (alreadyAllocated.contains(handle)) {
            TODO();
        }

        return allocateRegister(handle, false);
    }
};
