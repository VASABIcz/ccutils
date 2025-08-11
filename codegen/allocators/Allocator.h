#pragma once

#include "codegen/SSARegisterHandle.h"
#include "codegen/IRInstruction.h"
#include "codegen/LiveRanges.h"


template<typename CTX>
struct Allocator {
    typedef size_t RegisterHandle;
    using IRInstruction = IRInstruction<CTX>;

    virtual ~Allocator() = default;

    virtual RegisterHandle allocateTmp(size_t size) = 0;

    virtual void freeTmp(RegisterHandle handle) = 0;

    virtual RegisterHandle getReg(SSARegisterHandle reg) = 0;

    virtual void beforeInst(IRInstruction& instruction, size_t time) = 0;

    virtual void afterInst(IRInstruction& instruction, size_t time) = 0;

    virtual void setup(CTX::ASSEMBLER& assembler, LiveRanges currentLiveRanges, CTX::IRGEN& irGen) {}
};