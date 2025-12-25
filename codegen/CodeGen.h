#pragma once
#include <map>

#include "../utils/utils.h"
#include "../utils/Errorable.h"
#include "../utils/CopyPtr.h"
#include "forward.h"
#include "JumpCondType.h"
#include "LiveRanges.h"
#include "BlockId.h"
#include "Assembler.h"

using namespace std;

template<typename CTX>
class CodeGen {
public:
    typedef CodeBlock<CTX> BaseBlock;
    using IRInstruction = IRInstruction<CTX>;
    typedef size_t RegisterHandle;

    CTX::ASSEMBLER& assembler;
    std::map<BlockId, size_t> labelMapping;

    void jmpBlock(BlockId blockId) {
        assembler.jmp(getJmpLabelForBlock(blockId));
    }

    void jmpBlockTrue(size_t reg, BlockId blockId) {
        assembler.jmpLabelTrue(reg, getJmpLabelForBlock(blockId));
    }

    void jmpBlockFalse(size_t reg, BlockId blockId) {
        assembler.jmpLabelFalse(reg, getJmpLabelForBlock(blockId));
    }

    void jmpBlockCond(JumpCondType type, size_t lhs, size_t rhs, BlockId blockId) {
        assembler.jmpCond(getJmpLabelForBlock(blockId), type, lhs, rhs);
    }

    size_t getJmpLabelForBlock(BlockId block);

    void doPrintLinearized();

    void doDumpGraphPNG();

    Result<void> gen();

    void generateIR();

    CodeGen::RegisterHandle allocateTemp(const size_t size) const;

    void freeTemp(const CodeGen::RegisterHandle handle) const;

    CodeGen::RegisterHandle getReg(const SSARegisterHandle& reg);

    std::optional<CodeGen::RegisterHandle> getReg(std::optional<SSARegisterHandle> reg) {
        if (not reg.has_value()) return {};

        return getReg(*reg);
    }

    std::vector<CodeGen::RegisterHandle> getReg(std::span<SSARegisterHandle> regs) {
        std::vector<CodeGen::RegisterHandle> out;

        for (auto reg : regs) {
            out.push_back(getReg(reg));
        }

        return out;
    }

    optional<CodeGen::RegisterHandle> getRegOrNull(const SSARegisterHandle& reg) {
        if (not reg.isValid()) return {};

        return getReg(reg);
    }

// http://www.christianwimmer.at/Publications/Wimmer10a/Wimmer10a.pdf
    LiveRanges liveRanges(vector<BlockId> blocks) const {
        return irGen.graph.simpleLiveRanges(blocks);
    }

    void generateCodeBlock(const BaseBlock& block);

    void assignPhis(BlockId targetBlock);

    size_t nextLabel() {
        return assembler.allocateJmpLabel();
    }

    vector<CodeGen::RegisterHandle> getRegs(span<SSARegisterHandle> regs);

    auto getBlock(BlockId id) -> BaseBlock&;

    size_t sizeBytes(SSARegisterHandle handle) const;

    bool mustAssignPhis(BlockId targetBlock);

    size_t getTmp(size_t id) {
        TODO();
    }

    auto getType(SSARegisterHandle handle) const;

    size_t getBasicBlockOffset(BlockId id, std::span<BlockId> linearized);

    void fixUnliveRanges();

    void fixupPhiLiveRanges(std::span<BlockId> linearized);

    CodeGen(CTX::ASSEMBLER& assembler, CTX::IRGEN& irGen, string name) : assembler(assembler), irGen(irGen), name(name) {}

    bool dumpGraphPNG = false, printLinearized = false, printLiveRanges = false, warnLeak = false;
// private:
    void assignRegisters(IRInstruction& instruction);

    void beforeInstruction(IRInstruction* instruction);

    template<typename FN>
    void assertStableRegs(IRInstruction* instruction, const FN& fn) {
        auto oldRegs = this->assembler.numRegs();
        fn();
        auto newRegs = this->assembler.numRegs();

        if (newRegs != oldRegs && warnLeak) {
            println("[gen] instruction {} leaks registers old: {} vs new: {}", instruction->toString(irGen), oldRegs, newRegs);
            PANIC();
        }
    }

    void generateInstruction(IRInstruction* instruction);

    void generateInstructions(const vector<CopyPtr<IRInstruction>>& instructions);

    CTX::IRGEN& irGen;

    LiveRanges currentLiveRanges;
    BlockId currentBlock = BlockId::invalid();
    size_t currentInstructionCounter = 0;
    std::vector<BlockId> flatBlocks;
    string name;
    Allocator<CTX>* allocator;
};