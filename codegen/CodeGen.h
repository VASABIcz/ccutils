#pragma once
#include <map>

#include "../utils/utils.h"
#include "../utils/Errorable.h"
#include "../utils/CopyPtr.h"
#include "SSARegisterHandle.h"
#include "IRInstruction.h"
#include "forward.h"
#include "Assembler.h"
#include "optimizations.h"

using namespace std;

template<typename CTX>
class CodeGen {
public:
    typedef CodeBlock<CTX> BaseBlock;
    CTX::ASSEMBLER& assembler;
    typedef size_t RegisterHandle;
    std::map<size_t, size_t> labelMapping;

    void jmpBlock(size_t blockId) {
        assembler.jmp(getJmpLabelForBlock(blockId));
    }

    void jmpBlockTrue(size_t reg, size_t blockId) {
        assembler.jmpLabelTrue(reg, getJmpLabelForBlock(blockId));
    }

    void jmpBlockFalse(size_t reg, size_t blockId) {
        assembler.jmpLabelFalse(reg, getJmpLabelForBlock(blockId));
    }

    void jmpBlockCond(JumpCondType type, size_t lhs, size_t rhs, size_t blockId) {
        assembler.jmpCond(getJmpLabelForBlock(blockId), type, lhs, rhs);
    }

    size_t getJmpLabelForBlock(size_t block) {
        if (labelMapping.contains(block)) return labelMapping.at(block);

        auto label = assembler.allocateJmpLabel();
        labelMapping[block] = label;

        return label;
    }

    CodeGen::RegisterHandle allocateRegister(const SSARegisterHandle& tgt) {
        if (!tgt.isValid()) PANIC();
        size_t registerHandle;
        if (this->irGen.getRecord(tgt).isForceStack()) {
            registerHandle = assembler.allocateStack(sizeBytes(tgt));
        }
        else {
            registerHandle = assembler.allocateRegister(sizeBytes(tgt));
        }
        auto handle = internalAllocateRegister(tgt, registerHandle);
        return handle;
    }

    CodeGen::RegisterHandle internalAllocateRegister(SSARegisterHandle tgt, const RegisterHandle registerHandle) {
        if (alreadyAllocated.contains(tgt)) {
            println("trying to double allocate: {} at {}", tgt.toString(), currentInstructionCounter);
            assert(false);
        }

        assert(tgt.isValid());

        assert(!regs.contains(tgt));
        regs.emplace(tgt, registerHandle);
        alreadyAllocated.insert({tgt, registerHandle});
        // println("[GEN] allocating {} -> {}", tgt, assembler.toString(registerHandle));

        return registerHandle;
    }

    void initializeArgs(const BaseBlock& root) {
        auto asmRegs = assembler.getArgHandles();
        auto ssaRegs = root.getArgRegisters();

        assert(asmRegs.size() == ssaRegs.size());

        for (const auto& [asmReg, ssaReg]: views::zip(asmRegs, ssaRegs)) {
            internalAllocateRegister(root.getTarget(ssaReg->id), asmReg);
        }
    }

    void deallocatePending() {
        vector<SSARegisterHandle> toFree;

        auto canBeDeallocated = [&](SSARegisterHandle reg) {
            auto& bits = currentLiveRanges[reg];

            for (auto i = currentInstructionCounter; i < bits.size(); i++) {
                if (bits[i]) return false;
            }

            return true;
        };

        for (const auto& reg : regs) {
            if (canBeDeallocated(reg.first)) {
                toFree.push_back(reg.first);
            }
        }

        // here bcs if it would be done inline it would case sigsegv
        for (auto reg : toFree) {
            freeRegister(reg);
        }
    }

    void doPrintLiveRanges() {
        for (auto& range: currentLiveRanges) {
            vector<string> subranges;

            optional<size_t> start = nullopt;
            for (auto [index, v]: range.second | views::enumerate) {
                if (!start.has_value() && v) {
                    start = index;
                    continue;
                }
                if (not v && start.has_value()) {
                    subranges.push_back(stringify("{}..{}", *start, index - 1));
                    start = nullopt;
                }
            }
            if (start.has_value() && not range.second.empty()) {
                subranges.push_back(stringify("{}..{}", *start, range.second.size() - 1));
                start = nullopt;
            }

            println("{} {} {}", stringify(range.second, {StringifyCtx{"", "", ""}}), range.first, subranges);
        }
    }

    void doPrintLinearized() {
        if (!currentLiveRanges.empty() && !(*currentLiveRanges.begin()).second.empty()) {
            auto max_value = (*currentLiveRanges.begin()).second.size()-1; // FIXME this aint right
            auto max_value_len = to_string(max_value).size();

            println("== BINARY LAYOUT == {}", name);
            auto id = 0u;
            for (auto blockId : flatBlocks) {
                cout << string(max_value_len, ' ') << " # " << blockId << " (" << getBlock(blockId).tag << ")" << (getBlock(blockId).isLoopHeader() ? " loop-header" : "") << endl;
                for (const auto& inst : irGen.getBlock(blockId).instructions) {
                    auto ajd = to_string(id);
                    cout << ajd << string(max_value_len-ajd.size(), ' ') << " | ";
                    inst->print(irGen);
                    id++;
                }
            }
            println("== END ==");
        }
    }

    void doDumpGraphPNG() {
        string buf = "digraph {\n";
        for (const auto& node : irGen.graph.validNodesConst()) {
            /*       if (node->previousId().has_value()) {
                        buf += stringify("{} -> {} [color=green]\n", *node->previousId(), node->blockId);
                    }*/
            buf += stringify("{} [label=\\\"{}\\\"]\n", node->blockId, node->tag);
            for (auto tgt : node->getTargets())
                buf += stringify("{} -> {} [color=red]\n", node->blockId, tgt);
        }
        for (auto i : views::iota(0u, flatBlocks.size()-1)) {
            buf += stringify("{} -> {} [color=blue]\n", flatBlocks[i], flatBlocks[i+1]);
        }
        buf += "}";

        system(stringify("echo \"{}\" | dot -Tpng > {}.png", buf, name).c_str());
    }

    Result<void> gen() {
        optimizePhis<CTX>(irGen);

        flatBlocks = irGen.graph.flattenBlocks();
        currentLiveRanges = liveRanges(flatBlocks);

        if (printLiveRanges) doPrintLiveRanges();

        if (printLinearized) doPrintLinearized();

        bool didOptimize = false;

        do {
            didOptimize = false;
            didOptimize |= optimizeAssign<CTX>(irGen);
            didOptimize |= mergeLiveRanges<CTX>(irGen, currentLiveRanges);
            didOptimize |= removeFallJumps<CTX>(irGen, flatBlocks);
        } while (didOptimize);
        optimizeCumulativeOps<CTX>(irGen, flatBlocks, currentLiveRanges);

        forceStackAlloc<CTX>(irGen);

        if (printLiveRanges) doPrintLiveRanges();

        if (printLinearized) doPrintLinearized();

        if (dumpGraphPNG) doDumpGraphPNG();

        if (irGen.graph.nodeCount() != 0)
            initializeArgs(irGen.root());

        for (auto blockId : flatBlocks) {
            auto& current = irGen.getBlock(blockId);

            generateCodeBlock(current);
        }

        return {};
    }

    void freeRegister(SSARegisterHandle reg) {
        if (!regs.contains(reg)) std::terminate();
        auto r = regs.at(reg);
        // println("[GEN] freeing {} -> {}", reg, assembler.toString(r));
        regs.erase(reg);
        assembler.freeRegister(r);
    }

    CodeGen::RegisterHandle allocateTemp(const size_t size) const {
        return assembler.allocateRegister(size);
    }

    void freeTemp(const CodeGen::RegisterHandle handle) const {
        assembler.freeRegister(handle);
    }

    CodeGen::RegisterHandle getReg(const SSARegisterHandle& reg) {
        assert(reg.isValid());

        if (regs.contains(reg)) {
            return regs.at(reg);
        }

        return allocateRegister(reg);
    }

    std::optional<CodeGen::RegisterHandle> getReg(std::optional<SSARegisterHandle> reg) {
        if (not reg.has_value()) return {};

        return getReg(*reg);
    }

    optional<CodeGen::RegisterHandle> getRegOrNull(const SSARegisterHandle& reg) {
        if (not reg.isValid()) return {};

        return getReg(reg);
    }

    void assignPhi(SSARegisterHandle target, SSARegisterHandle source) {
        assembler.movReg(getRegTotallyUnsafeDontUseThis(target), getReg(source));
    }

    void validate() {
        if (!regs.empty()) {
            println("[ERROR] leaked registers: {}", regs);
            // PANIC();
        }
    }

// http://www.christianwimmer.at/Publications/Wimmer10a/Wimmer10a.pdf
    map<SSARegisterHandle, vector<bool>> liveRanges(vector<size_t> blocks) const {
        return irGen.graph.liveRanges(blocks);
    }

    void generateCodeBlock(const BaseBlock& block) {
        // println("[GEN] generating block {}", block.blockId);
        assembler.createLabel(getJmpLabelForBlock(block.id()));
        assembler.bindHint(stringify("== CODE_BLOCK {}", block.id()));

        const auto& instructions = block.getInstructions();

        currentBlock = block.blockId;

        if (instructions.empty()) {
            assembler.generateRet();
            return;
        }

        generateInstructions(instructions);
    }

    void assignPhis(size_t targetBlock) {
        const auto& block = irGen.getBlock(targetBlock);
        auto pis = block.getPhis(currentBlock);

        vector<pair<size_t, pair<size_t, size_t>>> idks;

        // move register to temps before actual write
        // this allows us stuff like:
        // 1:0 := phi 0: 0:0, 1: 1:1
        // 1:1 := phi 0: 0:1, 1: 1:0
        // FIXME this could be optimized to use only single temp register
        // TODO we would need to find dependency cicles
        for (auto pi : pis) {
            // std::cout << "ALLOCATING TMP FOR " << sizeBytes(pi.first) << " - " << pi.first.toString() << std::endl;
            auto reg = allocateTemp(sizeBytes(pi.first));
            assembler.movReg(reg, getReg(pi.first), 0, 0, sizeBytes(pi.first));
            idks.emplace_back(getRegTotallyUnsafeDontUseThis(pi.second), make_pair(reg, sizeBytes(pi.first)));
        }

        for (auto idk : idks) {
            assembler.movReg(idk.first, idk.second.first, 0, 0, idk.second.second);
            freeTemp(idk.second.first);
        }
    }

    size_t nextLabel() {
        return assembler.allocateJmpLabel();
    }

    vector<CodeGen::RegisterHandle> getRegs(span<SSARegisterHandle> regs) {
        vector<RegisterHandle> buf;

        for (auto reg : regs) {
            buf.push_back(getReg(reg));
        }

        return buf;
    }

    size_t getRegTotallyUnsafeDontUseThis(SSARegisterHandle handle) {
        if (regs.contains(handle)) {
            return regs.at(handle);
        }
        if (alreadyAllocated.contains(handle)) {
            return alreadyAllocated[handle];
        }

        return getReg(handle);
    }

    auto getBlock(size_t id) -> BaseBlock& {
        return irGen.getBlock(id);
    }

    size_t sizeBytes(SSARegisterHandle handle) const {
        return irGen.getRecord(handle).sizeBytes();
    }

    bool mustAssignPhis(size_t targetBlock) {
        const auto& block = irGen.getBlock(targetBlock);
        auto pis = block.getPhis(currentBlock);

        return not pis.empty();
    }

    auto getType(SSARegisterHandle handle) const {
        return irGen.getRecord(handle).getType();
    }


    CodeGen(CTX::ASSEMBLER& assembler, CTX::IRGEN& irGen, string name) : assembler(assembler), irGen(irGen), name(name) {}

    bool dumpGraphPNG = false, printLinearized = false, printLiveRanges = false, warnLeak = false;
private:
    CodeGen::RegisterHandle doAlloca(const SSARegisterHandle& reg, size_t size) {
        return internalAllocateRegister(reg, assembler.allocateRegister(size));
    }

    void generateInstructions(const vector<CopyPtr<IRInstruction<CTX>>>& instructions) {
        for (const auto& [id, instruction] : instructions | views::enumerate) {
            this->assembler.nop();
            assembler.instructionNumberHint(currentInstructionCounter);
            std::stringstream ss;
            instruction->print(irGen, ss);
            assembler.bindHint(ss.str());

            // cout << "[GEN] generating: ";
            // instruction->print(this->irGen);

            for (auto& [reg, range] : currentLiveRanges) {
                if (range.empty()) continue; // aka. if register is never used
                if (regs.contains(reg) || !range[currentInstructionCounter]) continue;

                // handle allocating of alloca instructions
                auto alloca = dynamic_cast<instructions::Alloca<CTX>*>(instruction.get());
                if (alloca != nullptr) {
                    doAlloca(alloca->target, alloca->size);
                    continue;
                }
                allocateRegister(reg);
            }

            auto tgt = instruction->target;
            if (tgt.isValid() && not currentLiveRanges.contains(tgt)) {
                allocateRegister(tgt);
            }

            auto oldRegs = this->assembler.numRegs();

            this->assembler.doStuff();
            instruction->generate(static_cast<CTX::GEN&>(*this));
            currentInstructionCounter++;

            auto newRegs = this->assembler.numRegs();

            if (newRegs != oldRegs && warnLeak) {
                println("[gen] instruction {} leaks registers old: {} vs new: {}", instruction->name, oldRegs, newRegs);
            }

            deallocatePending();

            if (instruction->isTerminal()) break;
        }
    }

    CTX::IRGEN& irGen;

    size_t labelCounter = 0;
    map<SSARegisterHandle, size_t> regs;
    map<SSARegisterHandle, size_t> alreadyAllocated;
    size_t currentBlock = 0;
    size_t currentInstructionCounter = 0;
    map<SSARegisterHandle, vector<bool>> currentLiveRanges;
    std::vector<size_t> flatBlocks;
    string name;
};