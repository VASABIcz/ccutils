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
#include "allocators/SimpleAllocator.h"

using namespace std;

template<typename CTX>
class CodeGen {
public:
    typedef CodeBlock<CTX> BaseBlock;
    using IRInstruction = IRInstruction<CTX>;
    typedef size_t RegisterHandle;

    CTX::ASSEMBLER& assembler;
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

    void doPrintLinearized() {
        irGen.graph.printRegisters();
        if (currentLiveRanges.regCount() != 0 && currentLiveRanges.length() != 0) {
            auto max_value = currentLiveRanges.length()-1; // FIXME this aint right
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
            auto text = node->toString(irGen);
            buf += stringify("{} [label=\\\"{}\\\"] [shape=box] [xlabel=\\\"{}\\\"]\n", node->blockId, text, node->tag);
            bool pepa = true;
            for (auto tgt : node->getTargets()) {
                buf += stringify("{} -> {} [color=red] [label=\\\"{}\\\"]\n", node->blockId, tgt, pepa ? "true" : "false");
                pepa = !pepa;
            }
        }
        for (auto i : views::iota(0u, flatBlocks.size()-1)) {
            buf += stringify("{} -> {} [color=blue]\n", flatBlocks[i], flatBlocks[i+1]);
        }
        buf += "}";

        system(stringify("echo \"{}\" | dot -Tpng > {}.png", buf, name).c_str());
    }

    Result<void> gen() {
        // optimizeAssign<CTX>(irGen);
        // optimizePhis<CTX>(irGen);
        // optimizePhis<CTX>(irGen);
        // optimizePhis<CTX>(irGen);

        if (false) {
            println("=== BEFOR FLAT ===");
            irGen.print();
            println("=== AFTER FLAT ===");
        }

        flatBlocks = irGen.graph.flattenBlocks();
        currentLiveRanges = LiveRanges{liveRanges(flatBlocks)};

        fixUnliveRanges();
        fixupPhiLiveRanges(flatBlocks);
        currentLiveRanges.clumpLiveRanges();

        if (printLiveRanges) currentLiveRanges.doPrintLiveRanges();

        // if (printLinearized) doPrintLinearized();

        // bool didOptimize = false;

/*        do {
            didOptimize = false;
            didOptimize |= mergeLiveRanges<CTX>(irGen, currentLiveRanges);
            didOptimize |= removeFallJumps<CTX>(irGen, flatBlocks);
        } while (didOptimize);
        optimizeCumulativeOps<CTX>(irGen, flatBlocks, currentLiveRanges);*/

        forceStackAlloc<CTX>(irGen);

        if (printLiveRanges) currentLiveRanges.doPrintLiveRanges();

        if (printLinearized) doPrintLinearized();

        if (dumpGraphPNG) doDumpGraphPNG();

        allocator->setup(assembler, currentLiveRanges, irGen);

        generateIR();

        return {};
    }

    void generateIR() {
        for (auto blockId : flatBlocks) {
            auto& current = irGen.getBlock(blockId);

            generateCodeBlock(current);
        }
    }

    CodeGen::RegisterHandle allocateTemp(const size_t size) const {
        return allocator->allocateTmp(size);
    }

    void freeTemp(const CodeGen::RegisterHandle handle) const {
        allocator->freeTmp(handle);
    }

    CodeGen::RegisterHandle getReg(const SSARegisterHandle& reg) {
        return allocator->getReg(reg);
    }

    std::optional<CodeGen::RegisterHandle> getReg(std::optional<SSARegisterHandle> reg) {
        if (not reg.has_value()) return {};

        return getReg(*reg);
    }

    optional<CodeGen::RegisterHandle> getRegOrNull(const SSARegisterHandle& reg) {
        if (not reg.isValid()) return {};

        return getReg(reg);
    }

// http://www.christianwimmer.at/Publications/Wimmer10a/Wimmer10a.pdf
    map<SSARegisterHandle, vector<bool>> liveRanges(vector<size_t> blocks) const {
        return irGen.graph.simpleLiveRanges(blocks);
    }

    void generateCodeBlock(const BaseBlock& block) {
        assembler.createLabel(getJmpLabelForBlock(block.id()));
        assembler.bindHint(stringify("== CODE_BLOCK {}", block.id()));

        const auto& instructions = block.getInstructions();

        currentBlock = block.blockId;

        if (instructions.empty()) {
            // assembler.generateRet();
            return;
        }

        generateInstructions(instructions);
    }

    void assignPhis(size_t targetBlock) {
        const auto& block = irGen.getBlock(targetBlock);
        auto phiFunctions = block.getPhis(currentBlock);

        vector<pair<size_t, pair<size_t, size_t>>> atomicPhis;

        // move register to temps before actual write
        // this allows us stuff like:
        // 1:0 := phi 0: 0:0, 1: 1:1
        // 1:1 := phi 0: 0:1, 1: 1:0
        // FIXME this could be optimized to use only single temp register
        // TODO we would need to find dependency cicles
        for (auto [phiSources, phiTarget] : phiFunctions) {
            // std::cout << "ALLOCATING TMP FOR " << sizeBytes(pi.first) << " - " << pi.first.toString() << std::endl;
            auto tmpPhi = allocateTemp(sizeBytes(phiSources));
            assembler.movReg(tmpPhi, getReg(phiSources), 0, 0, sizeBytes(phiSources));
            atomicPhis.emplace_back(getReg(phiTarget), make_pair(tmpPhi, sizeBytes(phiSources)));
        }

        for (auto [targetPhi, other] : atomicPhis) {
            auto [tmpSourcePhi, size] = other;
            assembler.movReg(targetPhi, tmpSourcePhi, 0, 0, size);
            freeTemp(tmpSourcePhi);
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

    size_t getTmp(size_t id) {
        TODO();
    }

    auto getType(SSARegisterHandle handle) const {
        return irGen.getRecord(handle).getType();
    }

    size_t getBasicBlockOffset(size_t id, std::span<size_t> linearized) {
        size_t acu = 0;
        for (auto bb : linearized) {
            if (bb == id) return acu;
            acu += getBlock(bb).instructions.size();
        }
        return 0;
    }

    void fixUnliveRanges() {
        irGen.graph.forEachInstruction([&](auto& inst) {
            auto tgt = inst.target;
            if (not tgt.isValid()) return;
            if (currentLiveRanges.contains(tgt)) return;

            currentLiveRanges.addRange(tgt);
        });
    }

    void fixupPhiLiveRanges(std::span<size_t> linearized) {
        irGen.graph.forEachInstruction([&](auto& inst) {
            if (auto phi = inst.template cst<instructions::PhiFunction<CTX>>(); phi) {
                for (auto [block, value] : phi->getRawVersions()) {
                    auto offset = getBasicBlockOffset(block, linearized);
                    auto size = getBlock(block).getInstructions().size();
                    if (not currentLiveRanges.contains(phi->target)) {
                        println("JSEM UPPPPPPPPPPPPPLNEEEEEEEEEEE V PICIIIIIIIIIIIIIIIIII {}", phi->target);
                        return;
                    }
                    assert(currentLiveRanges.contains(phi->target));
                    assert(offset+size-1 < currentLiveRanges.length());
                    currentLiveRanges.appendRange(phi->target, offset+size-1, true);
                }
            }
        });
    }

    CodeGen(CTX::ASSEMBLER& assembler, CTX::IRGEN& irGen, string name) : assembler(assembler), irGen(irGen), name(name) {}

    bool dumpGraphPNG = false, printLinearized = false, printLiveRanges = false, warnLeak = false;
// private:
    void assignRegisters(IRInstruction& instruction) {
        allocator->beforeInst(instruction, currentInstructionCounter);
    }

    void beforeInstruction(IRInstruction* instruction) {
        this->assembler.nop();
        assembler.instructionNumberHint(currentInstructionCounter);
        std::stringstream ss;
        instruction->print(irGen, ss);
        assembler.bindHint(ss.str());
    }

    template<typename FN>
    void assertStableRegs(IRInstruction* instruction, const FN& fn) {
        auto oldRegs = this->assembler.numRegs();
        fn();
        auto newRegs = this->assembler.numRegs();

        if (newRegs != oldRegs && warnLeak) {
            println("[gen] instruction {} leaks registers old: {} vs new: {}", instruction->name, oldRegs, newRegs);
        }
    }

    void generateInstruction(IRInstruction* instruction) {
        // std::cout << "INDEX " << currentInstructionCounter << std::endl;
        beforeInstruction(instruction);

        assignRegisters(*instruction);
        // std::cout << "INDEX AFTER" << currentInstructionCounter << std::endl;

        assertStableRegs(instruction, [&] {
            this->assembler.beforeInstruction();
            instruction->generate(static_cast<CTX::GEN &>(*this));
        });

        allocator->afterInst(*instruction, currentInstructionCounter);
    }

    void generateInstructions(const vector<CopyPtr<IRInstruction>>& instructions) {
        for (const auto& instruction : instructions) {
            generateInstruction(instruction.get());
            currentInstructionCounter++;
            if (instruction->isTerminal()) break;
        }
    }

    CTX::IRGEN& irGen;

    LiveRanges currentLiveRanges;
    size_t currentBlock = 0;
    size_t currentInstructionCounter = 0;
    std::vector<size_t> flatBlocks;
    string name;
    Allocator<CTX>* allocator;
};