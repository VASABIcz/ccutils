#pragma once
#include "CodeGen.h"
#include "allocators/Allocator.h"
#include "optimizations.h"

template<typename CTX>
size_t CodeGen<CTX>::getJmpLabelForBlock(BlockId block) {
    if (labelMapping.contains(block)) return labelMapping.at(block);

    auto label = assembler.allocateJmpLabel();
    labelMapping[block] = label;

    return label;
}
template<typename CTX>
void CodeGen<CTX>::doDumpGraphPNG() {
    string buf = "digraph {\n";
    for (const auto& node: irGen.graph.validNodesConst()) {
        auto text = node->toString();
        buf += stringify("{} [label=\\\"{}\\\"] [shape=box] [xlabel=\\\"{}\\\"]\n", node->blockId, text, node->tag);
        bool pepa = true;
        for (auto tgt: node->getTargets()) {
            buf += stringify("{} -> {} [color=red] [label=\\\"{}\\\"]\n", node->blockId, tgt, pepa ? "true" : "false");
            pepa = !pepa;
        }
    }
    for (auto i: views::iota(0u, flatBlocks.size() - 1)) {
        buf += stringify("{} -> {} [color=blue]\n", flatBlocks[i], flatBlocks[i + 1]);
    }
    buf += "}";

    system(stringify("echo \"{}\" | dot -Tpng > {}.png", buf, name).c_str());
}
template<typename CTX>
Result<void> CodeGen<CTX>::gen() {
    // optimizeAssign<CTX>(irGen);
    // optimizePhis<CTX>(irGen);
    // optimizePhis<CTX>(irGen);
    // optimizePhis<CTX>(irGen);
    // irGen.graph.genPhis();

    if (false) {
        println("=== BEFOR FLAT ===");
        irGen.print();
        println("=== AFTER FLAT ===");
    }

    flatBlocks = irGen.graph.flattenBlocks();

    if (printLinearized) doPrintLinearized();

    currentLiveRanges = LiveRanges{liveRanges(flatBlocks)};

    ControlFlowGraph<CTX>& g = irGen.graph;
/*

    auto [tt, lookup] = g.blockOffsets(flatBlocks);
            g.forEachInstruction([&](auto& inst, auto& block, auto id) {
            if (inst.template is<X86Call>()) {
                for (auto reg : x86::SYSV_CALLER) {
                    auto intId = currentLiveRanges.addRange(SSARegisterHandle::invalid());
                    currentLiveRanges.collorReg(intId, RegAlloc::regToHandle(reg));
                    currentLiveRanges.appendRange(intId, lookup[block.id()]+id, true);
                }
            }
        });
*/

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

    // optimizations::forceStackAlloc<CTX>(irGen);

    if (printLiveRanges) currentLiveRanges.doPrintLiveRanges();

    if (printLinearized) doPrintLinearized();

    if (dumpGraphPNG) doDumpGraphPNG();

    allocator->setup(assembler, currentLiveRanges, irGen);

    generateIR();

    return {};
}

template<typename CTX>
void CodeGen<CTX>::generateIR() {
    for (auto blockId: flatBlocks) {
        auto& current = irGen.getBlock(blockId);

        generateCodeBlock(current);
    }
}

template<typename CTX>
void CodeGen<CTX>::generateCodeBlock(const CodeGen::BaseBlock& block) {
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

template<typename CTX>
void CodeGen<CTX>::assignPhis(BlockId targetBlock) {
    const auto& block = irGen.getBlock(targetBlock);
    auto phiFunctions = block.getPhis(currentBlock);

    vector<pair<size_t, pair<size_t, size_t>>> atomicPhis;

    // move register to temps before actual write
    // this allows us stuff like:
    // 1:0 := phi 0: 0:0, 1: 1:1
    // 1:1 := phi 0: 0:1, 1: 1:0
    // FIXME this could be optimized to use only single temp register
    // TODO we would need to find dependency cicles
    for (auto [phiSources, phiTarget]: phiFunctions) {
        // std::cout << "ALLOCATING TMP FOR " << sizeBytes(pi.first) << " - " << pi.first.toString() << std::endl;
        auto tmpPhi = allocateTemp(sizeBytes(phiSources));
        assembler.movReg(tmpPhi, getReg(phiSources), 0, 0, sizeBytes(phiSources));
        atomicPhis.emplace_back(getReg(phiTarget), make_pair(tmpPhi, sizeBytes(phiSources)));
    }

    for (auto [targetPhi, other]: atomicPhis) {
        auto [tmpSourcePhi, size] = other;
        assembler.movReg(targetPhi, tmpSourcePhi, 0, 0, size);
        freeTemp(tmpSourcePhi);
    }
}

template<typename CTX>
vector<typename CodeGen<CTX>::RegisterHandle> CodeGen<CTX>::getRegs(span<SSARegisterHandle> regs) {
    vector<RegisterHandle> buf;

    for (auto reg: regs) {
        buf.push_back(getReg(reg));
    }

    return buf;
}

template<typename CTX>
bool CodeGen<CTX>::mustAssignPhis(BlockId targetBlock) {
    const auto& block = irGen.getBlock(targetBlock);
    auto pis = block.getPhis(currentBlock);

    return not pis.empty();
}

template<typename CTX>
size_t CodeGen<CTX>::getBasicBlockOffset(BlockId id, std::span<BlockId> linearized) {
    size_t acu = 0;
    for (auto bb: linearized) {
        if (bb == id) return acu;
        acu += getBlock(bb).instructions.size();
    }
    return 0;
}

template<typename CTX>
void CodeGen<CTX>::fixUnliveRanges() {
    irGen.graph.forEachInstruction([&](auto& inst) {
        auto tgt = inst.target;
        if (not tgt.isValid()) return;
        if (currentLiveRanges.contains(tgt)) return;

        currentLiveRanges.addRange(tgt);
    });
}

template<typename CTX>
void CodeGen<CTX>::fixupPhiLiveRanges(std::span<BlockId> linearized) {
    irGen.graph.forEachInstruction([&](auto& inst) {
        if (auto phi = inst.template cst<instructions::PhiFunction<CTX>>(); phi) {
            for (auto [block, value]: phi->getRawVersions()) {
                auto offset = getBasicBlockOffset(block, linearized);
                auto size = getBlock(block).getInstructions().size();
                if (not currentLiveRanges.contains(phi->target)) {
                    return;
                }
                assert(currentLiveRanges.contains(phi->target));
                assert(offset + size - 1 < currentLiveRanges.length());
                currentLiveRanges.appendRange(phi->target, offset + size - 1, true);
            }
        }
    });
}

template<typename CTX>
void CodeGen<CTX>::beforeInstruction(IRInstruction* instruction) {
    this->assembler.nop();
    assembler.instructionNumberHint(currentInstructionCounter);
    std::stringstream ss;
    instruction->print(ss);
    assembler.bindHint(ss.str());
}

template<typename CTX>
void CodeGen<CTX>::generateInstruction(IRInstruction* instruction) {
    beforeInstruction(instruction);

    assignRegisters(*instruction);

    assertStableRegs(instruction, [&] {
        this->assembler.beforeInstruction();
        instruction->generate(static_cast<CTX::GEN&>(*this));
    });

    allocator->afterInst(*instruction, currentInstructionCounter);
}

template<typename CTX>
void CodeGen<CTX>::generateInstructions(const vector<CopyPtr<IRInstruction>>& instructions) {
    for (const auto& instruction: instructions) {
        generateInstruction(instruction.get());
        currentInstructionCounter++;
        if (instruction->isTerminal()) break;
    }
}

template<typename CTX>
void CodeGen<CTX>::doPrintLinearized() {
    irGen.graph.printRegisters();

    auto max_value = currentLiveRanges.length() - 1; // FIXME this aint right
    auto max_value_len = to_string(max_value).size();

    println("== BINARY LAYOUT == {}", name);
    auto id = 0u;
    for (auto blockId: flatBlocks) {
        cout << string(max_value_len, ' ') << " # " << blockId.toString() << " (" << getBlock(blockId).tag << ")" << (getBlock(blockId).isLoopHeader() ? " loop-header" : "") << endl;
        for (const auto& inst: irGen.getBlock(blockId).instructions) {
            auto ajd = to_string(id);
            cout << ajd << string(max_value_len - ajd.size(), ' ') << " | ";
            inst->print();
            id++;
        }
    }
    println("== END ==");
}

template<typename CTX>
CodeGen<CTX>::RegisterHandle CodeGen<CTX>::getReg(const SSARegisterHandle& reg) {
    return allocator->getReg(reg);
}
template<typename CTX>
CodeGen<CTX>::RegisterHandle CodeGen<CTX>::allocateTemp(const size_t size) const {
    return allocator->allocateTmp(size);
}
template<typename CTX>
void CodeGen<CTX>::freeTemp(const CodeGen::RegisterHandle handle) const {
    allocator->freeTmp(handle);
}
template<typename CTX>
auto CodeGen<CTX>::getBlock(BlockId id) -> CodeGen::BaseBlock& {
    return irGen.getBlock(id);
}
template<typename CTX>
size_t CodeGen<CTX>::sizeBytes(SSARegisterHandle handle) const {
    return irGen.getRecord(handle).sizeBytes();
}
template<typename CTX>
auto CodeGen<CTX>::getType(SSARegisterHandle handle) const {
    return irGen.getRecord(handle).getType();
}
template<typename CTX>
void CodeGen<CTX>::assignRegisters(IRInstruction& instruction) {
    allocator->beforeInst(instruction, currentInstructionCounter);
}
