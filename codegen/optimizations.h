#pragma once

#include "forward.h"
#include "IRInstructions.h"
#include "IRInstructions.h"

namespace optimizations {
    /*bool optimizeSimpleReturns(IRGen& gen) {
    bool didOptimize = false;

    for (const auto& block: gen.nodes()) {
        if (block->getInstructions().size() != 1 || gen.root().blockId == block->blockId) continue;

        if (block->getInstruction(0)->is<instructions::Return>()) {
            auto ingress = gen.lookupIngress(block->blockId);

            for (auto srcId: ingress) {
                gen.getBlock(srcId).getInstruction(-1) = make_unique<instructions::Return>(SSARegisterHandle::invalid(), SSARegisterHandle::invalid());
            }
        }

        gen.graph.destroy(block->blockId);
        didOptimize = true;
    }

    return didOptimize;
}*/

// replace all ocuracnes of virgins with chad
    template<typename CTX>
    void replaceInstr(typename CTX::IRGEN& gen, SSARegisterHandle chad, const std::set<SSARegisterHandle>& virgins) {
        forEachInstruction(gen.graph, [&](auto& inst, auto& node) -> bool {
            // patch writes
            if (virgins.contains(inst->target)) {
                inst->target = chad;
            }
            // patch reads
            inst->visitSrcs([&](auto& src) {
                if (virgins.contains(src)) {
                    src = chad;
                }
            });

            return true;
        });
    }

/// get rid of phi functions that have only one source
    template<typename CTX>
    bool optimizePhis(typename CTX::IRGEN& gen) {
        bool didOptimize = false;

        for (auto& block: gen.nodes()) {
            for (const auto& instruction: block->getInstructions() | views::reverse) {
                if (!instruction->template is<instructions::PhiFunction<CTX>>()) continue;

                auto* phi = instruction->template cst<instructions::PhiFunction<CTX>>();
                if (phi->getAllVersions().size() > 1) continue;
                auto target = phi->target;
                auto valu = *phi->getVersions().begin();

                for (const auto& otherBlock: gen.nodes()) {
                    for (const auto& instruction1: otherBlock->getInstructions()) {
                        if (instruction1->template is<instructions::PhiFunction<CTX>>()) {
                            auto phaj = instruction1->template cst<instructions::PhiFunction<CTX>>();
                            if (phaj->target == valu) {
                                phaj->remove(target);
                                continue;
                            }
                        }
                        instruction1->visitSrcs([&](auto& reg) {
                            if (reg == target) {
                                reg = valu;
                            }
                        });
                    }
                }

                block->removeInstruction(instruction);
                didOptimize = true;
            }
        }

        return didOptimize;
    }

    template<typename CTX>
    bool optimizeAssign(typename CTX::IRGEN& gen) {
        bool didOptimize = false;

        std::vector<std::pair<CodeBlock<CTX>*, IRInstruction<CTX>*>> toRemove;

        forEachInstruction(gen.graph, [&](auto& inst, auto& node) -> bool {
            if (!inst->template is<instructions::Assign<CTX>>()) return true;

            auto* assign = inst->template cst<instructions::Assign<CTX>>();
            auto tgt = assign->target;
            auto value = assign->value;

            replaceInstr<CTX>(gen, value, set{tgt});
            toRemove.emplace_back(node.get(), inst.get());
            didOptimize = true;

            return true;
        });

        for (auto [block, instruction] : toRemove) {
            block->removeInstruction(instruction);
        }

        return didOptimize;
    }

    template<typename CTX>
    bool ssa2cfg(typename CTX::IRGEN& gen) {
        bool didOptimize = false;

        std::vector<std::pair<CodeBlock<CTX>*, IRInstruction<CTX>*>> toRemove;

        forEachInstruction(gen.graph, [&](auto& inst, auto& node) -> bool {
            if (!inst->template is<instructions::Assign<CTX>>()) return true;

            auto* assign = inst->template cst<instructions::Assign<CTX>>();
            auto tgt = assign->target;
            auto value = assign->value;

            replaceInstr<CTX>(gen, value, set{tgt});
            toRemove.emplace_back(node.get(), inst.get());
            didOptimize = true;

            return true;
        });

        for (auto [block, instruction] : toRemove) {
            block->removeInstruction(instruction);
        }

        return didOptimize;
    }

    template<typename CTX>
    bool optimizePhis1(typename CTX::IRGEN& gen) {
        bool didOptimize = false;

        std::vector<std::pair<CodeBlock<CTX> *, IRInstruction<CTX> *>> toRemove;

        forEachInstruction(gen.graph, [&](auto &inst, auto &node) -> bool {
            if (!inst->template is<instructions::PhiFunction<CTX>>()) return true;

            instructions::PhiFunction<CTX> *phi = inst->template cst<instructions::PhiFunction<CTX>>();

            auto vers = phi->getVersions();

            // phi has only 1 unique input
            if (vers.size() == 1) {
                toRemove.emplace_back(node.get(), inst.get());
                replaceInstr<CTX>(gen, *vers.begin(), set{phi->target});
                didOptimize = true;
                return true;
            }

            // phi has 1 unique input + other input is target
            if (vers.size() == 2 && vers.contains(phi->target)) {
                toRemove.emplace_back(node.get(), inst.get());
                vers.erase(phi->target);
                replaceInstr<CTX>(gen, *vers.begin(), set{phi->target});
                didOptimize = true;
                return true;
            }

            return true;
        });

        for (auto [block, instruction]: toRemove) {
            block->removeInstruction(instruction);
        }

        return didOptimize;
    }

// remove blocks that only contain jmp
    template<typename CTX>
    void removeUselessBlocks(typename CTX::IRGEN& gen) {
        auto inv = gen.graph.inverseTreeP();
        std::vector<BlockId> toRemove;

        for (auto [target, sources] : inv) {
            CodeBlock<CTX>* xd = &gen.getBlock(target);

            // only 1 inst, only 1 jmp tgt
            if (xd->getTargets().size() == 1 && xd->instructionCount() == 1) {
                auto targetId = xd->getTargets()[0];

                // patch sources to jump to our target
                for (auto src : sources) {
                    CodeBlock<CTX>* b = &gen.getBlock(src);
                    b->patchJumps(target, targetId);
                }

                CodeBlock<CTX>* tgtB = &gen.getBlock(targetId);

                // patch target blocks phis to account for new sources
                tgtB->patchPhis(target, sources);

                toRemove.push_back(target);
            }
        }

        for (auto tgt : toRemove) {
            gen.graph.removeBlock(tgt);
        }
    }

    template<typename CTX>
    void forEachInstruction(ControlFlowGraph<CTX>& g, auto&& fn) {
        for (auto& node : g.validNodes()) {
            for (auto& inst : node->instructions) {
                if (!fn(inst, node)) return;
            }
        }
    }

    template<typename CTX>
    CopyPtr<IRInstruction<CTX>>* findInstructionByTGT(ControlFlowGraph<CTX>& g, SSARegisterHandle reg) {
        CopyPtr<IRInstruction<CTX>>* res = nullptr;
        forEachInstruction(g, [&](auto&& it) -> bool {
            if (it->target == reg) {
                res = &it;
                return false;
            }
            return true;
        });

        return res;
    }

    template<typename IRGEN>
    size_t calcUseCount(IRGEN& gen, SSARegisterHandle reg) {
        size_t useCount = 0;

        forEachInstruction(gen.graph, [&](auto& inst) -> bool {
            inst->visitSrc([&](auto& src) {
                if (src == reg) useCount += 1;
            });

            return true;
        });

        return useCount;
    }

/// remove instructions that produce value that isnt used
    template<typename IRGEN>
    bool removeUnusedInstructions(IRGEN& gen) {
        bool didOptimize = false;

        for (auto& block: gen.nodes()) {
            for (auto& instruction: block->getInstructionsMut() | views::reverse) {
                if (!instruction->target.isValid()) continue;

                if (calcUseCount(gen, instruction->target) == 0 && instruction->isPure()) {
                    instruction->visitSrc([&](auto& reg) {
                        gen.getRecord(reg).decUseCount();
                    });

                    block->removeInstruction(instruction);
                    didOptimize = true;
                }
            }
        }

        return didOptimize;
    }

// TODO can we do partial merge?
// FIXME we propably shouldent do anymore optimizations after this
/// merge live ranges of phi function
/// if phi function sources dont overlap merge them
/// does this break SSA? yes it does break SSA
    template<typename CTX>
    bool mergeLiveRanges(typename CTX::IRGEN& gen, map<SSARegisterHandle, vector<bool>>& liveRanges) {
        bool didOptimize = false;

        for (auto& block: gen.nodes()) {
            for (auto& instruction: block->getInstructionsMut() | views::reverse) {
                if (!instruction->template is<instructions::PhiFunction<CTX>>()) continue;

                auto* phi = instruction->template cst<instructions::PhiFunction<CTX>>();

                vector<bool> merged(liveRanges.begin()->second.size());

                const auto apply = [&](SSARegisterHandle idk) {
                    size_t overlaps = 0;
                    for (auto [i, b] : liveRanges[idk] | views::enumerate) {
                        if (merged[i] && b) overlaps += 1;
                        if (b) merged[i] = true;
                    }
                    return overlaps <= 1;
                };

                auto toApply = phi->getVersions();
                toApply.insert(phi->target);
                auto sources = phi->getVersions();
                sources.erase(phi->target);

                bool wasSucess = true;
                for (auto& idk : toApply) {
                    if (!apply(idk)) {
                        wasSucess = false;
                        break;
                    }
                }

                if (wasSucess) {
                    didOptimize = true;

                    // patch nodes
                    replaceInstr<CTX>(gen, phi->target, sources);

                    // set new live range
                    liveRanges[phi->target] = merged;

                    // remove source registers
                    for (auto& source : sources) {
                        liveRanges.erase(source);
                    }

                    // replace phi instruction with noop, this makes sure we dont break live ranges
                    instruction.uPtrRef() = std::make_unique<instructions::NoOp<CTX>>();
                }
            }
        }

        return didOptimize;
    }

/// if we do jump to block that is lineary bellow us remove it
/// if we do branch and one of our blocks is lineary bellow use replace it with conditional jump
    template<typename CTX>
    bool removeFallJumps(typename CTX::IRGEN& gen, span<size_t> linearized) {
        bool didOptimize = false;

        for (auto i: views::iota(0u, linearized.size()-1)) {
            auto& inst = gen.getBlock(linearized[i]).getInstruction(-1);
            if (inst->template is<instructions::Jump<CTX>>()) {
                auto next = inst->template cst<instructions::Jump<CTX>>()->value;
                if (next == linearized[i+1]) {
                    inst.uPtrRef() = make_unique<instructions::FallTrough<CTX>>(next);
                    didOptimize = true;
                }
            }
            else if (inst->template is<instructions::Branch<CTX>>()) {
                auto* branch = inst->template cst<instructions::Branch<CTX>>();
                if (linearized[i+1] == branch->scopeT) {
                    inst.uPtrRef() = make_unique<instructions::JumpFalse<CTX>>(branch->condition, branch->scopeT, branch->scopeF);
                    didOptimize = true;
                }
                else if (linearized[i+1] == branch->scopeF) {
                    inst.uPtrRef() = make_unique<instructions::JumpTrue<CTX>>(branch->condition, branch->scopeT, branch->scopeF);
                    didOptimize = true;
                }
            } else if (inst->template is<instructions::BranchCond<CTX>>()) {
                auto* branch = inst->template cst<instructions::BranchCond<CTX>>();
                if (linearized[i+1] == branch->scopeT) {
                    inst.uPtrRef() = make_unique<instructions::JumpCond<CTX>>(negateType(branch->type), branch->lhs, branch->rhs, branch->scopeF, branch->scopeT);
                    didOptimize = true;
                }
                else if (linearized[i+1] == branch->scopeF) {
                    inst.uPtrRef() = make_unique<instructions::JumpCond<CTX>>(branch->type, branch->lhs, branch->rhs, branch->scopeT, branch->scopeF);
                    didOptimize = true;
                }
            }
        }

        return didOptimize;
    }

/// DEOPTIMIZATION - :D
/// mark all registers that we need to know their address
    template<typename CTX>
    bool forceStackAlloc(typename CTX::IRGEN& gen) {
        bool didOptimize = false;

        for (auto& block: gen.nodes()) {
            for (auto& instruction: block->getInstructionsMut() | views::reverse) {
                if (not instruction->template is<instructions::AddressOf<CTX>>()) continue;
                auto* inst = instruction->template cst<instructions::AddressOf<CTX>>();

                gen.getRecord(inst->obj).forceStack();
                didOptimize = true;
            }
        }

        return didOptimize;
    }

/// merge cmp + branch instructtions
    template<typename CTX>
    bool optimizeJmpCond(typename CTX::IRGEN& gen) {
        bool didOptimize = false;

        enum class Type {
            BRANCH,
            JUMP_TRUE,
            JUMP_FALSE
        };

        for (auto& block: gen.nodes()) {
            for (auto& instruction: block->getInstructionsMut()) {
                if (not instruction->template is<instructions::Branch<CTX>>() && not instruction->template is<instructions::JumpFalse<CTX>>() && not instruction->template is<instructions::JumpTrue<CTX>>()) continue;

                Type type;
                size_t scopeT;
                size_t scopeF;
                SSARegisterHandle subject;

                instruction->template ifIs<instructions::Branch<CTX>>([&](auto& it) {
                    type = Type::BRANCH;
                    scopeT = it.scopeT;
                    scopeF = it.scopeF;
                    subject = it.condition;
                }) || instruction->template ifIs<instructions::JumpTrue<CTX>>([&](auto& it) {
                    type = Type::JUMP_TRUE;
                    scopeT = it.scopeT;
                    scopeF = it.scopeF;
                    subject = it.condition;
                }) || instruction->template ifIs<instructions::JumpFalse<CTX>>([&](auto& it) {
                    type = Type::JUMP_FALSE;
                    scopeT = it.scopeT;
                    scopeF = it.scopeF;
                    subject = it.condition;
                });

                auto cnd1 = findInstructionByTGT(gen.graph, subject);

                if (cnd1 == nullptr) continue;
                auto& cnd = *cnd1;

                JumpCondType tajp;
                SSARegisterHandle rhs;
                SSARegisterHandle lhs;

                if (cnd->template is<instructions::BinaryInstruction>()) {
                    continue;
                }

                instructions::BinaryInstruction<CTX>* xd = cnd->template as<instructions::BinaryInstruction>();

                Assembler::BinaryOp op = xd->op;
                switch (op) {
                    case Assembler::BinaryOp::EQ:
                        tajp = JumpCondType::EQUALS;
                        break;
                    case Assembler::BinaryOp::NEQ:
                        tajp = JumpCondType::NOT_EQUALS;
                        break;
                    case Assembler::BinaryOp::GT:
                        tajp = JumpCondType::GREATER;
                        break;
                    case Assembler::BinaryOp::GE:
                        tajp = JumpCondType::GREATER_OR_EQUAL;
                        break;
                    case Assembler::BinaryOp::LS:
                        tajp = JumpCondType::LESS;
                        break;
                    case Assembler::BinaryOp::LE:
                        tajp = JumpCondType::LESS_OR_EQUAL;
                        break;
                    default:
                        continue;
                }

                switch (type) {
                    case Type::BRANCH:
                        instruction = CopyPtr<IRInstruction<CTX>>(make_unique<instructions::BranchCond<CTX>>(tajp, lhs, rhs, scopeT, scopeF));
                        break;
                    case Type::JUMP_TRUE:
                        instruction = CopyPtr<IRInstruction<CTX>>(make_unique<instructions::JumpCond<CTX>>(tajp, lhs, rhs, scopeT, scopeF));
                        break;
                    case Type::JUMP_FALSE:
                        instruction = CopyPtr<IRInstruction<CTX>>(make_unique<instructions::JumpCond<CTX>>(negateType(tajp), lhs, rhs, scopeT, scopeF));
                        break;
                }

                didOptimize = true;
            }
        }
        return didOptimize;
    }

    inline bool isLastUse(map<SSARegisterHandle, vector<bool>>& liveRanges, SSARegisterHandle reg, size_t idex) {
        assert(liveRanges.contains(reg));
        assert(idex < liveRanges[reg].size());

        // its last item in live ranges
        if (idex+1 == liveRanges[reg].size()) return true;

        for (auto i = idex+1; i < liveRanges[reg].size(); i++) {
            if (i) return false;
        }

        return true;
    }

    inline bool isFirstUse(map<SSARegisterHandle, vector<bool>>& liveRanges, SSARegisterHandle reg, size_t idex) {
        assert(liveRanges.contains(reg));
        assert(idex < liveRanges[reg].size());

        // its last item in live ranges
        if (idex == 0) return true;

        for (auto i = 0ul; i < idex; i++) {
            if (i) return false;
        }

        return true;
    }

/// merge live ranges `a` `b` into `out`
/// returns number of overlaps
    inline size_t mergeLiveRange(const vector<bool>& a, const vector<bool>& b, vector<bool>& out) {
        assert(a.size() == b.size());
        out.resize(a.size());
        size_t overlaps = 0;

        for (auto i = 0UL; i < a.size(); i++) {
            overlaps += a[i] && b[i];
            out[i] = a[i] || b[i];
        }

        return overlaps;
    }

/// merge live ranges of binary instructions, preferably use lhs
/// reduce number of registers overall
/// allow backends to emit efficent code eg.: add rax, rdi
/// 0:2 := add 0:0 0:1 -> 0:0 := add 0:0 0:1
// TODO maybe there is more sim
    template<typename CTX>
    bool optimizeCumulativeOps(typename CTX::IRGEN& gen, span<size_t> linearized, map<SSARegisterHandle, vector<bool>>& liveRanges) {
        // FIXME we are assuming that no instructions were removed
        size_t ip = 0; // instruction counter

        for (auto i = 0UL; i < linearized.size(); i++) {
            auto blockId = linearized[i];

            auto& block = gen.getBlock(blockId);

            for (auto& instruction: block.getInstructionsMut()) {
                if (auto* it = instruction->template cst<instructions::BinaryInstruction<CTX>>(); it != nullptr) {
                    auto tgt = it->target;
                    auto lhs = it->lhs;
                    // auto rhs = it->rhs;

                    vector<bool> merged;
                    // FIXME proper handling of dead registers
                    if (not liveRanges.contains(tgt)) continue;
                    assert(liveRanges.contains(tgt));
                    assert(liveRanges.contains(lhs));
                    auto overlaps = mergeLiveRange(liveRanges[tgt], liveRanges[lhs], merged);
                    // assert(overlaps != 0); // WTF is this case????

                    if (overlaps == 1) {
                        std::set<SSARegisterHandle> toReplace{tgt};
                        replaceInstr<CTX>(gen, lhs, toReplace);
                        liveRanges[lhs] = merged;
                        liveRanges[tgt] = vector(merged.size(), false);
                        assert(liveRanges[tgt].size() == merged.size());
                        assert(liveRanges[lhs].size() == merged.size());
                    }
                }

                ip += 1;
            }
        }

        return false;
    }
}