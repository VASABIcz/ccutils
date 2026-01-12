#pragma once
#include "Graph.h"
#include "X86Instruction.hpp"
#include "utils/Logger.h"
#include "codegen/x86/x86_insts.h"
#include <map>
#include <set>

template<typename CTX>
struct Lower {
    using IM = InstructionMatcher<CTX>;
    std::map<SSARegisterHandle, BaseRegister*> virtRegs;
    std::map<void*, StackSlot> allocaSlots;
    Graph g;

    Lower(std::shared_ptr<Logger> logger = Logger::NOP()) : g(logger) {}

    StackSlot allocateStack(size_t size, size_t alignment) {
        // FIXME this is retarded
        return g.allocStackSlot(size, alignment);
    }

    void insertPrologueEpilogue(std::set<x86::X64Register> regs) {
        std::map<x86::X64Register, StackSlot> ss;
        for (auto reg: regs) {
            auto saveType = x86::sysVSave(reg);
            if (saveType != x86::X64Register::SaveType::Callee) continue;

            ss[reg] = this->allocateStack(8, 8);
            g.root->insertBefore(g.root->first, new STORESTACK(g.getReg(reg), ss[reg], 0));
        }

        for (auto block: g.blocks) {
            for (auto inst: block->iterator()) {
                if (inst->is<RET>()) {
                    for (auto reg: regs) {
                        auto saveType = x86::sysVSave(reg);
                        if (saveType != x86::X64Register::SaveType::Callee) continue;

                        block->insertBefore(inst, new LOADSTACK(g.getReg(reg), ss[reg], 0, 8));
                    }
                }
            }
        }
    }

    // template<typename CTX>
    struct RewriteRule {
        std::function<void(Lower& l, typename IM::MatchedInstructions inst, Block* block)> rewrite;
        BasePattern* pattern;
    };

    static RewriteRule* makeRewrite(BasePattern* pat, std::function<void(Lower& l, typename IM::MatchedInstructions inst, Block* block)> rev) { return new RewriteRule{rev, pat}; }

    void doConsuming(ControlFlowGraph<CTX>& cfg, std::set<IRInstruction<CTX>*>& globalConsumed, std::span<RewriteRule*> patterns, IRInstruction<CTX>* inst, Block* block) {
        if (globalConsumed.contains(inst)) return;

        for (auto pattern: patterns) {
            std::vector<IRInstruction<CTX>*> wildcards;
            std::vector<IRInstruction<CTX>*> consumed;
            typename IM::MatchedInstructions matched;
            auto didMatch = matcher.matchPattern(cfg, inst, pattern->pattern, wildcards, consumed, &matched);
            if (!didMatch) continue;
            globalConsumed.insert(inst);

            // block->insertPoint = 0;
            pattern->rewrite(*this, matched, block);

            // for (auto con : consumed) globalConsumed.insert(con);

            // for (auto wild: wildcards) doConsuming(cfg, globalConsumed, patterns, wild, block);

            return;
        }

        PANIC("failed to match inst {}", inst->name);
    }

    std::map<BlockId, Block*> blocks;
    Block* getBlockForId(BlockId id) {
        if (not blocks.contains(id)) {
            blocks.emplace(id, g.makeBlock());
        }

        return blocks.at(id);
    }

    void matchBlock(ControlFlowGraph<CTX>& cfg, CodeBlock<CTX>& block, std::span<RewriteRule*> patterns) {
        std::set<IRInstruction<CTX>*> globalConsumed;

        auto bb = getBlockForId(block.id());
        for (auto b: block.getTargets()) {
            bb->addTarget(getBlockForId(b));
        }

        for (auto i = 0ul; i < block.instructions.size(); i++) {
            auto cur = block.instructions[i].get();

            doConsuming(cfg, globalConsumed, patterns, cur, bb);
            // bb->insertPoint = 0;
        }
    }

    BaseRegister* getReg(SSARegisterHandle hand) {
        assert(hand.isValid());
        if (!virtRegs.contains(hand)) {
            virtRegs[hand] = g.allocaVirtReg();
        }

        return virtRegs[hand];
    }

    BaseRegister* getReg(IRInstruction<CTX>* hand) { return getReg(hand->target); }

    long getImm(IRInstruction<CTX>* inst) {
        if (auto intLit = inst->template cst<instructions::IntLiteral>(); intLit) {
            return intLit->value;
        }
        TODO()
    }

    void emitLoadAddImm(Lower<CTX>& l, IM::MatchedInstructions inst, Block* block) { block->push<MOVMEMIMM>(l.getReg(inst->target), l.getReg(inst[0, 0]->target), l.getImm(inst[0, 1]->target)); }

    void matchCFG(ControlFlowGraph<CTX>& cfg, std::span<RewriteRule*> patterns) {
        cfg.forEachBlock([&](CodeBlock<CTX>& block) { matchBlock(cfg, block, patterns); });
    }

    void emitCall(std::vector<SSARegisterHandle> ret, std::span<IRInstruction<CTX>*> argz, Block* block, std::variant<BaseRegister*, size_t> isRipOffset) {
        std::vector<x86::X64Register> argRegs{x86::Rdi, x86::Rsi, x86::Rdx, x86::Rcx, x86::R8, x86::R9};

        std::vector<x86::X64Register> kills{x86::Rax, x86::R10, x86::R11, x86::R9, x86::R8, x86::Rcx, x86::Rdx, x86::Rsi, x86::Rdi};
        std::vector<x86::X64Register> uses;
        std::vector<x86::X64Register> defs;

        for (auto i = 0ul; i < argz.size(); i++) {
            block->push<MOV>(g.getReg(argRegs[i]), getReg(argz[i]));
            uses.push_back(kills.back());
            kills.pop_back();
        }

        assert(ret.size() <= 2);

        if (ret.size() == 2) {
            defs.push_back(x86::Rax);
            defs.push_back(x86::Rdx);
        } else if (ret.size() == 1) {
            defs.push_back(x86::Rax);
        }

        std::vector<BaseRegister*> killsR;
        std::vector<BaseRegister*> usesR;
        std::vector<BaseRegister*> defsR;

        for (auto it: kills)
            killsR.push_back(g.getReg(it));
        for (auto it: uses)
            usesR.push_back(g.getReg(it));
        for (auto it: defs)
            defsR.push_back(g.getReg(it));

        // code gen
        Variant(isRipOffset).match(
            CASE_VAL(BaseRegister*) { block->push<CALLREG>(defsR, usesR, killsR, it); },
            CASE_VAL(size_t) { block->push<CALLRIP>(defsR, usesR, killsR, it); }
        );

        if (ret.size() == 2) {
            block->push<MOV>(getReg(ret[0]), g.getReg(x86::Rax));
            block->push<MOV>(getReg(ret[1]), g.getReg(x86::Rdx));
        } else if (ret.size() == 1) {
            block->push<MOV>(getReg(ret[0]), g.getReg(x86::Rax));
        }

    }

    IM matcher;

    struct RewriteBuilder {
        std::vector<RewriteRule*> rules;

        RewriteBuilder& makeRewrite(BasePattern* p, std::function<void(Lower& l, typename IM::MatchedInstructions inst, Block* block)> rev) {
            rules.push_back(Lower::makeRewrite(p, rev));
            return *this;
        }
    };

    void genLiveRanges() {
        g.print();
        g.calcLiveRanges();
        println("=== X86 GRAPH ===");
        g.print();
        println("=== RANGES ===");
        g.printLiveRange();
        println("=== END X86 GRAPH ===");
    }

    BaseRegister* FLAGS = g.allocaVirtReg();

    void lowerIR(ControlFlowGraph<CTX>& cfg) {
        using OP = Assembler::BinaryOp;
        using TYP = Assembler::BaseDataType;
        auto argCounter = 0;
        RewriteBuilder rewrites;

        auto WILD = matcher.getWild();
#define PAT(...) matcher.template makePattern<__VA_ARGS__>
#define BIN(...) matcher.template makeBin<__VA_ARGS__>
#define WILD(...) matcher.template makeWildPattern<__VA_ARGS__>

        rewrites
            .makeRewrite(
                PAT(instructions::Branch)(BIN(OP::EQ, TYP::I64)(WILD, PAT(instructions::IntLiteral)())),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMPIMM>(FLAGS, l.getReg(inst[0, 0]->target), l.getImm(*inst[0, 1]));
                    block->push<JZ>(FLAGS, l.getBlockForId(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlockForId(inst->branchTargets()[1]));
                }
            )

            .makeRewrite(
                PAT(instructions::Branch)(BIN(OP::EQ, TYP::I64)(WILD, WILD)),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, l.getReg(inst[0, 0]->target), l.getReg(inst[0, 1]->target));
                    block->push<JZ>(FLAGS, l.getBlockForId(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlockForId(inst->branchTargets()[1]));
                }
            )

            .makeRewrite(
                PAT(instructions::Branch)(WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMPIMM>(FLAGS, l.getReg(*inst[0]), 1);
                    block->push<JZ>(FLAGS, l.getBlockForId(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlockForId(inst->branchTargets()[1]));
                }
            )

            // cmp *, *
            // jmpe true
            // jmp false
            // makePattern<instructions::Branch<CTX>>(BIN(OP::EQ, TYP::I64)(WILD,WILD)),

            // mov rax, *
            // ret
            .makeRewrite(
                PAT(instructions::Return)(WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(g.getReg(x86::Rax), getReg(*inst[0]));
                    block->push<RET>(std::initializer_list<BaseRegister*>{g.getReg(x86::Rax)});
                }
            )

            .makeRewrite(
                PAT(instructions::ReturnCompound)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(g.getReg(x86::Rax), getReg(*inst[0]));
                    block->push<MOV>(g.getReg(x86::Rdx), getReg(*inst[1]));
                    block->push<RET>(std::initializer_list<BaseRegister*>{g.getReg(x86::Rax), g.getReg(x86::Rdx)});
                }
            )

            // mov rax, imm
            // ret
            .makeRewrite(PAT(instructions::Return)(PAT(instructions::IntLiteral)()), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { TODO() })

            // mov rax, [*+imm]
            .makeRewrite(
                PAT(instructions::PointerLoad)(BIN(OP::ADD, TYP::I64)(WILD, PAT(instructions::IntLiteral)())),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<MOVMEMIMM>(getReg(*inst), getReg(*inst[0][0]), getImm(*inst[0][1])); }
            )

            .makeRewrite(
                PAT(instructions::PointerLoad)(PAT(instructions::AllocaPtr)()),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    auto pLoad = inst->template cst<instructions::PointerLoad>();
                    block->push<LOADSTACK>(getReg(*inst), allocaSlots[inst[0].inst], pLoad->offset, cfg.getSize(pLoad->target));
                }
            )

            .makeRewrite(
                PAT(instructions::PointerStore)(PAT(instructions::AllocaPtr)(), WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<STORESTACK>(getReg(*inst[1]), allocaSlots[inst[0].inst], inst->template cst<instructions::PointerStore>()->offset);
                }
            )

            .makeRewrite(
                PAT(instructions::PointerLoad)(WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<LOADMEM>(getReg(*inst), getReg(*inst[0]), inst->template cst<instructions::PointerLoad>()->offset); }
            )

            // mov rax, [*]
            .makeRewrite(
                PAT(instructions::PointerStore)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<STOREMEM>(getReg(*inst[0]), getReg(*inst[1]), inst->template cst<instructions::PointerStore>()->offset); }
            )

            .makeRewrite(
                BIN(OP::MUL, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MUL>(getReg(*inst), getReg(*inst[1]));
                }
            )

            .makeRewrite(
                BIN(OP::ADD, TYP::I64)(WILD, PAT(instructions::IntLiteral)()),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<ADDIMM>(getReg(*inst), getImm(*inst[1]));
                }
            )

            .makeRewrite(
                BIN(OP::SUB, TYP::I64)(WILD, PAT(instructions::IntLiteral)()),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<SUBIMM>(getReg(*inst), getImm(*inst[1]));
                }
            )

            .makeRewrite(
                BIN(OP::ADD, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<ADD>(getReg(*inst), getReg(*inst[1]));
                }
            )

            .makeRewrite(
                BIN(OP::SUB, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<SUB>(getReg(*inst), getReg(*inst[1]));
                }
            )

            // mov rax, imm
            .makeRewrite(PAT(instructions::IntLiteral)(), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<MOVIMM>(getReg(*inst), getImm(*inst)); })

            // ret
            .makeRewrite(PAT(instructions::VoidReturn)(), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<RET>(std::initializer_list<BaseRegister*>{}); })

            // rax
            .makeRewrite(
                PAT(instructions::Arg)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    auto reg = x86::SYSV_REGS[argCounter++];
                    block->push<FAKE_DEF>(g.getReg(reg));
                    block->push<MOV>(getReg(*inst), g.getReg(reg));
                }
            )

            .makeRewrite(
                PAT(instructions::AllocaPtr)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    auto alloc = allocateStack(inst->template cst<instructions::AllocaPtr>()->size, 8);
                    block->push<LEASTACK>(getReg(*inst), alloc);
                    allocaSlots[inst.inst] = alloc;
                }
            )

            .makeRewrite(
                BIN(OP::EQ, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETZ>(FLAGS, getReg(*inst));
                }
            )

            .makeRewrite(
                BIN(OP::NEQ, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETNZ>(FLAGS, getReg(*inst));
                }
            )

            .makeRewrite(
                BIN(OP::LE, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLE>(FLAGS, getReg(*inst));
                }
            )
            .makeRewrite(
                BIN(OP::GE, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETGE>(FLAGS, getReg(*inst));
                }
            )

            .makeRewrite(PAT(x86::inst::MovRip)(), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<MOVRIP>(getReg(*inst), inst->template cst<x86::inst::MovRip>()->id); })

            .makeRewrite(PAT(instructions::Assign)(WILD), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<MOV>(getReg(*inst), getReg(*inst[0])); })

            .makeRewrite(
                PAT(instructions::BoolLiteral)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<MOVIMM>(getReg(*inst), (int)inst->template cst<instructions::BoolLiteral>()->value); }
            )

            .makeRewrite(
                PAT(instructions::CharLiteral)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<MOVIMM>(getReg(*inst), (int)inst->template cst<instructions::CharLiteral>()->value); }
            )

            .makeRewrite(PAT(instructions::Jump)(), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { block->push<JMP>(getBlockForId(inst->branchTargets()[0])); })

            .makeRewrite(
                BIN(OP::LS, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLS>(FLAGS, getReg(*inst));
                }
            )
            .makeRewrite(
                BIN(OP::GT, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETGT>(FLAGS, getReg(*inst));
                }
            )
            .makeRewrite(
                BIN(OP::LE, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(FLAGS, getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLE>(FLAGS, getReg(*inst));
                }
            )
            .makeRewrite(
                BIN(OP::AND, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<AND>(getReg(*inst), getReg(*inst[1]));
                }
            )
            .makeRewrite(
                BIN(OP::OR, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<OR>(getReg(*inst), getReg(*inst[1]));
                }
            )
            .makeRewrite(
                BIN(OP::XOR, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<XOR>(getReg(*inst), getReg(*inst[1]));
                }
            )
            .makeRewrite(
                BIN(OP::SHL, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MOV>(g.getReg(x86::Rcx), getReg(*inst[1]));
                    block->push<SHL>(getReg(*inst));
                }
            )
            .makeRewrite(
                BIN(OP::SHR, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MOV>(g.getReg(x86::Rcx), getReg(*inst[1]));
                    block->push<ASR>(getReg(*inst));
                }
            )
            .makeRewrite(
                BIN(OP::DIV, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(g.getReg(x86::Rax), getReg(*inst[0]));
                    block->push<CQO>();
                    block->push<IDIV>(getReg(*inst[1]));
                    block->push<FAKE_USE>(g.getReg(x86::Rdx));
                    block->push<MOV>(getReg(*inst), g.getReg(x86::Rax));
                }
            )
            .makeRewrite(
                BIN(OP::MOD, TYP::I64)(WILD, WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(g.getReg(x86::Rax), getReg(*inst[0]));
                    block->push<CQO>();
                    block->push<IDIV>(getReg(*inst[1]));
                    block->push<FAKE_USE>(g.getReg(x86::Rax));
                    block->push<MOV>(getReg(*inst), g.getReg(x86::Rdx));
                }
            )

            .makeRewrite(
                WILD(x86::inst::CallRIP)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    auto insts = inst.params | views::transform([&](auto& it) { return it->inst; }) | ranges::to<vector>();
                    std::vector<SSARegisterHandle> xd;
                    if (inst->target.isValid()) xd.push_back(inst->target);
                    l.emitCall(xd, insts, block, inst->template cst<x86::inst::CallRIP>()->id);
                }
            )

            .makeRewrite(
                WILD(x86::inst::CallREG)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    auto insts = inst.params | views::transform([&](auto& it) { return it->inst; }) | ranges::to<vector>();
                    std::vector<SSARegisterHandle> xd;
                    if (inst->target.isValid()) xd.push_back(inst->target);
                    auto argz = insts | views::drop(1) | ranges::to<vector>();
                    l.emitCall(xd, argz, block, (BaseRegister*)getReg(insts[0]));
                }
            )

            .makeRewrite(
                WILD(x86::inst::CallRIP2)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    auto insts = inst.params | views::transform([&](auto& it) { return it->inst; }) | ranges::to<vector>();

                    auto pepa = inst.inst->template cst<x86::inst::CallRIP2<CTX>>();
                    l.emitCall(pepa->results, insts, block, inst->template cst<x86::inst::CallRIP2>()->id);
                }
            )

            .makeRewrite(
                PAT(instructions::BoolNot)(WILD),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    block->push<XOR>(getReg(*inst), getReg(*inst));
                    block->push<TEST>(FLAGS, getReg(*inst[0]), getReg(*inst[0]));
                    block->push<SETZ>(FLAGS, getReg(*inst));
                }
            )

            .makeRewrite(WILD(instructions::BitExtract)(), [&](Lower& l, IM::MatchedInstructions inst, Block* block) { TODO() })

            .makeRewrite(
                WILD(instructions::Builtin<CTX>)(),
                [&](Lower& l, IM::MatchedInstructions inst, Block* block) {
                    if (inst->template cst<instructions::Builtin>()->type == "trap") {
                        block->push<INT3>();
                    } else {
                        TODO()
                    }
                }
            )

            .makeRewrite(WILD(instructions::Dummy)(), [&](Lower& l, IM::MatchedInstructions inst, Block* block) {});

#undef PAT
#undef WILD
#undef BIN

        matchCFG(cfg, rewrites.rules);

        g.root = getBlockForId(cfg.root().id());
    }
};
