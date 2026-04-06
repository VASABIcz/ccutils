#pragma once

#include "utils/BetterOption.h"
#include "gen64/X86mc.h"
#include "utils/Variant.h"
#include "utils/dispatch.h"
#include "instructions.hpp"
#include "Graph.h"
#include "X86Instruction.hpp"
#include "Registers.hpp"
#include "instructions.hpp"
#include <any>

// base type: VIRTUAL/PHYSICAL/STACK?
// allocated type (only for virtual): STACK/PHYSICAL

struct EmitCtx {
    // ins
    const std::map<VHAND, x86::X64Register>& regMap;
    X86mc& mc;
    Graph& g;
    long stackAdend = 0;

    // outs
    std::map<size_t, std::vector<ImmSpace>> symbols;
    std::map<size_t, std::vector<std::pair<ImmSpace, long>>> stackSlots;
    std::map<Block*, std::vector<ImmSpace>> jumps;
    std::map<Block*, size_t> blockOffs;
    std::map<size_t, size_t> stackOffsets;
    size_t stackSize = 0;

    auto getReg(BaseRegister* r) -> x86::X64Register {
        if (not regMap.contains(r)) {
            assert(r->hintReg.has_value());
            return *r->hintReg;
        }
        assert(regMap.contains(r->getId()));
        return regMap.at(r->getId());
    };

    auto requestOffset(size_t id, ImmSpace space) { symbols[id].push_back(space); };

    auto requestJmp(Block* id, ImmSpace space) { jumps[id].push_back(space); };

    auto requestStack(StackSlot id, ImmSpace space, long adend = 0) { stackSlots[id.id].emplace_back(space, adend); };

    void linkJmp() {
        for (auto jmp: jumps) {
            auto label = blockOffs[jmp.first];

            for (auto slot: jmp.second) {
                mc.patchImm(slot, label);
            }
        }
    }

    // turn "abstract" StackSlot LOAD/STORE/LEA stack into concreate RSP/RBP moves
    void refineStack() {
        for (auto block: g.blocks) {
            for (auto inst: block->iterator()) {
                if (inst->is<LEASTACK>()) {
                    auto i = inst->as<LEASTACK>();
                    block->replace<LOADMEM>(inst, i->getDef(0), g.getReg(x86::Rsp), getStackOffset(i->slot));
                } else if (inst->is<LOADSTACK>()) {
                    auto i = inst->as<LOADSTACK>();
                    block->replace<LOADMEM>(inst, i->getDef(0), g.getReg(x86::Rsp), getStackOffset(i->slot)); // FIXME size
                } else if (inst->is<STORESTACK>()) {
                    auto i = inst->as<STORESTACK>();
                    block->replace<STOREMEM>(inst, g.getReg(x86::Rsp), i->getUse(0), getStackOffset(i->slot)); // FIXME size
                }
            }
        }
    }

    void insertCallingConvention() {
        std::set<x86::X64Register> regs;
        for (auto [_h, reg] : regMap) {
            regs.insert(reg);
        }
        g.insertPrologueEpilogue(regs);
    }

    void insertStackFrame() {
        // push rbp
        // mov rbp, rsp
        // sub rsp, {stack_size}

        auto first = g.root->first;
        g.root->insertBefore<PUSH>(first, g.getReg(x86::Rbp));
        g.root->insertBefore<MOV>(first, g.getReg(x86::Rbp), g.getReg(x86::Rsp));
        g.root->insertBefore<SUBIMM>(first, g.getReg(x86::Rsp), stackSize);

        g.forEachInst([&](auto block, auto inst){
            if (not inst->template is<RET>()) return;

            block->template insertBefore<LEAVE>(inst);
        });
    }

    void calculateStack() {
        auto [_stackOffsets, _stackSize] = g.stackOffsets();
        stackSize = _stackSize+16;
        stackOffsets = _stackOffsets;
    }

    void megaLink() {
        insertCallingConvention();
        calculateStack();
        // no allocas after this point
        insertStackFrame();
        refineStack();
        emit();
        linkJmp();
    }

    long getStackOffset(StackSlot slot, long adend = 0) {
        return stackOffsets.at(slot.id) + adend + stackAdend;
    }

    void patchStack() {
        for (auto stack: stackSlots) {
            for (auto [slot, adend]: stack.second) {
                mc.patchRaw(slot, stackOffsets.at(stack.first) + adend + stackAdend);
            }
        }
    }

    bool dispatchInst(X86Instruction* inst) {
        bool didMatch = true;
        dispatch(
            inst,
            CASE_VAL(MOV*) { mc.movReg(getReg(it->getDef(0)), getReg(it->getUse(0))); },
            CASE_VAL(MOVIMM*) { mc.movFast(getReg(it->getDef(0)), it->value); },
            CASE_VAL(ADD*) { mc.writeRegInst(X64Instruction::add, getReg(it->getDef(0)), getReg(it->getUse(1))); },
            CASE_VAL(SUB*) { mc.writeRegInst(X64Instruction::sub, getReg(it->getDef(0)), getReg(it->getUse(1))); },
            CASE_VAL(RET*) { mc.ret(); },
            CASE_VAL(LEAVE*) { mc.leave(); },
            CASE_VAL(MOVRIP*) {
                auto sym = mc.writeRegRipInst(X64Instruction::mov, getReg(it->getDef(0)), 0);
                requestOffset(it->id, sym);
            },
            CASE_VAL(LEARIP*) {
                auto sym = mc.writeRegRipInst(X64Instruction::lea, getReg(it->getDef(0)), 0);
                requestOffset(it->id, sym);
            },
            CASE_VAL(ADDIMM*) { mc.addImm32(getReg(it->getDef(0)), it->imm); },
            CASE_VAL(SUBIMM*) { mc.subImm32(getReg(it->getDef(0)), it->imm); },
            CASE_VAL(MOVMEMIMM*) { mc.readMem(getReg(it->getDef(0)), getReg(it->getUse(0)), it->imm, 8); },
            CASE_VAL(LOADMEM*) { mc.readMem(getReg(it->getDef(0)), getReg(it->getUse(0)), it->offset, 8); },
            CASE_VAL(STOREMEM*) { mc.writeMem(getReg(it->getUse(0)), getReg(it->getUse(1)), it->offset, 8); },
            CASE_VAL(CMP*) { mc.writeRegInst(X64Instruction::cmp, getReg(it->getUse(0)), getReg(it->getUse(1))); },
            CASE_VAL(TEST*) { mc.writeRegInst(X64Instruction::test, getReg(it->getUse(0)), getReg(it->getUse(1))); },
            CASE_VAL(CALLREG*) { mc.call(getReg(it->reg)); },
            CASE_VAL(CALLRIP*) {
                auto imm = mc.callRIP(0);
                requestOffset(it->id, imm);
            },
            CASE_VAL(CMPIMM*) { mc.cmp64Imm(getReg(it->getUse(0)), it->imm); },
            CASE_VAL(JZ*) {
                auto imm = mc.writeJmp(CmpType::Equal, 0_u32);
                requestJmp(it->tgt, imm);
            },
            CASE_VAL(JGT*) {
                auto imm = mc.writeJmp(CmpType::Greater, 0_u32);
                requestJmp(it->tgt, imm);
            },
            CASE_VAL(JLS*) {
                auto imm = mc.writeJmp(CmpType::Less, 0_u32);
                requestJmp(it->tgt, imm);
            },
            CASE_VAL(JGE*) {
                auto imm = mc.writeJmp(CmpType::GreaterOrEqual, 0_u32);
                requestJmp(it->tgt, imm);
            },
            CASE_VAL(JLE*) {
                auto imm = mc.writeJmp(CmpType::LessOrEqual, 0_u32);
                requestJmp(it->tgt, imm);
            },
            CASE_VAL(JMP*) {
                auto imm = mc.writeJmp(0_u32);
                requestJmp(it->tgt, imm);
            },
            // CASE_VAL(LEASTACK*) {
            // auto imm = mc.lea(getReg(it->getDef(0)), x86::Rsp, 0);
            // requestStack(it->slot, imm);
            // },
            // CASE_VAL(STORESTACK*) {
            // auto imm = mc.writeStack(500'000, getReg(it->getUse(0)));
            // requestStack(it->slot, imm, it->offset);
            // },
            // CASE_VAL(LOADSTACK*) {
            // auto imm = mc.readStack(getReg(it->getDef(0)), it->size, 500'000);
            // requestStack(it->slot, imm, it->offset);
            // },
            CASE_VAL(MUL*) { mc.writeRegInst(SusX64Instruction::imul, getReg(it->getDef(0)), getReg(it->getUse(1))); },
            CASE_VAL(INT3*) { mc.trap(); },
            CASE_VAL(SETZ*) {
                mc.setCC(getReg(it->getDef(0)), CmpType::Equal);
                mc.writeMovZX8(getReg(it->getDef(0)), getReg(it->getDef(0)));
            },
            CASE_VAL(SETNZ*) {
                mc.setCC(getReg(it->getDef(0)), CmpType::NotEqual);
                mc.writeMovZX8(getReg(it->getDef(0)), getReg(it->getDef(0)));
            },
            CASE_VAL(SETGT*) {
                mc.setCC(getReg(it->getDef(0)), CmpType::Greater);
                mc.writeMovZX8(getReg(it->getDef(0)), getReg(it->getDef(0)));
            },
            CASE_VAL(SETGE*) {
                mc.setCC(getReg(it->getDef(0)), CmpType::GreaterOrEqual);
                mc.writeMovZX8(getReg(it->getDef(0)), getReg(it->getDef(0)));
            },
            CASE_VAL(SETLS*) {
                mc.setCC(getReg(it->getDef(0)), CmpType::Less);
                mc.writeMovZX8(getReg(it->getDef(0)), getReg(it->getDef(0)));
            },
            CASE_VAL(SETLE*) {
                mc.setCC(getReg(it->getDef(0)), CmpType::LessOrEqual);
                mc.writeMovZX8(getReg(it->getDef(0)), getReg(it->getDef(0)));
            },
            CASE_VAL(XOR*) { mc.writeRegInst(X64Instruction::xorI, getReg(it->getDef(0)), getReg(it->getUse(1))); },
            CASE_VAL(CQO*) { mc.cqo(); },
            CASE_VAL(IDIV*) { mc.signedDivide(getReg(it->getUse(0))); },
            CASE_VAL(IDIV*) { mc.signedDivide(getReg(it->getUse(0))); },
            CASE_VAL(AND*) { mc.writeRegInst(X64Instruction::And, getReg(it->getDef(0)), getReg(it->getUse(1))); },
            CASE_VAL(OR*) { mc.writeRegInst(X64Instruction::Or, getReg(it->getDef(0)), getReg(it->getUse(1))); },
            CASE_VAL(ASR*) { mc.shiftRightByCl(getReg(it->getDef(0))); },
            CASE_VAL(SHR*) { TODO() },
            CASE_VAL(SHL*) { mc.shiftLeftByCl(getReg(it->getDef(0))); },
            CASE_VAL(PUSH*) { mc.push(getReg(it->getUse(0))); },
            CASE_VAL(POP*) { mc.pop(getReg(it->getDef(0))); },
            CASE_VAL(FAKE_DEF*){},
            CASE_VAL(FAKE_USE*){},
            CASE_VAL(X86Instruction*) { didMatch = false; }
        );

        return didMatch;
    }

    void emit() {
        for (auto& b: g.blocks) {
            blockOffs[b] = mc.getOffset();
            for (auto inst: b->iterator()) {
                println("dispatching {}", inst->className());
                auto didDispatch = dispatchInst(inst);
                if (not didDispatch) PANIC("DID NOT MATCH {}", inst->className());
            }
        }
    }
};