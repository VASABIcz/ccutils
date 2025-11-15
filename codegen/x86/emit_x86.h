#pragma once
#include "x86IR.h"
#include "gen64/X86mc.h"
#include "utils/Variant.h"
#include <any>

template<typename T, typename Ret, typename Class, typename... Args>
bool tryDispatch(Class obj, Ret (Class::*lambda)(Args...) const, T* value) {
    auto cst = dynamic_cast<typename First<Args...>::TYPE>(value);
    if (cst != nullptr) {
        (obj.*lambda)(cst);
        return true;
    } else {
        return false;
    }
}

template<typename T, typename FN>
bool tryDispatch(T* value, FN fn) {
    return tryDispatch(fn, &FN::operator(), value);
}

template<typename T, typename FN, typename... FNs>
bool tryDispatch(T* value, FN fn, FNs... fns) {
    if (tryDispatch(value, fn)) return true;

    return tryDispatch(value, fns...);
}

template<typename T, typename... Args>
bool dispatch(T* value, Args... args) {
    return tryDispatch(value, args...);
}

struct EmitCtx {
    // ins
    const std::map<size_t, x86::X64Register>& regMap;
    X86mc& mc;
    Graph& g;

    // outs
    std::map<size_t, std::vector<ImmSpace>> symbols;
    std::map<size_t, std::vector<std::pair<ImmSpace, long>>> stackSlots;
    std::map<size_t, size_t> stackOffsets;
    std::map<Block*, std::vector<ImmSpace>> jumps;
    std::map<Block*, size_t > blockOffs;

    auto getReg(BaseRegister* r) -> x86::X64Register {
        if (auto v = r->as<Virtual>(); v) {
            assert(regMap.contains(v->id));
            return regMap.at(v->id);
        } else {
            return r->as<Physical>()->id;
        }
    };

    auto requestOffset(size_t id, ImmSpace space) {
        symbols[id].push_back(space);
    };

    auto requestJmp(Block* id, ImmSpace space) {
        jumps[id].push_back(space);
    };

    auto requestStack(StackSlot id, ImmSpace space, long adend = 0) {
        stackSlots[id.id].emplace_back(space, adend);
    };

    void linkJmp() {
        for (auto jmp : jumps) {
            auto label  = blockOffs[jmp.first];

            for (auto slot : jmp.second) {
                mc.patchImm(slot, label);
            }
        }
    }

    void patchStack() {
        for (auto stack : stackSlots) {
            for (auto [slot, adend] : stack.second) {
                mc.patchRaw(slot, stackOffsets[stack.first]+8+adend);
            }
        }
    }

    void emit() {
        for (auto& b : g.blocks) {
            blockOffs[b] = mc.getOffset();
            for (auto inst : b->iterator()) {
                auto didDispatch = dispatch(inst,
                CASE_VAL(MOV*) {
                    mc.movReg(getReg(it->defs[0]), getReg(it->uses[0]));
                },
                CASE_VAL(MOVIMM*) {
                    mc.movFast(getReg(it->defs[0]), it->value);
                },
                CASE_VAL(ADD*) {
                    mc.writeRegInst(X64Instruction::add, getReg(it->defs[0]), getReg(it->uses[1]));
                },
                CASE_VAL(SUB*) {
                    mc.writeRegInst(X64Instruction::sub, getReg(it->defs[0]), getReg(it->uses[1]));
                },
                CASE_VAL(RET*) {
                    mc.leave();
                    mc.ret();
                },
                CASE_VAL(MOVRIP*) {
                    auto sym = mc.writeRegRipInst(X64Instruction::mov, getReg(it->defs[0]), 0);
                    requestOffset(it->id, sym);
                },
                CASE_VAL(ADDIMM*) {
                    mc.addImm32(getReg(it->defs[0]), it->imm);
                },
                CASE_VAL(SUBIMM*) {
                    mc.subImm32(getReg(it->defs[0]), it->imm);
                },
                CASE_VAL(MOVMEMIMM*) {
                    mc.readMem(getReg(it->defs[0]), getReg(it->uses[0]), it->imm, 8);
                },
                CASE_VAL(LOADMEM*) {
                    mc.readMem(getReg(it->defs[0]), getReg(it->uses[0]), it->offset, 8);
                },
                CASE_VAL(STOREMEM*) {
                    mc.writeMem(getReg(it->uses[0]), getReg(it->uses[1]), it->offset, 8);
                },
                CASE_VAL(CMP*) {
                    mc.writeRegInst(X64Instruction::cmp, getReg(it->uses[0]), getReg(it->uses[1]));
                },
                CASE_VAL(TEST*) {
                    mc.writeRegInst(X64Instruction::test, getReg(it->uses[0]), getReg(it->uses[1]));
                },
                CASE_VAL(CALLREG*) {
                    mc.call(getReg(it->reg));
                },
                CASE_VAL(CMPIMM*) {
                    mc.cmp64Imm(getReg(it->uses[0]), it->imm);
                },
                CASE_VAL(JZ*) {
                    auto imm = mc.writeJmp(CmpType::Equal, 0_u32);
                    requestJmp(it->tgt, imm);
                },
                CASE_VAL(JMP*) {
                    auto imm = mc.writeJmp(0_u32);
                    requestJmp(it->tgt, imm);
                },
                CASE_VAL(LEASTACK*) {
                    auto imm = mc.lea(getReg(it->defs[0]), x86::Rsp, 0);
                    requestStack(it->slot, imm);
                },
                CASE_VAL(STORESTACK*) {
                    auto imm = mc.writeStack(500'000, getReg(it->uses[0]));
                    requestStack(it->slot, imm, it->offset);
                },
                CASE_VAL(LOADSTACK*) {
                    auto imm = mc.readStack(500'000, getReg(it->defs[0]));
                    requestStack(it->slot, imm, it->offset);
                },
                CASE_VAL(MUL*) {
                    mc.writeRegInst(SusX64Instruction::imul, getReg(it->defs[0]), getReg(it->uses[1]));
                },
                CASE_VAL(INT3*) {
                    mc.trap();
                },
                CASE_VAL(SETZ*) {
                    mc.setCC(getReg(it->defs[0]), CmpType::Equal);
                    mc.writeMovZX8(getReg(it->defs[0]), getReg(it->defs[0]));
                },
                CASE_VAL(SETNZ*) {
                    mc.setCC(getReg(it->defs[0]), CmpType::NotEqual);
                },
                CASE_VAL(SETGT*) {
                    mc.setCC(getReg(it->defs[0]), CmpType::Greater);
                    mc.writeMovZX8(getReg(it->defs[0]), getReg(it->defs[0]));
                },
                CASE_VAL(SETGE*) {
                    mc.setCC(getReg(it->defs[0]), CmpType::LessOrEqual);
                },
                CASE_VAL(SETLS*) {
                    mc.setCC(getReg(it->defs[0]), CmpType::Less);
                    mc.writeMovZX8(getReg(it->defs[0]), getReg(it->defs[0]));
                },
                CASE_VAL(SETLE*) {
                    mc.setCC(getReg(it->defs[0]), CmpType::LessOrEqual);
                },
                CASE_VAL(XOR*) {
                    mc.writeRegInst(X64Instruction::xorI, getReg(it->defs[0]), getReg(it->uses[1]));
                },
                CASE_VAL(CQO*) {
                    mc.cqo();
                },
                CASE_VAL(IDIV*) {
                    mc.signedDivide(getReg(it->uses[0]));
                },
                CASE_VAL(IDIV*) {
                    mc.signedDivide(getReg(it->uses[0]));
                },
                CASE_VAL(AND*) {
                    mc.writeRegInst(X64Instruction::And, getReg(it->defs[0]), getReg(it->uses[1]));
                },
                CASE_VAL(OR*) {
                    mc.writeRegInst(X64Instruction::Or, getReg(it->defs[0]), getReg(it->uses[1]));
                },
                CASE_VAL(ASR*) {
                    mc.shiftRightByCl(getReg(it->defs[0]));
                },
                CASE_VAL(SHR*) {
                    TODO()
                },
                CASE_VAL(SHL*) {
                    mc.shiftLeftByCl(getReg(it->defs[0]));
                },
                CASE_VAL(FAKE_DEF*) {},
                CASE_VAL(FAKE_USE*) {}
                );
                if (not didDispatch) PANIC("DID NOT MATCH {}", inst->className());
            }
        }
    }
};