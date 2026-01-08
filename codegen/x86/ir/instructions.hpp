#pragma once
#include "X86Instruction.hpp"
#include "Registers.hpp"
#include "utils/BetterSpan.h"

struct RET: X86Instruction {
    DEBUG_INFO2(RET)
    RET(std::initializer_list<BaseRegister*> uses) { this->uses = uses; }
};

struct FAKE_DEF: X86Instruction {
    DEBUG_INFO2(FAKE_DEF)
    FAKE_DEF(BaseRegister* def) { this->defs.push_back(def); }
};

struct FAKE_USE: X86Instruction {
    DEBUG_INFO2(FAKE_USE)
    FAKE_USE(BaseRegister* def) { this->uses.push_back(def); }
};

struct CALLRIP: X86Instruction {
    DEBUG_INFO2(CALLRIP)
    size_t id;

    /*    CALLRIP(std::initializer_list<BaseRegister*> defs, std::initializer_list<BaseRegister*> uses, std::initializer_list<BaseRegister*> kills, size_t id): id(id) {
            this->defs = defs;
            this->kills = kills;
            this->uses = uses;
        }*/

    CALLRIP(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills, size_t id): id(id) {
        this->defs = std::vector<BaseRegister*>{defs.begin(), defs.end()};
        this->kills = std::vector<BaseRegister*>{kills.begin(), kills.end()};
        this->uses = std::vector<BaseRegister*>{uses.begin(), uses.end()};
    }
};

struct CALLREG: X86Instruction {
    DEBUG_INFO2(CALLREG)

    BaseRegister* reg = nullptr;

    /*    CALLREG(std::initializer_list<BaseRegister*> defs, std::initializer_list<BaseRegister*> uses, std::initializer_list<BaseRegister*> kills, BaseRegister* reg) : reg(reg) {
            defs.b
            this->defs = defs;
            this->kills = kills;
            this->uses = uses;
            this->uses.push_back(reg);
        }*/

    CALLREG(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills, BaseRegister* reg) : reg(reg) {
        this->defs = std::vector<BaseRegister*>{defs.begin(), defs.end()};
        this->kills = std::vector<BaseRegister*>{kills.begin(), kills.end()};
        this->uses = std::vector<BaseRegister*>{uses.begin(), uses.end()};
        this->uses.push_back(reg);
    }
};

struct MOVRIP: X86Instruction {
    DEBUG_INFO2(MOVRIP)
    size_t id;
    MOVRIP(BaseRegister* dst, size_t id) : id(id) { this->defs.push_back(dst); }
};

struct MOV: X86Instruction {
    DEBUG_INFO2(MOV)
    MOV(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct LEAVE: X86Instruction {
    DEBUG_INFO2(LEAVE)
    LEAVE() {
        // FIXME do we def esp/ebp here?
    }
};

struct PUSH: X86Instruction {
    DEBUG_INFO2(PUSH)
    PUSH(BaseRegister* value) {
        this->uses.push_back(value);
    }
};

struct POP: X86Instruction {
    DEBUG_INFO2(POP)
    POP(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct MOVIMM: X86Instruction {
    DEBUG_INFO2(MOVIMM)
    uint64_t value;

    MOVIMM(BaseRegister* lhs, uint64_t value) : value(value) { this->defs.push_back(lhs); }
};

struct MUL: X86Instruction {
    DEBUG_INFO2(MUL)
    MUL(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct SUB: X86Instruction {
    DEBUG_INFO2(SUB)
    SUB(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct ADD: X86Instruction {
    DEBUG_INFO2(ADD)
    ADD(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct XOR: X86Instruction {
    DEBUG_INFO2(XOR)
    XOR(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct AND: X86Instruction {
    DEBUG_INFO2(AND)
    AND(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct OR: X86Instruction {
    DEBUG_INFO2(OR)
    OR(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct ADDIMM: X86Instruction {
    DEBUG_INFO2(ADDIMM)

    long imm;

    ADDIMM(BaseRegister* lhs, long imm) : imm(imm) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
    }
};

struct SUBIMM: X86Instruction {
    DEBUG_INFO2(SUBIMM)

    long imm;

    SUBIMM(BaseRegister* lhs, long imm) : imm(imm) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
    }
};

struct Block;

struct JMP: X86Instruction {
    DEBUG_INFO2(JMP)
    Block* tgt;
    JMP(Block* tgt) : tgt(tgt) {}
};

struct CMP: X86Instruction {
    DEBUG_INFO2(CMP)
    CMP(BaseRegister* lhs, BaseRegister* rhs) {
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct TEST: X86Instruction {
    DEBUG_INFO2(TEST)
    TEST(BaseRegister* lhs, BaseRegister* rhs) {
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct CQO: X86Instruction {
    DEBUG_INFO2(CQO)
    CQO() {
        this->uses.push_back(new Physical(x86::Rax));

        this->defs.push_back(new Physical(x86::Rdx));
        this->defs.push_back(new Physical(x86::Rax));
    }
};

struct IDIV: X86Instruction {
    DEBUG_INFO2(IDIV)
    IDIV(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rdx));
        this->uses.push_back(new Physical(x86::Rax));

        this->defs.push_back(new Physical(x86::Rdx));
        this->defs.push_back(new Physical(x86::Rax));
    }
};

struct SHL: X86Instruction {
    DEBUG_INFO2(SHL)
    SHL(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rcx));

        this->defs.push_back(reg);
    }
};

struct SHR: X86Instruction {
    DEBUG_INFO2(SHR)
    SHR(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rcx));

        this->defs.push_back(reg);
    }
};

struct ASR: X86Instruction {
    DEBUG_INFO2(ASR)
    ASR(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rcx));

        this->defs.push_back(reg);
    }
};

struct SETZ: X86Instruction {
    DEBUG_INFO2(SETZ)
    SETZ(BaseRegister* dst) { this->defs.push_back(dst); }
};

struct SETNZ: X86Instruction {
    DEBUG_INFO2(SETNZ)
    SETNZ(BaseRegister* dst) { this->defs.push_back(dst); }
};

struct SETLE: X86Instruction {
    DEBUG_INFO2(SETLE)
    SETLE(BaseRegister* dst) { this->defs.push_back(dst); }
};

struct SETLS: X86Instruction {
    DEBUG_INFO2(SETLS)
    SETLS(BaseRegister* dst) { this->defs.push_back(dst); }
};

struct SETGT: X86Instruction {
    DEBUG_INFO2(SETGT)
    SETGT(BaseRegister* dst) { this->defs.push_back(dst); }
};

struct SETGE: X86Instruction {
    DEBUG_INFO2(SETGE)
    SETGE(BaseRegister* dst) { this->defs.push_back(dst); }
};

struct INT3: X86Instruction {
    DEBUG_INFO2(INT3)
};

struct HLT: X86Instruction {
    DEBUG_INFO2(HLT)
};

struct CMPIMM: X86Instruction {
    DEBUG_INFO2(CMPIMM)

    long imm;

    CMPIMM(BaseRegister* lhs, long imm) : imm(imm) { this->uses.push_back(lhs); }
};

struct LOADMEM: X86Instruction {
    DEBUG_INFO2(LOADMEM)

    long offset;

    LOADMEM(BaseRegister* dst, BaseRegister* src, long offset) : offset(offset) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct STOREMEM: X86Instruction {
    DEBUG_INFO2(STOREMEM)

    long offset;

    STOREMEM(BaseRegister* dst, BaseRegister* src, long offset) : offset(offset) {
        this->uses.push_back(dst);
        this->uses.push_back(src);
    }
};

struct STORESTACK: X86Instruction {
    DEBUG_INFO2(STORESTACK)

    StackSlot slot;
    long offset;

    STORESTACK(BaseRegister* value, StackSlot slot, long offset) : slot(slot), offset(offset) { this->uses.push_back(value); }
};

struct LOADSTACK: X86Instruction {
    DEBUG_INFO2(LOADSTACK)

    StackSlot slot;
    long offset;
    size_t size;

    LOADSTACK(BaseRegister* dst, StackSlot slot, long offset, size_t size) : slot(slot), offset(offset), size(size) { this->defs.push_back(dst); }
};

struct LOADRIP: X86Instruction {
    DEBUG_INFO2(LOADRIP)
    LOADRIP(BaseRegister* dst, long offset) { this->defs.push_back(dst); }
};

struct LEASTACK: X86Instruction {
    DEBUG_INFO2(LEASTACK)

    StackSlot slot;

    LEASTACK(BaseRegister* dst, StackSlot slot) : slot(slot) { this->defs.push_back(dst); }
};

struct MOVMEMIMM: X86Instruction {
    DEBUG_INFO2(MOVMEMIMM)

    long imm;

    MOVMEMIMM(BaseRegister* dst, BaseRegister* src, long imm) : imm(imm) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct JZ: X86Instruction {
    DEBUG_INFO2(JZ)
    Block* tgt;
    JZ(Block* tgt) : tgt(tgt) {}
};

struct SYSCALL: X86Instruction {
    DEBUG_INFO2(SYSCALL)

    SYSCALL(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills) {
        this->defs = std::vector<BaseRegister*>{defs.begin(), defs.end()};
        this->kills = std::vector<BaseRegister*>{kills.begin(), kills.end()};
        this->uses = std::vector<BaseRegister*>{uses.begin(), uses.end()};
    }
};