#pragma once
#include "Registers.hpp"
#include "X86Instruction.hpp"
#include "utils/BetterSpan.h"

struct RET: X86Instruction {
    DEBUG_INFO2(RET)
    RET(std::initializer_list<BaseRegister*> uses) { setUse(uses); }
};

struct FAKE_DEF: X86Instruction {
    DEBUG_INFO2(FAKE_DEF)
    FAKE_DEF(BaseRegister* def) { addDef(def); }
};

struct FAKE_USE: X86Instruction {
    DEBUG_INFO2(FAKE_USE)
    FAKE_USE(BaseRegister* def) { addUse(def); }
};

struct CALLRIP: X86Instruction {
    DEBUG_INFO2(CALLRIP)
    size_t id;

    /*    CALLRIP(std::initializer_list<BaseRegister*> defs, std::initializer_list<BaseRegister*> uses, std::initializer_list<BaseRegister*> kills, size_t id): id(id) {
            this->defs = defs;
            this->kills = kills;
            this->uses = uses;
        }*/

    CALLRIP(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills, size_t id) : id(id) {
        setDef(std::vector<BaseRegister*>{defs.begin(), defs.end()});
        setKill(std::vector<BaseRegister*>{kills.begin(), kills.end()});
        setUse(std::vector<BaseRegister*>{uses.begin(), uses.end()});
    }
};

struct CALLREG: X86Instruction {
    DEBUG_INFO2(CALLREG)

    BaseRegister* reg = nullptr;

    CALLREG(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills, BaseRegister* reg) : reg(reg) {
        setDef(std::vector<BaseRegister*>{defs.begin(), defs.end()});
        setKill(std::vector<BaseRegister*>{kills.begin(), kills.end()});
        setUse(std::vector<BaseRegister*>{uses.begin(), uses.end()});
        addUse(reg);
    }
};

struct MOVRIP: X86Instruction {
    DEBUG_INFO2(MOVRIP)
    size_t id;
    MOVRIP(BaseRegister* dst, size_t id) : id(id) { addDef(dst); }
};

struct MOV: X86Instruction {
    DEBUG_INFO2(MOV)
    MOV(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(rhs);
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
    PUSH(BaseRegister* value) { addUse(value); }
};

struct POP: X86Instruction {
    DEBUG_INFO2(POP)
    POP(BaseRegister* dst) { addDef(dst); }
};

struct MOVIMM: X86Instruction {
    DEBUG_INFO2(MOVIMM)
    uint64_t value;

    MOVIMM(BaseRegister* lhs, uint64_t value) : value(value) { addDef(lhs); }
};

struct MUL: X86Instruction {
    DEBUG_INFO2(MUL)
    MUL(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct SUB: X86Instruction {
    DEBUG_INFO2(SUB)
    SUB(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct ADD: X86Instruction {
    DEBUG_INFO2(ADD)
    ADD(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct XOR: X86Instruction {
    DEBUG_INFO2(XOR)
    XOR(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct AND: X86Instruction {
    DEBUG_INFO2(AND)
    AND(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct OR: X86Instruction {
    DEBUG_INFO2(OR)
    OR(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct ADDIMM: X86Instruction {
    DEBUG_INFO2(ADDIMM)

    long imm;

    ADDIMM(BaseRegister* lhs, long imm) : imm(imm) {
        addDef(lhs);
        addUse(lhs);
    }
};

struct SUBIMM: X86Instruction {
    DEBUG_INFO2(SUBIMM)

    long imm;

    SUBIMM(BaseRegister* lhs, long imm) : imm(imm) {
        addDef(lhs);
        addUse(lhs);
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
    CMP(BaseRegister* flags, BaseRegister* lhs, BaseRegister* rhs) {
        addUse(lhs);
        addUse(rhs);
        addDef(flags);
    }
};

struct TEST: X86Instruction {
    DEBUG_INFO2(TEST)
    TEST(BaseRegister* flags, BaseRegister* lhs, BaseRegister* rhs) {
        addUse(lhs);
        addUse(rhs);
    }
};

struct CQO: X86Instruction {
    DEBUG_INFO2(CQO)
    CQO() {
        addUse(PHY(x86::Rax));

        addDef(PHY(x86::Rdx));
        addDef(PHY(x86::Rax));
    }
};

struct IDIV: X86Instruction {
    DEBUG_INFO2(IDIV)
    IDIV(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rdx));
        addUse(PHY(x86::Rax));

        addDef(PHY(x86::Rdx));
        addDef(PHY(x86::Rax));
    }
};

struct SHL: X86Instruction {
    DEBUG_INFO2(SHL)
    SHL(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rcx));

        addDef(reg);
    }
};

struct SHR: X86Instruction {
    DEBUG_INFO2(SHR)
    SHR(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rcx));

        addDef(reg);
    }
};

struct ASR: X86Instruction {
    DEBUG_INFO2(ASR)
    ASR(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rcx));

        addDef(reg);
    }
};

struct SETZ: X86Instruction {
    DEBUG_INFO2(SETZ)
    SETZ(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETNZ: X86Instruction {
    DEBUG_INFO2(SETNZ)
    SETNZ(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETLE: X86Instruction {
    DEBUG_INFO2(SETLE)
    SETLE(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETLS: X86Instruction {
    DEBUG_INFO2(SETLS)
    SETLS(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETGT: X86Instruction {
    DEBUG_INFO2(SETGT)
    SETGT(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETGE: X86Instruction {
    DEBUG_INFO2(SETGE)
    SETGE(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
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

    CMPIMM(BaseRegister* flags, BaseRegister* lhs, long imm) : imm(imm) {
        addUse(lhs);
        addDef(flags);
    }
};

struct LOADMEM: X86Instruction {
    DEBUG_INFO2(LOADMEM)

    long offset;

    LOADMEM(BaseRegister* dst, BaseRegister* src, long offset) : offset(offset) {
        addDef(dst);
        addUse(src);
    }
};

struct STOREMEM: X86Instruction {
    DEBUG_INFO2(STOREMEM)

    long offset;

    STOREMEM(BaseRegister* dst, BaseRegister* src, long offset) : offset(offset) {
        addUse(dst);
        addUse(src);
    }
};

struct STORESTACK: X86Instruction {
    DEBUG_INFO2(STORESTACK)

    StackSlot slot;
    long offset;

    STORESTACK(BaseRegister* value, StackSlot slot, long offset) : slot(slot), offset(offset) { addUse(value); }
};

struct LOADSTACK: X86Instruction {
    DEBUG_INFO2(LOADSTACK)

    StackSlot slot;
    long offset;
    size_t size;

    LOADSTACK(BaseRegister* dst, StackSlot slot, long offset, size_t size) : slot(slot), offset(offset), size(size) { addDef(dst); }
};

struct LOADRIP: X86Instruction {
    DEBUG_INFO2(LOADRIP)
    LOADRIP(BaseRegister* dst, long offset) { addDef(dst); }
};

struct LEASTACK: X86Instruction {
    DEBUG_INFO2(LEASTACK)

    StackSlot slot;

    LEASTACK(BaseRegister* dst, StackSlot slot) : slot(slot) { addDef(dst); }
};

struct MOVMEMIMM: X86Instruction {
    DEBUG_INFO2(MOVMEMIMM)

    long imm;

    MOVMEMIMM(BaseRegister* dst, BaseRegister* src, long imm) : imm(imm) {
        addDef(dst);
        addUse(src);
    }
};

struct JZ: X86Instruction {
    DEBUG_INFO2(JZ)
    Block* tgt;
    JZ(BaseRegister* flags, Block* tgt) : tgt(tgt) { addUse(flags); }
};

struct SYSCALL: X86Instruction {
    DEBUG_INFO2(SYSCALL)

    SYSCALL(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills) {
        setDef(std::vector<BaseRegister*>{defs.begin(), defs.end()});
        setDef(std::vector<BaseRegister*>{kills.begin(), kills.end()});
        setDef(std::vector<BaseRegister*>{uses.begin(), uses.end()});
    }
};