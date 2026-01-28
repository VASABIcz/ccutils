#pragma once
#include "Registers.hpp"
#include "X86Instruction.hpp"
#include "utils/BetterSpan.h"

struct RET: X86Instruction {
    DEBUG_INFO2(RET)
    void init(std::initializer_list<BaseRegister*> uses) { setUse(uses); }
};

struct FAKE_DEF: X86Instruction {
    DEBUG_INFO2(FAKE_DEF)
    void init(BaseRegister* def) { addDef(def); }
};

struct FAKE_USE: X86Instruction {
    DEBUG_INFO2(FAKE_USE)
    void init(BaseRegister* def) { addUse(def); }
};

struct CALLRIP: X86Instruction {
    DEBUG_INFO2(CALLRIP)
    size_t id;

    void init(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills, size_t id) {
        this->id = id;
        markSideEffect();
        setDef(std::vector<BaseRegister*>{defs.begin(), defs.end()});
        setKill(std::vector<BaseRegister*>{kills.begin(), kills.end()});
        setUse(std::vector<BaseRegister*>{uses.begin(), uses.end()});
    }
};

struct CALLREG: X86Instruction {
    DEBUG_INFO2(CALLREG)

    BaseRegister* reg = nullptr;

    void init(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills, BaseRegister* reg) {
        markSideEffect();
        this->reg = reg;
        setDef(std::vector<BaseRegister*>{defs.begin(), defs.end()});
        setKill(std::vector<BaseRegister*>{kills.begin(), kills.end()});
        setUse(std::vector<BaseRegister*>{uses.begin(), uses.end()});
        addUse(reg);
    }
};

struct MOVRIP: X86Instruction {
    DEBUG_INFO2(MOVRIP)
    size_t id;
    void init(BaseRegister* dst, size_t id) { this->id = id; addDef(dst); }
};

struct MOV: X86Instruction {
    DEBUG_INFO2(MOV)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(rhs);
    }
};

struct LEAVE: X86Instruction {
    DEBUG_INFO2(LEAVE)
    void init() {
        // FIXME do we def esp/ebp here?
    }
};

struct PUSH: X86Instruction {
    DEBUG_INFO2(PUSH)
    void init(BaseRegister* value) { addUse(value); }
};

struct POP: X86Instruction {
    DEBUG_INFO2(POP)
    void init(BaseRegister* dst) { addDef(dst); }
};

struct MOVIMM: X86Instruction {
    DEBUG_INFO2(MOVIMM)
    uint64_t value;

    void init(BaseRegister* lhs, uint64_t value) {
        this->value = value;
        addDef(lhs);
    }
};

struct MUL: X86Instruction {
    DEBUG_INFO2(MUL)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct SUB: X86Instruction {
    DEBUG_INFO2(SUB)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct ADD: X86Instruction {
    DEBUG_INFO2(ADD)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct XOR: X86Instruction {
    DEBUG_INFO2(XOR)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct AND: X86Instruction {
    DEBUG_INFO2(AND)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct OR: X86Instruction {
    DEBUG_INFO2(OR)
    void init(BaseRegister* lhs, BaseRegister* rhs) {
        addDef(lhs);
        addUse(lhs);
        addUse(rhs);
    }
};

struct ADDIMM: X86Instruction {
    DEBUG_INFO2(ADDIMM)

    long imm;

    void init(BaseRegister* lhs, long imm) {
        this->imm = imm;
        addDef(lhs);
        addUse(lhs);
    }
};

struct SUBIMM: X86Instruction {
    DEBUG_INFO2(SUBIMM)

    long imm;

    void init(BaseRegister* lhs, long imm) {
        this->imm = imm;
        addDef(lhs);
        addUse(lhs);
    }
};

struct Block;

struct JMP: X86Instruction {
    DEBUG_INFO2(JMP)
    Block* tgt;
    void init(Block* tgt) { this->tgt = tgt; }
};

struct CMP: X86Instruction {
    DEBUG_INFO2(CMP)
    void init(BaseRegister* flags, BaseRegister* lhs, BaseRegister* rhs) {
        addUse(lhs);
        addUse(rhs);
        addDef(flags);
    }
};

struct TEST: X86Instruction {
    DEBUG_INFO2(TEST)
    void init(BaseRegister* flags, BaseRegister* lhs, BaseRegister* rhs) {
        addUse(lhs);
        addUse(rhs);
        addDef(flags);
    }
};

struct CQO: X86Instruction {
    DEBUG_INFO2(CQO)
    void init() {
        addUse(PHY(x86::Rax));

        addDef(PHY(x86::Rdx));
        addDef(PHY(x86::Rax));
    }
};

struct IDIV: X86Instruction {
    DEBUG_INFO2(IDIV)
    void init(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rdx));
        addUse(PHY(x86::Rax));

        addDef(PHY(x86::Rdx));
        addDef(PHY(x86::Rax));
    }
};

struct SHL: X86Instruction {
    DEBUG_INFO2(SHL)
    void init(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rcx));

        addDef(reg);
    }
};

struct SHR: X86Instruction {
    DEBUG_INFO2(SHR)
    void init(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rcx));

        addDef(reg);
    }
};

struct ASR: X86Instruction {
    DEBUG_INFO2(ASR)
    void init(BaseRegister* reg) {
        addUse(reg);

        addUse(PHY(x86::Rcx));

        addDef(reg);
    }
};

struct SETZ: X86Instruction {
    DEBUG_INFO2(SETZ)
    void init(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETNZ: X86Instruction {
    DEBUG_INFO2(SETNZ)
    void init(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETLE: X86Instruction {
    DEBUG_INFO2(SETLE)
    void init(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETLS: X86Instruction {
    DEBUG_INFO2(SETLS)
    void init(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETGT: X86Instruction {
    DEBUG_INFO2(SETGT)
    void init(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct SETGE: X86Instruction {
    DEBUG_INFO2(SETGE)
    void init(BaseRegister* flags, BaseRegister* dst) {
        addDef(dst);
        addUse(flags);
    }
};

struct INT3: X86Instruction {
    DEBUG_INFO2(INT3)

    void init() {}
};

struct HLT: X86Instruction {
    DEBUG_INFO2(HLT)

    void init() {}
};

struct CMPIMM: X86Instruction {
    DEBUG_INFO2(CMPIMM)

    long imm;

    void init(BaseRegister* flags, BaseRegister* lhs, long imm) {
        this->imm = imm;
        addUse(lhs);
        addDef(flags);
    }
};

struct LOADMEM: X86Instruction {
    DEBUG_INFO2(LOADMEM)

    long offset;

    void init(BaseRegister* dst, BaseRegister* src, long offset) {
        this->offset = offset;
        addDef(dst);
        addUse(src);
    }
};

struct STOREMEM: X86Instruction {
    DEBUG_INFO2(STOREMEM)

    long offset;

    void init(BaseRegister* dst, BaseRegister* src, long offset) {
        this->offset = offset;
        addUse(dst);
        addUse(src);
    }
};

struct STORESTACK: X86Instruction {
    DEBUG_INFO2(STORESTACK)

    StackSlot slot;
    long offset;

    void init(BaseRegister* value, StackSlot slot, long offset) {
        this->slot = slot;
        this->offset = offset;
        addUse(value);
    }
};

struct LOADSTACK: X86Instruction {
    DEBUG_INFO2(LOADSTACK)

    StackSlot slot;
    long offset;
    size_t size;

    void init(BaseRegister* dst, StackSlot slot, long offset, size_t size) {
        this->slot = slot;
        this->offset = offset;
        this->size = size;
        addDef(dst);
    }
};

struct LOADRIP: X86Instruction {
    DEBUG_INFO2(LOADRIP)
    void init(BaseRegister* dst, long offset) { addDef(dst); }
};

struct LEASTACK: X86Instruction {
    DEBUG_INFO2(LEASTACK)

    StackSlot slot;

    void init(BaseRegister* dst, StackSlot slot) { this->slot = slot; addDef(dst); }
};

struct MOVMEMIMM: X86Instruction {
    DEBUG_INFO2(MOVMEMIMM)

    long imm;

    void init(BaseRegister* dst, BaseRegister* src, long imm) {
        this->imm = imm;
        addDef(dst);
        addUse(src);
    }
};

struct JZ: X86Instruction {
    DEBUG_INFO2(JZ)
    Block* tgt;
    void init(BaseRegister* flags, Block* tgt) { this->tgt = tgt; addUse(flags); }
};

struct SYSCALL: X86Instruction {
    DEBUG_INFO2(SYSCALL)

    void init(BetterSpan<BaseRegister*> defs, BetterSpan<BaseRegister*> uses, BetterSpan<BaseRegister*> kills) {
        markSideEffect();
        setDef(std::vector<BaseRegister*>{defs.begin(), defs.end()});
        setDef(std::vector<BaseRegister*>{kills.begin(), kills.end()});
        setDef(std::vector<BaseRegister*>{uses.begin(), uses.end()});
    }
};