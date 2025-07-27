#pragma once
#include <vector>
#include <string>
#include <ranges>
#include "definitions.h"
#include "../utils/utils.h"

struct ImmSpace {
    size_t offset;
    size_t size;
};

struct Arg {
    enum Type {
        REGISTER,
        IMMEDIATE,
        // SYMBOL, // space = *(&symbol-&space+offset)
        REG_OFFSET,
        REG_OFFSET_VALUE,
        SYMBOL_RIP_OFF_32, // space = &symbol-&space+offset
        SYMBOL_RIP_VALUE_32, // space = *(&symbol-&space+offset)
    };
    bool isFloat = false;
    size_t size;
    X64Register reg = X64Register::Rsp;
    int offset;
    Type type;
    size_t immValue;
    std::optional<size_t> symbol;
    std::optional<size_t> symbolType;

    size_t sizeBytes() const {
        switch (type) {
            case REGISTER:
                return 8;
            case IMMEDIATE:
                return 8;
            // case SYMBOL:
            //    return 8;
            case REG_OFFSET:
                return 8;
            case REG_OFFSET_VALUE:
                return size;
            case SYMBOL_RIP_OFF_32:
                return 8;
            case SYMBOL_RIP_VALUE_32:
                return 8;
        }
        PANIC();
    }

    static Arg Reg(X64Register reg) {
        Arg idk;
        idk.reg = reg;
        idk.type = Type::REGISTER;

        return idk;
    }

    static Arg StackValue(int offset, size_t size) {
        Arg idk;
        idk.type = Type::REG_OFFSET_VALUE;
        idk.reg = X64Register::Rsp;
        idk.offset = offset;
        idk.size = size;

        return idk;
    }

    static Arg MemoryValue(X64Register reg, int offset, size_t size) {
        Arg idk;
        idk.type = Type::REG_OFFSET_VALUE;
        idk.reg = reg;
        idk.offset = offset;
        idk.size = size;

        return idk;
    }

    static Arg MemoryPtr(X64Register reg, int offset) {
        Arg idk;
        idk.type = Type::REG_OFFSET;
        idk.reg = reg;
        idk.offset = offset;

        return idk;
    }

    static Arg StackPtr(int offset) {
        Arg idk;
        idk.type = Type::REG_OFFSET;
        idk.reg = X64Register::Rsp;
        idk.offset = offset;

        return idk;
    }

    static Arg Imm(size_t value) {
        Arg idk;
        idk.type = Type::IMMEDIATE;
        idk.immValue = value;

        return idk;
    }

    static Arg ImmPtr(const void* value) {
        Arg idk;
        idk.type = Type::IMMEDIATE;
        idk.immValue = std::bit_cast<size_t>(value);

        return idk;
    }

    template<class T, class... Args>
    static Arg FunPtr(T(*value)(Args...)) {
        Arg idk;
        idk.type = Type::IMMEDIATE;
        idk.immValue = std::bit_cast<size_t>(value);

        return idk;
    }

    static Arg ImmPtrC(const void* value) {
        Arg idk;
        idk.type = Type::IMMEDIATE;
        idk.immValue = std::bit_cast<size_t>(value);

        return idk;
    }

    static Arg Rel32Adr(size_t type, size_t symbol, int32_t adent) {
        Arg idk;
        idk.symbol = symbol;
        idk.type = Type::SYMBOL_RIP_OFF_32;
        idk.offset = adent;
        idk.symbolType = type;

        return idk;
    }

    static Arg Rel32Val(size_t symbol, int32_t adent) {
        Arg idk;
        idk.symbol = symbol;
        idk.type = Type::SYMBOL_RIP_VALUE_32;
        idk.offset = adent;

        return idk;
    }

    string toString() const {
        switch (type) {
            case REGISTER:
                return stringify("Reg({})", reg.toString());
            case IMMEDIATE:
                return stringify("Imm({})", immValue);
            // case SYMBOL:
            //    return stringify("Sym({})", symbol);
            case REG_OFFSET:
                return stringify("Off({}+{})", reg, offset);
            case REG_OFFSET_VALUE:
                return stringify("Value({}+{}:{})", reg, offset, size);
            case SYMBOL_RIP_OFF_32:
            case SYMBOL_RIP_VALUE_32:
                TODO()
        }
        PANIC();
    }
};

class X86mc {
    vector<u8>& bytes;
public:
    explicit X86mc(vector<u8>& bytes): bytes(bytes) {}

    template<typename T>
    ImmSpace writeImmValue(T b) {
        auto current = currentImm(sizeof b);

        auto rawPtr = reinterpret_cast<u8*>(&b);

        for (auto i = 0u; i < sizeof b; i++) {
            bytes.push_back(rawPtr[i]);
        }

        return current;
    }

    void rdtscp() {
        // rdtscp
        pushBack(0x0F);
        pushBack(0x01);
        pushBack(0xF9);
    }

    void rdtsc() {
        // rdtsc
        pushBack(0x0F);
        pushBack(0x31);
    }

    void writeRex(bool immediate64 = false, bool extDest = false, bool extSrc = false, bool extIndex = false);

    void writeModRM(const X64Register& dest, const X64Register& src, bool isRegDirect = true, bool isNoOffset = false, u8 orFlag = 0x5);

    void writeModMR(const X64Register& dest, const X64Register& src, bool isRegDirect = true, bool isNoOffset = false);

    void writeSIB(SibScale scale, const X64Register& base, const X64Register& index);

    void setIfFlag(X64Register dest, CmpType type);

    void pushBack(u8 value);

    void writeMRCode(X64Instruction inst);

    void writeRMCode(X64Instruction inst);

    void writeRegInst(SusX64Instruction inst, const X64Register& dest, const X64Register& src);

    void writeRegInst(X64Instruction inst, const X64Register& dest, const X64Register& src);

    void writeRegMemInst(X64Instruction inst, const X64Register& dest, const X64Register& subj, int32_t offset, bool immediate64 = true) {
        writeRex(immediate64, dest.isExt(), subj.isExt());
        writeRMCode(inst);

        someOffsetStuffForMov(dest, subj, offset);
    }

    void doNot(X64Register reg) {
        writeRex(true, false, reg.isExt());
        pushBack(0xF7);
        writeModRM(X64Register::Two, reg);
    }

    void writeMemRegInst(X64Instruction inst, const X64Register& dest, int32_t offset, const X64Register& value) {
        writeRex(true, value.isExt(), dest.isExt());
        writeMRCode(inst);

        someOffsetStuffForMov(value, dest, offset);
    }

    enum class ModRmType {
        DEREF    = 0b00, // rsp needs to use sib, rbp is rip + 32bit offset
        DEREF_8  = 0b01, // rsp needs to use sib
        DEREF_32 = 0b10, // rsp needs to use sib
        NO_DEREF = 0b11, // just the register no offset
    };

    void writeModRm(ModRmType type, X64Register a, X64Register b) {
        pushBack((u8)type << 6 | a.getEncoding() << 3 | b.getEncoding() << 0);
    }

    ImmSpace writeRipRegInst(X64Instruction inst, int32_t offset, const X64Register& value) {
        writeRex(true, value.isExt(), X64Register::Rbp.isExt());
        writeMRCode(inst);

        writeModRm(ModRmType::DEREF, value, X64Register::Rbp);
        return writeImmValue(offset);
    }

    ImmSpace writeRegRipInst(X64Instruction inst, const X64Register& value, int32_t offset) {
        writeRex(true, value.isExt(), X64Register::Rbp.isExt());
        writeRMCode(inst);

        writeModRm(ModRmType::DEREF, value, X64Register::Rbp);
        return writeImmValue(offset);
    }

    void movReg(const X64Register& dest, const X64Register& src);

    void writeStack(int byteOffset, const X64Register& reg);

    void shiftRImm(const X64Register& dst, u8 amount) {
        writeRex(true, false, dst.isExt());
        pushBack(0xC1);
        writeModRM(X64Register::Five, dst);
        pushBack(amount);
    }

    void shiftRImmSigned(const X64Register& dst, u8 amount) {
        writeRex(true, false, dst.isExt());
        pushBack(0xC1);
        writeModRM(X64Register::Seven, dst);
        pushBack(amount);
    }

    void shiftLImm(const X64Register& dst, u8 amount) {
        writeRex(true, false, dst.isExt());
        pushBack(0xC1);
        writeModRM(X64Register::Four, dst);
        pushBack(amount);
    }

    void writeStackAmount(int offset, const X64Register& src, size_t amount) {
        if (amount == 1) {
            write1(X64Register::Rsp, src, offset);
        } else if (amount == 2) {
            write2(X64Register::Rsp, src, offset);
        } else if (amount == 4) {
            write4(X64Register::Rsp, src, offset);
        } else {
            // println("[WARN] THIS ALSO DOESNT SEEM GUD, INVESTIGATE ONE DAY WHY CANT WE ASSERT 8 BYTES");
            // assert(amount == 8);
            writePtr(X64Register::Rsp, src, offset);
        }
    }

    void readStack(int byteOffset, const X64Register& tgt);

    void readPtr(const X64Register& dst, const X64Register& src, int32_t offset, SibScale scale = SibScale::One);

    ImmSpace read4(const X64Register& dst, const X64Register& src, int32_t offset) {
        writeRex(false, dst.isExt(), src.isExt());
        pushBack(0x8B);
        pushBack(0x80 | dst.getEncoding() << 3 | src.getEncoding());
        if (src == X64Register::Rsp || src == X64Register::R12) {
            pushBack(0x24);
        }

        return writeImmValue(offset);
    }

    ImmSpace read2(const X64Register& dst, const X64Register& src, int32_t offset) {
        pushBack(0x66);
        writeRex(false, dst.isExt(), src.isExt());
        pushBack(0x8B);
        pushBack(0x80 | dst.getEncoding() << 3 | src.getEncoding());
        if (src == X64Register::Rsp || src == X64Register::R12) {
            pushBack(0x24);
        }

        return writeImmValue(offset);
    }

    ImmSpace read1(const X64Register& dst, const X64Register& src, int32_t offset) {
        writeRex(false, dst.isExt(), src.isExt());
        pushBack(0x8A);
        pushBack(0x80 | dst.getEncoding() << 3 | src.getEncoding());
        if (src == X64Register::Rsp || src == X64Register::R12) {
            pushBack(0x24);
        }

        return writeImmValue(offset);
    }

    ImmSpace write1(const X64Register& dst, const X64Register& src, int32_t offset) {
        writeRex(false, src.isExt(), dst.isExt());
        pushBack(0x88);
        pushBack(0x80 | src.getEncoding() << 3 | dst.getEncoding());
        if (dst == X64Register::Rsp || dst == X64Register::R12) {
            pushBack(0x24);
        }

        return writeImmValue(offset);
    }

    ImmSpace write4(const X64Register& dst, const X64Register& src, int32_t offset) {
        writeRex(false, src.isExt(), dst.isExt());
        pushBack(0x89);
        pushBack(0x80 | src.getEncoding() << 3 | dst.getEncoding());
        if (dst == X64Register::Rsp || dst == X64Register::R12) {
            pushBack(0x24);
        }

        return writeImmValue(offset);
    }

    ImmSpace write2(const X64Register& dst, const X64Register& src, int offset) {
        pushBack(0x66);
        writeRex(false, src.isExt(), dst.isExt());
        pushBack(0x89);
        pushBack(0x80 | src.getEncoding() << 3 | dst.getEncoding());
        if (dst == X64Register::Rsp || dst == X64Register::R12) {
            pushBack(0x24);
        }

        return writeImmValue(offset);
    }

    ImmSpace mov32(const X64Register& dest, i32 value) {
        writeRex(true, false, dest.isExt());
        pushBack(0xC7);
        writeModRM(X64Register::Zero, dest);
        return writeImmValue(value);
    }

    void movFast(X64Register dest, uint64_t value) {
        if (value == 0) {
            writeRegInst(X64Instruction::xorI, dest, dest);
        } else if (value <= UINT32_MAX) {
            mov32(dest, bit_cast<size_t>(value));
        } else {
            mov(dest, bit_cast<size_t>(value));
        }
    }

    void writePtr(const X64Register& dest, const X64Register& src, int32_t offset, SibScale scale = SibScale::One);

    ImmSpace mov(const X64Register& dest, size_t value);

    void writeBytes(u8* data, size_t amount);

    void writeMovF(const X64Register& dest, double value);

    static u8 applyRegExt(u8 value, const X64Register& dest, const X64Register& src);

    static u8 applyDestExt(u8 value, const X64Register& reg);

    static u8 applySrcExt(u8 value, const X64Register& reg);

    static u8 applySrcReg(u8 value, const X64Register& reg);

    static u8 applyDestReg(u8 value, const X64Register& reg);

    static u8 applyRegs(u8 value, const X64Register& dest, const X64Register& src);

    ImmSpace currentImm(size_t sizeBytes) const;

    ImmSpace writeJmp(CmpType jmp, u8 offset);

    ImmSpace writeJmp(u8 offset);

    ImmSpace writeJmp(CmpType jmp, u32 offset);

    ImmSpace writeJmp(u32 offset);

/*    0:   48 0f 44 c0             cmove  rax,rax
    4:   48 0f 45 c0             cmovne rax,rax*/

    void cmov(CmpType jmpType, const X64Register& dest, const X64Register& src);

    void push(const X64Register& src);

    void pop(const X64Register& dest);

    void ret();

    void cmpI(CmpType type, const X64Register& dest, const X64Register& left, const X64Register& right);

    void shiftLeftByCl(const X64Register& dest);

    void shiftRightByCl(const X64Register& dest);

    void makeIsTrue(const X64Register& reg);

    void writeSimple(SimpleX64Instruction inst);

    void writeSimpleRex(SimpleX64Instruction inst);

    void writeMovZX8(const X64Register& dest, const X64Register& src);
    void writeMovZX16(const X64Register& dest, const X64Register& src) {
        writeRex(true, dest.isExt(), src.isExt());
        writeSusInst(SusX64Instruction::movzx8);
        writeModRM(dest, src);
    }

    void call(const X64Register& reg);

    void call(const X64Register& reg, i32 offset) {
        writeRegMemInst(X64Instruction::Call, X64Register::Two, reg, offset, false);
    }

    void callMC(size_t address);

    void writeSusInst(SusX64Instruction inst);

    void setCC(const X64Register& dest, CmpType type);

    ImmSpace addImm32(const X64Register& dest, int value);

    ImmSpace subImm32(const X64Register& dest, int value);

    void signedDivide(const X64Register& divisor);

    void cqo();

    void getStackPtr(const X64Register& reg, int32_t stackOffset);

    void nop();

    ImmSpace lea(const X64Register& dest, const X64Register& value, int32_t offset);

    ImmSpace leaRip(const X64Register& dest, int32_t offset);

    void garbageMemCpy(const X64Register& dest, const X64Register& src, size_t amountBytes, const X64Register& tmp, int32_t srcOffset = 0, int dstOffset = 0);

    ImmSpace relativeRead(X64Register dst, int32_t offset) {
        writeRex(true, dst.isExt());
        pushBack((u8)X64Instruction::mov);
        pushBack(0b00 << 6 | dst.getEncoding() << 3 | 0b101); // RM
        return writeImmValue(offset);
    }

    ImmSpace relativeWrite(X64Register dst, int32_t offset) {
        writeRex(true, dst.isExt());
        pushBack(0x89);
        pushBack(0b00 << 6 | dst.getEncoding() << 3 | 0b101); // RM
        return writeImmValue(offset);
    }

    void leave();

    void someOffsetStuffForMov(X64Register dst, X64Register src, int32_t offset);

    void addFloat(const X64Register& dst, const X64Register& lhs, X64Register rhs, bool is64Bit);

    void subFloat(const X64Register& dst, const X64Register& lhs, X64Register rhs, bool is64Bit);

    void mulFloat(const X64Register& dst, const X64Register& lhs, X64Register rhs, bool is64Bit);

    void divFloat(const X64Register& dst, const X64Register& lhs, X64Register rhs, bool is64Bit);

    void i64ToF64(X64Register dst, X64Register src) {
        cvtsi2sd(0, src);
        movq(dst, 0);
    }

    void f64ToI64(X64Register dst, X64Register src) {
        movq(0, src);
        cvttsd2si(dst, 0);
    }

    void i32ToF64(X64Register dst, X64Register src) {
        TODO();
    }

    void f64ToI32(X64Register dst, X64Register src) {
        TODO();
    }

    void i64ToF32(X64Register dst, X64Register src) {
        TODO();
    }

    void f32ToI64(X64Register dst, X64Register src) {
        TODO();
    }

    void i32ToF32(X64Register dst, X64Register src) {
        TODO();
    }

    void f32ToI32(X64Register dst, X64Register src) {
        TODO();
    }

    void movsx8(const X64Register& dst, const X64Register& src) {
        writeRex(true, dst.isExt(), src.isExt());
        pushBack(0x0F);
        pushBack(0xBE);
        writeModRM(dst, src);
    }

    void movsx16(const X64Register& dst, const X64Register& src) {
        writeRex(true, dst.isExt(), src.isExt());
        pushBack(0x0F);
        pushBack(0xBF);
        writeModRM(dst, src);
    }

    void movsx32(const X64Register& dst, const X64Register& src) {
        writeRex(true, dst.isExt(), src.isExt());
        pushBack(0x63);
        writeModRM(dst, src);
    }

    void floatCompare(CmpType type, X64Register dst, X64Register lhs, X64Register rhs, bool is64Bit = true) {
        movq(0, lhs, is64Bit);
        movq(1, rhs, is64Bit);
        comisd(0, 1, is64Bit);
        setCC(dst, type);
        writeMovZX8(dst, dst);
    }

    void floatToDouble(X64Register dst, X64Register src) {
        movq(0, src);
        cvtss2sd(0, 0);
        movq(dst, 0);
    }

    void doubleToFloat(X64Register dst, X64Register src) {
        movq(0, src);
        cvtsd2ss(0, 0);
        movq(dst, 0);
    }

    // f64 -> f32
    void cvtsd2ss(u8 lhs, u8 rhs) {
        pushBack(0xF2);
        pushBack(0x0F);
        pushBack(0x5A);
        pushBack(0xC0 | (lhs << 3) | rhs);
    }

    // f32 -> f64
    void cvtss2sd(u8 lhs, u8 rhs) {
        pushBack(0xF3);
        pushBack(0x0F);
        pushBack(0x5A);
        pushBack(0xC0 | (lhs << 3) | rhs);
    }

    void movq(u8 dstFpu, X64Register src, bool is64Bit = true);

    void movq(X64Register dst, u8 srcFpu, bool is64Bit = true);

    void addsd(u8 dstFpu, u8 srcFpu, bool is64Bit = true);

    void subsd(u8 dstFpu, u8 srcFpu, bool is64Bit = true);

    void mulsd(u8 dstFpu, u8 srcFpu, bool is64Bit = true);

    void divsd(u8 dstFpu, u8 srcFpu, bool is64Bit = true);

    void comisd(u8 lhs, u8 rhs, bool is64Bit = true) {
        // 66 0f 2f c0             comisd xmm0,xmm0
        if (is64Bit) pushBack(0x66);
        pushBack(0x0F);
        pushBack(0x2F);
        pushBack(0xC0 | (lhs << 3) | rhs);
    }

    void ucomisd(u8 lhs, u8 rhs, bool is64Bit = true) {
        if (is64Bit) pushBack(0x66);
        pushBack(0x0F);
        pushBack(0x2E);
        pushBack(0xC0 | (lhs << 3) | rhs);
    }

    void cvttsd2si(X64Register dst, u8 srcFpu) {
        // f2 48 0f 2c c0          cvttsd2si rax,xmm0
        // f2 4c 0f 2c c0          cvttsd2si r8,xmm0
        pushBack(0xF2);
        pushBack(0x48 | (dst.isExt() << 2));
        pushBack(0x0F);
        pushBack(0x2C);
        pushBack(0xC0 | (dst.getEncoding() << 3) | srcFpu);
    }

    void cvtsi2sd(u8 dstFpu, X64Register src) {
        // f2 48 0f 2a c0          cvtsi2sd xmm0,rax
        // f2 49 0f 2a c0          cvtsi2sd xmm0,r8
        pushBack(0xF2);
        pushBack(0x48 | src.isExt());
        pushBack(0x0F);
        pushBack(0x2A);
        pushBack(0xC0 | (dstFpu << 3) | src.getEncoding());
    }

    void* getPtr(ImmSpace spejc) {
        return this->bytes.data()+spejc.offset;
    }

    void readMem(X64Register dst, X64Register src, int32_t offset, size_t size) {
        if (size == 8) {
            readPtr(dst, src, offset);
        } else if (size == 4) {
            read4(dst, src, offset);
        } else if (size == 2) {
            read2(dst, src, offset);
        } else if (size == 1) {
            read1(dst, src, offset);
        }
        else {
            PANIC("invalid input");
        }
    }

    void writeMem(X64Register dst, X64Register src, int32_t offset, size_t size) {
        if (size == 8) {
            writePtr(dst, src, offset);
        } else if (size == 4) {
            write4(dst, src, offset);
        } else if (size == 2) {
            write2(dst, src, offset);
        } else if (size == 1) {
            write1(dst, src, offset);
        }
        else {
            PANIC("invalid input");
        }
    }

    void invokeScuffedSYSV(Arg func, span<Arg> args, optional<Arg> ret, const map<X64Register, size_t>& saved, std::function<void(X64Register, Arg, ImmSpace)> movLabel) {
        /// 6GP registers used for passing arguments
        const std::array<X64Register, 6> SYSV_REGS{X64Register::Rdi, X64Register::Rsi, X64Register::Rdx, X64Register::Rcx, X64Register::R8, X64Register::R9};
        /// temporaries for local use
        const auto TMP_REG = X64Register::R10;
        const auto TMP_REG2 = X64Register::Rax;
        /// 2GP registers that are used to return values
        const auto RET_REG = X64Register::Rax;
        const auto RET_REG2 = X64Register::Rdx;

        /// list of registers that cant be accesed, bcs they are regs or temporaries for local use
        /// the stack preserved version should be used
        set<X64Register> CLOBBER_SET;
        CLOBBER_SET.insert(SYSV_REGS.begin(), SYSV_REGS.end());
        CLOBBER_SET.insert(TMP_REG);
        CLOBBER_SET.insert(TMP_REG2);

        size_t allocatedStack = 0;
        size_t allocatedArgs = 0;

        auto movArgToReg = [&](const X64Register& dst, const X64Register& reg) {
            if (CLOBBER_SET.contains(reg)) {
                assert(saved.contains(reg));
                readStack(saved.at(reg)+allocatedStack, dst);
            } else {
                movReg(dst, reg);
            }
        };

        /// LAMBDA THAT returins register or if its clobbered reads it from stack and returns temporary one
        auto movToReg = [&](const X64Register& reg) {
            if (CLOBBER_SET.contains(reg)) {
                assert(saved.contains(reg));
                readStack(saved.at(reg)+allocatedStack, TMP_REG);
                return TMP_REG;
            }
            return reg;
        };

        /// if return type is larger than 16, caller passes buffer in rdi
        if (ret.has_value() && ret->sizeBytes() > 16) {
            /// I can just take the address of th reg + offset size change and pass it
            assert(ret->type == Arg::Type::REG_OFFSET_VALUE);
            lea(X64Register::Rdi, ret->reg, ret->offset);
            allocatedArgs += 1;
        }

        /// values that are passed over stack
        vector<Arg> toBeStackPassed;

        const auto getAllocatedReg = [&](size_t idex) {
          assert(idex < SYSV_REGS.size());

            return SYSV_REGS[idex];
        };

        // try to populate arg regs
        for (const auto& arg : args) {
            // value can fit into single arg reg
            if (arg.sizeBytes() <= 8 && allocatedArgs < SYSV_REGS.size()) {
                auto argReg = getAllocatedReg(allocatedArgs);
                switch (arg.type) {
                    case Arg::REGISTER:
                        movArgToReg(argReg, arg.reg);
                        break;
                    case Arg::IMMEDIATE:
                        movFast(argReg, arg.immValue);
                        break;
                    // case Arg::SYMBOL:
                    case Arg::SYMBOL_RIP_OFF_32:
                        movLabel(argReg, arg, leaRip(argReg, arg.offset));
                        break;
                    case Arg::SYMBOL_RIP_VALUE_32:
                        movLabel(argReg, arg, relativeRead(argReg, arg.offset));
                        break;
                    case Arg::REG_OFFSET:
                        lea(argReg, movToReg(arg.reg), arg.offset);
                        break;
                    case Arg::REG_OFFSET_VALUE:
                        readMem(argReg, movToReg(arg.reg), arg.offset, arg.sizeBytes());
                        break;
                }
                allocatedArgs += 1;
            }
            else if (arg.sizeBytes() <= 16 && allocatedArgs <= SYSV_REGS.size()-2) { // value can fit into 2 arg regs
                auto argRegA = getAllocatedReg(allocatedArgs);
                auto argRegB = getAllocatedReg(allocatedArgs+1);
                switch (arg.type) {
                    case Arg::REGISTER:
                    case Arg::IMMEDIATE:
                    // case Arg::SYMBOL:
                    case Arg::REG_OFFSET:
                    case Arg::SYMBOL_RIP_OFF_32:
                    case Arg::SYMBOL_RIP_VALUE_32:
                        PANIC("these values are only 8B");
                    case Arg::REG_OFFSET_VALUE: {
                        auto srcReg = movToReg(arg.reg);
                        readMem(argRegA, srcReg, arg.offset, 8);
                        readMem(argRegB, srcReg, arg.offset+8, arg.sizeBytes()-8);
                        break;
                    }
                }
                allocatedArgs += 2;
            }
            else { // value cant fit into arg regs
                toBeStackPassed.push_back(arg);
            }
        }

        /// DO WE NEED TO PASS ANYTHING BY STACK?
        if (not toBeStackPassed.empty()) {
            /// calculate required stack size
            allocatedStack = 0;
            for (auto arg : toBeStackPassed) {
                allocatedStack += align(arg.sizeBytes(), 8);
            }

            /// ALIGN STACK TO 16B
            allocatedStack = align(allocatedStack, 16);

            /// allocate stack for args
            subImm32(X64Register::Rsp, allocatedStack);

            size_t currentOffset = 0;
            // NOTE: arguments are pushed reversed, but we are iterating normaly bcs we are population stack from lower to upper addresses
            /// move values to stack slots
            for (auto arg : toBeStackPassed) {
                switch (arg.type) {
                    case Arg::REGISTER:
                        // NOTE: we can move the whole 8B register onto stack bcs all args should be extended to 8B
                        // so if we have 4B reg and the othe 4B has junk we can freely move it bcs user shouldnt access them
                        writeStack(currentOffset, movToReg(arg.reg));
                        break;
                    case Arg::IMMEDIATE:
                        movFast(TMP_REG, arg.immValue);
                        writeStack(currentOffset, TMP_REG);
                        break;
                    // case Arg::SYMBOL:
                    case Arg::SYMBOL_RIP_OFF_32:
                        movLabel(TMP_REG, arg, leaRip(TMP_REG, arg.offset));
                        writeStack(currentOffset, TMP_REG);
                        break;
                    case Arg::SYMBOL_RIP_VALUE_32:
                        movLabel(TMP_REG, arg, relativeRead(TMP_REG, arg.offset));
                        writeStack(currentOffset, TMP_REG);
                        break;
                    case Arg::REG_OFFSET: {
                        auto srcReg = movToReg(arg.reg);
                        // NOTE: we need to callculate the original Rsp offset bcs we FUCKED IT UP with the sub Rsp
                        auto srcOffset = (srcReg == X64Register::Rsp) ? arg.offset + allocatedStack : arg.offset;
                        lea(TMP_REG, srcReg, srcOffset); // FIXME do i need TMP_REG2 instead TMP_REG here????? i dont right
                        writeStack(currentOffset, TMP_REG);
                        break;
                    }
                    case Arg::REG_OFFSET_VALUE: {
                        auto srcReg = movToReg(arg.reg);
                        auto srcOffset = (srcReg == X64Register::Rsp) ? arg.offset + allocatedStack : arg.offset;
                        garbageMemCpy(X64Register::Rsp, srcReg, arg.sizeBytes(), TMP_REG2, srcOffset, currentOffset);
                        break;
                    }
                }
                currentOffset += align(arg.sizeBytes(), 8);
            }
        }

        /// move function ptr to CALL_REG
        X64Register CALL_REG = TMP_REG;
        switch (func.type) {
            case Arg::REGISTER:
                CALL_REG = movToReg(func.reg);
                break;
            case Arg::IMMEDIATE:
                movFast(TMP_REG, func.immValue);
                break;
            // case Arg::SYMBOL:
            case Arg::SYMBOL_RIP_OFF_32:
                movLabel(TMP_REG, func, leaRip(TMP_REG, func.offset));
                break;
            case Arg::SYMBOL_RIP_VALUE_32:
                movLabel(TMP_REG, func, relativeRead(TMP_REG, func.offset));
                break;
            case Arg::REG_OFFSET: {
                auto srcReg = movToReg(func.reg);
                auto srcOffset = (srcReg == X64Register::Rsp) ? func.offset + allocatedStack : func.offset;
                lea(TMP_REG, srcReg, srcOffset);
                break;
            }
            case Arg::REG_OFFSET_VALUE: {
                auto srcReg = movToReg(func.reg);
                auto srcOffset = (srcReg == X64Register::Rsp) ? func.offset + allocatedStack : func.offset;
                readPtr(TMP_REG, srcReg, srcOffset);
                break;
            }
        }

        /// invoke function
        call(CALL_REG);

        /// restore stack if any
        if (allocatedStack != 0) {
            addImm32(X64Register::Rsp, allocatedStack);
        }

        /// DONT! compensate for any allocated stack its freed now
        allocatedStack = 0;

        /// move return value if any to desired place
        if (ret.has_value()) {
            switch (ret->type) {
                case Arg::IMMEDIATE:
                // case Arg::SYMBOL:
                case Arg::REG_OFFSET:
                case Arg::SYMBOL_RIP_OFF_32:
                case Arg::SYMBOL_RIP_VALUE_32:
                    PANIC("invalid ret type");
                case Arg::REGISTER:
                    movReg(ret->reg, RET_REG);
                    break;
                case Arg::REG_OFFSET_VALUE:
                    if (ret->size <= 8) {
                        writeMem(movToReg(ret->reg), RET_REG, ret->offset, ret->size);
                    }
                    else if (ret->size <= 16) {
                        writeMem(movToReg(ret->reg), RET_REG, ret->offset, 8);
                        writeMem(movToReg(ret->reg), RET_REG2, ret->offset+8, ret->size-8);
                    }
                    else {
                        // do nothing, we passed ptr in Rdi that will be filled with return value
                    }
                    break;
            }
        }
    }

    ImmSpace cmpImm(const X64Register& subj, i32 imm) {
        if (subj.isExt()) writeRex(false, false, subj.isExt());
        pushBack(0x81);
        writeModMR(subj, X64Register::Seven);
        return this->writeImmValue(imm);
    }

    bool canFitToI8(i32 v) {
        return v <= 127 && v >= -128;
    }

    ImmSpace cmpImm(const X64Register& subj, i32 offset, i32 imm) {
        writeRex(true, false, subj.isExt());
        auto canFit = canFitToI8(imm);
        pushBack(canFit ? 0x83 : 0x81);
        someOffsetStuffForMov(X64Register::Seven, subj, offset);
        return (canFit ? this->writeImmValue((i8)imm) : this->writeImmValue(imm));
    }

    void invokeScuffedSYSV2(Arg func, span<Arg> args, optional<Arg> ret, const map<X64Register, size_t>& saved, std::function<void(X64Register, Arg, ImmSpace)> movLabel) {
        /// 6GP registers used for passing arguments
        const std::array<X64Register, 6> SYSV_REGS{X64Register::Rdi, X64Register::Rsi, X64Register::Rdx, X64Register::Rcx, X64Register::R8, X64Register::R9};
        /// temporaries for local use
        const auto TMP_REG = X64Register::R10;
        const auto TMP_REG2 = X64Register::Rax;
        /// 2GP registers that are used to return values
        const auto RET_REG = X64Register::Rax;
        const auto RET_REG2 = X64Register::Rdx;

        /// list of registers that cant be accesed, bcs they are regs or temporaries for local use
        /// the stack preserved version should be used
        set<X64Register> CLOBBER_SET;

        size_t allocatedStack = 0;
        size_t allocatedArgs = 0;

        auto markClobbered = [&](const X64Register& reg) {
            CLOBBER_SET.insert(reg);
        };

        auto movArgToReg = [&](const X64Register& dst, const X64Register& reg) {
            if (CLOBBER_SET.contains(reg)) {
                assert(saved.contains(reg));
                readStack(saved.at(reg)+allocatedStack, dst);
            } else {
                movReg(dst, reg);
            }

            markClobbered(dst);
        };

        /// LAMBDA THAT returins register or if its clobbered reads it from stack and returns temporary one
        auto movToReg = [&](const X64Register& reg) {
            if (CLOBBER_SET.contains(reg)) {
                assert(saved.contains(reg));
                readStack(saved.at(reg)+allocatedStack, TMP_REG);
                markClobbered(TMP_REG);
                return TMP_REG;
            }
            return reg;
        };

        /// if return type is larger than 16, caller passes buffer in rdi
        if (ret.has_value() && ret->sizeBytes() > 16) {
            /// I can just take the address of th reg + offset size change and pass it
            assert(ret->type == Arg::Type::REG_OFFSET_VALUE);
            lea(X64Register::Rdi, ret->reg, ret->offset);
            markClobbered(X64Register::Rdi);
            allocatedArgs += 1;
        }

        /// values that are passed over stack
        vector<Arg> toBeStackPassed;

        const auto getAllocatedReg = [&](size_t idex) {
          assert(idex < SYSV_REGS.size());

            return SYSV_REGS[idex];
        };

        // try to populate arg regs
        for (const auto& arg : args) {
            // value can fit into single arg reg
            if (arg.sizeBytes() <= 8 && allocatedArgs < SYSV_REGS.size()) {
                auto argReg = getAllocatedReg(allocatedArgs);
                switch (arg.type) {
                    case Arg::REGISTER:
                        movArgToReg(argReg, arg.reg);
                        break;
                    case Arg::IMMEDIATE:
                        movFast(argReg, arg.immValue);
                        break;
                    // case Arg::SYMBOL:
                    case Arg::SYMBOL_RIP_OFF_32:
                        movLabel(argReg, arg, leaRip(argReg, arg.offset));
                        break;
                    case Arg::SYMBOL_RIP_VALUE_32:
                        movLabel(argReg, arg, relativeRead(argReg, arg.offset));
                        break;
                    case Arg::REG_OFFSET:
                        lea(argReg, movToReg(arg.reg), arg.offset);
                        break;
                    case Arg::REG_OFFSET_VALUE:
                        readMem(argReg, movToReg(arg.reg), arg.offset, arg.sizeBytes());
                        break;
                }
                markClobbered(argReg);
                allocatedArgs += 1;
            }
            else if (arg.sizeBytes() <= 16 && allocatedArgs <= SYSV_REGS.size()-2) { // value can fit into 2 arg regs
                auto argRegA = getAllocatedReg(allocatedArgs);
                auto argRegB = getAllocatedReg(allocatedArgs+1);
                switch (arg.type) {
                    case Arg::REGISTER:
                    case Arg::IMMEDIATE:
                    // case Arg::SYMBOL:
                    case Arg::REG_OFFSET:
                    case Arg::SYMBOL_RIP_OFF_32:
                    case Arg::SYMBOL_RIP_VALUE_32:
                        PANIC("these values are only 8B");
                    case Arg::REG_OFFSET_VALUE: {
                        auto srcReg = movToReg(arg.reg);
                        readMem(argRegA, srcReg, arg.offset, 8);
                        markClobbered(argRegA);
                        readMem(argRegB, srcReg, arg.offset+8, arg.sizeBytes()-8);
                        markClobbered(argRegB);
                        break;
                    }
                }
                allocatedArgs += 2;
            }
            else { // value cant fit into arg regs
                toBeStackPassed.push_back(arg);
            }
        }

        /// DO WE NEED TO PASS ANYTHING BY STACK?
        if (not toBeStackPassed.empty()) {
            /// calculate required stack size
            allocatedStack = 0;
            for (auto arg : toBeStackPassed) {
                allocatedStack += align(arg.sizeBytes(), 8);
            }

            /// ALIGN STACK TO 16B
            allocatedStack = align(allocatedStack, 16);

            /// allocate stack for args
            subImm32(X64Register::Rsp, allocatedStack);

            size_t currentOffset = 0;
            // NOTE: arguments are pushed reversed, but we are iterating normaly bcs we are population stack from lower to upper addresses
            /// move values to stack slots
            for (auto arg : toBeStackPassed) {
                switch (arg.type) {
                    case Arg::REGISTER:
                        // NOTE: we can move the whole 8B register onto stack bcs all args should be extended to 8B
                        // so if we have 4B reg and the othe 4B has junk we can freely move it bcs user shouldnt access them
                        writeStack(currentOffset, movToReg(arg.reg));
                        break;
                    case Arg::IMMEDIATE:
                        movFast(TMP_REG, arg.immValue);
                        writeStack(currentOffset, TMP_REG);
                        markClobbered(TMP_REG);
                        break;
                    // case Arg::SYMBOL:
                    case Arg::SYMBOL_RIP_OFF_32:
                        movLabel(TMP_REG, arg, leaRip(TMP_REG, arg.offset));
                        writeStack(currentOffset, TMP_REG);
                        markClobbered(TMP_REG);
                        break;
                    case Arg::SYMBOL_RIP_VALUE_32:
                        movLabel(TMP_REG, arg, relativeRead(TMP_REG, arg.offset));
                        writeStack(currentOffset, TMP_REG);
                        markClobbered(TMP_REG);
                        break;
                    case Arg::REG_OFFSET: {
                        auto srcReg = movToReg(arg.reg);
                        // NOTE: we need to callculate the original Rsp offset bcs we FUCKED IT UP with the sub Rsp
                        auto srcOffset = (srcReg == X64Register::Rsp) ? arg.offset + allocatedStack : arg.offset;
                        lea(TMP_REG, srcReg, srcOffset); // FIXME do i need TMP_REG2 instead TMP_REG here????? i dont right
                        writeStack(currentOffset, TMP_REG);
                        markClobbered(TMP_REG);
                        break;
                    }
                    case Arg::REG_OFFSET_VALUE: {
                        auto srcReg = movToReg(arg.reg);
                        auto srcOffset = (srcReg == X64Register::Rsp) ? arg.offset + allocatedStack : arg.offset;
                        garbageMemCpy(X64Register::Rsp, srcReg, arg.sizeBytes(), TMP_REG2, srcOffset, currentOffset);
                        markClobbered(TMP_REG2);
                        break;
                    }
                }
                currentOffset += align(arg.sizeBytes(), 8);
            }
        }

        /// move function ptr to CALL_REG
        X64Register CALL_REG = TMP_REG;
        switch (func.type) {
            case Arg::REGISTER:
                movReg(TMP_REG, movToReg(func.reg));
                /// invoke function
                call(CALL_REG);
                break;
            case Arg::IMMEDIATE:
                movFast(TMP_REG, func.immValue);
                /// invoke function
                call(CALL_REG);
                break;
            case Arg::SYMBOL_RIP_OFF_32:
                movLabel(TMP_REG, func, leaRip(TMP_REG, func.offset));
                /// invoke function
                call(CALL_REG);
                break;
            case Arg::SYMBOL_RIP_VALUE_32:
                movLabel(TMP_REG, func, relativeRead(TMP_REG, func.offset));
                /// invoke function
                call(CALL_REG);
                break;
            case Arg::REG_OFFSET: {
                auto srcReg = movToReg(func.reg);
                auto srcOffset = (srcReg == X64Register::Rsp) ? func.offset + allocatedStack : func.offset;
                lea(TMP_REG, srcReg, srcOffset);
                /// invoke function
                call(CALL_REG);
                break;
            }
            case Arg::REG_OFFSET_VALUE: {
                auto srcReg = movToReg(func.reg);
                auto srcOffset = (srcReg == X64Register::Rsp) ? func.offset + allocatedStack : func.offset;
                // readPtr(TMP_REG, srcReg, srcOffset);
                /// invoke function
                call(srcReg, srcOffset);
                break;
            }
        }

        /// restore stack if any
        if (allocatedStack != 0) {
            addImm32(X64Register::Rsp, allocatedStack);
        }

        /// DONT! compensate for any allocated stack its freed now
        allocatedStack = 0;

        /// move return value if any to desired place
        if (ret.has_value()) {
            switch (ret->type) {
                case Arg::IMMEDIATE:
                    // case Arg::SYMBOL:
                case Arg::REG_OFFSET:
                case Arg::SYMBOL_RIP_OFF_32:
                case Arg::SYMBOL_RIP_VALUE_32:
                    PANIC("invalid ret type");
                case Arg::REGISTER:
                    movReg(ret->reg, X64Register::Rax);
                    break;
                case Arg::REG_OFFSET_VALUE:
                    if (ret->size <= 8) {
                        writeMem(movToReg(ret->reg), RET_REG, ret->offset, ret->size);
                    }
                    else if (ret->size <= 16) {
                        writeMem(movToReg(ret->reg), RET_REG, ret->offset, 8);
                        writeMem(movToReg(ret->reg), RET_REG2, ret->offset+8, ret->size-8);
                    }
                    else {
                        // do nothing, we passed ptr in Rdi that will be filled with return value
                    }
                    break;
            }
        }
    }

    // NOTE: we can theorteicaly make `allocateStack` optional dependency, allocate our stack as fallback
    void invokeScuffedFastCall(Arg func, span<Arg> args, optional<Arg> ret, const map<X64Register, size_t>& saved, const std::function<void(X64Register, Arg, ImmSpace)>& movLabel, const std::function<size_t(size_t)>& allocateStack) {
        /// 6GP registers used for passing arguments
        const array<X64Register, 4> ARG_REGS{X64Register::Rcx, X64Register::Rdx, X64Register::R8, X64Register::R9};
        const size_t STACK_BYTES_RESERVED = 8 * ARG_REGS.size();
        const X64Register TMP_REG = X64Register::R10;
        const X64Register TMP_REG2 = X64Register::Rax;
        const X64Register RET_REG = X64Register::Rax;
        set<X64Register> CLOBBER_SET;
        CLOBBER_SET.insert(ARG_REGS.begin(), ARG_REGS.end());
        CLOBBER_SET.insert(TMP_REG);
        CLOBBER_SET.insert(TMP_REG2);

        size_t allocatedStack = 0;
        size_t argCounter = 0;

        vector<X64Register> ignoredRegs;
        if (ret.has_value() && ret->type == Arg::REGISTER) {
            ignoredRegs.push_back(ret->reg);
        }

        /// LAMBDA THAT returins register or if its clobbered reads it from stack and returns temporary one
        auto movToReg = [&](const X64Register& reg) {
            if (CLOBBER_SET.contains(reg)) {
                assert(saved.contains(reg));
                readStack(saved.at(reg)+allocatedStack, TMP_REG);
                return TMP_REG;
            }
            return reg;
        };

        /// if return type is larger than 16, caller passes buffer in rdi
        if (ret.has_value() && ret->sizeBytes() > 8) {
            /// I can just take the address of th reg + offset size change and pass it
            assert(ret->type == Arg::Type::REG_OFFSET_VALUE);
            lea(X64Register::Rdi, ret->reg, ret->offset);
            argCounter += 1;
        }

        vector<Arg> toBeStackPassed;

        for (auto [argId, arg] : args | views::enumerate) {
            if (arg.sizeBytes() <= 8 && argCounter < ARG_REGS.size()) {
                auto dstReg = ARG_REGS[argCounter];
                switch (arg.type) {
                    case Arg::REGISTER:
                        movReg(dstReg, movToReg(arg.reg));
                        break;
                    case Arg::IMMEDIATE:
                        movFast(dstReg, arg.immValue);
                        break;
                    // case Arg::SYMBOL:
                    case Arg::SYMBOL_RIP_OFF_32:
                        movLabel(dstReg, arg, leaRip(dstReg, arg.offset));
                        break;
                    case Arg::SYMBOL_RIP_VALUE_32:
                        movLabel(dstReg, arg, relativeRead(dstReg, arg.offset));
                        break;
                    case Arg::REG_OFFSET:
                        lea(dstReg, movToReg(arg.reg), arg.offset);
                        break;
                    case Arg::REG_OFFSET_VALUE:
                        readMem(dstReg, movToReg(arg.reg), arg.offset, arg.sizeBytes());
                        break;
                }
                argCounter += 1;
            }
            else if (argCounter < ARG_REGS.size()) {
                println("asdadasdas {}", (size_t)arg.type);
                assert(arg.type == Arg::REG_OFFSET_VALUE);

                auto dstReg = ARG_REGS[argCounter];
                auto size = arg.sizeBytes();

                auto stackOffset = allocateStack(size);

                garbageMemCpy(X64Register::Rsp, movToReg(arg.reg), size, TMP_REG2, arg.offset, stackOffset);

                lea(dstReg, X64Register::Rsp, stackOffset);

                argCounter += 1;
            }
            else {
                // FIXME if argId < 4 is passed over stack we use its special lil ABI spot :3
                // TODO we need to pass the index down the pipeline and work with it
                assert(argId >= 4);
                toBeStackPassed.push_back(arg);
            }
        }

        /// calcluate required stack
        for (auto arg : toBeStackPassed) {
            allocatedStack += align(arg.sizeBytes(), 8);
        }
        allocatedStack += STACK_BYTES_RESERVED;
        /// align stack to 16 B
        allocatedStack = align(allocatedStack, 16);

        /// allocate stack
        subImm32(X64Register::Rsp, allocatedStack);

        size_t currentOffset = STACK_BYTES_RESERVED;
        // NOTE: arguments are pushed reversed, but we are iterating normaly bcs we are population stack from lower to upper addresses
        /// move values to stack slots
        for (auto arg : toBeStackPassed) {
            switch (arg.type) {
                case Arg::REGISTER:
                    // NOTE: we can move the whole 8B register onto stack bcs all args should be extended to 8B
                    // so if we have 4B reg and the othe 4B has junk we can freely move it bcs user shouldnt access them
                    writeStack(currentOffset, movToReg(arg.reg));
                    break;
                case Arg::IMMEDIATE:
                    movFast(TMP_REG, arg.immValue);
                    writeStack(currentOffset, TMP_REG);
                    break;
                // case Arg::SYMBOL:
                case Arg::SYMBOL_RIP_OFF_32:
                    movLabel(TMP_REG, arg, leaRip(TMP_REG, arg.offset));
                    writeStack(currentOffset, TMP_REG);
                    break;
                case Arg::SYMBOL_RIP_VALUE_32:
                    movLabel(TMP_REG, arg, relativeRead(TMP_REG, arg.offset));
                    writeStack(currentOffset, TMP_REG);
                    break;
                case Arg::REG_OFFSET: {
                    auto srcReg = movToReg(arg.reg);
                    // NOTE: we need to callculate the original Rsp offset bcs we FUCKED IT UP with the sub Rsp
                    auto srcOffset = (srcReg == X64Register::Rsp) ? arg.offset + allocatedStack : arg.offset;
                    lea(TMP_REG, srcReg, srcOffset); // FIXME do i need TMP_REG2 instead TMP_REG here????? i dont right
                    writeStack(currentOffset, TMP_REG);
                    break;
                }
                case Arg::REG_OFFSET_VALUE: {
                    auto srcReg = movToReg(arg.reg);
                    auto srcOffset = (srcReg == X64Register::Rsp) ? arg.offset + allocatedStack : arg.offset;
                    garbageMemCpy(X64Register::Rsp, srcReg, arg.sizeBytes(), TMP_REG2, srcOffset, currentOffset);
                    break;
                }
            }
            currentOffset += align(arg.sizeBytes(), 8);
        }

        /// move function ptr to TMP_REG
        X64Register CALL_REG = TMP_REG;
        switch (func.type) {
            case Arg::REGISTER:
                CALL_REG = movToReg(func.reg);
                break;
            case Arg::IMMEDIATE:
                movFast(TMP_REG, func.immValue);
                break;
            // case Arg::SYMBOL:
            case Arg::SYMBOL_RIP_OFF_32:
                movLabel(TMP_REG, func, leaRip(TMP_REG, func.offset));
                writeStack(currentOffset, TMP_REG);
                break;
            case Arg::SYMBOL_RIP_VALUE_32:
                movLabel(TMP_REG, func, relativeRead(TMP_REG, func.offset));
                writeStack(currentOffset, TMP_REG);
                break;
            case Arg::REG_OFFSET: {
                auto srcReg = movToReg(func.reg);
                auto srcOffset = (srcReg == X64Register::Rsp) ? func.offset + allocatedStack : func.offset;
                lea(TMP_REG, srcReg, srcOffset);
                break;
            }
            case Arg::REG_OFFSET_VALUE: {
                auto srcReg = movToReg(func.reg);
                auto srcOffset = (srcReg == X64Register::Rsp) ? func.offset + allocatedStack : func.offset;
                readPtr(TMP_REG, srcReg, srcOffset);
                break;
            }
        }

        /// invoke function
        call(TMP_REG);

        /// restore stack
        if (allocatedStack != 0) {
            addImm32(X64Register::Rsp, allocatedStack);
            allocatedStack = 0;
        }

        /// move return value if any to desired place
        if (ret.has_value()) {
            switch (ret->type) {
                case Arg::IMMEDIATE:
                // case Arg::SYMBOL:
                case Arg::REG_OFFSET:
                case Arg::SYMBOL_RIP_OFF_32:
                case Arg::SYMBOL_RIP_VALUE_32:
                    PANIC("invalid ret type");
                case Arg::REGISTER:
                    movReg(ret->reg, RET_REG);
                    break;
                case Arg::REG_OFFSET_VALUE:
                    if (ret->size <= 8) {
                        writeMem(movToReg(ret->reg), RET_REG, ret->offset, ret->size);
                    }
                    else {
                        // do nothing, we passed ptr in Rdi that will be filled with return value
                    }
                    break;
            }
        }
    }

    void generateNativeCallWrapper(span<size_t> argSizes1, size_t retSize1, Arg idk, std::function<void(X64Register, Arg, ImmSpace)> movLabel) {
#ifdef __unix__
        return generateCallWrapperSYSV(argSizes1, retSize1, idk, movLabel);
#else
        return generateCallWrapperFast(argSizes1, retSize1, idk, movLabel);
#endif
    }

    void generateCallWrapperSYSV(span<size_t> argSizes1, size_t retSize1, Arg idk, std::function<void(X64Register, Arg, ImmSpace)> movLabel) {
        // Rdi - args, Rsi - ret, Rcx - runtime
        nop();
        nop();
        nop();
        push(X64Register::Rbp);
        movReg(X64Register::Rbp, X64Register::Rsp);

        push(X64Register::Rdi);
        push(X64Register::Rsi);

        map<X64Register, size_t> saved;
        saved.emplace(X64Register::Rdi, 8);
        saved.emplace(X64Register::Rsi, 0);

        vector<Arg> argz;
        size_t currentOffset = 0;
        for (auto size : argSizes1) {
            argz.push_back(Arg::MemoryValue(X64Register::Rdi, currentOffset, align(size, 8)));
            currentOffset += size;
        }

        optional<Arg> ret;
        if (retSize1 != 0) {
            ret = Arg::MemoryValue(X64Register::Rsi, 0, retSize1);
        }

        invokeScuffedSYSV(idk, argz, ret, saved, movLabel);

        leave();
        this->ret();
    }

    void generateCallWrapperFast(span<size_t> argSizes1, size_t retSize1, Arg idk, std::function<void(X64Register, Arg, ImmSpace)> movLabel) {
        nop();
        nop();
        nop();

        frameInit();

        push(X64Register::Rcx);
        push(X64Register::Rdx);

        map<X64Register, size_t> saved;
        saved.emplace(X64Register::Rcx, 8);
        saved.emplace(X64Register::Rdx, 0);

        vector<Arg> argz;
        size_t currentOffset = 0;
        for (auto size : argSizes1) {
            argz.push_back(Arg::MemoryValue(X64Register::Rcx, currentOffset, size));
            currentOffset += size;
        }

        optional<Arg> ret;
        if (retSize1 != 0) {
            ret = Arg::MemoryValue(X64Register::Rdx, 0, retSize1);
        }

        auto imm = subImm32(X64Register::Rsp, 0);
        int32_t allocatedStack = 0;

        invokeScuffedFastCall(idk, argz, ret, saved, movLabel, [&](auto amount) {
            auto a = amount;
            allocatedStack += amount;
            return a;
        });

        auto res = (int32_t*)getPtr(imm);
        *res = allocatedStack;

        leave();
        this->ret();
    }

    void push(const X64Register& obj, i32 offset) {
        if (obj.isExt()) writeRex(false, false, true);

        pushBack(0xFF);

        someOffsetStuffForMov(X64Register::Six, obj, offset);
    }

    void pop(const X64Register& obj, i32 offset) {
        if (obj.isExt()) writeRex(false, false, true);

        pushBack(0x8F);

        someOffsetStuffForMov(X64Register::Zero, obj, offset);
    }

    void frameInit() {
        push(X64Register::Rbp);
        movReg(X64Register::Rbp, X64Register::Rsp);
    }

    void hlt() {
        pushBack(0xF4);
    }
};