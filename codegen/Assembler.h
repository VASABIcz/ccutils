#pragma once
#include <utility>
#include <vector>
#include <string>
#include <memory>

#include "BinaryOutput.h"

using namespace std;

#define WRAPPED(t) { arithmeticInt(ArithmeticOp::t, dest, left, right); }
#define WRAPPED_FLOAT(t) { arithmeticFloat(ArithmeticOp::t, FloatingPointType::Float, dest, left, right); }
#define WRAPPED_DOUBLE(t) { arithmeticFloat(ArithmeticOp::t, FloatingPointType::Double, dest, left, right); }

enum class JumpCondType {
    EQUALS,
    NOT_EQUALS,
    GREATER,
    GREATER_OR_EQUAL,
    LESS,
    LESS_OR_EQUAL
};

inline string_view toString1(JumpCondType type) {
    switch (type) {
        case JumpCondType::EQUALS: return "==";
        case JumpCondType::NOT_EQUALS: return "!=";
        case JumpCondType::GREATER: return ">";
        case JumpCondType::GREATER_OR_EQUAL: return ">=";
        case JumpCondType::LESS: return "<";
        case JumpCondType::LESS_OR_EQUAL: return "<=";
        default: PANIC();
    }
}

inline JumpCondType negateType(JumpCondType type) {
    switch (type) {
        case JumpCondType::EQUALS: return JumpCondType::NOT_EQUALS;
        case JumpCondType::NOT_EQUALS: return JumpCondType::EQUALS;
        case JumpCondType::GREATER: return JumpCondType::LESS_OR_EQUAL;
        case JumpCondType::GREATER_OR_EQUAL: return JumpCondType::LESS;
        case JumpCondType::LESS: return JumpCondType::GREATER_OR_EQUAL;
        case JumpCondType::LESS_OR_EQUAL: return JumpCondType::GREATER;
        default:PANIC();
    }
}

class Assembler {
public:
    enum class ArithmeticOp {
        ADD,
        SUB,
        MUL,
        DIV,
        MOD,
        SHR,
        SHL,
        OR,
        AND,
        EQ,
        GT,
        LS,
        XOR,
        NEGATE,
        GE,
        LE,
        NEQ
    };

    enum class FloatingPointType {
        Float,
        Double
    };

    void derefChainI(size_t reg, std::initializer_list<int> offsets) {
        derefChain(reg, offsets);
    }

    virtual std::string toString(size_t hand) {
        return "NO-STUFF";
    }

    // 0 -> reg+0
    // 8 -> reg+8
    // 0 0 -> [reg+0]+0
    // 8 8 -> [reg+8]+8
    // 8 8 8 -> [[reg+8]+8]+8
    void derefChain(size_t reg, span<const int> offsets) {
        if (offsets.empty()) return;

        for (size_t i = 0; i < offsets.size() - 1; i++) {
            readMem(reg, reg, offsets[i], 8);
        }

        if (offsets.back() != 0) {
            auto v = movImmValueToReg(offsets.back());
            addInt(reg, reg, v);
            freeRegister(v);
        }
    }

    virtual ~Assembler() = default;

    typedef size_t RegisterHandle;

    virtual vector<RegisterHandle> getArgHandles() = 0;

    // rets
    virtual void generateRet() = 0;
    virtual void generateRet(RegisterHandle value) = 0;

    // value movement
    virtual void movReg(RegisterHandle dest, RegisterHandle src, size_t destOffset = 0, size_t srcOffset=0, size_t amount=8) = 0;
    // FIXME multiple width versions, eg> mov8, mov16, mov32, mov64, movPtr
    virtual void movInt(RegisterHandle dest, u64 value, bool isSigned, size_t offsetBytes = 0) = 0;
    virtual void movSigned(RegisterHandle dest, u64 value, size_t offsetBytes = 0) {
        movInt(dest, value, true, offsetBytes);
    }
    virtual void movUnsigned(RegisterHandle dest, u64 value, size_t offsetBytes = 0) {
        movInt(dest, value, false, offsetBytes);
    }
    virtual void movPtr(RegisterHandle dest, void* value, size_t offsetBytes = 0) {
        movUnsigned(dest, std::bit_cast<uintptr_t>(value), offsetBytes);
    }
    virtual void writeMem(RegisterHandle obj, RegisterHandle value, size_t byteOffset, size_t amount = 8) = 0;
    virtual void readMem(RegisterHandle tgt, RegisterHandle obj, size_t byteOffset, size_t amount = 8) = 0;
    virtual void addressOf(RegisterHandle tgt, RegisterHandle obj) = 0;
    virtual void signExtend(RegisterHandle dst, RegisterHandle src) = 0;

    virtual void doStuff() {
        // TODO();
    }

    // int ops
    virtual void arithmeticInt(ArithmeticOp op, RegisterHandle tgt, RegisterHandle lhs, RegisterHandle rhs) {};
    virtual void subInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(SUB);
    virtual void addInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(ADD);
    virtual void mulInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(MUL);
    virtual void divInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(DIV);
    virtual void eqInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(EQ);
    virtual void neqInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(NEQ);
    virtual void gtInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(GT);
    virtual void lessInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(LS);
    virtual void negateInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(NEGATE);
    virtual void xorInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(XOR);
    virtual void andInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(AND);
    virtual void orInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(OR);
    virtual void shlInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(SHL);
    virtual void shrInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(SHR);
    virtual void modInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(MOD);
    virtual void geInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(GE);
    virtual void leInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED(LE);

    void notBits(RegisterHandle dst, RegisterHandle src) {
        TODO();
    }

    // float ops
    virtual void arithmeticFloat(ArithmeticOp op, FloatingPointType type, RegisterHandle tgt, RegisterHandle lhs, RegisterHandle rhs) {};
    virtual void subFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(SUB);
    virtual void addFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(ADD);
    virtual void mulFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(MUL);
    virtual void divFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(DIV);
    virtual void eqFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(EQ);
    virtual void gtFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(GT);
    virtual void lessFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(LS);
    virtual void leFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(LE);
    virtual void geFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(GE);
    virtual void neqFloat(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_FLOAT(NEQ);

    virtual void subDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(SUB);
    virtual void addDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(ADD);
    virtual void mulDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(MUL);
    virtual void divDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(DIV);
    virtual void eqDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(EQ);
    virtual void gtDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(GT);
    virtual void lessDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(LS);
    virtual void leDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(LE);
    virtual void geDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(GE);
    virtual void neqDouble(RegisterHandle dest, RegisterHandle left, RegisterHandle right) WRAPPED_DOUBLE(NEQ);

    // type conversion
    virtual void int2f64(RegisterHandle dest, RegisterHandle value) = 0;
    virtual void f64ToInt(RegisterHandle dest, RegisterHandle value) = 0;
    virtual void f32ToF64(RegisterHandle dest, RegisterHandle value) = 0;
    virtual void f64ToF32(RegisterHandle dest, RegisterHandle value) = 0;

    size_t jmpLabel = 0;

    // control flow
    virtual size_t allocateJmpLabel() {
        return jmpLabel++;
    }
    virtual void jmpLabelTrue(RegisterHandle cond, size_t label) = 0;
    virtual void jmpLabelFalse(RegisterHandle cond, size_t label) = 0;
    virtual void jmp(size_t label) = 0;
    virtual void jmpCond(size_t label, JumpCondType type, RegisterHandle lhs, RegisterHandle rhs) {
        auto reg = allocateRegister(8);
        arithmeticInt(jmpToArithmetic(type), reg, lhs, rhs);
        jmpLabelTrue(reg, label);
        freeRegister(reg);
    }
    virtual void createLabel(size_t name) = 0;

    virtual void bindHint(std::string_view s) {}

    virtual void dumpHints(std::string_view s) {}

    ArithmeticOp jmpToArithmetic(JumpCondType type) {
        switch (type) {
            case JumpCondType::EQUALS: return ArithmeticOp::EQ;
            case JumpCondType::NOT_EQUALS: return ArithmeticOp::NEQ;
            case JumpCondType::GREATER: return ArithmeticOp::GT;
            case JumpCondType::GREATER_OR_EQUAL: return ArithmeticOp::GE;
            case JumpCondType::LESS: return ArithmeticOp::LS;
            case JumpCondType::LESS_OR_EQUAL: return ArithmeticOp::LE;
        }
        UNREACHABLE();
    }

    // function calls

    size_t movImmValueToReg(size_t value) {
        auto reg = allocateRegister(8);
        movUnsigned(reg, value);
        return reg;
    }

    size_t movImmPtrToReg(void* value) {
        return movImmValueToReg(bit_cast<i64>(value));
    }

    virtual RegisterHandle allocateRegister(RegisterHandle size) = 0;
    virtual RegisterHandle allocateStack(RegisterHandle size) = 0;
    virtual void freeRegister(RegisterHandle handle) = 0;
    virtual void resurect(RegisterHandle handle) = 0;

    virtual size_t numRegs() const = 0;

    virtual void print() const = 0;
    virtual void instructionNumberHint(size_t id) {}
    virtual void nop() = 0;
    virtual void trap() {
    }
};