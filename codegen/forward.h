#pragma once

class SSARegisterHandle;

class Assembler;

template<typename CTX>
class CodeGen;

template<typename CTX>
class IRGen;

template<typename CTX>
class CodeBlock;

namespace instructions {
    template<typename CTX>
    struct PhiFunction;
    template<typename CTX>
    class NoOp;
    template<typename CTX>
    struct Branch;
    template<typename CTX>
    struct JumpTrue;
    template<typename CTX>
    struct JumpFalse;
    template<typename CTX>
    struct Jump;
    template<typename CTX>
    struct FallTrough;
    template<typename CTX>
    class Assign;
    template<typename CTX>
    struct AddressOf;
    template<typename CTX>
    struct IntEquals;
    template<typename CTX>
    struct IntGt;
    template<typename CTX>
    struct IntGe;
    template<typename CTX>
    struct IntSub;
    template<typename CTX>
    struct IntLe;
    template<typename CTX>
    struct IntLess;
    template<typename CTX>
    struct BranchCond;
    template<typename CTX>
    struct JumpCond;
    template<typename CTX>
    struct BoolNot;
    template<typename CTX>
    struct BinaryInstruction;
    template<typename CTX>
    struct Alloca;
}