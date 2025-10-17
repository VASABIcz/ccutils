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
    struct BranchCond;
    template<typename CTX>
    struct JumpCond;
    template<typename CTX>
    struct BoolNot;
    template<typename CTX>
    struct BinaryInstruction;
    template<typename CTX>
    struct Alloca;
    template<typename CTX>
    struct AllocaPtr;
    template<typename CTX>
    struct Arg;
    template<typename CTX>
    struct PointerStore;
    template<typename CTX>
    struct PointerLoad;
    template<typename CTX>
    struct IntLiteral;
    template<typename CTX>
    struct Return;
    template<typename CTX>
    struct VoidReturn;
}