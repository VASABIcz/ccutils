#pragma once
#include <utility>
#include <vector>
#include <string>

#include "CodeBlock.h"
#include "SSARegisterHandle.h"
#include "CodeGen.h"
#include "Assembler.h"

using namespace std;

namespace instructions {
    template<typename CTX>
    struct BinaryInstruction: public IR2Instruction2<CTX> {
        PUB_VIRTUAL_COPY(BinaryInstruction)
        Assembler::BinaryOp op;
        Assembler::BaseDataType type;

        BinaryInstruction(
                SSARegisterHandle tgt,
                SSARegisterHandle lhs,
                SSARegisterHandle rhs,
                Assembler::BinaryOp op,
                Assembler::BaseDataType type
        ): IR2Instruction2<CTX>(tgt, lhs, rhs, stringify("{}_{}", Assembler::baseTypeToString(type), Assembler::binaryOpToString(op))), op(op), type(type) {}

        void generate(CTX::GEN& ctx) override {
            ctx.assembler.binaryInst(op, ctx.getReg(this->target), ctx.getReg(this->lhs), ctx.getReg(this->rhs), type);
        }
    };

    template<typename CTX>
    class NoOp: public IR0Instruction<"noop", CTX> {
        PUB_VIRTUAL_COPY(NoOp)
        using IR0Instruction<"noop", CTX>::IR0Instruction;

        void generate(CTX::GEN &gen) override {}
    };

    template<typename CTX>
    struct FallTrough: public IRBaseInstruction<size_t, "fall_trough", CTX> {
        PUB_VIRTUAL_COPY(FallTrough)
        FallTrough(size_t value): IRBaseInstruction<size_t, "fall_trough", CTX>(SSARegisterHandle::invalid(), value) {}

        void generate(CTX::GEN& ctx) override {
            ctx.assignPhis(this->value);
        }

        [[nodiscard]] vector<size_t> branchTargets() const override {return {this->value};}

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct BoolNot: public IR1Instruction<"bool_not", CTX> {
    PUB_VIRTUAL_COPY(BoolNot)
        using IR1Instruction<"bool_not", CTX>::IR1Instruction;

        void generate(CTX::GEN &gen) override {
            auto temp = gen.allocateTemp(1);
            auto tgt = gen.getReg(this->target);
            auto val1 = gen.getReg(this->value);
            gen.assembler.movUnsigned(temp, 1);
            gen.assembler.xorInt(tgt, val1, temp);
            gen.freeTemp(temp);
        }
    };

    template<typename CTX>
    struct I2F: IR1Instruction<"i64ToF64", CTX> {
    PUB_VIRTUAL_COPY(I2F)
        using IR1Instruction<"i64ToF64", CTX>::IR1Instruction;

        void generate(CTX::GEN&gen) override {
            gen.assembler.int2f64(gen.getReg(this->target), gen.getReg(this->value));
        }
    };

    template<typename CTX>
    struct F2I: IR1Instruction<"f64ToI64", CTX> {
    PUB_VIRTUAL_COPY(F2I)
        using IR1Instruction<"f64ToI64", CTX>::IR1Instruction;

        void generate(CTX::GEN& gen) override {
            gen.assembler.f64ToInt(gen.getReg(this->target), gen.getReg(this->value));
        }
    };

    template<typename CTX>
    struct PhiFunction: public NamedIrInstruction<"phi", CTX> {
    PUB_VIRTUAL_COPY(PhiFunction)
        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", stringify(versions, {{", ", "", ""}}));
        }

        void visitSrc(function<void (SSARegisterHandle &)> fn) override {
            for (auto& [block, handle]: versions) {
                fn(handle);
            }
        }

        PhiFunction(SSARegisterHandle target, map<BlockId, SSARegisterHandle> versions) : NamedIrInstruction<"phi", CTX>(target),
                                                                                         versions(std::move(versions)) {}

        PhiFunction(SSARegisterHandle target, span<pair<SSARegisterHandle, BlockId>> versions) : NamedIrInstruction<"phi", CTX>(target) {
            for (auto version: versions) {
                this->versions.emplace(version.second, version.first);
            }
        }

        void generate(CTX::GEN&) override {
            // we consistently have one more use count than we need
            // gen.getReg(target);
            // this should be nop
        }

        void pushVersion(SSARegisterHandle handle, BlockId block) {
            versions[block] = handle;
        }

        size_t versionsCount() const {
            return versions.size();
        }

        [[nodiscard]] set<SSARegisterHandle> getVersions() const {
            set<SSARegisterHandle> buf;

            for (auto [block, handle]: versions) {
                buf.insert(handle);
            }

            return buf;
        }

        void remove(SSARegisterHandle reg) {
            for(auto it = versions.begin(); it != versions.end(); it++) {
                if(it->second == reg) {
                    versions.erase(it);
                    break;
                }
            }
        }

        void remove(BlockId block) {
            versions.erase(block);
        }

        [[nodiscard]] vector<SSARegisterHandle> getAllVersions() const {
            vector<SSARegisterHandle> buf;

            for (auto [block, handle]: versions) {
                buf.push_back(handle);
            }

            return buf;
        }

        map<BlockId, SSARegisterHandle> getRawVersions() const {
            return versions;
        }

        optional<SSARegisterHandle> getBySource(BlockId src) {
            if (not versions.contains(src)) return {};

            return versions[src];
        }

    private:
        map<BlockId, SSARegisterHandle> versions;
    };

    template<typename CTX>
    class Assign: public IR1Instruction<"assign", CTX> {
    PUB_VIRTUAL_COPY(Assign)
        using IR1Instruction<"assign", CTX>::IR1Instruction;

        void generate(CTX::GEN& gen) override {
            auto destHandle = gen.getReg(this->target);
            auto valueHandle = gen.getReg(this->value);

            gen.assembler.movReg(destHandle, valueHandle, 0, 0, gen.sizeBytes(this->target));
        }
    };

    template<typename CTX>
    struct Branch: public NamedIrInstruction<"branch", CTX> {
    PUB_VIRTUAL_COPY(Branch)
        SSARegisterHandle condition;
        BlockId scopeT;
        BlockId scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(condition);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}, {}, {}", condition.toString(), scopeT, scopeF);
        }

        Branch(SSARegisterHandle condition, BlockId t, BlockId f): NamedIrInstruction<"branch", CTX>(SSARegisterHandle::invalid()), condition(condition), scopeT(t), scopeF(f) {

        };

        void generate(CTX::GEN &ctx) override {
            auto condReg = ctx.getReg(condition);
            auto label = ctx.nextLabel();

            // TODO fast path to prevent unnecessary jump

            // branch to phi handling
            ctx.assembler.jmpLabelTrue(condReg, label);

            // false block PHI handling
            ctx.assignPhis(scopeF);

            // jump to FALSE block
            ctx.jmpBlock(scopeF);

            // TRUE block phi handling
            ctx.assembler.createLabel(label);
            ctx.assignPhis(scopeT);

            // jmp to TRUE block
            ctx.jmpBlock(scopeT);
        }

        [[nodiscard]] vector<BlockId*> branchTargetsPtr() const override {
            return {(BlockId*)&scopeT, (BlockId*)&scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct BranchCond: public NamedIrInstruction<"branch_cond", CTX> {
        PUB_VIRTUAL_COPY(BranchCond)
        JumpCondType type;
        SSARegisterHandle lhs;
        SSARegisterHandle rhs;
        BlockId scopeT;
        BlockId scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(lhs);
            fn(rhs);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{} {} {} ? @{} : @{}", lhs.toString(), toString1(type), rhs.toString(), scopeT, scopeF);
        }

        BranchCond(JumpCondType type, SSARegisterHandle lhs, SSARegisterHandle rhs, BlockId t, BlockId f): NamedIrInstruction<"branch_cond", CTX>(SSARegisterHandle::invalid()), type(type), lhs(lhs), rhs(rhs), scopeT(t), scopeF(f) {

        };

        void generate(CTX::GEN &ctx) override {
            auto rhsReg = ctx.getReg(rhs);
            auto lhsReg = ctx.getReg(lhs);
            auto label = ctx.nextLabel();

            // fast path to prevent unnecesary jump ... this can still be improved if we specialize for falltrough
            if (not ctx.mustAssignPhis(scopeT)) {
                // jump true
                ctx.jmpBlockCond(type, lhsReg, rhsReg, scopeT);

                // assign phis, this is noop if there are none
                ctx.assignPhis(scopeF);
                ctx.jmpBlock(scopeF);

                return;
            }

            // branch to phi handling
            // NOTE: assembler call is explicit bcs we are jumping to **assembler** label not BasicBlock label
            ctx.assembler.jmpCond(label, type, lhsReg, rhsReg);

            // false block PHI handling
            ctx.assignPhis(scopeF);

            // jump to FALSE block
            ctx.jmpBlock(scopeF);

            // TRUE block phi handling
            ctx.assembler.createLabel(label);
            ctx.assignPhis(scopeT);

            // jmp to TRUE block
            ctx.jmpBlock(scopeT);
        }

        [[nodiscard]] vector<BlockId*> branchTargetsPtr() const override {
            return vector<BlockId*>{(BlockId*)&scopeT, (BlockId*)&scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct JumpFalse: public NamedIrInstruction<"jump_false", CTX> {
        PUB_VIRTUAL_COPY(JumpFalse)
        SSARegisterHandle condition;
        BlockId scopeT;
        BlockId scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(condition);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}, {}, {}", condition.toString(), scopeT, scopeF);
        }

        JumpFalse(SSARegisterHandle condition, BlockId t, BlockId f): NamedIrInstruction<"jump_false", CTX>(SSARegisterHandle::invalid()), condition(condition), scopeT(t), scopeF(f) {

        };

        void generate(CTX::GEN &ctx) override {
            auto condReg = ctx.getReg(condition);
            auto label = ctx.nextLabel();

            // TODO fast path to prevent unnecessary jump

            if (ctx.mustAssignPhis(scopeF)) {
                // branch to phi handling
                ctx.assembler.jmpLabelTrue(condReg, label);

                // false block PHI handling
                ctx.assignPhis(scopeF);

                // jump to FALSE block
                ctx.jmpBlock(scopeF);
            }
            else {
                // false branch doesnt require phi assigment just jump there
                ctx.jmpBlockFalse(condReg, scopeF);
            }

            // TRUE block phi handling
            ctx.assembler.createLabel(label);
            ctx.assignPhis(scopeT);
        }

        [[nodiscard]] vector<BlockId> branchTargetsPtr() const override {
            return {&scopeT, &scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct JumpTrue: public NamedIrInstruction<"jump_true", CTX> {
        PUB_VIRTUAL_COPY(JumpTrue)
        SSARegisterHandle condition;
        BlockId scopeT;
        BlockId scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(condition);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}, {}, {}", condition.toString(), scopeT, scopeF);
        }

        JumpTrue(SSARegisterHandle condition, BlockId t, BlockId f): NamedIrInstruction<"jump_true", CTX>(SSARegisterHandle::invalid()), condition(condition), scopeT(t), scopeF(f) {

        };

        void generate(CTX::GEN &ctx) override {
            auto condReg = ctx.getReg(condition);
            auto label = ctx.nextLabel();

            if (ctx.mustAssignPhis(scopeT)) {
                // branch to phi handling
                ctx.assembler.jmpLabelFalse(condReg, label);

                // false block PHI handling
                ctx.assignPhis(scopeT);

                // jump to FALSE block
                ctx.jmpBlock(scopeT);
            }
            else {
                // false branch doesnt require phi assigment just jump there
                ctx.jmpBlockTrue(condReg, scopeT);
            }

            // TRUE block phi handling
            ctx.assembler.createLabel(label);
            ctx.assignPhis(scopeF);
        }

        [[nodiscard]] vector<BlockId*> branchTargetsPtr() const override {
            return {(BlockId*)&scopeT, (BlockId*)&scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct Jump: public IRBaseInstruction<BlockId, "jump", CTX> {
    PUB_VIRTUAL_COPY(Jump)
        bool shouldAssign = true;

        Jump(BlockId value): IRBaseInstruction<BlockId, "jump", CTX>(SSARegisterHandle::invalid(), value) {}

        void generate(CTX::GEN& ctx) override {
            if (shouldAssign) ctx.assignPhis(this->value);

            ctx.jmpBlock(this->value);
        }

        [[nodiscard]] vector<BlockId*> branchTargetsPtr() const override { return std::vector<BlockId*>{(BlockId*)&this->value}; }

        bool isTerminal() const override { return true; }
    };


    template<typename CTX>
    struct JumpCond: public NamedIrInstruction<"jump_cond", CTX> {
        PUB_VIRTUAL_COPY(JumpCond)
        JumpCondType type;
        SSARegisterHandle lhs;
        SSARegisterHandle rhs;
        BlockId scopeT;
        BlockId scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(rhs);
            fn(lhs);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, " {} {} {} ? @{} : @{}", lhs, toString1(type), rhs, scopeT, scopeF);
        }

        JumpCond(JumpCondType type, SSARegisterHandle lhs, SSARegisterHandle rhs, BlockId t, BlockId f): NamedIrInstruction<"jump_cond", CTX>(SSARegisterHandle::invalid()), type(type), lhs(lhs), rhs(rhs), scopeT(t), scopeF(f) {

        };

        void generate(CTX::GEN &ctx) override {
            auto rhsReg = ctx.getReg(rhs);
            auto lhsReg = ctx.getReg(lhs);
            auto label = ctx.nextLabel();

            if (ctx.mustAssignPhis(scopeT)) {
                // if false skip phi assignment
                ctx.assembler.jmpCond(label, negateType(type), lhsReg, rhsReg);

                // TRUE block PHI handling
                ctx.assignPhis(scopeT);

                // jump to TRUE block
                ctx.jmpBlock(scopeT);
            }
            else {
                // TRUE branch doesnt require phi assigment just jump there
                ctx.jmpBlockCond(type, lhsReg, rhsReg, scopeT);
            }

            // FALSE block phi handling
            ctx.assembler.createLabel(label);
            ctx.assignPhis(scopeF);
        }

        [[nodiscard]] vector<BlockId*> branchTargetsPtr() const override {
            return {(BlockId*)&scopeT, (BlockId*)&scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct Return: public IR1Instruction<"return", CTX> {
    PUB_VIRTUAL_COPY(Return)
        using IR1Instruction<"return", CTX>::IR1Instruction;

        explicit Return(SSARegisterHandle ret): IR1Instruction<"return", CTX>(SSARegisterHandle::invalid(), ret) {}

        void generate(CTX::GEN& gen) override {
            if (!this->value.isValid()) return gen.assembler.generateRet();

            auto res = gen.getReg(this->value);

            gen.assembler.generateRet(res);
        }

        bool isTerminal() const override {return true;}
    };

    template<typename CTX>
    struct ReturnCompound: public NamedIrInstruction<"return_compound", CTX> {
        PUB_VIRTUAL_COPY(ReturnCompound)

        std::vector<SSARegisterHandle> parts;

        explicit ReturnCompound(std::vector<SSARegisterHandle> parts): NamedIrInstruction<"return_compound", CTX>(SSARegisterHandle::invalid()), parts(std::move(parts)) {}


        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            for (auto& s : parts) fn(s);
        }

        void generate(CTX::GEN&) override {
            TODO()
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", parts);
        }

        bool isTerminal() const override {return true;}
    };

    template<typename CTX>
    struct IntLiteral: public NamedIrInstruction<"int", CTX> {
        PUB_VIRTUAL_COPY(IntLiteral)
        size_t value;
        bool isSigned;

        IntLiteral(SSARegisterHandle target, size_t value, bool isSigned): NamedIrInstruction<"int", CTX>(target), value(value), isSigned(isSigned) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

        void generate(CTX::GEN& gen) override {
            auto res = gen.getReg(this->target);
            if (isSigned)
                gen.assembler.movSigned(res, value);
            else
                gen.assembler.movUnsigned(res, value);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", isSigned ? stringify(std::bit_cast<intmax_t>(value)) : stringify(value));
        }
    };

    template<typename CTX>
    struct CharLiteral: public IRBaseInstruction<char, "char", CTX> {
    PUB_VIRTUAL_COPY(CharLiteral)
        using IRBaseInstruction<char, "char", CTX>::IRBaseInstruction;

        void generate(CTX::GEN& gen) override {
            auto res = gen.getReg(this->target);
            gen.assembler.movUnsigned(res, this->value);
        }
    };

    template<typename CTX>
    struct FloatLiteral: public IRBaseInstruction<float, "float", CTX> {
    PUB_VIRTUAL_COPY(FloatLiteral)
        using IRBaseInstruction<float, "float", CTX>::IRBaseInstruction;

        void generate(CTX::GEN& gen) override {
            auto res = gen.getReg(this->target);
            gen.assembler.movUnsigned(res, bit_cast<u32>(this->value));
        }
    };

    template<typename CTX>
    struct DoubleLiteral: public IRBaseInstruction<double, "double", CTX> {
        PUB_VIRTUAL_COPY(DoubleLiteral)
        using IRBaseInstruction<double, "double", CTX>::IRBaseInstruction;

        void generate(CTX::GEN& gen) override {
            auto res = gen.getReg(this->target);
            gen.assembler.movUnsigned(res, bit_cast<u64>(this->value));
        }
    };

    template<typename CTX>
    struct BoolLiteral: public IRBaseInstruction<bool, "bool", CTX> {
    PUB_VIRTUAL_COPY(BoolLiteral)
        using IRBaseInstruction<bool, "bool", CTX>::IRBaseInstruction;

        void generate(CTX::GEN &gen) override {
            auto res = gen.getReg(this->target);
            gen.assembler.movUnsigned(res, this->value);
        }
    };

    template<typename CTX>
    struct VoidReturn: public IR0Instruction<"void_return", CTX> {
    PUB_VIRTUAL_COPY(VoidReturn)
        using IR0Instruction<"void_return", CTX>::IR0Instruction;

        // explicit VoidReturn(): IR0Instruction<"void_return", CTX>(SSARegisterHandle::invalid()) {}

        void generate(CTX::GEN& gen) override {
            gen.assembler.generateRet();
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct Invalid: public IR0Instruction<"invalid", CTX> {
    PUB_VIRTUAL_COPY(Invalid)
        void generate(CTX::GEN &gen) override {
            TODO();
        }
    };

    template<typename CTX>
    struct Arg: public NamedIrInstruction<"arg", CTX> {
        PUB_VIRTUAL_COPY(Arg)

        Arg(SSARegisterHandle tgt): NamedIrInstruction<"arg", CTX>(tgt) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "");
        }

        void generate(CTX::GEN& gen) override {}
    };


    template<typename CTX>
    struct PointerStore: public NamedIrInstruction<"ptr_store", CTX> {
        PUB_VIRTUAL_COPY(PointerStore)
        SSARegisterHandle ptr;
        SSARegisterHandle value;
        i64 offset;

        PointerStore(SSARegisterHandle ptr, SSARegisterHandle value, i64 offset = 0): NamedIrInstruction<"ptr_store", CTX>(SSARegisterHandle::invalid()), ptr(ptr), value(value), offset(offset) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(ptr);
            fn(value);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}+{} <- {}", ptr, offset, value);
        }

        void generate(CTX::GEN& gen) override {
            auto ptrReg = gen.getReg(ptr);
            auto valueReg = gen.getReg(value);
            auto valueSize = gen.sizeBytes(value);

            gen.assembler.writeMem(ptrReg, valueReg, offset, valueSize);
        }
    };

    template<typename CTX>
    struct PointerLoad: public NamedIrInstruction<"ptr_load", CTX> {
        PUB_VIRTUAL_COPY(PointerLoad)
        SSARegisterHandle ptr;
        i64 offset;

        PointerLoad(SSARegisterHandle target, SSARegisterHandle ptr, size_t size, i64 offset): NamedIrInstruction<"ptr_load", CTX>(target), ptr(ptr), offset(offset) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(ptr);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}+{}", ptr, offset);
        }

        void generate(CTX::GEN& gen) override {
            auto tgt = gen.getReg(this->target);
            auto ptrReg = gen.getReg(ptr);
            auto tgtSize = gen.sizeBytes(this->target);

            gen.assembler.readMem(tgt, ptrReg, tgtSize*offset, tgtSize);
        }
    };

    template<typename CTX>
    struct ValueSize: public NamedIrInstruction<"value_size", CTX> {
        PUB_VIRTUAL_COPY(ValueSize)
        SSARegisterHandle subject;

        ValueSize(SSARegisterHandle target, SSARegisterHandle subject): NamedIrInstruction<"value_size", CTX>(target), subject(subject) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(subject);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", subject);
        }

        void generate(CTX::GEN& gen) override {
            auto tgt = gen.getReg(this->target);
            auto tgtSize = gen.sizeBytes(subject);

            gen.assembler.movUnsigned(tgt, tgtSize);
        }
    };

    /// stack allocation - allocate **Pointer** to nB sized memory location
    template<typename CTX>
    struct AllocaPtr: public NamedIrInstruction<"alloca_ptr", CTX> {
        PUB_VIRTUAL_COPY(AllocaPtr)
        size_t size;

        AllocaPtr(SSARegisterHandle target, size_t size): NamedIrInstruction<"alloca_ptr", CTX>(target), size(size) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", size);
        }

        void generate(CTX::GEN& gen) override {
            // handled by CodeGen
        }
    };

    template<typename CTX>
    struct BitExtract: public NamedIrInstruction<"bit_extract", CTX> {
        PUB_VIRTUAL_COPY(BitExtract)
        SSARegisterHandle subject;
        i64 offset;
        i64 size;

        BitExtract(SSARegisterHandle target, SSARegisterHandle subject, i64 offset, i64 size): NamedIrInstruction<"bit_extract", CTX>(target), subject(subject), offset(offset), size(size) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(subject);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}[{}..<{}]", subject, offset, offset+size);
        }

        void generate(CTX::GEN& gen) override {
            TODO();
        }
    };

    template<typename CTX>
    struct MakeCompound: public NamedIrInstruction<"make_compound", CTX> {
        PUB_VIRTUAL_COPY(MakeCompound)
        std::vector<SSARegisterHandle> inputs;

        MakeCompound(SSARegisterHandle target, std::vector<SSARegisterHandle> inputs): NamedIrInstruction<"make_compound", CTX>(target), inputs(inputs) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            for (auto& input : inputs) fn(input);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", inputs);
        }

        void generate(CTX::GEN& gen) override {
            TODO();
        }
    };

    template<typename CTX>
    struct Dummy: public NamedIrInstruction<"dummy", CTX> {
        PUB_VIRTUAL_COPY(Dummy)

        Dummy(SSARegisterHandle target): NamedIrInstruction<"dummy", CTX>(target) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "");
        }

        void generate(CTX::GEN& gen) override {
            TODO();
        }
    };

    template<typename CTX>
    struct Builtin: public NamedIrInstruction<"builtin", CTX> {
        PUB_VIRTUAL_COPY(Builtin)
        std::string type;

        Builtin(std::string type): NamedIrInstruction<"builtin", CTX>(SSARegisterHandle::invalid()), type(type) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", type);
        }

        void generate(CTX::GEN& gen) override {
            TODO();
        }
    };

    template<typename CTX>
    struct COW: public NamedIrInstruction<"cow", CTX> {
        PUB_VIRTUAL_COPY(COW)

        SSARegisterHandle src;
        SSARegisterHandle sub;
        long offset;

        COW(SSARegisterHandle target, SSARegisterHandle sub, SSARegisterHandle src, long offset): NamedIrInstruction<"cow", CTX>(target), src(src), sub(sub), offset(offset) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(src);
            fn(sub);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{} | {} << {}", sub, src, offset);
        }

        void generate(CTX::GEN& gen) override {
            TODO();
        }
    };

    template<typename CTX>
    struct AddressOf: public NamedIrInstruction<"address_of", CTX> {
        PUB_VIRTUAL_COPY(AddressOf)
        SSARegisterHandle obj;
        size_t offset;

        AddressOf(SSARegisterHandle target, SSARegisterHandle obj, size_t offset = 0): NamedIrInstruction<"address_of", CTX>(target), obj(obj) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(obj);
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", obj);
        }

        void generate(CTX::GEN& gen) override {
            PANIC("AddressOf is a virtual instruction that must be lowered first to alloca")
        }
    };
}

/*
 * fn fact(n: int): int {
 * if n == 0 {
 *  return 1
 * }
 * return fact(n-1)*n;
 * }
 */

/*
 * BLOCK0:
 *  tempA := 0
 *  tempB := eq n, tempA1
 *  branch tempB, BLOCK1:, IF0
 *
 * IF0:
 *  tempC := 1
 *  return tempB1
 *
 * BLOCK1:
 *  tempD := 1
 *  tempE := sub tempD, 1
 *  tempF := call fact, tempE
 *  tempG := mul tempF, n1
 *  return tempG
 */


/*
 * BLOCK0:
 *  tempA := 0
 *  tempB := eq n, tempA1
 *  _ := branch tempB, BLOCK1:, IF0
 *
 * IF0:
 *  tempC := 1
 *  _ := return tempB1
 *
 * BLOCK1:
 *  tempD := 1
 *  tempE := sub tempD, 1
 *  tempF := call fact, tempE
 *  tempG := mul tempF, n1
 *  _ := return tempG
 */
