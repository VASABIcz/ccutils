#pragma once
#include <utility>
#include <vector>
#include <string>

#include "CodeBlock.h"
#include "SSARegisterHandle.h"
#include "IRGen.h"
#include "CodeGen.h"
#include "Assembler.h"

using namespace std;

/*template<typename ASSEMBLER>
using AssemblerMethodRef = void(ASSEMBLER::*)(size_t, size_t, size_t);*/

// template<typename CTX>struct className: public IR2Instruction<name, CTX> { PUB_VIRTUAL_COPY(className) using IR2Instruction<name, CTX>::IR2Instruction; void generate(CTX::GEN& gen) override {auto tgt = gen.getReg(this->target);auto l = gen.getReg(this->lhs);auto r = gen.getReg(this->rhs);gen.assembler.func(tgt, l, r);}};


namespace instructions {
    template<typename CTX>
    struct BinaryInstruction: public IR2Instruction2<CTX> {
        BinaryInstruction(SSARegisterHandle tgt, SSARegisterHandle lhs, SSARegisterHandle rhs, bool isComutative, std::string name): IR2Instruction2<CTX>(tgt, lhs, rhs, name), isComutative(isComutative) {}

        // OMG meth is ekshulay useful?
        // nvm
        bool isComutative = false;
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

#define SIMPLE_INSTR(_CLASS_NAME, name, ptr, isCom) template<typename CTX>struct _CLASS_NAME: public BinaryInstruction<CTX> { PUB_VIRTUAL_COPY(_CLASS_NAME) _CLASS_NAME(SSARegisterHandle tgt, SSARegisterHandle lhs, SSARegisterHandle rhs): BinaryInstruction<CTX>(tgt, lhs, rhs, isCom, name) {}; void generate(CTX::GEN& gen) override {auto tgt = gen.getReg(this->target);auto l = gen.getReg(this->lhs);auto r = gen.getReg(this->rhs);gen.assembler.ptr(tgt, l, r);}};
    // int stuff IDK
    SIMPLE_INSTR(IntAdd, "int_add", addInt, true) // opt+com
    SIMPLE_INSTR(IntSub, "int_sub", subInt, false) // opt
    SIMPLE_INSTR(IntLess, "int_less", lessInt, false) // opt
    SIMPLE_INSTR(IntGt, "int_gt", gtInt, false) // opt
    SIMPLE_INSTR(IntMul, "int_mul", mulInt, true) // opt+com
    SIMPLE_INSTR(IntDiv, "int_div", divInt, false) // opt
    SIMPLE_INSTR(IntMod, "int_mod", modInt, false) // opt
    SIMPLE_INSTR(IntEquals, "int_eq", eqInt, true) // opt+com
    SIMPLE_INSTR(IntNotEquals, "int_neq", neqInt, true) // opt+com
    SIMPLE_INSTR(IntXor, "int_xor", xorInt, true) // opt+com
    SIMPLE_INSTR(IntGe, "int_ge", geInt, false) // opt
    SIMPLE_INSTR(IntLe, "int_le", leInt, false) // opt
    // bitwise
    SIMPLE_INSTR(IntAnd, "int_and", andInt, true) // opt+com
    SIMPLE_INSTR(IntOr, "int_or", orInt, true) // opt+com
    SIMPLE_INSTR(IntShr, "int_shr", shrInt, false) // opt
    SIMPLE_INSTR(IntShl, "int_shl", shlInt, false) // opt

    // f32
    SIMPLE_INSTR(FloatAdd, "f32_add", addFloat, true)
    SIMPLE_INSTR(FloatSub, "f32_sub", subFloat, false)
    SIMPLE_INSTR(FloatMul, "f32_mul", mulFloat, true)
    SIMPLE_INSTR(FloatDiv, "f32_div", divFloat, false)
    SIMPLE_INSTR(FloatEq, "f32_eq", eqFloat, true)
    SIMPLE_INSTR(FloatNotEquals, "f32_neq", neqFloat, true)
    SIMPLE_INSTR(FloatGt, "f32_gt", gtFloat, false)
    SIMPLE_INSTR(FloatLess, "f32_ls", lessFloat, false)
    SIMPLE_INSTR(FloatGE, "f32_ge", geFloat, false)
    SIMPLE_INSTR(FloatLE, "f32_le", leFloat, false)

    // f64
    SIMPLE_INSTR(DoubleAdd, "f64_add", addDouble, true)
    SIMPLE_INSTR(DoubleSub, "f64_sub", subDouble, false)
    SIMPLE_INSTR(DoubleMul, "f64_mul", mulDouble, true)
    SIMPLE_INSTR(DoubleDiv, "f64_div", divDouble, false)
    SIMPLE_INSTR(DoubleEq, "f64_eq", eqDouble, true)
    SIMPLE_INSTR(DoubleNotEq, "f64_neq", neqDouble, true)
    SIMPLE_INSTR(DoubleGt, "f64_gt", gtDouble, false)
    SIMPLE_INSTR(DoubleLess, "f64_ls", lessDouble, false)
    SIMPLE_INSTR(DoubleGE, "f64_ge", geDouble, false)
    SIMPLE_INSTR(DoubleLE, "f64_le", leDouble, false)
#undef SIMPLE_INSTR

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
        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, "{}", stringify(versions, {{", ", "", ""}}));
        }

        void visitSrc(function<void (SSARegisterHandle &)> fn) override {
            for (auto& [block, handle]: versions) {
                fn(handle);
            }
        }

        PhiFunction(SSARegisterHandle target, map<size_t, SSARegisterHandle> versions) : NamedIrInstruction<"phi", CTX>(target),
                                                                                         versions(std::move(versions)) {}

        PhiFunction(SSARegisterHandle target, span<pair<SSARegisterHandle, size_t>> versions) : NamedIrInstruction<"phi", CTX>(target) {
            for (auto version: versions) {
                this->versions.emplace(version.second, version.first);
            }
        }

        void generate(CTX::GEN&) override {
            // we consistently have one more use count than we need
            // gen.getReg(target);
            // this should be nop
        }

        void pushVersion(SSARegisterHandle handle, size_t block) {
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

        [[nodiscard]] vector<SSARegisterHandle> getAllVersions() const {
            vector<SSARegisterHandle> buf;

            for (auto [block, handle]: versions) {
                buf.push_back(handle);
            }

            return buf;
        }

        map<size_t, SSARegisterHandle> getRawVersions() const {
            return versions;
        }

        optional<SSARegisterHandle> getBySource(size_t src) {
            if (not versions.contains(src)) return {};

            return versions[src];
        }

    private:
        map<size_t, SSARegisterHandle> versions;
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
        size_t scopeT;
        size_t scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(condition);
        }

        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, "{}, {}, {}", condition.toString(), scopeT, scopeF);
        }

        Branch(SSARegisterHandle condition, size_t t, size_t f): NamedIrInstruction<"branch", CTX>(SSARegisterHandle::invalid()), condition(condition), scopeT(t), scopeF(f) {

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

        [[nodiscard]] vector<size_t> branchTargets() const override {
            return {scopeT, scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct BranchCond: public NamedIrInstruction<"branch_cond", CTX> {
        PUB_VIRTUAL_COPY(BranchCond)
        JumpCondType type;
        SSARegisterHandle lhs;
        SSARegisterHandle rhs;
        size_t scopeT;
        size_t scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(lhs);
            fn(rhs);
        }

        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, "{} {} {} ? @{} : @{}", lhs.toString(), toString1(type), rhs.toString(), scopeT, scopeF);
        }

        BranchCond(JumpCondType type, SSARegisterHandle lhs, SSARegisterHandle rhs, size_t t, size_t f): NamedIrInstruction<"branch_cond", CTX>(SSARegisterHandle::invalid()), type(type), lhs(lhs), rhs(rhs), scopeT(t), scopeF(f) {

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

        [[nodiscard]] vector<size_t> branchTargets() const override {
            return {scopeT, scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct JumpFalse: public NamedIrInstruction<"jump_false", CTX> {
        PUB_VIRTUAL_COPY(JumpFalse)
        SSARegisterHandle condition;
        size_t scopeT;
        size_t scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(condition);
        }

        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, "{}, {}, {}", condition.toString(), scopeT, scopeF);
        }

        JumpFalse(SSARegisterHandle condition, size_t t, size_t f): NamedIrInstruction<"jump_false", CTX>(SSARegisterHandle::invalid()), condition(condition), scopeT(t), scopeF(f) {

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

        [[nodiscard]] vector<size_t> branchTargets() const override {
            return {scopeT, scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct JumpTrue: public NamedIrInstruction<"jump_true", CTX> {
        PUB_VIRTUAL_COPY(JumpTrue)
        SSARegisterHandle condition;
        size_t scopeT;
        size_t scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(condition);
        }

        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, "{}, {}, {}", condition.toString(), scopeT, scopeF);
        }

        JumpTrue(SSARegisterHandle condition, size_t t, size_t f): NamedIrInstruction<"jump_true", CTX>(SSARegisterHandle::invalid()), condition(condition), scopeT(t), scopeF(f) {

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

        [[nodiscard]] vector<size_t> branchTargets() const override {
            return {scopeT, scopeF};
        }

        bool isTerminal() const override { return true; }
    };

    template<typename CTX>
    struct Jump: public IRBaseInstruction<size_t, "jump", CTX> {
    PUB_VIRTUAL_COPY(Jump)
        Jump(size_t value): IRBaseInstruction<size_t, "jump", CTX>(SSARegisterHandle::invalid(), value) {}

        void generate(CTX::GEN& ctx) override {
            ctx.assignPhis(this->value);

            ctx.jmpBlock(this->value);
        }

        [[nodiscard]] vector<size_t> branchTargets() const override {return {this->value};}

        bool isTerminal() const override { return true; }
    };


    template<typename CTX>
    struct JumpCond: public NamedIrInstruction<"jump_cond", CTX> {
        PUB_VIRTUAL_COPY(JumpCond)
        JumpCondType type;
        SSARegisterHandle lhs;
        SSARegisterHandle rhs;
        size_t scopeT;
        size_t scopeF;

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(rhs);
            fn(lhs);
        }

        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, " {} {} {} ? @{} : @{}", lhs, toString1(type), rhs, scopeT, scopeF);
        }

        JumpCond(JumpCondType type, SSARegisterHandle lhs, SSARegisterHandle rhs, size_t t, size_t f): NamedIrInstruction<"jump_cond", CTX>(SSARegisterHandle::invalid()), type(type), lhs(lhs), rhs(rhs), scopeT(t), scopeF(f) {

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

        [[nodiscard]] vector<size_t> branchTargets() const override {
            return {scopeT, scopeF};
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

        void print(CTX::IRGEN& gen, std::ostream& stream) override {
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
    struct AddressOf: public NamedIrInstruction<"address_of", CTX> {
        PUB_VIRTUAL_COPY(AddressOf)
        SSARegisterHandle obj;

        AddressOf(SSARegisterHandle target, SSARegisterHandle obj): NamedIrInstruction<"address_of", CTX>(target), obj(obj) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(obj);
        }

        void print(CTX::IRGEN& gen, std::ostream& stream) override {
            this->basePrint(stream, "{}", obj);
        }

        void generate(CTX::GEN& gen) override {
            gen.assembler.addressOf(gen.getReg(this->target), gen.getReg(obj));
        }
    };

    template<typename CTX>
    struct Arg: public NamedIrInstruction<"arg", CTX> {
        PUB_VIRTUAL_COPY(Arg)

        Arg(SSARegisterHandle tgt): NamedIrInstruction<"arg", CTX>(tgt) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

        void print(CTX::IRGEN& gen, std::ostream& stream) override {
            this->basePrint(stream, "");
        }

        void generate(CTX::GEN& gen) override {}
    };


    template<typename CTX>
    struct PointerStore: public NamedIrInstruction<"ptr_store", CTX> {
        PUB_VIRTUAL_COPY(PointerStore)
        SSARegisterHandle ptr;
        SSARegisterHandle value;

        PointerStore(SSARegisterHandle ptr, SSARegisterHandle value): NamedIrInstruction<"ptr_store", CTX>(SSARegisterHandle::invalid()), ptr(ptr), value(value) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(ptr);
            fn(value);
        }

        void print(CTX::IRGEN&, std::ostream& stream) override {
            this->basePrint(stream, "{} {}", ptr, value);
        }

        void generate(CTX::GEN& gen) override {
            auto ptrReg = gen.getReg(ptr);
            auto valueReg = gen.getReg(value);
            auto valueSize = gen.sizeBytes(value);

            gen.assembler.writeMem(ptrReg, valueReg, 0, valueSize);
        }
    };

    template<typename CTX>
    struct PointerLoad: public NamedIrInstruction<"ptr_load", CTX> {
        PUB_VIRTUAL_COPY(PointerLoad)
        SSARegisterHandle ptr;
        i64 offset;

        PointerLoad(SSARegisterHandle target, SSARegisterHandle ptr, i64 offset = 0): NamedIrInstruction<"ptr_load", CTX>(target), ptr(ptr), offset(offset) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(ptr);
        }

        void print(CTX::IRGEN& gen, std::ostream& stream) override {
            this->basePrint(stream, "{}", ptr);
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

        void print(CTX::IRGEN& gen, std::ostream& stream) override {
            this->basePrint(stream, "{}", subject);
        }

        void generate(CTX::GEN& gen) override {
            auto tgt = gen.getReg(this->target);
            auto tgtSize = gen.sizeBytes(subject);

            gen.assembler.movUnsigned(tgt, tgtSize);
        }
    };

    /// stack allocation
    template<typename CTX>
    struct Alloca: public NamedIrInstruction<"alloca", CTX> {
        PUB_VIRTUAL_COPY(Alloca)
        size_t size;

        Alloca(SSARegisterHandle target, size_t size): NamedIrInstruction<"alloca", CTX>(target), size(size) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void print(CTX::IRGEN& gen, std::ostream& stream) override {
            this->basePrint(stream, "{}", size);
        }

        void generate(CTX::GEN& gen) override {
            // handled by CodeGen
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