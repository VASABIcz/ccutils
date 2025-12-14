#pragma once
#include <functional>

#include "forward.h"
#include "SSARegisterHandle.h"
#include "../utils/StringLiteral.h"
#include "../utils/VirtualCopy.h"
#include "../utils/stringify.h"
#include "BlockId.h"

template<typename CTX>
struct IRInstruction: public VirtualCopy<IRInstruction<CTX>> {
    SSARegisterHandle target;
    string name;
    CodeBlock<CTX>* codeBlock = nullptr;

    typedef CodeBlock<CTX> BaseGen;

    virtual ~IRInstruction() = default;

    IRInstruction(SSARegisterHandle target, string name): target(target), name(std::move(name)) {}

    virtual void print() {
        print(std::cout);
        std::cout << std::endl;
    }

    std::set<SSARegisterHandle> validDests() const {
        if (!target.isValid()) return {};

        return {target};
    }

    CodeBlock<CTX>* getBB() {
        return codeBlock;
    }

    CTX::CFG* getCFG() {
        return getBB()->getCFG();
    }

    virtual void print(std::ostream& stream) = 0;

    virtual void generate(CTX::GEN& gen) = 0;

    [[nodiscard]] virtual vector<BlockId*> branchTargetsPtr() const {
        return {};
    }

    [[nodiscard]] vector<BlockId> branchTargets() const {
        vector<BlockId> targets;
        for (auto tgt : branchTargetsPtr()) {
            targets.push_back(*tgt);
        }
        return targets;
    }

    [[nodiscard]] virtual bool isTerminal() const {
        return false;
    }

    template<class T>
    [[nodiscard]] bool is() const {
        bool b = dynamic_cast<const T *>(this) != nullptr;
        return b;
    }

    template<template<typename> typename T>
    [[nodiscard]] bool is() const {
        bool b = dynamic_cast<const T<CTX>*>(this) != nullptr;
        return b;
    }

    template<class T>
    T* cst() {
        return dynamic_cast<T*>(this);
    }

    template<template<typename> typename T>
    T<CTX>* cst() {
        return dynamic_cast<T<CTX>*>(this);
    }

    template<typename T>
    bool ifIs(std::function<void(T&)> fn) {
        if (is<T>()) {
            fn(*cst<T>());
            return true;
        }
        return false;
    }

    virtual bool isPure() const {
        return true;
    }

    template<typename... Args>
    constexpr void basePrint(std::ostream& stream, StringChecker<type_identity_t<Args>...> strArg, Args&&... argz) {
        stream << stringify("{} := {} {}", target, name, stringify(strArg, std::forward<Args>(argz)...));
    }

    std::string toString(CTX::IRGEN& gen) {
        std::stringstream stream;
        this->print(stream);

        return stream.str();
    }

    void visitSrcs(std::function<void(SSARegisterHandle&)> fn) {
        visitSrc(fn);
        visitSrc([](SSARegisterHandle& reg) {
            assert(reg.isValid());
        });
    }

    std::vector<SSARegisterHandle> getSources() {
        std::vector<SSARegisterHandle> s;
        visitSrcs([&](auto x) {
            s.push_back(x);
        });
        return s;
    }

    SSARegisterHandle getSrc(size_t idex) {
        auto srcs = getSources();

        return srcs[idex];
    }

    size_t srcCount() {
        return getSources().size();
    }

    std::set<SSARegisterHandle> allValidRegs() {
        std::set<SSARegisterHandle> s;
        visitSrcs([&](auto x) {
            if (x.isValid()) s.insert(x);
        });
        if (target.isValid()) s.insert(target);
        return s;
    }

    std::set<SSARegisterHandle> getSrces() {
        std::set<SSARegisterHandle> res;

        for (auto src : getSources()) {
            res.insert(src);
        }

        return res;
    }

    void patchSrc(SSARegisterHandle old, SSARegisterHandle newer) {
        visitSrcs([&](auto& src) {
            if (src == old) src = newer;
        });
    }

    void patchDst(SSARegisterHandle old, SSARegisterHandle newer) {
        if (target == old) target = newer;
    }
private:
    virtual void visitSrc(function<void(SSARegisterHandle&)> fn) {}
};

template<StringLiteral V, typename CTX>
struct NamedIrInstruction: public IRInstruction<CTX> {
    NamedIrInstruction(SSARegisterHandle tgt): IRInstruction<CTX>(tgt, string(V.asView())) {}
};

template<StringLiteral V, typename CTX>
struct IR2Instruction: public IRInstruction<CTX> {
    SSARegisterHandle lhs;
    SSARegisterHandle rhs;

    using BaseGen = CTX::GEN;

    IR2Instruction(SSARegisterHandle tgt, SSARegisterHandle lhs, SSARegisterHandle rhs):
    IRInstruction<CTX>(tgt, string(V.asView())), lhs(lhs), rhs(rhs) {}

    void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        fn(rhs);
        fn(lhs);
    }

    void print(CTX::IRGEN &gen) override {
        println("{} := {} {}, {}", this->target.toTextString(), V.value, lhs.toString(), rhs.toString());
    }

    void generate(BaseGen& gen) override = 0;
};

template<typename CTX>
struct IR2Instruction2: public IRInstruction<CTX> {
    SSARegisterHandle lhs;
    SSARegisterHandle rhs;

    using BaseGen = CTX::GEN;

    IR2Instruction2(SSARegisterHandle tgt, SSARegisterHandle lhs, SSARegisterHandle rhs, string name):
    IRInstruction<CTX>(tgt, std::move(name)), lhs(lhs), rhs(rhs) {}

    void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        fn(lhs);
        fn(rhs);
    }

    void print(std::ostream& stream) override {
        this->basePrint(stream, "{}, {}", lhs.toString(), rhs.toString());
    }

    void generate(BaseGen& gen) override = 0;
};

template<StringLiteral V, typename CTX>
struct IR1Instruction: public IRInstruction<CTX> {
    SSARegisterHandle value;

    IR1Instruction(SSARegisterHandle tgt, SSARegisterHandle value): IRInstruction<CTX>(tgt, string(V.asView())), value(value) {

    }

    void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        fn(value);
    }

    void print(std::ostream& stream) override {
        this->basePrint(stream, "{}", value.toString());
    }
};

template<StringLiteral V, typename CTX>
struct IR0Instruction: public IRInstruction<CTX> {
    IR0Instruction(): IRInstruction<CTX>(SSARegisterHandle::invalid(), string(V.asView())) {}

    void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

    void print(std::ostream& stream) override {
        this->basePrint(stream, "");
    }
};

template<typename T, StringLiteral V, typename CTX>
struct IRBaseInstruction: public NamedIrInstruction<V, CTX> {
    T value;
    using BaseGen = CTX::GEN;

    IRBaseInstruction(SSARegisterHandle tgt, T value): NamedIrInstruction<V, CTX>(tgt), value(value) {

    }

    void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

    void print(std::ostream& stream) override {
        this->basePrint(stream, "{}", stringify(value));
    }

    void generate(BaseGen& gen) override = 0;
};