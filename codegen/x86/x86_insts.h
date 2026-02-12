#pragma once
#include <utility>

#include "codegen/IRInstruction.h"

namespace x86::inst {
    template<typename CTX>
    struct BaseCall: public NamedIrInstruction<CTX> {
        PUB_VIRTUAL_COPY(BaseCall)
        std::vector<SSARegisterHandle> results;
        std::vector<std::vector<SSARegisterHandle>> bundles;
        std::variant<size_t, SSARegisterHandle> id;

        BaseCall(std::vector<SSARegisterHandle> target, std::vector<std::vector<SSARegisterHandle>> bundles, std::variant<size_t, SSARegisterHandle> id): NamedIrInstruction<CTX>(SSARegisterHandle::invalid(), "call", {}), results(target), bundles(bundles), id(id) {
        }

        BaseCall(SSARegisterHandle target, std::vector<SSARegisterHandle> bundles, std::variant<size_t, SSARegisterHandle> id): NamedIrInstruction<CTX>(target, "call", {}) {
            if (target.isValid()) this->results.push_back(target);
            this->bundles = bundles | views::transform([](auto b) { return std::vector<SSARegisterHandle>{b}; }) | ranges::to<std::vector<std::vector<SSARegisterHandle>>>();
            this->id = id;
        }

        std::vector<SSARegisterHandle> simpleArgs() {
            std::vector<SSARegisterHandle> args;

            for (auto bundle : bundles) {
                assert(bundle.size() == 1);
                args.push_back(bundle[0]);
            }

            return args;
        }

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            if (holds_alternative<SSARegisterHandle>(id)) {
                fn(get<SSARegisterHandle>(id));
            }

            for (auto& bundle : bundles) {
                for (auto& arg : bundle) fn(arg);
            }
        }

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{} <== {}", this->results, this->bundles);
        }

        void patchDst(SSARegisterHandle old, SSARegisterHandle newer) override {
            for (auto& tgt : this->results) {
                if (tgt == old) tgt = newer;
            }
        }

        void generate(CTX::GEN& gen) override {}
    };

    template<typename CTX>
    struct LeaRip: public NamedIrInstruction<CTX> {
        PUB_VIRTUAL_COPY(LeaRip)
        size_t id;

        LeaRip(SSARegisterHandle target, size_t id): NamedIrInstruction<CTX>(target, "x86_lea_rip"), id(id) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", id);
        }
    };

    template<typename CTX>
    struct Arg2: public NamedIrInstruction<CTX> {
        PUB_VIRTUAL_COPY(Arg2)
        std::vector<SSARegisterHandle> targets;

        Arg2(std::vector<SSARegisterHandle> targets): NamedIrInstruction<CTX>(SSARegisterHandle::invalid(), "marg", {}), targets(targets) {
            // this->additionalTargets.insert(this->additionalTargets.end(), targets.begin(), targets.end());
        }

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", targets);
        }
    };

    template<typename CTX>
    struct MovRip: public IR0Instruction<CTX> {
        PUB_VIRTUAL_COPY(MovRip)
        size_t id;

        MovRip(SSARegisterHandle target, size_t id): IR0Instruction<CTX>(target, "x86_mov_rip"), id(id) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", id);
        }
    };
}
