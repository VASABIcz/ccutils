#pragma once
#include <utility>

#include "codegen/IRInstruction.h"

namespace x86::inst {
    template<typename CTX>
    struct CallRIP: public NamedIrInstruction<"x86_call_rip", CTX> {
        PUB_VIRTUAL_COPY(CallRIP)
        std::vector<SSARegisterHandle> argz;
        size_t id;

        CallRIP(SSARegisterHandle target, std::vector<SSARegisterHandle> argz, size_t id): NamedIrInstruction<"x86_call_rip", CTX>(target), argz(std::move(argz)), id(id) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            for (auto& arg : argz) fn(arg);
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", argz);
        }
    };

    template<typename CTX>
    struct CallREG: public NamedIrInstruction<"x86_call_reg", CTX> {
        PUB_VIRTUAL_COPY(CallREG)
        std::vector<SSARegisterHandle> argz;
        SSARegisterHandle reg;

        CallREG(SSARegisterHandle target, std::vector<SSARegisterHandle> argz, SSARegisterHandle reg): NamedIrInstruction<"x86_call_reg", CTX>(target), argz(std::move(argz)), reg(reg) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            fn(reg);
            for (auto& arg : argz) fn(arg);
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}#{}", reg, argz);
        }
    };

    template<typename CTX>
    struct CallRIP2: public NamedIrInstruction<"x86_call2", CTX> {
        PUB_VIRTUAL_COPY(CallRIP2)
        std::vector<SSARegisterHandle> argz;
        std::vector<SSARegisterHandle> results;
        size_t id;

        CallRIP2(std::vector<SSARegisterHandle> target, std::vector<SSARegisterHandle> argz, size_t id): NamedIrInstruction<"x86_call2", CTX>(SSARegisterHandle::invalid()), argz(std::move(argz)), results(target), id(id) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            for (auto& arg : argz) fn(arg);
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{} <== {}", results, argz);
        }
    };

    template<typename CTX>
    struct CallREG2: public NamedIrInstruction<"x86_call2", CTX> {
        PUB_VIRTUAL_COPY(CallREG2)
        std::vector<SSARegisterHandle> argz;
        std::vector<SSARegisterHandle> results;
        SSARegisterHandle reg;

        CallREG2(std::vector<SSARegisterHandle> target, std::vector<SSARegisterHandle> argz, SSARegisterHandle reg): NamedIrInstruction<"x86_call2", CTX>(SSARegisterHandle::invalid()), argz(std::move(argz)), results(target), reg(reg) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
            for (auto& arg : argz) fn(arg);
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{} <== {}", results, argz);
        }
    };

    template<typename CTX>
    struct LeaRip: public NamedIrInstruction<"x86_lea_rip", CTX> {
        PUB_VIRTUAL_COPY(LeaRip)
        size_t id;

        LeaRip(SSARegisterHandle target, size_t id): NamedIrInstruction<"x86_lea_rip", CTX>(target), id(id) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {
        }

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", id);
        }
    };

    template<typename CTX>
    struct MovRip: public NamedIrInstruction<"x86_mov_rip", CTX> {
        PUB_VIRTUAL_COPY(MovRip)
        size_t id;

        MovRip(SSARegisterHandle target, size_t id): NamedIrInstruction<"x86_mov_rip", CTX>(target), id(id) {}

        void visitSrc(std::function<void (SSARegisterHandle &)> fn) override {}

        void generate(CTX::GEN& gen) override {}

        void print(std::ostream& stream) override {
            this->basePrint(stream, "{}", id);
        }
    };
}