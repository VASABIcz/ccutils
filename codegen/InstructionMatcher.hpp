#pragma once
#include "codegen/IRInstruction.h"
#include "codegen/Assembler.h"
#include "utils/Variant.h"

struct BasePattern {
    virtual ~BasePattern() = default;

    template<class T>
    T* as() {
        return dynamic_cast<T*>(this);
    }
};

struct SimplePattern : BasePattern {
    SimplePattern(const char* base, std::vector<BasePattern*> params) : base(base), params(params) {
    }

    const char* base;
    std::vector<BasePattern*> params;
};

struct SimpleWildPattern : BasePattern {
    SimpleWildPattern(const char* base) : base(base) {
    }

    const char* base;
};

struct WildCardPattern : BasePattern {
};

struct BinOpPattern : BasePattern {
    BinOpPattern(Assembler::BinaryOp op, Assembler::BaseDataType type, BasePattern* lhs,
                 BasePattern* rhs) : op(op), type(type), lhs(lhs), rhs(rhs) {
    }

    Assembler::BinaryOp op;
    Assembler::BaseDataType type;
    BasePattern* lhs;
    BasePattern* rhs;
};

template<typename CTX>
struct InstructionMatcher {
    struct MatchedInstructions {
        IRInstruction<CTX>* mInst = nullptr;
        SSARegisterHandle reg;
        std::vector<MatchedInstructions*> mParams;

        template<typename... Args>
        MatchedInstructions& operator[](Args... args) {
            MatchedInstructions* self = this;
            for (auto arg: {args...}) {
                self = self->mParams[arg];
            }
            return *self;
        }

        SSARegisterHandle dst() { return reg; }

        IRInstruction<CTX>* inst() { return this->mInst; }

        std::span<MatchedInstructions*> params() { return this->mParams; }

        std::vector<IRInstruction<CTX>*> getParams() {
            std::vector<IRInstruction<CTX>*> res;

            for (auto arg: mParams) {
                res.push_back(arg->inst());
            }

            return res;
        }
    };

    template<typename T, typename... Args>
    BasePattern* makePattern(Args... arg) {
        return new SimplePattern{typeid(T).name(), std::vector<typename First<Args...>::TYPE>{arg...}};
    }

    template<template<typename> typename T, typename... Args>
    BasePattern* makePattern(Args... arg) {
        return makePattern<T<CTX> >(std::forward<Args>(arg)...);
    }

    template<template<typename> typename T>
    BasePattern* makePattern() {
        return makePattern<T<CTX> >();
    }

    template<typename T>
    BasePattern* makePattern() {
        return new SimplePattern{typeid(T).name(), std::vector<BasePattern*>{}};
    }

    template<typename T>
    BasePattern* makeWildPattern() {
        return new SimpleWildPattern{typeid(T).name()};
    }

    template<template<typename> typename T>
    BasePattern* makeWildPattern() {
        return makeWildPattern<T<CTX> >();
    }

    template<Assembler::BinaryOp OP, Assembler::BaseDataType TYP>
    BasePattern* makeBin(BasePattern* lhs, BasePattern* rhs) {
        return new BinOpPattern{OP, TYP, lhs, rhs};
    }

    MatchedInstructions* tryMatch(
        ControlFlowGraph<CTX>& cfg,
        SSARegisterHandle reg,
        BasePattern* pattern
    ) {
        // FIXME this should handle case if multiple definitions exist and fail
        auto inst = cfg.resolveInstruction(reg);

        return matchPattern(cfg, inst, reg, pattern);
    }

    MatchedInstructions* matchPattern(
        ControlFlowGraph<CTX>& cfg,
        IRInstruction<CTX>* inst, // FIXME null is valid value??? but only wildcard matches it???
        SSARegisterHandle reg,
        BasePattern* pattern
    ) {
        // assert(inst != nullptr);
        return dispatch(
            pattern,
            CASE_VAL(BinOpPattern*, bin) -> MatchedInstructions* {
                if (inst == nullptr) return nullptr;
                instructions::BinaryInstruction<CTX>* bbin = inst->template cst<instructions::BinaryInstruction<CTX>>();
                if (bbin == nullptr) return nullptr;
                if (bbin->op != bin->op) return nullptr;
                if (bbin->type != bin->type) return nullptr;
                auto lm = tryMatch(cfg, bbin->getSrc(0), bin->lhs);
                if (lm == nullptr) return nullptr;
                auto rm = tryMatch(cfg, bbin->getSrc(1), bin->rhs);
                if (rm == nullptr) return nullptr;
                return new MatchedInstructions{inst, reg, {lm, rm}};
            },
            CASE_VAL(WildCardPattern*) {
                return new MatchedInstructions{inst, reg};
            },
            CASE_VAL(SimpleWildPattern*, sim1) -> MatchedInstructions* {
                if (inst == nullptr) return nullptr;
                if (typeid(*inst).name() != sim1->base) return nullptr;

                std::vector<MatchedInstructions*> matched;
                for (auto i = 0ul; i < inst->srcCount(); i++) {
                    auto didMatch = tryMatch(cfg, inst->getSrc(i), getWild());
                    if (didMatch == nullptr) return nullptr;
                    matched.push_back(didMatch);
                }
                return new MatchedInstructions{inst, reg, matched};
            },
            CASE_VAL(SimplePattern*, sim) -> MatchedInstructions* {
                if (inst == nullptr) return nullptr;
                if (typeid(*inst).name() != sim->base) return nullptr;

                std::vector<MatchedInstructions*> matched;
                for (auto i = 0ul; i < sim->params.size(); i++) {
                    auto didMatch = tryMatch(cfg, inst->getSrc(i), sim->params[i]);
                    if (didMatch == nullptr) return nullptr;
                    matched.push_back(didMatch);
                }
                return new MatchedInstructions{inst, reg, matched};
            },
            CASE_VAL(BasePattern*) -> MatchedInstructions* { TODO("unahndled case") }
        ).unwrap();
    }

    static BasePattern* getWild() { return new WildCardPattern(); }
};
