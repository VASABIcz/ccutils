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

struct SimplePattern: BasePattern {
    SimplePattern(const char* base, std::vector<BasePattern*> params) : base(base), params(params) {}
    const char* base;
    std::vector<BasePattern*> params;
};

struct SimpleWildPattern: BasePattern {
    SimpleWildPattern(const char* base) : base(base) {}
    const char* base;
};

struct WildCardPattern: BasePattern {};

struct BinOpPattern: BasePattern {
    BinOpPattern(Assembler::BinaryOp op, Assembler::BaseDataType type, BasePattern* lhs, BasePattern* rhs) : op(op), type(type), lhs(lhs), rhs(rhs) {}
    Assembler::BinaryOp op;
    Assembler::BaseDataType type;
    BasePattern* lhs;
    BasePattern* rhs;
};

template<typename CTX>
struct InstructionMatcher {
    struct MatchedInstructions {
        IRInstruction<CTX>* inst = nullptr;
        std::vector<MatchedInstructions*> params;

        template<typename... Args>
        MatchedInstructions& operator[](Args... args) {
            MatchedInstructions* self = this;
            for (auto arg: {args...}) {
                self = self->params[arg];
            }
            return *self;
        }

        IRInstruction<CTX>* operator->() { return inst; }

        IRInstruction<CTX>* operator*() { return inst; }
    };

    template<typename T, typename... Args>
    BasePattern* makePattern(Args... arg) {
        return new SimplePattern{typeid(T).name(), std::vector<typename First<Args...>::TYPE>{arg...}};
    }

    template<template<typename> typename T, typename... Args>
    BasePattern* makePattern(Args... arg) {
        return makePattern<T<CTX>>(std::forward<Args>(arg)...);
    }

    template<template<typename> typename T>
    BasePattern* makePattern() {
        return makePattern<T<CTX>>();
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
        return makeWildPattern<T<CTX>>();
    }

    template<Assembler::BinaryOp OP, Assembler::BaseDataType TYP>
    BasePattern* makeBin(BasePattern* lhs, BasePattern* rhs) {
        return new BinOpPattern{OP, TYP, lhs, rhs};
    }

    static BasePattern* getWild() { return new WildCardPattern(); }

    bool matchPattern(
        ControlFlowGraph<CTX>& cfg,
        IRInstruction<CTX>* inst,
        BasePattern* pattern,
        std::vector<IRInstruction<CTX>*>& wildcards,
        std::vector<IRInstruction<CTX>*>& consumed,
        MatchedInstructions* matched
    ) {
        matched->inst = inst;
        return dispatch(
                   pattern,
                   CASE_VAL(BinOpPattern*, bin) {
                       instructions::BinaryInstruction<CTX>* bbin = inst->template cst<instructions::BinaryInstruction<CTX>>();
                       if (bbin == nullptr) return false;
                       if (bbin->op != bin->op) return false;
                       if (bbin->type != bin->type) return false;
                       consumed.push_back(inst);
                       auto l = new MatchedInstructions{};
                       auto r = new MatchedInstructions{};
                       matched->params.push_back(l);
                       matched->params.push_back(r);
                       return matchPattern(cfg, cfg.resolveInstruction(bbin->getSrc(0)), bin->lhs, wildcards, consumed, l) &&
                              matchPattern(cfg, cfg.resolveInstruction(bbin->getSrc(1)), bin->rhs, wildcards, consumed, r);
                   },
                   CASE_VAL(WildCardPattern*) {
                       wildcards.push_back(inst);
                       return true;
                   },
                   CASE_VAL(SimpleWildPattern*, sim1) {
                       if (typeid(*inst).name() != sim1->base) return false;

                       for (auto i = 0ul; i < inst->srcCount(); i++) {
                           auto ii = new MatchedInstructions{};
                           auto didMatch = matchPattern(cfg, cfg.resolveInstruction(inst->getSrc(i)), getWild(), wildcards, consumed, ii);
                           matched->params.push_back(ii);
                           if (!didMatch) return false;
                       }
                       consumed.push_back(inst);
                       return true;
                   },
                   CASE_VAL(SimplePattern*, sim) {
                       if (typeid(*inst).name() != sim->base) return false;

                       for (auto i = 0ul; i < sim->params.size(); i++) {
                           auto ii = new MatchedInstructions{};
                           auto didMatch = matchPattern(cfg, cfg.resolveInstruction(inst->getSrc(i)), sim->params[i], wildcards, consumed, ii);
                           matched->params.push_back(ii);
                           if (!didMatch) return false;
                       }
                       consumed.push_back(inst);
                       return true;
                   },
                   CASE_VAL(BasePattern*)->bool { TODO("unahndled case") }
        ).unwrap();
    }
};