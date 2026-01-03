#pragma once
#include "IRInstruction.h"

template<typename A, typename B, typename C>
struct triple {
    A first;
    B second;
    C third;
};

template<typename CTX>
struct CfgPatcher {
    struct PatchContext {
        ControlFlowGraph<CTX>& cfg;
        ControlFlowGraph<CTX>::InstHandle handle;
        size_t idex = 0;

        template<typename INST, typename... Args>
        void patch(Args... args) {
            cfg.template insertInstruction<INST>({handle.first, handle.second + idex}, std::forward<Args>(args)...);
            idex += 1;
        }

        template<template<typename> typename INST, typename... Args>
        void patch(Args... args) {
            patch<INST<CTX>>(std::forward<Args>(args)...);
        }
    };

    using PATCH_FN = std::function<void(ControlFlowGraph<CTX>& cfg, PatchContext& ctx)>;

    enum class PatchType {
        REWRITE,
        BEFORE,
        AFTER,
        REMOVE
    };

    ControlFlowGraph<CTX>& cfg;
    std::vector<triple<IRInstruction<CTX>*, PatchType, PATCH_FN>> insts;

    void addPatch(IRInstruction<CTX>* reg, PATCH_FN fn) {
        insts.emplace_back(reg, PatchType::REWRITE, fn);
    }

    void addBefore(IRInstruction<CTX>* reg, PATCH_FN fn) {
        insts.emplace_back(reg, PatchType::BEFORE, fn);
    }

    void addAfter(IRInstruction<CTX>* reg, PATCH_FN fn) {
        insts.emplace_back(reg, PatchType::AFTER, fn);
    }

    void remove(IRInstruction<CTX>* reg) {
        insts.emplace_back(reg, PatchType::REMOVE, [](auto&, auto&) {});
    }

    void execute() {
        for (auto [inst, patchType, patchfn]: insts) {
            auto pos = cfg.getInstPos(inst);
            switch (patchType) {
                case PatchType::BEFORE: {
                    PatchContext ctx{cfg, pos};
                    patchfn(cfg, ctx);
                    break;
                }
                case PatchType::AFTER: {
                    PatchContext ctx{cfg, {pos.first, pos.second + 1}};
                    patchfn(cfg, ctx);
                    break;
                }
                case PatchType::REWRITE: {
                    PatchContext ctx{cfg, pos};
                    patchfn(cfg, ctx);
                    cfg.template removeInstruction({ctx.handle.first, ctx.handle.second + ctx.idex});
                    break;
                }
                case PatchType::REMOVE: {
                    cfg.template removeInstruction(pos);
                }
            }
        }
    }
};