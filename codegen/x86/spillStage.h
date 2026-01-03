#pragma once
#include "codegen/SSARegisterHandle.h"
#include "codegen/IRInstruction.h"
#include "codegen/CFGPatcher.h"

// 1. find all values that are taken as pointers
// 2. move those values to stack
// 3. replace all accesses with load stores
// 4. remove all AddressOf calls
template<typename CTX>
void spillStage(ControlFlowGraph<CTX>& cfg) {
    std::map<SSARegisterHandle, pair<SSARegisterHandle, size_t>> leaked;
    std::map<SSARegisterHandle, SSARegisterHandle> rewrite;

    // find leaked locals
    cfg.forEachInstruction([&](IRInstruction<CTX>& inst) {
        if (inst.template is<instructions::AddressOf>()) {
            auto size = cfg.getRecord(inst.getSrc(0)).sizeBytes();
            if (!leaked.contains(inst.getSrc(0))) {
                leaked.emplace(inst.getSrc(0), make_pair(inst.target, size));
                return;
            }
            auto [ptr, _size] = leaked[inst.getSrc(0)];
            rewrite[inst.target] = ptr;
        }
    });

    // allocate stack for leaked locals
    for (auto [reg, _it]: leaked) {
        auto [stackReg, size] = _it;
        cfg.root().template insertInst<instructions::AllocaPtr>(0, stackReg, size);
    }

    CfgPatcher<CTX> patcher{cfg};

    cfg.forEachInstruction([&](IRInstruction<CTX>& inst) {
        auto srces = inst.getSrces();
        auto target = inst.target;
        auto i = &inst;

        if (inst.template is<instructions::AddressOf>()) {
            patcher.remove(&inst);
            return;
        }

        if (rewrite.contains(target)) {
            i->patchDst(target, rewrite[target]);
        }
        for (auto src : srces) {
            if (rewrite.contains(src)) {
                i->patchSrc(src, rewrite[src]);
            }
        }

        for (auto [leak, _it]: leaked) {
            auto [ptr, size] = _it;
            if (srces.contains(leak)) {
                patcher.addBefore(&inst, [=](ControlFlowGraph<CTX>& cfg, auto& ctx) {
                    auto dummy = cfg.allocateDummy(size);
                    ctx.template patch<instructions::PointerLoad>(dummy, ptr, size, 0);
                    i->patchSrc(leak, dummy);
                });
            }
            if (leak == target) {
                patcher.addAfter(&inst, [=](ControlFlowGraph<CTX>& cfg, auto& ctx) {
                    auto dummy = cfg.allocateDummy(size);
                    ctx.template patch<instructions::PointerStore>(ptr, dummy);
                    i->patchDst(leak, dummy);
                    assert(i->target == dummy);
                });
            }
        }
    });

    patcher.execute();

    // create Alloca's at beggining
    // replace read with load to tmp
    // replace write with tmp and store
    // remove AddressOf
}