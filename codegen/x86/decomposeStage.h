#pragma once
#include "codegen/CFGPatcher.h"
#include "codegen/IRInstruction.h"
#include "utils/Logger.h"
#include "utils/Variant.h"
#include "utils/dispatch.h"
#include "x86_insts.h"

// decompose values larger than targets register, to smaller ones
// subjects for rewriting:
// - load => multiple loads + extends
// - store => multiple stores
// - make_compound => decomposition
// - bit_extract => usage of decomposed parts + smaller bit extract
// - phi - we need to emit more phis
// - assign - split into smaller assigns
// - call - pass as "bundle object" to target ABI needs to se the entire struct layout
// - call results - call2
// - return - compound_return
// - dummy - split
template<typename CTX>
inline void decomposeStage(ControlFlowGraph<CTX>& cfg, Logger& logger) {
    CfgPatcher<CTX> patcher{cfg};

    auto sorted = cfg.getReversePostOrder();
    std::map<SSARegisterHandle, std::vector<SSARegisterHandle>> splitRegs;
    auto splitPtr = &splitRegs;

    for (auto node: sorted) {
        cfg.getBlock(node).forEach([&](IRInstruction<CTX>& inst) {
            dispatch(
                &inst,
                CASEP(instructions::PointerLoad<CTX>, load) {
                    auto size = cfg.getRecord(load->target).sizeBytes();
                    if (size > 8) {
                        logger.DEBUG("[dec] should split load {} its size is {}", load->target, size);
                        splitRegs.emplace(load->target, std::vector<SSARegisterHandle>{});
                        assert(size % 8 == 0);
                        patcher.addPatch(load, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            for (auto i = 0UL; i < size / 8; i++) {
                                auto newDst = cfg.allocateDummy();
                                (*splitPtr)[load->target].push_back(newDst);
                                ctx.template patch<instructions::PointerLoad>(newDst, load->ptr, 8, load->offset + i * 8);
                            }
                        });
                    }
                },
                CASEP(instructions::PointerStore<CTX>, store) {
                    auto size = cfg.getRecord(store->value).sizeBytes();
                    if (size > 8) {
                        logger.DEBUG("[dec] should split store {} its size is {}", store->value, size);

                        patcher.addPatch(store, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            for (auto [i, arg]: (*splitPtr)[store->value] | views::enumerate) {
                                ctx.template patch<instructions::PointerStore>(store->ptr, arg, store->offset + (i * 8));
                            }
                        });
                    }
                },
                CASEP(instructions::BitExtract<CTX>, extract) {
                    // auto size = cfg.getRecord(extract->subject).sizeBytes();
                    logger.DEBUG("[dec] will split extract {} its marked", extract->target);
                    // assert(extract->size % 8 == 0);
                    assert(extract->offset % 8 == 0);
                    // assert(extract->size == 8); // for now assume 8B, we can support larger sizes in future

                    patcher.addPatch(extract, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                        ctx.template patch<instructions::Assign>(extract->target, (*splitPtr)[extract->subject][extract->offset / 8]);
                    });
                },
                CASEP(instructions::PhiFunction<CTX>, phi) { logger.DEBUG("[dec] TODO phi {}", phi->target); },
                CASEP(instructions::MakeCompound<CTX>, mC) {
                    auto size = cfg.getRecord(mC->target).sizeBytes();

                    logger.DEBUG("[dec] should split make_compound {} its size is {}", mC->target, size);
                    splitRegs.emplace(mC->target, std::vector<SSARegisterHandle>{});
                    assert(size % 8 == 0);
                    patcher.addPatch(mC, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                        for (auto input: mC->inputs) {
                            (*splitPtr)[mC->target].push_back(input);
                        }
                    });
                },
                CASEP(instructions::Return<CTX>, ret) {
                    auto size = cfg.getRecord(ret->value).sizeBytes();
                    if (size > 8) {
                        logger.DEBUG("[dec] should rewrite return {} it uses split value", ret->value);

                        patcher.addPatch(ret, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            assert((*splitPtr).at(ret->value).size() >= 2);
                            ctx.template patch<instructions::ReturnCompound>((*splitPtr).at(ret->value));
                        });
                    }
                },
                CASEP(x86::inst::CallRIP<CTX>, callRip) {
                    auto size = callRip->target.isValid() ? cfg.getRecord(callRip->target).sizeBytes() : 0;

                    if (size > 8) {
                        assert(size % 8 == 0);
                        logger.DEBUG("[dec] should rewrite call {} it returns {}", callRip->target, size);

                        patcher.addPatch(callRip, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            std::vector<SSARegisterHandle> dummies;
                            for (auto i = 0UL; i < size / 8; i++) {
                                auto dummy = cfg.allocateDummy();
                                (*splitPtr)[callRip->target].push_back(dummy);
                                dummies.push_back(dummy);
                            }
                            for (auto dummy: dummies) {
                                ctx.template patch<instructions::Dummy>(dummy);
                            }
                            ctx.template patch<x86::inst::CallRIP2>(dummies, callRip->argz, callRip->id);
                        });
                    }
                },
                CASEP(instructions::Dummy<CTX>, dummy) {
                    auto size = cfg.getRecord(dummy->target).sizeBytes();

                    if (size > 8) {
                        assert(size % 8 == 0);
                        logger.DEBUG("[dec] should rewrite dummy {} it returns {}", dummy->target, size);

                        patcher.addPatch(dummy, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            for (auto i = 0ul; i < size; i += 8) {
                                auto dummyReg = cfg.allocateDummy();
                                (*splitPtr)[dummy->target].push_back(dummyReg);
                                ctx.template patch<instructions::Dummy>(dummyReg);
                            }
                        });
                    }
                },
                CASEP(instructions::COW<CTX>, cow) {
                    auto size = cfg.getRecord(cow->target).sizeBytes();

                    assert(size % 8 == 0);
                    assert(cow->offset % 8 == 0);
                    logger.DEBUG("[dec] should rewrite cow {} it returns {}", cow->target, size);

                    patcher.addPatch(cow, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                        (*splitPtr)[cow->target] = (*splitPtr)[cow->sub];
                        (*splitPtr)[cow->target][cow->offset / 8] = cow->src;
                    });
                }
            );
        });
    }

    patcher.execute();
}