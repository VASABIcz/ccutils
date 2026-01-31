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
    println("SORT IS {}", sorted);

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
                                ctx.template patch<
                                    instructions::PointerStore>(store->ptr, arg, store->offset + (i * 8));
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
                        ctx.template patch<instructions::Assign>(extract->target,
                                                                 (*splitPtr)[extract->subject][extract->offset / 8]);
                    });
                },
                CASEP(instructions::PhiFunction<CTX>, phi) { logger.DEBUG("[dec] TODO phi {}", phi->target); },
                CASEP(instructions::MakeCompound<CTX>, mC) {
                    auto size = cfg.getRecord(mC->target).sizeBytes();

                    logger.DEBUG("[dec] should split make_compound {} its size is {}", mC->target, size);
                    assert(size % 8 == 0);
                    patcher.addPatch(mC, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                        if (splitPtr->contains(mC->target)) {
                            for (auto [tgt, input]: views::zip((*splitPtr)[mC->target], mC->inputs)) {
                                ctx.template patch<instructions::Assign>(tgt, input);
                            }
                        } else {
                            for (auto input: mC->inputs) {
                                (*splitPtr)[mC->target].push_back(input);
                            }
                        }
                    });
                },
                CASEP(instructions::Return<CTX>, ret) {
                    auto size = cfg.getRecord(ret->getValue()).sizeBytes();
                    if (size > 8) {
                        logger.DEBUG("[dec] should rewrite return {} it uses split value", ret->getValue());

                        patcher.addPatch(ret, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            assert((*splitPtr).at(ret->getValue()).size() >= 2);
                            ctx.template patch<instructions::ReturnCompound>(splitPtr->at(ret->getValue()));
                        });
                    }
                },
                CASEP(x86::inst::BaseCall<CTX>, callRip) {
                    assert(callRip->results.size() <= 1);
                    auto tgt = SSARegisterHandle::invalid();
                    auto size = 0ul;
                    if (callRip->results.size() != 0) {
                        tgt = callRip->results[0];
                        size = cfg.getRecord(tgt).sizeBytes();
                    }

                    assert(size <= 7 || size % 8 == 0);
                    logger.DEBUG("[dec] should rewrite call {} it returns {}", tgt, size);

                    patcher.addPatch(callRip, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                        std::vector<SSARegisterHandle> dummies;
                        if (size > 8) {
                            for (auto i = 0UL; i < size / 8; i++) {
                                auto dummy = cfg.allocateDummy();
                                (*splitPtr)[tgt].push_back(dummy);
                                dummies.push_back(dummy);
                            }
                            for (auto dummy: dummies) {
                                ctx.template patch<instructions::Dummy>(dummy);
                            }
                        } else {
                            dummies = callRip->results;
                        }

                        std::vector<std::vector<SSARegisterHandle> > bundles;
                        for (auto arg: callRip->simpleArgs()) {
                            if (splitPtr->contains(arg)) {
                                auto v = splitPtr->at(arg);
                                bundles.push_back(v);
                            } else {
                                bundles.push_back(std::vector{arg});
                            }
                        }

                        ctx.template patch<x86::inst::BaseCall>(dummies, bundles, callRip->id);
                    });
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
                },
                CASEP(instructions::Arg<CTX>, arg) {
                    auto size = cfg.getRecord(arg->target).sizeBytes();
                    if (size > 8) {
                        logger.DEBUG("[dec] should split arg {} its size is {}", arg->target, size);
                        assert(size % 8 == 0);

                        patcher.addPatch(arg, [=](auto& cfg, CfgPatcher<CTX>::PatchContext& ctx) {
                            for (auto i = 0UL; i < size / 8; i++) {
                                auto newDst = cfg.allocateDummy();
                                (*splitPtr)[arg->target].push_back(newDst);
                                ctx.template patch<instructions::Dummy>(newDst);
                            }
                            ctx.template patch<x86::inst::Arg2>((*splitPtr)[arg->target]);
                        });
                    }
                }
            );
        });
    }

    patcher.execute();
}
