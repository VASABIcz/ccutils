#pragma once
#include "utils/utils.h"
#include "utils/Logger.h"
#include "GraphColoring.hpp"
#include "forward.hpp"
#include "Block.h"
#include "ranges.hpp"
#include "print_utils.h"

struct Graph {
    Graph(std::shared_ptr<Logger> logger) : logger(logger) {}

    void forEachInst(std::function<void(Block*, X86Instruction*)> fn) {
        for (auto block : blocks) {
            for (auto inst : block->iterator()) {
                fn(block, inst);
            }
        }
    }

    size_t totalStackSize() const {
        size_t acu = 0;
        for (auto size: stackSizes) {
            acu += size;
        }

        return acu;
    }

    std::map<size_t, size_t> stackOffsets() const {
        std::map<size_t, size_t> offsets;
        size_t acu = 0;
        for (auto [i, size]: stackSizes | views::enumerate) {
            offsets[i] = acu;
            acu += size;
        }

        return offsets;
    }

    GraphColoring registerAllocate(const std::function<void(size_t)>& onFailedAlloc = [](auto) {}) {
        size_t spillCounter = 0;

        while (true) {
            this->calcLiveRanges();

            auto gc = this->buildAllocatorGraph();

            auto res = gc.regAllocFast();
            if (not res.has_value()) return gc;
            logger->DEBUG("[graph-color] FAILED to color {}", *res);

            auto toSpill = gc.getInterferenceMaxNeighbour(*res);

            logger->DEBUG("[graph-color] spilling {}", toSpill);
            this->spill(new Virtual{toSpill});

            onFailedAlloc(spillCounter);

            spillCounter += 1;
        }
    }

    size_t allocVirtId() { return virtCounter++; }

    Virtual* allocaVirtReg() { return new Virtual{allocVirtId()}; }

    size_t allocStack(size_t size, size_t alignment) {
        stackSizes.push_back(size);
        return stackSizes.size() - 1;
    }

    StackSlot allocStackSlot(size_t size, size_t alignment) { return StackSlot{allocStack(size, alignment)}; }

    void prune() {
        std::set<size_t> used;

        for (auto b: blocks) {
            for (auto inst: b->iterator()) {
                for (auto use: inst->uses) {
                    if (auto v = use->as<Virtual>(); v) {
                        used.insert(v->id);
                    }
                }
            }
        }

        auto canDelete = [&](std::span<BaseRegister*> defs) {
            if (defs.empty()) return false;

            for (auto reg: defs) {
                if (auto v = reg->as<Virtual>(); v) {
                    if (used.contains(v->id)) return false;
                } else {
                    return false;
                }
            }
            return true;
        };

        for (auto b: blocks) {
            std::set<X86Instruction*> toDelete;
            for (auto inst: b->iterator()) {
                if (canDelete(inst->defs)) {
                    toDelete.insert(inst);
                }
            }
            for (auto inst: toDelete)
                b->erase(inst);
        }
    }

    void print() {
        for (auto [id, block]: blocks | views::enumerate) {
            println("BLOCK {}", id);
            printBlock(block);
        }
    }

    void printLiveRange() {
        for (auto [id, block]: blocks | views::enumerate) {
            println("BLOCK {} range 0..<{}", id, block->size - 1);
            for (auto range: virtualRanges[block]) {
                println("  v{} = {}..{}", range.first, range.second.first->orderId, range.second.last->orderId);
            }
        }
    }

    X86Instruction* findDefVirt(Block* block, X86Instruction* start, size_t virtId) {
        for (auto inst: Block::RevIter{start->prev}) {
            if (inst->hasDefVirt(virtId)) return inst;
        }
        return nullptr;
    }

    X86Instruction* findDefPhy(Block* block, X86Instruction* start, x86::X64Register phyId) {
        for (auto inst: Block::RevIter{start->prev}) {
            if (inst->hasDefPhy(phyId)) return inst;
        }
        return nullptr;
    }

    void markLiveVirt(Block* b, X86Instruction* start, X86Instruction* endIncl, size_t virtId) {
        if (virtualRanges[b].contains(virtId)) {
            virtualRanges[b][virtId] = virtualRanges[b][virtId].merge(Range{start, endIncl});
        } else {
            virtualRanges[b][virtId] = Range{start, endIncl};
        }
    }

    void markLivePhy(Block* b, X86Instruction* start, X86Instruction* endIncl, x86::X64Register phyId) {
        if (physicalRanges[b].contains(phyId)) {
            physicalRanges[b][phyId].add(Range{start, endIncl});
        } else {
            physicalRanges[b][phyId].add(Range{start, endIncl});
        }
    }

    bool virtVirtInt(size_t id, size_t other) {
        for (auto block: blocks) {
            auto x = virtualRanges[block];
            if (!x.contains(id) || !x.contains(other)) continue;
            auto a = virtualRanges[block][id];
            auto b = virtualRanges[block][other];
            if (a.intersects(b)) return true;
        }
        return false;
    }

    bool virtPhyInt(size_t id, x86::X64Register other) {
        for (auto block: blocks) {
            auto x = virtualRanges[block];
            auto x1 = physicalRanges[block];
            if (!x.contains(id) || !x1.contains(other)) continue;
            auto a = virtualRanges[block][id];
            auto b = physicalRanges[block][other];
            if (b.intersects(a)) return true;
        }
        return false;
    }

    void findDefVirtRec(Block* block, X86Instruction* foundUse, size_t virtId, std::set<Block*>& visited) {
        if (visited.contains(block)) return;
        visited.insert(block);

        if (foundUse == nullptr) foundUse = block->last;

        auto foundDef = findDefVirt(block, foundUse, virtId);
        if (foundDef != nullptr) {
            markLiveVirt(block, foundDef, foundUse, virtId);
        } else {
            markLiveVirt(block, block->first, foundUse, virtId); // implicitly mark live range, search in incoming

            for (auto inc: block->incoming) {
                findDefVirtRec(inc, nullptr, virtId, visited);
            }
        }

        visited.erase(block);
    }

    void calcLiveRanges() {
        this->virtualRanges.clear();
        this->physicalRanges.clear();
        for (auto block: blocks) {
            for (auto inst: block->iterator()) {
                for (auto use: inst->uses) {
                    auto v = use->as<Virtual>();
                    if (v == nullptr) continue;
                    std::set<Block*> visited;
                    findDefVirtRec(block, inst, v->id, visited);
                }

                for (auto use: inst->uses) {
                    auto v = use->as<Physical>();
                    if (v == nullptr) continue;
                    auto def = findDefPhy(block, inst, v->id);
                    if (def == nullptr) {
                        PANIC("definition for value {} not found", v->id.toString());
                    }
                    assert(def != nullptr);
                    markLivePhy(block, def, inst, v->id);
                }

                for (auto use: inst->kills) {
                    auto v = use->as<Physical>();
                    if (v == nullptr) continue;
                    markLivePhy(block, inst, inst, v->id);
                }
            }
        }
    }

    std::set<size_t> getVirtIds() {
        std::set<size_t> res;

        for (const auto& [block, ranges]: virtualRanges) {
            for (auto [vId, stuff]: ranges)
                res.insert(vId);
        }

        return res;
    }

    std::set<x86::X64Register> getPhyIds() {
        std::set<x86::X64Register> res;

        for (const auto& [block, ranges]: physicalRanges) {
            for (auto [vId, stuff]: ranges)
                res.insert(vId);
        }

        return res;
    }

    // find all definition / uses of register
    // insert mem load before each load
    // insert mem store after each store
    // replace all ocurances of register with temporaries
    // FIXME this creates non optimal instructions, some instructions can use MEM+REG operations
    // we could loose the tmp requiriment and simplify life for reg allocator?
    // mby create simple pass that can look at the generated instructions and try to merge them?
    void spill(Virtual* regId) {
        auto stackSlot = allocStackSlot(8, 8);

        for (auto block: this->blocks) {
            for (auto inst: block->iterator()) {
                if (inst->hasDefVirt(regId->id)) {
                    auto tmp = allocaVirtReg();
                    block->insertAfter(inst, new STORESTACK(tmp, stackSlot, 0));
                    inst->rewriteDef(regId, tmp);
                }
                if (inst->hasUseVirt(regId->id)) {
                    auto tmp = allocaVirtReg();
                    block->insertBefore(inst, new LOADSTACK(tmp, stackSlot, 0, 8));
                    inst->rewriteUse(regId, tmp);
                }
                assert(!inst->hasDefVirt(regId->id));
                assert(!inst->hasUseVirt(regId->id));
            }
        }
    }

    GraphColoring buildAllocatorGraph() {
        std::set<size_t> virtIds = getVirtIds();
        std::set<x86::X64Register> phyIds = getPhyIds();

        std::map<size_t, std::set<size_t>> virtToVirt;
        std::map<size_t, std::set<x86::X64Register>> virtToPhy;

        for (auto block: blocks) {
            for (auto rangeA: virtualRanges[block]) {
                for (auto rangeB: virtualRanges[block]) {
                    if (rangeA.second.intersects(rangeB.second)) {
                        virtToVirt[rangeB.first].insert(rangeA.first);
                        virtToVirt[rangeA.first].insert(rangeB.first);
                    }
                }
            }
        }

        for (auto self: virtIds) {
            for (auto other: phyIds) {
                auto ints = virtPhyInt(self, other);
                if (ints) {
                    virtToPhy[self].insert(other);
                }
            }
        }

        size_t physVirtBase = 50'000;
        std::map<size_t, std::set<size_t>> virtToVirt2 = virtToVirt;
        for (auto [virt, physs]: virtToPhy) {
            for (auto phys: physs) {
                virtToVirt2[virt].insert(physVirtBase + phys.getRawValue());
                virtToVirt2[physVirtBase + phys.getRawValue()].insert(virt);
            }
        }

        GraphColoring gc = GraphColoring::create(logger, virtToVirt2);
        for (auto [virt, physs]: virtToPhy) {
            for (auto phys: physs) {
                gc.colorReg(physVirtBase + phys.getRawValue(), phys);
            }
        }

        return gc;
    }

    struct GVEmit {
        std::stringstream ss;
        size_t _ident = 0;

        template<class FN>
        void ident(FN&& fn) {
            _ident += 1;
            fn();
            _ident -= 1;
        }

        template<typename... Args>
        void appendLine(StringChecker<type_identity_t<Args>...> strArg, Args&&... argz) {
            ss << stringify(strArg, std::forward<Args>(argz)...) << std::endl;
        }

        void appendLine() { ss << std::endl; }
    };

    void emitGVBlock(GVEmit& e, Block* b, size_t id) {
        e.appendLine("subgraph cluster_{} {", id);

        e.ident([&] {
            e.appendLine("label = \"process #1\";");
            e.appendLine("style=filled;");
            e.appendLine("color=lightgrey;");
            e.appendLine("node [style=filled,color=white];");
            e.appendLine("edge[rank=same];");
            e.appendLine();

            for (auto i = 0ul; i < b->size - 1; i++) {
                e.appendLine("b{}i{} -> b{}i{}", id, i, id, i + 1);
            }
            e.appendLine();

            for (auto inst: b->iterator()) {
                e.appendLine("b{}i{} [shape=rect]", id, inst->orderId);
                std::stringstream ss;
                printBlockInst(inst, ss);
                e.appendLine("b{}i{} [label=\"{}\"]", id, inst->orderId, ss.str());
            }
        });

        e.appendLine("}");
    }

    void emitGVBlock2(GVEmit& e, Block* b, size_t id) {
        e.appendLine("b{} [", id);

        e.ident([&] {
            e.appendLine(R"(xlabel="B{}", label = "{)", id);

            for (auto inst: b->iterator()) {
                std::stringstream ss;
                printBlockInst(inst, ss);
                e.appendLine("<i{}>{}", inst->orderId, ss.str());

                if (inst->next != nullptr) e.appendLine("|");
            }
            e.appendLine("}\";");
        });

        e.appendLine("]");
    }

    void emitGV2(GVEmit& e) {
        e.appendLine("digraph G {");
        e.appendLine("node [shape=Mrecord, fontsize=66];");
        e.appendLine("dummy0 -> b0:i0;");
        e.appendLine("dummy0 -> v1;");

        e.ident([&] {
            // blocks
            for (auto item: blocks) {
                emitGVBlock2(e, item, item->id);
            }

            // block edges
            for (auto src: blocks) {
                for (auto dst: src->outgoing) {
                    e.appendLine("b{}:i{} -> b{}:i{} [tailport=s, headport=n];", src->id, src->last->orderId, dst->id, dst->first->orderId);
                }
            }

            // virt regs
            std::optional<std::size_t> lastId;
            e.appendLine("subgraph {");
            for (auto block: blocks) {
                for (auto [regId, range]: virtualRanges[block]) {
                    e.appendLine("v{} [shape=oval]", regId);
                    auto color = (range.first->hasDefVirt(regId)) ? "green"sv : "red"sv;
                    auto xd = false;
                    e.appendLine("v{} -> b{}:i{} [color={}, constraint={}, concentrate=false];", regId, block->id, range.first->orderId, color, (xd ? "true" : "false"));
                    e.appendLine("v{} -> b{}:i{} [color=blue, constraint={}, concentrate=false];", regId, block->id, range.last->orderId, (xd ? "true" : "false"));
                    e.appendLine("v{} -> b{} [style=invis, constraint=true, concentrate=false];", regId, block->id);
                    if (lastId.has_value()) {
                        e.appendLine("v{} -> v{} [style=invis];", *lastId, regId);
                    }
                    lastId = regId;
                }
            }
            e.appendLine("}");

            // phy regs
            e.appendLine("subgraph {");
            for (auto block: blocks) {
                for (auto [regId, rangeSet]: physicalRanges[block]) {
                    // e.appendLine("{} [shape=oval]", regId.toString());
                    for (auto range: rangeSet.ranges) {
                        e.appendLine("b{}:i{} -> b{}:i{} [color=purple] [label={}, tailport=w];", block->id, range.first->orderId, block->id, range.last->orderId, regId.toString());
                    }
                }
            }
            e.appendLine("}");
        });

        e.appendLine("}");
    }

    void emitGV(GVEmit& e) {
        e.appendLine("digraph G {");

        e.ident([&] {
            // blocks
            for (auto item: blocks) {
                emitGVBlock(e, item, item->id);
            }

            // block edges
            for (auto src: blocks) {
                for (auto dst: src->outgoing) {
                    e.appendLine("b{}i{} -> b{}i{};", src->id, src->size - 1, dst->id, 0);
                }
            }

            // regs
            for (auto block: blocks) {
                for (auto [regId, range]: virtualRanges[block]) {
                    e.appendLine("v{} -> b{}i{} [color=red];", regId, block->id, range.first->orderId);
                    e.appendLine("v{} -> b{}i{} [color=red];", regId, block->id, range.last->orderId);
                }
            }
        });

        e.appendLine("}");
    }

    std::vector<Block*> blocks;
    Block* root = nullptr;

    size_t virtCounter = 0;
    std::vector<size_t> stackSizes;
    std::shared_ptr<Logger> logger;

    std::vector<size_t> linearized;

    // live ranges
    std::map<Block*, std::map<size_t, Range>> virtualRanges;
    std::map<Block*, std::map<x86::X64Register, RangeSet>> physicalRanges;

    std::string tag;
};