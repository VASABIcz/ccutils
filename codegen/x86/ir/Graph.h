#pragma once
#include "utils/utils.h"
#include "utils/Logger.h"
#include "GraphColoring.hpp"
#include "forward.hpp"
#include "Block.h"
#include "ranges.hpp"
#include "print_utils.h"
#include "Registers.hpp"
#include "instructions.hpp"

struct Graph {
    std::map<x86::X64Register, BaseRegister*> regMap;

    BaseRegister* getReg(x86::X64Register r) {
        if (!regMap.contains(r)) {
            regMap[r] = allocaVirtReg();
            regMap[r]->hintReg = r;
        }

        return regMap[r];
    }

    Graph(std::shared_ptr<Logger> logger) : logger(logger) {
        for (auto reg : x86::ALL_REGS) {
            getReg(reg);
        }
    }

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
            logger->DEBUG("[graph-color] FAILED to color {}", (*res)->toString());

            auto toSpill = gc.getInterferenceMaxNeighbour(*res);

            onFailedAlloc(spillCounter);

            logger->DEBUG("[graph-color] spilling {}", toSpill->toString());
            this->spill(toSpill);

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
        std::set<VHAND> used;

        for (auto b: blocks) {
            for (auto inst: b->iterator()) {
                inst->forEachUse([&](auto use) {
                    used.insert(use);
                });
            }
        }

        auto canDelete = [&](X86Instruction* inst) {
            if (inst->defCount() == 0) return false;

            for (auto i = 0u; i < inst->defCount(); i++) {
                if (used.contains(inst->getDef(i))) return false;
            }
            return true;
        };

        for (auto b: blocks) {
            std::set<X86Instruction*> toDelete;
            for (auto inst: b->iterator()) {
                if (canDelete(inst)) {
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
            for (auto [reg, range] : virtualRanges[block]) {
                for (auto inst : block->iterator()) {
                    if (range.intersects(Range{inst, inst})) {
                        std::cout << "X";
                    } else {
                        std::cout << ".";
                    }
                }
                println(" // reg={}", reg->toString());
            }
        }
    }

    void printLiveRangeGraph(std::ostream& ss, Block* block) {
        auto virts = virtualRanges[block];
        auto i = 0;

        for (auto inst : block->iterator()) {
            std::stringstream buffer;
            printBlockInst(inst, buffer);
            ss << stringify("<tr><td align=\"left\" port=\"i{}\">{}</td>", inst->orderId, buffer.str()); // row start
            inst->forEachDef([&](BaseRegister* def) {
                auto s = virts[def].getStart(inst);
                if (s.has_value()) {
                    ss << stringify("<td align=\"left\" rowspan=\"{}\" bgcolor=\"#{}\">{}</td>", s->getLength(), stringify("{:x}", ((uint32_t)hashInt(def->as<Virtual>()->id+31))>>8).substr(2, 6), def->toString());
                }
            });
            ss << "</tr>\n"; // row end
            i++;
        }
    }

    X86Instruction* findDefVirt(Block* block, X86Instruction* start, VHAND virtId) {
        for (auto inst: Block::RevIter{start->prev}) {
            if (inst->hasDefVirt(virtId)) return inst;
        }
        return nullptr;
    }

    void markLiveVirt(Block* b, X86Instruction* start, X86Instruction* endIncl, VHAND virtId) {
        if (!virtualRanges[b].contains(virtId)) {
            virtualRanges[b][virtId] = RangeSet{};
        }

        virtualRanges[b][virtId].add(Range{start, endIncl});
    }

    void findDefVirtRec(Block* block, X86Instruction* foundUse, VHAND virtId, std::set<Block*>& visited) {
        if (visited.contains(block)) return;
        visited.insert(block);

        if (foundUse == nullptr) foundUse = block->last;

        auto foundDef = findDefVirt(block, foundUse, virtId);
        if (foundDef != nullptr) {
            markLiveVirt(block, foundDef, foundUse, virtId);
        } else {
            println("DID NOT FIND DEF FOR {}", virtId->toString());
            markLiveVirt(block, block->first, foundUse, virtId); // implicitly mark live range, search in incoming

            for (auto inc: block->incoming) {
                findDefVirtRec(inc, nullptr, virtId, visited);
            }
        }

        visited.erase(block);
    }

    void calcLiveRanges() {
        this->virtualRanges.clear();
        for (auto block: blocks) {
            for (auto inst: block->iterator()) {
                inst->forEachUse([&](auto use) {
                    std::set<Block*> visited;
                    findDefVirtRec(block, inst, use, visited);
                });

                inst->forEachKill([&](auto kill) {
                    markLiveVirt(block, inst, inst, kill);
                });
            }
        }
        printLiveRange();
    }

    std::set<VHAND> getVirtIds() {
        std::set<VHAND> res;

        for (const auto& [block, ranges]: virtualRanges) {
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
    void spill(BaseRegister* regId) {
        auto stackSlot = allocStackSlot(8, 8);

        for (auto block: this->blocks) {
            for (auto inst: block->iterator()) {
                if (inst->hasDefVirt(regId)) {
                    auto tmp = allocaVirtReg();
                    block->insertAfter(inst, new STORESTACK(tmp, stackSlot, 0));
                    inst->rewriteDef(regId, tmp);
                }
                if (inst->hasUseVirt(regId)) {
                    auto tmp = allocaVirtReg();
                    block->insertBefore(inst, new LOADSTACK(tmp, stackSlot, 0, 8));
                    inst->rewriteUse(regId, tmp);
                }
                assert(!inst->hasDefVirt(regId));
                assert(!inst->hasUseVirt(regId));
            }
        }
    }

    GraphColoring buildAllocatorGraph() {
        auto virtIds = getVirtIds();

        std::map<VHAND, std::set<VHAND>> virtToVirt;

        for (auto block: blocks) {
            for (auto [regA, rangeA]: virtualRanges[block]) {
                for (auto [regB, rangeB]: virtualRanges[block]) {
                    if (rangeA.intersects(rangeB)) {
                        virtToVirt[regA].insert(regB);
                        virtToVirt[regB].insert(regA);
                    }
                }
            }
        }

        GraphColoring gc = GraphColoring::create(logger, virtToVirt);
        for (auto it : regMap) {
            gc.colorReg(it.second, it.first);
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

    void emitGVBlock2(GVEmit& e, Block* b, size_t id) {
        e.appendLine("b{} [", id);

        e.ident([&] {
            e.appendLine(R"(xlabel="B{}", label = "{)", id);

            for (auto inst: b->iterator()) {
                std::stringstream ss;
                printBlockInst(inst, ss);
                e.appendLine("<i{}>{}\\l", inst->orderId, ss.str());

                if (inst->next != nullptr) e.appendLine("|");
            }
            e.appendLine("}\";");
        });

        e.appendLine("]");
    }

    void emitGV2(GVEmit& e) {
        e.appendLine("digraph G {");
        e.appendLine("nodesep=2;");
        e.appendLine("splines=ortho;");
        e.appendLine("node [shape=Mrecord, nojustify=false, fontname=\"monospace\", fontsize=66];");
        e.appendLine("START -> b{}:i{} [tailport=s, headport=n];", root->id, root->first->orderId);

        e.ident([&] {
            // blocks
     /*       for (auto item: blocks) {
                emitGVBlock2(e, item, item->id);
            }*/

            // block edges
            for (auto src: blocks) {
                for (auto dst: src->outgoing) {
                    e.appendLine("b{}:i{} -> b{}:i{} [tailport=s, headport=n, minlen=5];", src->id, src->last->orderId, dst->id, dst->first->orderId);
                }
            }

            // virt regs
/*            std::optional<VHAND> lastId;
            e.appendLine("subgraph {");
            for (auto block: blocks) {
                for (auto [regId, range]: virtualRanges[block]) {
                    e.appendLine("v{} [shape=oval]", regId->toString());
                    auto color = (range.first()->hasDefVirt(regId)) ? "green"sv : "red"sv;
                    auto xd = false;
                    e.appendLine("v{} -> b{}:i{} [color={}, constraint={}, concentrate=false];", regId->toString(), block->id, range.first()->orderId, color, (xd ? "true" : "false"));
                    e.appendLine("v{} -> b{}:i{} [color=blue, constraint={}, concentrate=false];", regId->toString(), block->id, range.last()->orderId, (xd ? "true" : "false"));
                    e.appendLine("v{} -> b{} [style=invis, constraint=true, concentrate=false];", regId->toString(), block->id);
                    if (lastId.has_value()) {
                        e.appendLine("v{} -> v{} [style=invis];", (*lastId)->toString(), regId->toString());
                    }
                    lastId = regId;
                }
            }*/
            // e.appendLine("}");
        });

        for (auto block: blocks) {
            e.ss << stringify("  b{} [shape=none label=<<TABLE cellpadding=\"24\"  style='rounded'>", block->id);
            printLiveRangeGraph(e.ss, block);
            e.ss << "</TABLE>>];";
        }

/*        for (auto block: blocks) {
            for (auto inst : block->iterator()) {
                e.appendLine("b{}:i{} -> bb{}:i{};", block->id, inst->orderId, block->id, inst->orderId);
                e.appendLine("bb{}:i{} -> b{}:i{};", block->id, inst->orderId, block->id, inst->orderId);
            }
        }*/

        e.appendLine("}");
    }

    Block* makeBlock() {
        auto b = new Block();
        b->id = this->blocks.size();
        b->graph = this;
        blocks.push_back(b);

        return b;
    }

    std::vector<Block*> blocks;
    Block* root = nullptr;

    size_t virtCounter = 0;
    std::vector<size_t> stackSizes;
    std::shared_ptr<Logger> logger;

    std::vector<size_t> linearized;

    // live ranges
    std::map<Block*, std::map<VHAND, RangeSet>> virtualRanges;

    std::string tag;
};