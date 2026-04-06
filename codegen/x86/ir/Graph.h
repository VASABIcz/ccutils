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
    BaseRegister* getReg(x86::X64Register r) {
        if (!regMap.contains(r)) {
            regMap[r] = allocaVirtReg();
            regMap[r]->hintReg = r;
        }

        return regMap[r];
    }

    Graph(std::shared_ptr<Logger> logger) : logger(logger) {
        root = makeBlock();
        // why do this? ... causing issues with save/restore calee save regs, commenting won't fix but ... 
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

    std::pair<std::map<size_t, size_t>, size_t> stackOffsets() const {
        std::map<size_t, size_t> offsets;
        size_t acu = 0;
        for (auto [i, size]: stackSizes | views::enumerate) {
            offsets[i] = acu;
            acu += size;
        }

        return {offsets, acu};
    }

GraphColoring registerAllocate(const std::function<void(size_t)>& onFailedAlloc = [](auto) {}) {
        size_t spillCounter = 0;

        GraphColoring gc;

        while (true) {
            this->calcLiveRanges();

            gc = this->buildAllocatorGraph();

            auto res = gc.regAllocFast();
            allocated = gc.allocated;
            if (not res.has_value()) break;
            logger->DEBUG("[graph-color] FAILED to color {}", (*res)->toString());

            auto toSpill = gc.getInterferenceMaxNeighbour(*res);

            onFailedAlloc(spillCounter);

            logger->DEBUG("[graph-color] spilling {}", toSpill->toString());
            this->spill(toSpill);

            spillCounter += 1;
        }

        // prune coloring ... this is kind of a hack?

        return gc;
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
            if (inst->defCount() == 0 || inst->hasSideEffect()) return false;

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
            
            for (auto [def, set] : virtualRanges[block]) {
                auto s = set.getStart(inst);
                if (s.has_value()) {
                    auto color = stringify("{:x}", ((uint32_t)hashInt(def->as<Virtual>()->id+31))>>8).substr(2, 6);
                    std::string ss1;
                    if (allocated.contains(def)) {
                        ss1 = allocated.at(def).toString();
                    } else {
                        ss1 = def->toString();
                    }
                    ss << stringify("<td align=\"left\" rowspan=\"{}\" bgcolor=\"#{}\">{}</td>", s->getLength(), color, ss1);
                }
            }
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
            println("DID NOT FIND DEF FOR {} IN {}", virtId->toString(), block->id);
            markLiveVirt(block, block->first, foundUse, virtId); // implicitly mark live range, search in incoming

            for (auto inc: block->incoming) {
                findDefVirtRec(inc, nullptr, virtId, visited);
            }
        }
    }

    std::set<VHAND> getAliveHands() {
        std::set<VHAND> aliveHands;
        for (auto [block, _it] : virtualRanges) {
            for (auto [hand, range] : _it) {
                aliveHands.insert(hand);
            }
        }

        return aliveHands;
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
                    block->insertAfter<STORESTACK>(inst, tmp, stackSlot, 0);
                    inst->rewriteDef(regId, tmp);
                }
                if (inst->hasUseVirt(regId)) {
                    auto tmp = allocaVirtReg();
                    block->insertBefore<LOADSTACK>(inst, tmp, stackSlot, 0, 8);
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

        std::set<VHAND> phys;
        for (auto block: blocks) {
            for (auto [regA, rangeA]: virtualRanges[block]) {
                if (regA->hintReg.has_value()) phys.insert(regA);
                for (auto [regB, rangeB]: virtualRanges[block]) {
                    if (rangeA.intersects(rangeB)) {
                        virtToVirt[regA].insert(regB);
                        virtToVirt[regB].insert(regA);
                    }
                }
            }
        }

        GraphColoring gc = GraphColoring::create(logger, virtToVirt);
        for (auto hand : phys) {
            gc.colorReg(hand, *hand->hintReg);
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

    void dumpGraph(std::string path) {
        GVEmit e;
        emitGV2(e);

        writeBytesToFile(path, e.ss.str());
    }

    void emitGV2(GVEmit& e) {
        e.appendLine("digraph G {");
        e.appendLine("nodesep=2;");
        e.appendLine("splines=ortho;");
        e.appendLine("node [shape=Mrecord, nojustify=false, fontname=\"monospace\", fontsize=66];");
        assert(root != nullptr);
        assert(root->first != nullptr);
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
            e.ss << stringify("  b{} [shape=none xlabel=\"block-{}\" label=<<TABLE cellpadding=\"24\"  style='rounded'>", block->id, block->id);
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

    void insertPrologueEpilogue(std::set<x86::X64Register> regs) {
        std::map<x86::X64Register, StackSlot> ss;
        for (auto reg: regs) {
            auto saveType = x86::sysVSave(reg);
            if (saveType != x86::X64Register::SaveType::Callee) continue;

            ss[reg] = allocStackSlot(8, 8);
            root->insertBefore<STORESTACK>(root->first, getReg(reg), ss[reg], 0);
        }

        for (auto block: blocks) {
            for (auto inst: block->iterator()) {
                if (inst->is<RET>()) {
                    for (auto reg: regs) {
                        auto saveType = x86::sysVSave(reg);
                        if (saveType != x86::X64Register::SaveType::Callee) continue;

                        block->insertBefore<LOADSTACK>(inst, getReg(reg), ss[reg], 0, 8);
                    }
                }
            }
        }
    }

    std::map<x86::X64Register, BaseRegister*> regMap;
    std::vector<Block*> blocks;
    Block* root = nullptr;

    size_t virtCounter = 0;
    std::vector<size_t> stackSizes;
    std::shared_ptr<Logger> logger;

    // live ranges
    std::map<Block*, std::map<VHAND, RangeSet>> virtualRanges;
    std::map<VHAND, x86::X64Register> allocated;

    std::string tag;
};