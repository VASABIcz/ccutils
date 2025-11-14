#pragma once
#include <ranges>
#include <unordered_set>

#include "CodeBlock.h"
#include "../utils/Errorable.h"
#include "LiveRanges.h"
#include "BlockId.h"

template<typename CTX>
struct ControlFlowGraph {
    typedef CodeBlock<CTX> BaseBlock;
    typedef unordered_map<size_t, unordered_set<size_t>> DependencyMap;

    ControlFlowGraph() = default;

    void print() {
        println("CFG dump for {}", tag);
        this->printRegisters();
        for (auto& block : this->validNodes()) {
            const auto & instructions = block->getInstructions();
            println("{}", block->tag);
            if (instructions.empty()) {
                println("  _ := TERMINAL");
            }
            for (auto& instruction : instructions) {
                cout << "  ";
                instruction->print(*static_cast<CTX::IRGEN*>(nullptr));
            }
        }
    }

    std::vector<BlockId> getTargets(BlockId id) {
        return getBlock(id).getTargets();
    }

    enum COLOR {
        BLACK,
        GREY
    };

    // src: https://eli.thegreenplace.net/2015/directed-graph-traversal-orderings-and-applications-to-data-flow-analysis/
    void discoverBackEdgesRec(BlockId node, std::map<BlockId, COLOR>& color, std::map<BlockId, std::set<BlockId>>& res) {
        color[node] = GREY;
        for (auto succ : getTargets(node)) {
            if (color.contains(succ) && color.at(succ) == GREY) {
                res[node].insert(succ);
            }
            if (not color.contains(succ)) {
                discoverBackEdgesRec(succ, color, res);
            }
        }
        color[node] = BLACK;
    }

    std::map<BlockId, std::set<BlockId>> discoverBackEdges() {
        std::map<BlockId, COLOR> color;
        std::map<BlockId, std::set<BlockId>> res;

        discoverBackEdgesRec(root().id(), color, res);

        return res;
    }


    void filteredDFSRec(BlockId node, std::set<BlockId>& visited, std::vector<BlockId>& res, const std::map<BlockId, std::set<BlockId>>& ignore) {
        if (visited.contains(node)) return;

        visited.insert(node);

        for (auto succ : getTargets(node)) {
            // skip back edges
            if (ignore.contains(node) && ignore.at(node).contains(succ)) continue;
            filteredDFSRec(succ, visited, res, ignore);
        }
        res.push_back(node);
    }

    std::vector<BlockId> getPostOrder(BlockId node, const std::map<BlockId, std::set<BlockId>>& ignore) {
        std::set<BlockId> visited;
        std::vector<BlockId> res;
        filteredDFSRec(node, visited, res, ignore);

        return res;
    }

    std::vector<BlockId> getReversePostOrder() {
        std::map<BlockId, std::set<BlockId>> idk;
        auto postOrder = getPostOrder(root().id(), idk);

        std::reverse(postOrder.begin(), postOrder.end());

        return postOrder;
    }

    BaseBlock& getBlock(BlockId blockId) {
        if (not nodes.contains(blockId)) PANIC();
        return *nodes.at(blockId);
    }

    [[nodiscard]] const BaseBlock& getBlockConst(BlockId blockId) const {
        if (not nodes.contains(blockId)) PANIC();
        return *nodes.at(blockId);
    }

    void removeBlock(BlockId blockId) {
        if (not nodes.contains(blockId)) PANIC();
        nodes.erase(blockId);
    }

    auto validNodes() {
        return this->nodes | views::values;
    }

    [[nodiscard]] auto validNodesConst() const {
        return this->nodes | views::values;
    }

    [[nodiscard]] size_t nodeCount() const {
        return nodes.size();
    }

    void destroy(BlockId blockId) {
        println("destroying block: {}", blockId);
        (void)nodes[blockId].uPtrRef().reset(nullptr);
    }

    template<typename T, typename... Argz>
    void patchInst(SSARegisterHandle target, Argz... argz) {
        for (auto& block : this->validNodesConst()) {
            for (auto& inst : block->getInstructionsMut()) {
                if (inst->target == target) {
                    inst.uPtrRef() = make_unique<T>(target, argz...);
                }
            }
        }
    }

    template<typename T, typename... Argz>
    void patchInst(IRInstruction<CTX>* target, Argz... argz) {
        for (auto& block : this->validNodesConst()) {
            for (auto& inst : block->getInstructionsMut()) {
                if (inst.get() == target) {
                    inst.uPtrRef() = make_unique<T>(argz...);
                }
            }
        }
    }

    template<template<typename>typename T, typename... Argz>
    void patchInst(IRInstruction<CTX>* target, Argz... argz) {
        for (auto& block : this->validNodesConst()) {
            for (auto& inst : block->getInstructionsMut()) {
                if (inst.get() == target) {
                    inst.uPtrRef() = make_unique<T<CTX>>(argz...);
                }
            }
        }
    }

    template<template<typename> typename T, typename... Argz>
    void patchInst(SSARegisterHandle target, Argz... argz) {
        patchInst<T<CTX>>(target, argz...);
    }

    void dumpSVG(string_view name) const {
        string buf = "digraph Test {\n";
        for (const auto& node : validNodesConst()) {
            for (auto tgt : node->getTargets())
                buf += stringify("{} -> {}\n", node->blockId, tgt);
        }
        buf += "}";

        auto res = system(stringify("echo \"{}\" | dot -Tsvg > {}", buf, name).c_str());
        if (res) println("[gen] dumping exited with {}", res);
    }

    /// tries to look up the original non-shadowed variable
    optional<SSARegisterHandle> lookupRootLocal(BaseBlock& c, string_view name) {
        // current reg, root reg

        optional<SSARegisterHandle> best;
        map<SSARegisterHandle, SSARegisterHandle> modernMap;
        BaseBlock* cur = &c;

        while (true) {
            auto reg = cur->getRegisterHandleByName(name);

            if (reg.has_value()) {
                auto root = getRoot(*reg).getHandle();

                if (not modernMap.contains(root)) {
                    modernMap[root] = *reg;
                }

                if (!best.has_value() || best != root) {
                    best = root;
                }
            }


            if (not cur->previousId().has_value()) break;
            cur = &getBlock(*cur->previousId());
        }

        if (best.has_value()) return modernMap[*best];

        return {};
    }

    pair<SSARegisterHandle, instructions::PhiFunction<CTX>*> makePhi(BaseBlock& current, span<pair<SSARegisterHandle, size_t>> regs) {
        auto temp = generateNewVersion(regs.begin()->first, current);

        auto& tempRec = getRecord(temp);
        assert(tempRec.getHandle() == temp);

        for (std::weakly_incrementable auto _i : views::iota(0u, regs.size())) {
            (void)_i;
            tempRec.incUseCount();
        }

        for (const auto& reg: regs) {
            getRecord(reg.first).incUseCount();
        }

        auto ptr = current.template pushInstruction<instructions::PhiFunction>(temp, regs);

        return {temp, ptr};
    }

    SSARegisterHandle generateNewVersion(SSARegisterHandle previousHandle, BaseBlock& block) {
        assert(previousHandle.isValid());

        auto& previous = getRecord(previousHandle);
        auto cpy = previous.copy();
        cpy.setPrevious(previousHandle);

        return pushRegister(make_unique<typename CTX::REG>(cpy));
    }

    CTX::REG& getRoot(SSARegisterHandle target) {
        assert(target.isValid());
        auto* current = &getRecord(target);

        while (!current->isRoot()) {
            auto t1 = current->getPrevious();
            assert(t1.has_value());
            current = &getRecord(*t1);
        }

        return *current;
    }

    CTX::REG& getRecord(SSARegisterHandle target) {
        assert(target.isValid());

        auto& reg = getRecord(target.registerId);

        assert(reg.getHandle() == target);

        return reg;
    }

    size_t blockIdCounter = 0;
    BlockId allocBlockId() {
        return BlockId::raw(blockIdCounter++);
    }

    BaseBlock& createBlock(string tag) {
        auto id = allocBlockId();
        return *(nodes.emplace(id, make_unique<BaseBlock>(std::move(tag), id)).first->second);
    }

    void printRegisters() {
        for (const auto& reg : getRegistersConst()) {
            auto root = getRoot(reg->getHandle()).getHandle();
            println("id: {} - name: {} - parent: {} - type: {} - size: {}", reg->getHandle().toString(), reg->name, root == reg->getHandle() ? "_" : root.toString(), reg->typeString(), reg->sizeBytes());
        }
    }

    const CTX::REG& getRecordConst(SSARegisterHandle target) const {
        assert(target.isValid());

        return getRecordConst(target.registerId);
    }

    const CTX::REG& getRootConst(SSARegisterHandle target) const {
        assert(target.isValid());
        const auto* current = &getRecordConst(target.registerId);

        while (!current->isRoot()) {
            auto t1 = current->getPrevious();
            assert(t1.has_value());
            current = &getRecordConst(t1->registerId);
        }

        return *current;
    }

    Result<void> validate() {
        for (auto& block: validNodesConst()) {
            bool endOfPhis = false;
            bool encounteredTerminator = false;

            for (auto& inst: block->getInstructions()) {
                if (inst->template is<instructions::PhiFunction<CTX>>() && !endOfPhis) {
                    continue;
                }
                if (inst->isTerminal() && encounteredTerminator) {
                    return FAIL("multiple terminal instructions");
                }
                if (inst->isTerminal() && !encounteredTerminator) {
                    encounteredTerminator = true;
                    continue;
                }
                if (encounteredTerminator) {
                    return FAIL("instructions after terminator");
                }
                endOfPhis = true;
            }
        }

        return {};
    }

    void fixupUseCount() {
        for (auto& block : validNodes()) {
            for (auto& inst : block->getInstructionsMut()) {
                if (not inst->target.isValid()) continue;
                auto& reg = getRecord(inst->target);

                auto* phiPtr = dynamic_cast<instructions::PhiFunction<CTX>*>(inst.get());
                if (phiPtr != nullptr) {
                    reg.useCount = (calculateUseCount(inst->target) + phiPtr->versionsCount())-1;
                }
                else {
                    reg.useCount = calculateUseCount(inst->target);
                }
            }
        }
    }

    ControlFlowGraph::DependencyMap buildDependencyMap() const {
        DependencyMap buf;

        for (auto& block : validNodesConst()) {
            for (auto target : block->getTargets()) {
                buf[target].insert(block->blockId);
            }
        }

        return buf;
    }

    ControlFlowGraph::DependencyMap buildDependencyMapDeep() const {
        DependencyMap buf;

        for (auto& block : validNodesConst()) {
            for (auto target : block->getTargets()) {
                buf[target].insert(block->blockId);
                auto& src = buf[block->blockId];
                auto& wtf = buf[target];
                wtf.insert(src.begin(), src.end());
            }
        }

        return buf;
    }

    size_t calculateUseCount(SSARegisterHandle reg) {
        size_t useCount = 0;

        for (auto& block : validNodes()) {
            for (auto& inst : block->getInstructionsMut()) {
                inst->visitSrc([&](auto usedReg) {
                    if (usedReg == reg) {
                        useCount++;
                    }
                });
            }
        }

        return useCount;
    }

    std::map<SSARegisterHandle, size_t> calcUseCount() {
        std::map<SSARegisterHandle, size_t> useCount;

        forEachInstruction([&](IRInstruction<CTX>& instruction) {
            useCount[instruction.target] = 1;
            for (auto src : instruction.getSources()) {
                useCount[src] += 1;
            }
        });
        return useCount;
    }

    map<BlockId, set<BlockId>> inverseTreeP() const {
        map<BlockId, set<BlockId>> res;
        for (const auto& node : validNodesConst()) {
            for (auto tgt : node.get()->getTargets()) {
                res[tgt].insert(node.get()->blockId);
            }
        }

        return res;
    }

    optional<size_t> indexOfLast(const vector<BlockId>& pepa, const auto& pred) const {
        // we could iterate in reverse :)
        optional<size_t> n;
        for (const auto& [i, item] : pepa | views::enumerate) {
            if (pred(item)) n = i;
        }
        return n;
    }

    void removeInstruction(SSARegisterHandle tgt) {
        for (auto& [id, block] : this->nodes) {
            block->removeInstruction(tgt);
        }
    }

    vector<BlockId> flattenBlocks() const {
        vector<BlockId> toProcessStack;
        vector<BlockId> result;
        set<BlockId> resolved;
        auto inverseTree = inverseTreeP();
        vector<BlockId> priorityStack;

        toProcessStack.push_back(BlockId::raw(0));

        const auto visit = [&](BlockId n) {
            if (resolved.contains(n)) return;
            toProcessStack.push_back(n);
        };
        auto graph = *this;

        const auto canReach = [&](BlockId src, BlockId dst) -> bool {
            stack<BlockId> toVisit;
            set<BlockId> visited;
            toVisit.push(src);

            while (not toVisit.empty()) {
                auto cur = toVisit.top(); toVisit.pop();
                if (cur == dst) return true;
                if (resolved.contains(cur) || visited.contains(cur)) continue;
                visited.insert(cur);
                for (auto child : graph.getBlock(cur).getTargets()) {
                    toVisit.push(child);
                }
            }
            return false;
        };

        auto unresolvedIncomingEdges = [&](BlockId node) -> vector<BlockId> { return inverseTree[node] | views::filter([&](const auto& it) { return !resolved.contains(it); }) | vi::toVec; };
        auto isAllIncomingResolved = [&](BlockId node) -> bool { return unresolvedIncomingEdges(node).empty(); };

        const auto removePriorityIfResolve = [&]() {
            if (priorityStack.empty()) return;
            priorityStack.erase(std::remove_if(priorityStack.begin(), priorityStack.end(), [&](const auto& it) { return isAllIncomingResolved(it); }), priorityStack.end());
        };

        const auto markResolved = [&](BlockId n) {
            result.push_back(n);
            resolved.insert(n);
            removePriorityIfResolve();
        };

        while (not toProcessStack.empty()) {
            optional<size_t> res;
            if (not priorityStack.empty()) {
                res = indexOfLast(toProcessStack, [&](const auto& it) { return canReach(it, priorityStack.back()) && isAllIncomingResolved(it); });
            }
            if (not res.has_value()) {
                res = indexOfLast(toProcessStack, [&](const auto& it) { return !isAllIncomingResolved(it) && canReach(it, unresolvedIncomingEdges(it)[0]); });
            }
            if (not res.has_value()) {
                res = indexOfLast(toProcessStack, [&](const auto& it) { return isAllIncomingResolved(it); });
            }

            BlockId top = BlockId::invalid();
            if (res.has_value()) {
                top = toProcessStack[*res];
                toProcessStack.erase(toProcessStack.begin()+*res);
            }
            else {
                top = toProcessStack.back(); toProcessStack.pop_back();
            }

            if (resolved.contains(top)) continue;
            markResolved(top);

            auto children = graph.getBlock(top).getTargets();

            for (auto child : children) {
                visit(child);
            }

            if (!isAllIncomingResolved(top)) {
                if (children.empty()) TODO();
                priorityStack.push_back(top);
            }
        }

        return result;
    }

    vector<BlockId> lookupIngress(BlockId id) const {
        vector<BlockId> buf;

        for (const auto& block : validNodesConst()) {
            for (auto tgt : block->getTargets()) {
                if (tgt == id) buf.push_back(block->id());
            }
        }

        return buf;
    }

    LiveRanges simpleLiveRanges(vector<BlockId> blocks) {
        map<SSARegisterHandle, vector<bool>> ranges;
        auto [instructionCount, instructionRangeOffset] = blockOffsets(blocks);

        const auto render = [&](SSARegisterHandle reg, size_t start, size_t end) {
            if (not ranges.contains(reg)) {
                ranges[reg] = std::vector<bool>();
                ranges[reg].resize(instructionCount, false);
            }

            for (auto i = start; i <= end; i++) {
                ranges[reg][i] = true;
            }
        };

        map<SSARegisterHandle, size_t> regStarts;

        forEachInstruction([&](IRInstruction<CTX>& inst, CodeBlock<CTX>& block, auto n) {
            auto idex = instructionRangeOffset[block.id()]+n;
            for (auto reg : inst.allValidRegs()) {
                if (not regStarts.contains(reg)) {
                    regStarts[reg] = idex;
                    render(reg, idex, idex);
                } else {
                    auto [start, end] = minmax(regStarts[reg], idex);
                    render(reg, start, end);
                }
            }
        });

        LiveRanges l{ranges};

        for (auto b : blocks) {
            if (getBlock(b).isLoopHeader()) {
                auto bb = getLastBlockOfLoop(b, blocks);
                if (not bb.has_value()) continue;

                auto start = instructionRangeOffset[b];
                auto end = (instructionRangeOffset[*bb]+ getBlock(*bb).instructions.size())-1;

                for (auto ins : l.intersections({start, end})) {
                    l.appendRange(ins, start, end, true);
                }
            }
        }

        return l;
    }

    std::pair<size_t, std::map<BlockId, size_t>> blockOffsets(const std::vector<BlockId>& blocks) const {
        size_t instructionCount = 0;
        map<BlockId, size_t> instructionRangeOffset;

        for (auto block : blocks) {
            instructionRangeOffset[block] = instructionCount;

            auto instCount = getBlockConst(block).instructions.size();
            instructionCount += instCount;
        }

        return {instructionCount, instructionRangeOffset};
    }

    std::optional<BlockId> getLastBlockOfLoop(BlockId blockId, std::span<BlockId> blocks) const {
        long headerIndex = indexOf(blocks, blockId);

        auto ingressDist = collectVec(lookupIngress(blockId) | views::transform([&](auto it) { return make_pair(it, (long)indexOf(blocks, it)-headerIndex); }));
        assert(not ingressDist.empty());

        auto max = pair{ingressDist.front().first, ingressDist.front().second};
        for (auto [ingId, ingOffset] : ingressDist) {
            if (ingOffset >= max.second) max = {ingId, ingOffset}; // it can happend that this returns the inbound block that isnt actualy part of the loop
        }

        if (max.second < 0) return {};

        return max.first;
    };


    map<SSARegisterHandle, vector<bool>> liveRanges(vector<BlockId> blocks) const {
        map<SSARegisterHandle, vector<bool>> ranges;
        map<SSARegisterHandle, size_t> registerStarts;

        map<size_t, set<SSARegisterHandle>> liveIns;

        auto [instructionCount, instructionRangeOffset] = blockOffsets(blocks);

        // NOTE bypasssStart ignores start index of register needed for correct loop handling
        auto addRange = [&](SSARegisterHandle handle, size_t s, size_t e, bool bypassStart = false) {
            // println("AAAAAAAAAAAAAAAAaa addRange {} - {}..{}", handle, s, e);
            assert(handle.isValid());
            auto& range = ranges[handle];

            if (range.empty()) {
                range.insert(range.end(), instructionCount, false);
            }

            auto start = (registerStarts.contains(handle) && !bypassStart) ? max(min(s, e), registerStarts[handle]) : min(s, e);
            auto end = min(max(s, e)+1, range.size());

            if (end < start) return;

            for (auto id : views::iota(start, end)) {
                range[id] = true;
            }
        };

        auto getLiveIns = [&](size_t blockId) -> set<SSARegisterHandle> {
            return liveIns[blockId];
        };

        auto setLiveIns = [&](size_t blockId, set<SSARegisterHandle> regs1) {
            liveIns[blockId] = std::move(regs1);
        };

        auto getBlockPhis = [&](size_t blockId) {
            return getBlockConst(blockId).getPhiSpan();
        };

        auto getBlockRanges = [&](BlockId blockId) -> pair<size_t, size_t> {
            auto start = instructionRangeOffset[blockId];
            auto instructionCount = getBlockConst(blockId).instructionCount();
            // assert(instructionCount > 0);
            // it theoretically should make sense bcs views::iota iterates by (start...<end) and we add +1 at the place
            // but start + instructionCount gives index 1 larger than possible
            // so we should output (start..end) not (start..end+1)
            auto end = start+(instructionCount-1); // FIXME not sure about this -1

            return {start, end};
        };

        auto getBlock = [&](size_t blockId) -> const BaseBlock& {
            return getBlockConst(blockId);
        };

        // NOTE this indeed screws handling for loops, but it's fixed by forced fixup by is bypassStart
        auto setFrom = [&](SSARegisterHandle reg, size_t instId) {
            registerStarts[reg] = instId;
            if (not ranges.contains(reg)) return;
            auto& range = ranges[reg];
            for (auto id : views::iota(0u, instId)) {
                range[id] = false;
            }
        };

        auto isLoopHeader = [&](size_t blockId) -> bool {
            return getBlock(blockId).isLoopHeader();
        };

        auto getBlockSuccesors = [&](size_t blockId) -> vector<size_t> {
            return getBlock(blockId).getTargets();
        };

        // for each block b in reverse order do
        for (auto blockId : blocks | views::reverse) {
            auto currentSuccesors = getBlockSuccesors(blockId);
            auto blockRange = getBlockRanges(blockId);

            // live = union of successor.liveIn for each successor of b
            auto live = set<SSARegisterHandle>();
            for (auto succesorId: currentSuccesors) {
                auto liveIns1 = getLiveIns(succesorId);
                live.insert(liveIns1.begin(), liveIns1.end());
            }

            // for each phi function phi of successors of b do
            //     live.add(phi.inputOf(b))
            for (auto succesorId: currentSuccesors) {
                for (auto* phi: getBlockPhis(succesorId)) {
                    auto res = phi->getBySource(blockId);
                    if (res.has_value()) {
                        live.insert(*res);
                    }
                }
            }

            // for each opd in live do
            //    intervals[opd].addRange(b.from, b.to)
            for (auto reg: live) {
                addRange(reg, blockRange.first, blockRange.second);
            }

            // for each operation op of b in reverse order do
            for (auto [index, operation] : getBlock(blockId).instructions | views::enumerate | views::reverse) {
                // FIXME patch to make even unused registers alive
                if (operation->target.isValid()) {
                    addRange(operation->target, blockRange.first+index, blockRange.first+index);
                }
                // "Phi functions are not processed during this iteration of operations, instead they are iterated separately"
                if (operation->template is<instructions::PhiFunction<CTX>>()) continue;

                //    for each output operand opd of op do
                //       intervals[opd].setFrom(op.id)
                //       live.remove(opd)
                auto tgt = operation->target;
                if (tgt.isValid()) {
                    setFrom(tgt, blockRange.first+index);
                }
                live.erase(tgt);

                //    for each input operand opd of op do
                //       intervals[opd].addRange(b.from, op.id)
                //       live.add(opd)
                operation->visitSrcs([&](auto& source) {
                    if (!source.isValid()) return; // return can have invalid src
                    addRange(source, blockRange.first, blockRange.first+index);
                    live.insert(source);
                });

                // for each phi function phi of b do
                //    live.remove(phi.output)
                for (auto* phi: getBlockPhis(blockId)) {
                    live.erase(phi->target);
                }

                // println("LIVE INS FOR {} is {}", blockId, live);

                // if b is loop header then
                if (isLoopHeader(blockId)) {
                    //      loopEnd = last block of the loop starting at b
                    //      for each opd in live do
                    //          intervals[opd].addRange(b.from, loopEnd.to)
                    auto loopEnd = getLastBlockOfLoop(blockId, blocks);
                    // println("[LIVE] end for {} is {}", blockId, loopEnd);
                    auto loopEndEnd = getBlockRanges(loopEnd).second;
                    for (auto reg : live) {
                        // println("[LIVE] is {}", reg);
                        addRange(reg, blockRange.first, loopEndEnd, true);
                    }
                }

                // b.liveIn = live
                setLiveIns(blockId, live);
            }
        }

        return ranges;
    }

    void forEachInstruction(std::function<void(IRInstruction<CTX>&, CodeBlock<CTX>&)> body) {
        for (auto& block : this->validNodesConst()) {
            for (auto& inst : block->getInstructionsMut()) {
                body(*inst, *block);
            }
        }
    }

    void forEachInstruction(std::function<void(IRInstruction<CTX>&, CodeBlock<CTX>&, size_t n)> body) {
        for (auto& block : this->validNodesConst()) {
            size_t n = 0;
            for (auto& inst : block->getInstructionsMut()) {
                body(*inst, *block, n);
                n += 1;
            }
        }
    }

    void forEachInstruction(std::function<void(IRInstruction<CTX>&)> body) {
        for (auto& block : this->validNodesConst()) {
            for (auto& inst : block->getInstructionsMut()) {
                body(*inst);
            }
        }
    }

    void forEachBlock(std::function<void(CodeBlock<CTX>&)> body) {
        for (auto& block : this->validNodesConst()) {
            body(*block);
        }
    }

    IRInstruction<CTX>* resolveInstruction(SSARegisterHandle handle) {
            IRInstruction<CTX>* res = nullptr;
            forEachInstruction([&](auto& inst, auto&) {
                if (inst.target == handle) res = &inst;
            });
            return res;
    }

    map<SSARegisterHandle, SSARegisterHandle> reachableDefinitions(const BaseBlock& block) const {
        map<SSARegisterHandle, SSARegisterHandle> oldes;

        const BaseBlock* currentBlock = &block;
        while (currentBlock != nullptr) {
            // reverse used bcs we want to register the newest register version witch is at the end, THIS ASSUMPTION DOESNT HAVE TO HOLD TRUE
            // 1. FIXME this doesnt correctly represent the version and we should iterate over the instructions in the block and get their targets
            // 2. this whole function depends on block.previous which is kinda outdating and propably inacurate representation
            // we should do block.getPredecessors and iterate until we get to root ignoring already visited
            // if we encounter IF/ELSE it wont actualy affect us bcs if they modify regs then phi function will be generated after them
            for (const auto& regHandle : currentBlock->getInstructions() | views::transform([&](const CopyPtr<IRInstruction<CTX>>& it) { return it->target; }) | views::filter([](const SSARegisterHandle& it) { return it.isValid(); }) | views::reverse) {
                auto& reg = getRecordConst(regHandle.registerId);
                if (reg.isTemp()) continue;

                if (reg.getPrevious().has_value()) {
                    const auto root = getRootConst(*reg.getPrevious());

                    // NOTE: we are iterating from the youngest to the oldest if we already encountered younger version we shouldn't replace it with older none
                    // + reg.version doesn't seem to be working
                    if (oldes.contains(root.getHandle())) continue;

                    oldes[root.getHandle()] = regHandle;
                }
                else {
                    if (!oldes.contains(regHandle)) {
                        oldes[regHandle] = regHandle;
                    }
                }
            }

            currentBlock = currentBlock->previousId().has_value() ? &getBlockConst(*currentBlock->previousId()) : nullptr;
        }

        return oldes;
    }

    optional<SSARegisterHandle> lookupOptionalLocal(string_view name) {
        auto res = this->getRegisterByName(name);
        if (not res.has_value()) return {};
        return (*res)->getHandle();
    }

    optional<SSARegisterHandle> getRegisterHandleByName(string_view name) {
        auto reg = getRegisterByName(name);
        if (reg.has_value()) return (*reg)->getHandle();

        return {};
    }

    SSARegisterHandle pushRegister(unique_ptr<typename CTX::REG> reg) {
        reg->id = registers.size();

        auto hand = reg->getHandle();

        registers.push_back(std::move(reg));

        return hand;
    }

    SSARegisterHandle pushRegister(CopyPtr<typename CTX::REG> reg) {
        reg->id = registers.size();

        auto hand = reg->getHandle();

        registers.push_back(std::move(reg));

        return hand;
    }

    SSARegisterHandle allocateDummy(size_t size = 8) {
        return pushRegister(std::make_unique<typename CTX::REG>(CTX::REG::makeDummy(size)));
    }

    span<CopyPtr<typename CTX::REG>> getRegisters() {
        return registers;
    }

    BaseBlock* getBlock(IRInstruction<CTX>* inst1) {
        for (auto& block : this->validNodesConst()) {
            for (auto& inst : block->getInstructionsMut()) {
                if (inst.get() == inst1) return block;
            }
        }
    }

    using InstHandle = std::pair<BlockId, size_t>;

    InstHandle getInstPos(IRInstruction<CTX>* inst1) {
        for (auto& block : this->validNodesConst()) {
            size_t i = 0;
            for (auto& inst : block->getInstructionsMut()) {
                if (inst.get() == inst1) return {block->id(), i};
                i += 1;
            }
        }
        PANIC()
    }

    void removeInstruction(InstHandle h) {
        getBlock(h.first).remove(h.second);
    }

    void insertInstruction(InstHandle h, std::unique_ptr<IRInstruction<CTX>> inst, bool replace) {
        if (replace) {
            getBlock(h.first).replace(std::move(inst), h.second);
        } else {
            getBlock(h.first).insert(std::move(inst), h.second);
        }
    }

    template<typename INST, typename... Args>
    void insertInstruction(InstHandle h, Args... args) {
        insertInstruction(h, std::make_unique<INST>(std::forward<Args>(args)...), false);
    }

    template<typename INST, typename... Args>
    void replaceInstruction(InstHandle h, Args... args) {
        insertInstruction(h, std::make_unique<INST>(std::forward<Args>(args)...), true);
    }

    optional<typename CTX::REG*> getRegisterByName(string_view name) {
        for (auto& reg : views::reverse(registers)) {
            if (reg->name == name) {
                return reg.get();
            }
        }

        return {};
    }

    const vector<CopyPtr<typename CTX::REG>>& getRegistersConst() const {
        return registers;
    }

    const CTX::REG& getRecordConst(size_t id) const {
        assertInBounds(registers, id);
        return *registers[id];
    }

    CTX::REG& getRecord(size_t id) {
        assertInBounds(registers, id);
        return *registers[id];
    }

    static void intersect(std::set<size_t>& self, const std::set<size_t>& other) {
        std::vector<size_t> toRemove;

        for (auto s : self) {
            if (other.contains(s)) continue;
            toRemove.push_back(s);
        }

        for (auto rem : toRemove) {
            self.erase(rem);
        }
    }

    void checkConsistency() {
        std::map<SSARegisterHandle, bool> isAssigned; // isAssigned


        forEachInstruction([&](IRInstruction<CTX>& inst, CodeBlock<CTX>& body) {
            isAssigned[inst.target] = true;

            for (auto tgt : body.getTargets()) {
                if (not nodes.contains(tgt)) PANIC("invalid block {}", tgt);
            }

            for (auto src : inst.getSources()) {
                if (isAssigned[src]) continue;
            }

            isAssigned[inst.target] = true;
        });

        for (auto [reg, isA] : isAssigned) {
            if (not isA) PANIC("reg {} is not assigned", reg);
        }
    }

    void genPhiRec(BlockId reg, const std::set<SSARegisterHandle>& badBoys, std::map<SSARegisterHandle, SSARegisterHandle> reachable, std::set<BlockId>& visited) {
        if (visited.contains(reg)) return;
        visited.insert(reg);

        getBlock(reg).forEach([&](IRInstruction<CTX>& inst) {
            if (not inst.template is<instructions::PhiFunction>()) {
                inst.visitSrcs([&](SSARegisterHandle& hand) {
                    if (badBoys.contains(hand) && reachable.contains(hand)) {
                        hand = reachable[hand];
                    }
                });
            }

            auto oldTgt = inst.target;
            if (badBoys.contains(oldTgt)) {
                auto nH = pushRegister(std::make_unique<typename CTX::REG>(getRecord(inst.target).copy()));
                inst.target = nH;
                reachable[oldTgt] = nH;
            }
        });

        for (auto tgt : getBlock(reg).getTargets()) {
            for (instructions::PhiFunction<CTX>* phi : getBlock(tgt).getPhiSpan()) {
                auto ver = UNWRAP(phi->getBySource(reg));
                if (reachable.contains(ver)) {
                    phi->pushVersion(reachable[ver], reg);
                } else {
                    assert(badBoys.contains(ver));
                    phi->remove(reg);
                }
            }
        }

        for (auto tgt : getBlock(reg).getTargets()) {
            auto s = reachable;
            genPhiRec(tgt, badBoys, reachable, visited);
            assert(s == reachable);
        }
    }

    void genPhis() {
        auto preds = predecesors();
        auto badBoys = getBadBoys();

        forEachBlock([&](BaseBlock& block) {
            if (preds[block.id()].size() > 1) {
                for (auto bad : badBoys) {
                    std::map<BlockId, SSARegisterHandle> imps;

                    for (auto pred : preds[block.id()]) {
                        imps[pred] = bad;
                    }

                    block.template insertInst<instructions::PhiFunction>(0, bad, imps);
                }
            }
        });

        std::set<BlockId> visited;

        std::map<SSARegisterHandle, SSARegisterHandle> reachable;
        genPhiRec(root().id(), badBoys, reachable, visited);
    }

    BaseBlock& root() {
        return *this->nodes.at(BlockId::raw(0));
    }

    static std::map<size_t, std::set<size_t>> calcDom(std::set<size_t> nodes, std::function<std::vector<size_t>(size_t)> getPredecessors) {
        bool didChange = true;
        std::map<size_t, std::set<size_t>> res;

        while (didChange) {
            didChange = false;
            for (auto vertex : nodes) {
                auto preds = getPredecessors(vertex);
                auto base = res[preds[0]];
                for (auto pred : preds | views::drop(1)) {
                    intersect(base, res[pred]);
                }
                base.insert(vertex);
                if (base != res[vertex]) {
                    res[vertex] = base;
                    didChange = true;
                }
            }
        }

        return res;
    }

    static std::map<size_t, std::set<size_t>> doStuff(size_t root, std::function<std::vector<size_t>(size_t)> succesors) {
        std::map<size_t, std::set<size_t>> res;

        // doStuff(root, res, {}, succesors);

        return res;
    }

    std::map<BlockId, std::set<BlockId>> predecesors() {
        std::map<BlockId, std::set<BlockId>> preds;
        forEachBlock([&](BaseBlock& block) {
            for (auto tgt : block.getTargets()) {
                preds[tgt].insert(block.id());
            }
        });

        return preds;
    }

    std::map<size_t, std::set<size_t>> domMomies() const {
        std::map<size_t, std::set<size_t>> momies;

        // doStuff(nodes[0]->id(), momies, {}, [&](size_t id) { return getBlockConst(id).getTargets(); });

        return momies;
    }


    std::set<SSARegisterHandle> getBadBoys() {
        std::set<SSARegisterHandle> found;
        std::set<SSARegisterHandle> naughty;

        forEachInstruction([&](IRInstruction<CTX>& inst) {
            for (auto dst : inst.validDests()) {
                if (found.contains(dst)) {
                    naughty.insert(dst);
                } else {
                    found.insert(dst);
                }
            }
        });

        return naughty;
    }

    std::string tag;
private:
    std::map<BlockId, CopyPtr<BaseBlock>> nodes;
    vector<CopyPtr<typename CTX::REG>> registers{};
};