#pragma once
#include "forward.h"
#include "IRGen.h"
#include "CodeBlock.h"
#include "../utils/CopyPtr.h"
#include "IRInstructions.h"

template <typename SELF, typename CTX>
struct IRGenCtx {
    typedef CodeBlock<CTX> BaseBlock;
    
    typedef pair<SSARegisterHandle, CodeBlock<CTX>*> ValueBlockPair;
    CTX::IRGEN& gen;
    BaseBlock* currentBlock;
    BaseBlock* nextBlock;
    optional<size_t> loopBegin;
    optional<size_t> loopEnd;
    // NOTE: mby make it return CodeBlock* + mby pass IrGenCtx&
    function<Result<void>(SELF*, string_view, size_t, SSARegisterHandle)> hookAssign;
    function<Result<SSARegisterHandle>(SELF*, string_view, size_t)> hookRead;
    SSARegisterHandle stateReg;

    [[nodiscard]] CodeBlock<CTX>& current() const {
        assert(currentBlock != nullptr);
        return *currentBlock;
    }

    static bool canOmmit(vector<pair<SSARegisterHandle, size_t>>& regs) {
        if (regs.empty()) return true;

        if (regs.size() == 1) return false;

        auto reg = regs[0].first;

        for (auto i : views::iota(1u, regs.size())) {
            if (regs[i].first != reg) return false;
        }

        return true;
    }

    /**
     * nullptr is valid block AKA ignored
     * */
    void dumpPhis1(span<const CodeBlock<CTX>*> inbound) {
        map<SSARegisterHandle, vector<pair<SSARegisterHandle, size_t>>> toBuildPhis;

        for (auto incomingBlock: inbound) {
            if (incomingBlock == nullptr) continue;
            auto currenReachable = gen.graph.reachableDefinitions(*incomingBlock);

            for (auto [key, value] : currenReachable) {
                toBuildPhis[key].emplace_back(value, incomingBlock->blockId);
            }
        }

        for (auto& phi : toBuildPhis) {
            if (canOmmit(phi.second)) continue;

            makePhi(phi.second);
        }
    }

    void dumpPhis(std::initializer_list<const CodeBlock<CTX>*> inbound) {
        std::vector<const CodeBlock<CTX>*> pepa(inbound);
        dumpPhis1(pepa);
    }

    bool isTerminated() {
        return isNullOrTerminated(currentBlock);
    }

    template<typename T, typename ...Args>
    T* pushInstruction(Args&&... args) {
        assert(currentBlock != nullptr);
        return current().template pushInstruction<T>(std::forward<Args>(args)...);
    }

    template<template<typename> typename T, typename ...Args>
    T<CTX>* pushInstruction(Args&&... args) {
        assert(currentBlock != nullptr);
        return current().template pushInstruction<T<CTX>>(std::forward<Args>(args)...);
    }

    template<typename T, typename ...Args>
    T* pushNoRet(Args&&... args) {
        return current().template pushInstruction<T>(SSARegisterHandle::invalid(), std::forward<Args>(args)...);
    }

    template<template<typename> typename T, typename ...Args>
    T<CTX>* pushNoRet(Args&&... args) {
        return current().template pushInstruction<T<CTX>>(SSARegisterHandle::invalid(), std::forward<Args>(args)...);
    }

    IRGenCtx(const IRGenCtx& other): gen(other.gen), currentBlock(other.currentBlock), nextBlock(other.nextBlock), loopBegin(other.loopBegin), loopEnd(other.loopEnd), hookAssign(other.hookAssign), hookRead(other.hookRead), stateReg(other.stateReg) {}

    IRGenCtx(CTX::IRGEN& gen, CodeBlock<CTX>* currentBlock, BaseBlock* nextBlock, optional<size_t> begin, optional<size_t> end) : gen(gen), currentBlock(currentBlock),
                                                                                                                                  nextBlock(nextBlock), loopBegin(begin), loopEnd(end) {}

    [[nodiscard]] SELF withNext(BaseBlock* next) const {
        auto cpy = getSelf();
        cpy.nextBlock = next;

        return cpy;
    }

    [[nodiscard]] SELF withBlock(BaseBlock* current) const {
        auto cpy = getSelf();
        cpy.currentBlock = current;

        return cpy;
    }

    [[nodiscard]] SELF withLoop(size_t begin, size_t end) const {
        auto cpy = getSelf();
        cpy.loopBegin = begin;
        cpy.loopEnd = end;

        return cpy;
    }

    /// creates phi function in endBlock for each reachable variable of current
    map<SSARegisterHandle, instructions::PhiFunction<CTX>*> createPhis(BaseBlock& endBlock) const {
        auto phis = map<SSARegisterHandle, instructions::PhiFunction<CTX>*>();

        auto reachableDefinitions = gen.graph.reachableDefinitions(current());
        for (const auto& [root, child] : reachableDefinitions) {
            array<pair<SSARegisterHandle, size_t>, 1> idk{pair<SSARegisterHandle, size_t>{child, currentBlock->blockId}};
            auto idk1 = withBlock(&endBlock).makePhi(idk);
            phis[root] = idk1.second;
        }

        return phis;
    }

    void commitReachableDefs(map<SSARegisterHandle, instructions::PhiFunction<CTX>*> phis, const BaseBlock& endBlock) const {
        gen.graph.commitReachableDefs(std::move(phis), endBlock);
    }

    void commitReachableDefs(map<SSARegisterHandle, instructions::PhiFunction<CTX>*> phis) const {
        gen.graph.commitReachableDefs(std::move(phis), current());
    }

    [[nodiscard]] bool hasNext() const { return nextBlock != nullptr; }

    IRGenCtx::ValueBlockPair makeRet(SSARegisterHandle handle) {
        return {handle, currentBlock};
    }

    IRGenCtx& operator=(const IRGenCtx& other) {
        this->loopBegin = other.loopBegin;
        this->loopEnd = other.loopEnd;
        this->currentBlock = other.currentBlock;
        this->nextBlock = other.nextBlock;

        return *this;
    }

    pair<SSARegisterHandle, instructions::PhiFunction<CTX>*> makePhi(span<pair<SSARegisterHandle, size_t>> regs) const {
        for (auto idk : regs) {
            assert(idk.first.isValid());
        }
        auto temp = this->gen.generateNewVersion(regs.begin()->first, current());

        auto& tempRec = gen.getRecord(temp);
        assert(tempRec.getHandle() == temp);

        for (std::weakly_incrementable auto _i : views::iota(0u, regs.size())) {
            (void)_i;
            tempRec.incUseCount();
        }

        for (const auto& reg: regs) {
            gen.getRecord(reg.first).incUseCount();
        }

        auto ptr = current().template pushInstruction<instructions::PhiFunction<CTX>>(temp, regs);

        return {temp, ptr};
    }

    [[nodiscard]] Result<size_t> getLoopBegin() const {
        if (!loopBegin.has_value()) return FAIL("continue used outside loop");

        return *loopBegin;
    }

    [[nodiscard]] Result<size_t> getLoopEnd() const {
        if (!loopEnd.has_value()) return FAIL("break used outside loop");

        return *loopEnd;
    }

    Result<SSARegisterHandle> lookupLocal(string_view name) {
        return gen.lookupLocal(name, current());
    }

    Result<SSARegisterHandle> readLocal(string_view name) {
        TODO();
    }

    CodeBlock<CTX>* createBlock(string_view name) {
        return &this->gen.createBlock(name, current());
    }

    CodeBlock<CTX>* createBlock(string_view name, BaseBlock* b) {
        return &this->gen.createBlock(name, b);
    }

    SELF getSelf() const {
        return *static_cast<const SELF*>(this);
    }

    static bool isNullOrTerminated(BaseBlock* bb) {
        if (bb == nullptr) return true;
        if (bb->isTerminated()) return true;

        return false;
    }

    Result<BaseBlock*> makeIf(
            std::function<Result<pair<SSARegisterHandle, BaseBlock*>>(SELF)> cond,
            std::function<Result<BaseBlock*>(SELF)> body,
            std::function<Result<BaseBlock*>(SELF)> els
    ) {
        auto oof = TRY(cond(getSelf()));
        SSARegisterHandle reg = oof.first;
        auto ctx1 = withBlock(oof.second);

        auto ifBlock = ctx1.createBlock("if-body");
        auto elsBlock = ctx1.createBlock("else-body");

        BaseBlock* ifBlockEnd = TRY(body(ctx1.withBlock(ifBlock)));
        BaseBlock* elsBlockEnd = TRY(els(ctx1.withBlock(elsBlock)));

        ctx1.template pushInstruction<instructions::Branch<CTX>>(reg, ifBlock->id(), elsBlock->id());

        BaseBlock* nextBlock1 = nullptr;
        if (not isNullOrTerminated(ifBlockEnd) || not isNullOrTerminated(elsBlockEnd)) {
            nextBlock1 = this->createBlock("if-next");
        }
        if (not isNullOrTerminated(ifBlockEnd)) {
            ifBlockEnd->template pushInstruction<instructions::Jump<CTX>>(nextBlock1->id());
        }
        if (not isNullOrTerminated(elsBlockEnd)) {
            elsBlockEnd->template pushInstruction<instructions::Jump<CTX>>(nextBlock1->id());
        }
        if (not isNullOrTerminated(nextBlock1)) {
            this->withBlock(nextBlock1).dumpPhis({ifBlockEnd, elsBlockEnd});
        }
        return nextBlock1;
    }

    Result<BaseBlock*> makeIf(
            std::function<Result<pair<SSARegisterHandle, BaseBlock*>>(SELF)> cond,
            std::function<Result<BaseBlock*>(SELF)> body
    ) {
        auto oof = TRY(cond(getSelf()));
        SSARegisterHandle reg = oof.first;
        SELF ctx1 = withBlock(oof.second);

        auto ifBlock = ctx1.createBlock("if-body");
        auto nextBlock1 = this->createBlock("next-body");

        BaseBlock* ifBlockEnd = TRY(body(ctx1.withBlock(ifBlock)));

        ctx1.template pushInstruction<instructions::Branch<CTX>>(reg, ifBlock->id(), nextBlock1->id());

        if (not isNullOrTerminated(ifBlockEnd)) {
            ifBlockEnd->template pushInstruction<instructions::Jump<CTX>>(nextBlock1->id());
        }

        this->withBlock(nextBlock1).dumpPhis({ifBlockEnd, ctx1.currentBlock});
        return nextBlock1;
    }

    Result<BaseBlock*> makeLoop(
        optional<string_view> label,
        Result<std::function<Result<BaseBlock*>(SELF)>> body
    ) {
        auto& startBlock = createBlock("loop-body", current());
        auto& nextBlock1 = createBlock("loop-next", current());
        startBlock.setLoopHeader(true);

        pushInstruction<instructions::Jump<CTX>>(startBlock->id());
        // FIXME not sure what are we doing here
        auto phis = this->createPhis(startBlock);

        auto idk = withBlock(startBlock).withLoop(startBlock->id(), nextBlock1->id());
        BaseBlock* bodyEndBlock = TRY(body(idk));
        if (bodyEndBlock != nullptr) {
            withBlock(bodyEndBlock)->commitReachableDefs(phis);
            bodyEndBlock->template pushInstruction<instructions::Jump<CTX>>(startBlock->id());
        }

        return nextBlock1;
    }

    Result<BaseBlock*> makeWhile(
            optional<string_view> label,
            std::function<Result<pair<SSARegisterHandle, BaseBlock*>>(SELF)> cond,
            std::function<Result<BaseBlock*>(SELF)> body
    ) {
        auto nextBlock1 = makeLoop(label, [&](SELF ctx1) -> Result<BaseBlock*> {
            return ctx1.makeIf([&](SELF ctx2) -> Result<pair<SSARegisterHandle, BaseBlock*>> {
                return cond(ctx2);
            }, [&](SELF ctx3) -> Result<BaseBlock*> {
                return body(ctx3);
            }, [&](SELF ctx3) -> Result<BaseBlock*> {
                return ctx3.makeBreak();
            });
        });

        return nextBlock1;
    }

    Result<BaseBlock*> makeContinue(optional<string_view> label) {
        assert(loopBegin.has_value());
        auto& block = this->getBlock(*loopBegin);
        auto phis = block.getPhiFunctions();
        this->commitReachableDefs(phis);
        this->pushInstruction<instructions::Jump<CTX>>(*loopBegin);

        return nullptr;
    }

    BaseBlock& getBlock(size_t id) {
        return gen.getBlock(id);
    }

    Result<BaseBlock*> makeBreak() {
        assert(loopEnd.has_value());
        auto& block = this->getBlock(*loopEnd);
        auto phis = block.getPhiFunctions();
        this->commitReachableDefs(phis);
        this->pushInstruction<instructions::Jump>(*loopEnd);

        return nullptr;
    }
};