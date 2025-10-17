#pragma once

#include <memory>
#include <utility>
#include <vector>
#include <map>
#include <stack>
#include <ranges>

#include "IRInstruction.h"
#include "../utils/utils.h"
#include "../utils/CopyPtr.h"
#include "BlockId.h"

using namespace std;

template<typename CTX>
class CodeBlock: VirtualCopy<CodeBlock<CTX>> {
public:
    CodeBlock* deepCopy() const override {
        return new CodeBlock(*this);
    }

    // private copy constructor to prevent unwanted/unnecessary copies, use deepCopy
private:
    CodeBlock(const CodeBlock&) = default;
public:
    CodeBlock(CodeBlock&&) = default;

    virtual ~CodeBlock() = default;

    [[nodiscard]] const vector<CopyPtr<IRInstruction<CTX>>>& getInstructions() const { return instructions; }

    [[nodiscard]] vector<CopyPtr<IRInstruction<CTX>>>& getInstructionsMut() { return instructions; }

    // [[nodiscard]] optional<size_t> previousId() const { return previous; }

    bool isTerminated() {
        return !isEmpty() && getInstruction(-1)->isTerminal();
    }

    template<typename T, typename ...Args>
    T* pushInstruction(Args&&... args) {
        assert(isEmpty() || !getInstruction(-1)->isTerminal());
        auto instruction = createInstruction<T>(std::forward<Args>(args)...);
        auto ptr = instruction.get();
        push(std::move(instruction));

        return ptr;
    }

    void forEach(std::function<void(IRInstruction<CTX>&)> body) {
        for (auto& node : instructions) {
            body(*node);
        }
    }

    BlockId id() const {
        return blockId;
    }

    static bool canPutInstruction(CodeBlock* block) {
        if (block == nullptr) return false;
        if (block->isEmpty()) return true;

        if (block->getInstruction(-1)->isTerminal()) return false;

        return true;
    }

    bool mIsLoopHeader = false;
    string tag;
    BlockId blockId;
    vector<CopyPtr<IRInstruction<CTX>>> instructions{};

    std::string toString(CTX::IRGEN& gen) {
        std::stringstream stream;
        for (auto& inst : getInstructions()) {
            inst->print(gen, stream);
            stream << "\\l";
        }
        return stream.str();
    }

    CodeBlock clone() const {
        return *this;
    }

    CopyPtr<IRInstruction<CTX>>& getInstruction(int index) {
        if (index >= (int)instructions.size()) assert(false);
        if (index < 0 && (int)instructions.size()+index < 0) assert(false);

        return instructions[index < 0 ? instructions.size()+index : index];
    }

    const CopyPtr<IRInstruction<CTX>>& getInstructionConst(int index) const {
        if (index >= static_cast<int>(instructions.size())) assert(false);
        if (index < 0 && static_cast<int>(instructions.size())+index < 0) assert(false);

        return instructions[index < 0 ? instructions.size()+index : index];
    }

/// returns reg map in format:
/// ancestor to child
/// eg.: 0:3 to 0:0, 0:2 to 0:0

    map<SSARegisterHandle, instructions::PhiFunction<CTX>*> getPhiFunctions() const {
        map<SSARegisterHandle, instructions::PhiFunction<CTX>*> pis;

      vector<instructions::PhiFunction<CTX>*> phiSpan = getPhiSpan();
        for (size_t i = 0; i < phiSpan.size(); i++) {
          instructions::PhiFunction<CTX>* phi = phiSpan[i];
            for (SSARegisterHandle version : phi->getVersions()) {
                pis[version] = phi;
            }
        }

        return pis;
    }

    vector<BlockId> getTargets() const {
        if (instructions.empty()) return {};

        return getInstructionConst(-1)->branchTargets();
    }

    vector<BlockId*> getTargetsPtr() const {
        if (instructions.empty()) return {};

        return getInstructionConst(-1)->branchTargetsPtr();
    }

    void patchJumps(BlockId old, BlockId neww) {
        for (auto tgt : getTargetsPtr()) {
            if (*tgt == old) {
                *tgt = neww;
            }
        }
    }

    void patchPhis(BlockId oldBlock, std::set<BlockId> newbies) {
        for (instructions::PhiFunction<CTX>* phi : getPhiSpan()) {
            auto vv = phi->getBySource(oldBlock);
            if (vv.has_value()) {
                for (auto newb : newbies) {
                    phi->pushVersion(*vv, newb);
                }
                phi->remove(oldBlock);
            }
        }
    }

    vector<instructions::PhiFunction<CTX>*> getPhiSpan() const {
        if (isEmpty())
            return {};

        vector<instructions::PhiFunction<CTX>*> phis;

        for (auto& inst : instructions) {
            if (inst->template is<instructions::PhiFunction<CTX>>()) {
                phis.push_back(inst->template cst<instructions::PhiFunction<CTX>>());
            } else if (inst->template is<instructions::NoOp<CTX>>()) {
                continue;
            }
            else {
                break;
            }
        }

        return phis;
    }

    map<SSARegisterHandle, SSARegisterHandle> getPhis(BlockId source) const {
        map<SSARegisterHandle, SSARegisterHandle> pis;

        for (auto phi : getPhiSpan()) {
            auto value = phi->getBySource(source);
            if (value.has_value()) {
                pis[*value] = phi->target;
            }
        }

        return pis;
    }

    size_t phiCount() const {
        return getPhiSpan().size();
    }

    span<CopyPtr<IRInstruction<CTX>>> instructionsWithoutPhis() {
        return {instructions.data()+phiCount(), instructions.size()-phiCount()};
    }

    void removeInstruction(const CopyPtr<IRInstruction<CTX>>& inst) {
        erase_if(instructions, [&](auto& it) { return it.get() == inst.get(); });
    }

    void removeInstruction(const IRInstruction<CTX>* inst) {
        erase_if(instructions, [&](auto& it) { return it.get() == inst; });
    }

    void removeInstruction(SSARegisterHandle tgt) {
        erase_if(instructions, [&](auto& it) { return it->target == tgt; });
    }

    SSARegisterHandle getHandle(const CTX::REG& reg) const {
        return SSARegisterHandle::valid(blockId, reg.id);
    }

    bool isEmpty() const {
        return instructions.empty();
    }

    CodeBlock(string tag, BlockId blockId) : tag(std::move(tag)), blockId(blockId) {

    }

    size_t instructionCount() const {
        return instructions.size();
    }

    bool isLoopHeader() const {
        return mIsLoopHeader;
    }

    void setLoopHeader(bool isHeader) {
        mIsLoopHeader = isHeader;
    }

    [[nodiscard]] vector<SSARegisterHandle> getArgRegisters() const {
        vector<SSARegisterHandle> buf;

        for (const auto& inst : getInstructions()) {
            if (const auto& arg = inst->template cst<instructions::Arg<CTX>>(); arg) {
                buf.push_back(arg->target);
            }
        }

        return buf;
    }

    void push(unique_ptr<IRInstruction<CTX>> instruction) {
        instructions.emplace_back(std::move(instruction));
    }

    size_t indexOf(IRInstruction<CTX>* instruction) const {
        for (const auto& [i, inst] : instructions | views::enumerate) {
            if (inst.get() == instruction) return i;
        }
        PANIC()
    }

    size_t indexOf(SSARegisterHandle handle) const {
        for (const auto& [i, inst] : instructions | views::enumerate) {
            if (inst->target == handle) return i;
        }
        PANIC()
    }

    void insert(unique_ptr<IRInstruction<CTX>> instruction, size_t id) {
        instructions.insert(instructions.begin()+id, std::move(instruction));
    }

    void replace(unique_ptr<IRInstruction<CTX>> instruction, size_t id) {
        instructions[id] = std::move(instruction);
    }

    void remove(size_t id) {
        instructions.erase(instructions.begin()+id);
    }

    template<typename T, typename ...Args>
    void insertInst(size_t id, Args&&... args) {
        instructions.emplace(instructions.begin()+id, createInstruction<T>(std::forward<Args>(args)...));
    }

    template<template<typename> typename T, typename ...Args>
    void insertInst(size_t id, Args&&... args) {
        instructions.emplace(instructions.begin()+id, createInstruction<T<CTX>>(std::forward<Args>(args)...));
    }

    template<typename T, typename ...Args>
    unique_ptr<T> createInstruction(Args&&... args) {
        return make_unique<T>(std::forward<Args>(args)...);
    }
};