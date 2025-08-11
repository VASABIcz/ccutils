#pragma once

#include <memory>
#include <map>
#include <ranges>
#include <cstring>

#include "../Assembler.h"
#include "../../gen64/definitions.h"
#include "../../gen64/X86mc.h"
#include "RegAlloc.h"

using namespace std;

// TODO
// lazy register allocation
// writes will not be executed until required
// instruction can treat the value as immediate
// TODO stack allocation
// values larger than reg SIZE store on stack, if not said otherwise

Result<void> linkRelative(u8* data, const map<string , vector<Label>>& missing, const map<string, Label>& labels);

struct BoundLabel {
    size_t offset;
    size_t type;
    size_t id;
};

enum class BaseType {
    ABSOLUTE_8,
    ABSOLUTE_4,
    ABSOLUTE_2,
    ABSOLUTE_1,
    RIP_REL_4,
    RIP_REL_2,
    RIP_REL_1,
};

inline size_t labelTypeToSize(BaseType type) {
    switch (type) {
        case BaseType::ABSOLUTE_8: return 8;
        case BaseType::ABSOLUTE_4: return 4;
        case BaseType::ABSOLUTE_2: return 2;
        case BaseType::ABSOLUTE_1: return 1;
        case BaseType::RIP_REL_4: return 4;
        case BaseType::RIP_REL_2: return 2;
        case BaseType::RIP_REL_1: return 1;
    }
    PANIC();
}

struct SlotLabel {
    size_t type;
    size_t offset;
    size_t id;
    BaseType baseType;
    long adend;
};

class X86Assembler: virtual public Assembler {
public:
    RegAlloc allocator;
    vector<u8> bytes;
    // map<string, Label> labels;
    size_t labelId = 10;
    size_t labelTypeId = 10;
    // map<string, vector<Label>> spaces;
    std::map<size_t, BoundLabel> boundLabels;
    std::map<size_t, std::vector<SlotLabel>> slotLabels;
    X86mc mc;
    vector<size_t> argSizes;
    size_t retSize;
    vector<size_t> argHandles;

    size_t currentOffset() {
        return bytes.size();
    }

    void bindRawLabel(size_t id, size_t type) {
        assert(not boundLabels.contains(id));

        boundLabels[id] = BoundLabel{currentOffset(), type, id};
    }

    void bindHint(std::string_view h) override {}

    void bindJmp(size_t id) {
        bindRawLabel(id, LABEL_TYPE_JMP);
    }

    void bindReturn(size_t id) {
        bindRawLabel(id, LABEL_TYPE_RETURN);
    }

    size_t allocateLabel() {
        return labelId++;
    }

    size_t allocateLabelType() {
        return labelTypeId++;
    }

    void requestLabel(size_t id, ImmSpace space, size_t type, BaseType baseType) {
        assert(labelTypeToSize(baseType) == space.size);
        slotLabels[id].push_back(SlotLabel{type, space.offset, id, baseType, 0});
    }

    void requestLabelRel4(size_t id, ImmSpace space, size_t type, long adend) {
        assert(labelTypeToSize(BaseType::RIP_REL_4) == space.size);
        slotLabels[id].push_back(SlotLabel{type, space.offset, id, BaseType::RIP_REL_4, adend});
    }

    void requestJmpLabel(size_t id, ImmSpace space) {
        requestLabel(id, space, LABEL_TYPE_JMP, BaseType::RIP_REL_4);
    }

    void requestStackSize(size_t id, ImmSpace space) {
        requestLabel(id, space, LABEL_TYPE_STACK_SIZE, BaseType::ABSOLUTE_4);
    }

    static constexpr size_t REG_SIZE = 8;
    static constexpr size_t STACK_ALIGNMENT = 16;
    static constexpr string STACK_SIZE_LABEL = "_STACK_SIZE_";

    static constexpr size_t LABEL_TYPE_JMP = 0;
    static constexpr size_t LABEL_TYPE_RETURN = 1;
    static constexpr size_t LABEL_TYPE_STACK_SIZE = 2;
    static constexpr size_t LABEL_TYPE_DEBUG_HINT = 3;
    static constexpr size_t LABEL_STACK_BEGIN = 4;
    static constexpr size_t LABEL_SYMBOL = 5;

    X86Assembler(span<size_t> argSizes, size_t retSize);

    void initializeSYSV();

    void initializeFastCall();

    std::string toString(size_t handle) override {
        if (allocator.isStack(handle)) {
            return stringify("rsp+{}", allocator.getStackOffset(handle));
        }
        return allocator.getReg(handle).toString();
    }

    static unique_ptr<Assembler> create(span<size_t> argSizes, size_t retSize);

    vector<RegisterHandle> getArgHandles() override;
    void print() const override;

    void allocRegSpilling();

    // int operations
    void divInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) override;
    void modInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) override;
    void shlInt(Assembler::RegisterHandle dest, Assembler::RegisterHandle left, Assembler::RegisterHandle right) override;
    void shrInt(Assembler::RegisterHandle dest, Assembler::RegisterHandle left, Assembler::RegisterHandle right) override;
    void arithmeticInt(BinaryOp op, Assembler::RegisterHandle tgt, Assembler::RegisterHandle lhs, Assembler::RegisterHandle rhs) override;

    // float operations
    void arithmeticFloat(BinaryOp op, Assembler::FloatingPointType type, Assembler::RegisterHandle tgt, Assembler::RegisterHandle lhs, Assembler::RegisterHandle rhs) override;

    // float int conversion
    void f64ToInt(RegisterHandle dest, RegisterHandle value) override;
    void int2f64(RegisterHandle dest, RegisterHandle value) override;

    /*void numericCast(Assembler::RegisterHandle dst, Assembler::RegisterHandle src, NumericType dstType, NumericType srcType) {
        if (srcType.isSigned() && dstType.isFloating()) {
            auto srcSize = allocator.sizeOf(src);
            assertStableRegs([&](const x86::X64Register& srcReg, const x86::X64Register& dstReg) {
                if (srcSize == 8) {
                    mc.movsx8(srcReg, srcReg);
                }
                if (srcSize == 16) {
                    mc.movsx16(srcReg, srcReg);
                }
                if (srcSize == 32) {
                    mc.movsx32(srcReg, srcReg);
                }

                if (dstType.type == NumericType::F64) {
                    mc.i64ToF64(dstReg, srcReg);
                } else {
                    mc.i64ToF32(dstReg, srcReg);
                }

                // write back result
                movRegToHandle(dst, dstReg);
            }, src, dst);

            return;
        }
        if (srcType.isUnsigned() && dstType.isFloating()) {
            TODO(); // FIXME this one looks kinda hard idk how to do it right (eg: godbolt)
        }
    }*/

    // memory primitives
    void readMem(RegisterHandle tgt, RegisterHandle obj, size_t offset, size_t amount) override;
    void writeMem(RegisterHandle obj, RegisterHandle value, size_t offset, size_t amount) override;
    void addressOf(Assembler::RegisterHandle tgt, Assembler::RegisterHandle obj) override;

    CmpType toCmpType(JumpCondType it);
    CmpType toCmpType2(JumpCondType it);

    // register primitives
    void movInt(Assembler::RegisterHandle dest, u64 value, bool isSigned, size_t offsetBytes = 0) override;
    void movReg(Assembler::RegisterHandle dest, Assembler::RegisterHandle src, size_t destOffsetBytes = 0, size_t srcOffsetBytes = 0, size_t amount = 8) override;

    // flow control functions
    void nop() override;
    void jmp(size_t label) override;
    void createLabel(size_t name) override;
    void jmpLabelTrue(RegisterHandle cond, size_t label) override;
    void jmpLabelFalse(RegisterHandle cond, size_t label) override;
    void generateRet() override;
    void generateRet(RegisterHandle value) override;

    // register allocation
    RegisterHandle allocateRegister(size_t size) override;
    void freeRegister(Assembler::RegisterHandle rt) override;

    // complex functions, which boil down to builtin function invocation
    // void objectGetter(Assembler::RegisterHandle target, Assembler::RegisterHandle object, size_t fieldId) override;

    void garbageMemCpy(x86::X64Register ptrReg, size_t stackOffset, size_t stackSize);

    void garbageMemCpy(x86::X64Register dst, size_t dstOffset, x86::X64Register src, size_t srcOffset, size_t stackSize);

    size_t allocateJmpLabel() override {
        return this->labelId++;
    }

    // void invokeBuiltin(LinkSymbol symbol, span<const Assembler::RegisterHandle> args, optional<Assembler::RegisterHandle> ret) override;

    RegisterHandle allocateStack(size_t size) override {
        return allocator.allocateStack(size);
    }

    void signExtend(Assembler::RegisterHandle dst, Assembler::RegisterHandle src) override;

    void f32ToF64(Assembler::RegisterHandle dest, Assembler::RegisterHandle value) override;

    void f64ToF32(Assembler::RegisterHandle dest, Assembler::RegisterHandle value) override;

    // constexpr static string RETURN_LABEL = "return";
    size_t returnCounter = 0;

    string nextReturnLabel();

    // beginning of a block (IF0, ELSE1, SWITCH3, FOR7)
    // void makeLabel(string_view name, Label::Type type, int size = 8, size_t data = 0);

/*    void makeAbsolute(string_view name, int size) {
        makeLabel(name, Label::Type::ABSOLUTE1, size);
    }*/

    // void putRelative(string_view name, ImmSpace space);

    // void putAbsolute(string_view name, ImmSpace space);

    // void putLabel(string_view name, ImmSpace space, Label::Type type, size_t data = 0);

    void withSavedCallRegs(span<const x86::X64Register> exclude, span<const x86::X64Register> excluceRestore, const std::function<x86::X64Register::SaveType(const x86::X64Register&)>& convention, const std::function<void(const map<x86::X64Register, size_t>&)>& callback);


    void callC(Arg label, span<const RegisterHandle> args, optional<RegisterHandle> ret) {
        vector<Arg> argz;

        for (auto [i, arg] : args | views::enumerate) {
            argz.push_back(handleToArg(arg));
        }
        optional<Arg> idk;
        if (ret.has_value()) {
            idk = handleToArg(*ret);
        }

        chadCall(label, argz, idk);
    }

    void writeJmp(CmpType jmp, size_t id);

    void writeJmp(size_t id);

    void withSpecificReg(const x86::X64Register& reg, const function<void()>& callback);

    size_t preserveCalleeRegs(const std::function<x86::X64Register::SaveType(x86::X64Register)>& save);

    void forEachLabel(std::function<void(SlotLabel&)> funk) {
        for (auto& slots : this->slotLabels) {
            for (auto& slot : slots.second) {
                funk(slot);
            }
        }
    }

    void forEachBound(std::function<void(BoundLabel&)> funk) {
        for (auto& slots : this->boundLabels) {
            funk(slots.second);
        }
    }

    BoundLabel getUniqueBoundLabel(size_t type) {
        for (auto slots : this->boundLabels) {
            if (slots.second.type == type) return slots.second;
        }
        PANIC();
    }

    BoundLabel getBoundLabelById(size_t id){
        assert(boundLabels.contains(id));
        return boundLabels[id];
    }

    void insertBytes(std::span<u8> otherBytes, size_t offset) {
        for (auto& bound : this->boundLabels) {
            // not >= bcs, if we want to (for example) insert callee restore registers we want the new instructions to be the lable? ... we don't want to move the label
            if (bound.second.offset > offset) {
                bound.second.offset += otherBytes.size();
            }
        }

        for (auto& slots : this->slotLabels) {
            for (auto& idk : slots.second) {
                // not >= bcs, see comment above
                if (idk.offset > offset) idk.offset += otherBytes.size();
            }
        }

        bytes.insert(bytes.begin()+offset, otherBytes.begin(), otherBytes.end());
    }

    size_t preserveCalleeRegs();

    void patchStackSize(size_t stackSize);

    void instructionNumberHint(size_t id) override;

    void movToArgRegVIPLCallingConvention(const x86::X64Register& dst, Assembler::RegisterHandle src, const map<x86::X64Register, size_t>& preserved, bool moveAsIs = false);

    size_t writeStack(size_t offset, Assembler::RegisterHandle handle);

    size_t dumpToStack(x86::X64Register reg);

    template<typename Ret, typename Class, typename... Args>
    void finallCallbackWrapper(Class obj, Ret (Class::* lambda)(Args...) const, Args... args);

    template<typename Class, typename... Args>
    void callWrapper(Class obj, void (Class::* lambda)(Args...) const, span<const x86::X64Register> ignore1);

    template<typename T>
    void withTempReg(T callback, span<const x86::X64Register> a);

    struct RegAllocCtx {
        X86Assembler* self;
        std::set<x86::X64Register> used;
        std::map<x86::X64Register, size_t> toRestore;
        std::map<size_t, size_t> toWriteback;
        std::set<size_t> toFree;
        bool didRestore = false;

        RegAllocCtx(X86Assembler* self): self(self) {}

        RegAllocCtx(const RegAllocCtx&) = delete;

        ~RegAllocCtx() {
            assert(didRestore);
        }

        /// allocate register ensure its not stack
        size_t allocReg() {
            auto [reg, didSpill] = self->allocator.allocateEvenClobered(used);
            assert(not used.contains(reg));
            used.insert(reg);
            if (didSpill) {
                // println("[reg-ctx] did spill {}", reg);
                auto hand = self->dumpToStack(reg);
                assert(not toRestore.contains(reg));
                toRestore.insert({reg, hand});
            } else {
                assert(not toFree.contains(self->allocator.toHandleStupid(reg)));
                toFree.emplace(self->allocator.toHandleStupid(reg));
            }

            return self->allocator.toHandleStupid(reg);
        }

        // will return proper reg handle to slot where its stored, eg if we saved this reg to stack so we can use it as tmp, we will return stack hancle
        size_t originalTransform(size_t idk) {
            if (self->allocator.isStack(idk)) return idk;

            auto reg = self->allocator.getReg(idk);
            if (this->toRestore.contains(reg)) {
                return this->toRestore[reg];
            } else {
                return idk;
            }
        }

        std::vector<size_t> originalTransform(span<size_t> idk) {
            std::vector<size_t> res;
            res.reserve(idk.size());
            for (auto p : idk) {
                res.push_back(originalTransform(p));
            }

            return res;
        }

        /// get the backing register
        x86::X64Register REG(size_t handle) {
            return self->allocator.getReg(handle);
        }

        /// if handle is register do nothing, else move value to reg
        RegisterHandle ensureReg(RegisterHandle handle) {
            if (not self->allocator.isStack(handle)) {
                // assert(not used.contains(self->allocator.getReg(handle)));
                used.insert(self->allocator.getReg(handle));
                return handle;
            }

            auto reg = allocReg();

            self->movReg(reg, handle);

            return reg;
        }

        RegisterHandle ensureRegWriteback(RegisterHandle handle) {
            if (not self->allocator.isStack(handle)) {
                // assert(not used.contains(self->allocator.getReg(handle)));
                used.insert(self->allocator.getReg(handle));
                return handle;
            }

            auto reg = allocReg();

            self->movReg(reg, handle);
            assert(not toWriteback.contains(reg));
            auto ori = toWriteback.size();
            toWriteback.insert({reg, handle});
            assert(ori+1 == toWriteback.size());

            return reg;
        }

        /// if we had to spill any registers restore them
        void restore() {
            assert(!didRestore);
            didRestore = true;
            for (auto [reg, stack] : toWriteback) {
                self->movReg(stack, reg);
                self->freeRegister(reg);
            }
            for (auto [reg, hand] : toRestore) {
                self->movHandleToReg(reg, hand);
                self->freeRegister(hand);
            }
            for (auto f : toFree) {
                self->freeRegister(f);
            }
        }
    };

    RegAllocCtx getAllocCtx() {
        return {this};
    }

    // function that will either get reg or
    // allocate temp reg
    // if no reg is available it will swap it to stack
    // and move original value to it
    x86::X64Register idk(size_t handle, set<x86::X64Register>& clobered, vector<pair<x86::X64Register, optional<size_t>>>& toRestore);

    x86::X64Register idk2(set<x86::X64Register>& clobered, vector<pair<x86::X64Register, optional<size_t>>>& toRestore);

    void restoreRegState(vector<pair<x86::X64Register, optional<size_t>>>& regs);

    template<typename... Args, typename F>
    void withRegs(F fan, Args... args);

    void movRegToHandle(size_t dst, x86::X64Register src);

    void movHandleToReg(const x86::X64Register& dst, Assembler::RegisterHandle src);

    void threeWayWrapper(size_t tgt, size_t lhs, size_t rhs, auto fun);

    void twoWayWrapper(size_t tgt, size_t lhs, size_t rhs, auto fun);

    void movRegToReg(const x86::X64Register& dst, const x86::X64Register& src, size_t size);

    void movRegToStack(size_t handle, int _offset, const x86::X64Register& src);

    size_t addressOf(size_t stackHandle);

    void movStackToReg(const x86::X64Register& dst, size_t adrHandle, int srcOffset = 0);

    void freeHandles(span<const size_t> handles);

    void freeHandles(initializer_list<size_t> handles);

    size_t getLabelId(size_t name);

    size_t toHandleStupid(const x86::X64Register& reg) {
        return allocator.toHandleStupid(reg);
    }

    Arg handleToArg(size_t handle);

    Arg handleToArgAssume8(size_t handle);

    size_t calculateStackSizeFastCall(span<const RegisterHandle> args);

    /*    void fastCall(LinkSymbol label, span<const RegisterHandle> args, optional<RegisterHandle> ret) {
        const array<x86::X64Register, 4> FAST_CALL_REG_ARGS{x86::X64Register::Rcx, x86::X64Register::Rdx, x86::X64Register::R8, x86::X64Register::R9};
        const size_t STACK_BYTES_USED = calculateStackSizeFastCall(args);
        const size_t STACK_BYTES_RESERVED = REG_SIZE * FAST_CALL_REG_ARGS.size(); // 40B

        vector<x86::X64Register> ignoredRegs;
        if (ret.has_value() && !RegAlloc::isStack(*ret)) {
            ignoredRegs.push_back(allocator.getReg(*ret));
        }

        size_t currentStackOffset = 0;

        withSavedCallRegs(ignoredRegs, fastCallSave, [&](const auto& savedRegs) {
            // prepare arg stack
            auto stackAllocated = align(STACK_BYTES_USED + STACK_BYTES_RESERVED, 16);
            mc.subImm32(x86::X64Register::Rsp, stackAllocated);

            // FIXME the arguments are pused from right to left IDK if we are doing that fix if wrong :*
            for (auto [index, arg] : args | views::enumerate) {
                // try to pass first 4 args trough regs
                if (allocator.sizeOf(arg) <= 8 && index < 4) {
                    auto reeg = allocator.getReg(arg);
                    if (not allocator.isStack(arg) && savedRegs.contains(reeg)) {
                        mc.readStack(savedRegs.at(reeg)+stackAllocated, FAST_CALL_REG_ARGS[index]);
                    }
                    else {
                        movHandleToReg(FAST_CALL_REG_ARGS[index], arg);
                    }
                }
                // move onto stack
                // lower args go to lower addresses
                else {
                    auto bytesUsed = writeStack(STACK_BYTES_USED + currentStackOffset, arg);
                    // we assume alignment if it breaks (it will) fix it ._.
                    assert(bytesUsed % 8 == 0); // aligned to 8B
                    currentStackOffset += bytesUsed;
                }
            }

            // FIXME not sure about the return stuff

            // move address to rax
            movLabel(x86::X64Register::Rax, label);

            // call ret reg
            mc.call(x86::X64Register::Rax);

            // move rax to ret
            if (ret.has_value()) {
                movRegToHandle(*ret, x86::X64Register::Rax);
            }

            // restore arg stack
            mc.addImm32(x86::X64Register::Rsp, stackAllocated);
        });
    }*/

    // PRE: 16bit aligned stack, SAVED CALLER SAVED REGS
    // REQUIRED INFO/SERVICESS: clobbered regs
    // 8 < value <= 16 will be passed over 2 registers
    void invokeScuffedSYSV(Arg func, span<Arg> args, optional<Arg> ret);

    void generateArgMove(x86::X64Register ret, Arg arg, ImmSpace imm) {
        if (not arg.symbol.has_value()) return;

        switch (arg.type) {
            case Arg::REGISTER:
                break;
            case Arg::IMMEDIATE:
                assert(imm.size == 8);
                requestLabel(*arg.symbol, imm, UNWRAP(arg.symbolType), BaseType::ABSOLUTE_8);
                break;
            case Arg::REG_OFFSET:
                break;
            case Arg::REG_OFFSET_VALUE:
                break;
            case Arg::SYMBOL_RIP_OFF_32:
                assert(imm.size == 4);
                requestLabel(*arg.symbol, imm, UNWRAP(arg.symbolType), BaseType::RIP_REL_4);
                break;
            case Arg::SYMBOL_RIP_VALUE_32:
                assert(imm.size == 4);
                requestLabel(*arg.symbol, imm, UNWRAP(arg.symbolType), BaseType::RIP_REL_4);
                break;
        }
    }

    void invokeScuffedFastCall(Arg func, span<Arg> args, optional<Arg> ret);

    void chadCall(Arg fun, span<Arg> args, optional<Arg> ret);

    void chudCall(Arg fun, std::initializer_list<Arg> args, optional<Arg> ret) {
        chadCall(fun, std::span<Arg>{(Arg*)args.begin(), args.end()}, ret);
    }

    void jmpCond(size_t label, JumpCondType type, RegisterHandle lhs, RegisterHandle rhs) override;

    static constexpr size_t JMP_OFFSET_SIZE = 4;

    void linkRelative(size_t dst, size_t src, size_t size, long adend) {
        assert(size == 4);

        i32 offset = ((i32)src-((i32)dst+4))+adend;

        std::memcpy(bytes.data()+dst, &offset, size);
    }



    void linkJumps() {
        forEachLabel([&](SlotLabel& it) {
            if (it.type != LABEL_TYPE_JMP) return;

            auto bound = this->getBoundLabelById(it.id);

            linkRelative(it.offset, bound.offset, 4, it.adend);
        });
    }

    void simpleLink(size_t type) {
        forEachLabel([&](SlotLabel& it) {
            if (it.type != type) return;

            auto bound = this->getBoundLabelById(it.id);

            linkRelative(it.offset, bound.offset, labelTypeToSize(it.baseType), it.adend);
        });
    }

    template<typename T>
    void patchLabel(SlotLabel l, T value) {
        assert(labelTypeToSize(l.baseType) == sizeof value);
        patchOffset<T>(l.offset, value);
    }

    template<typename T>
    void patchOffset(size_t offset, T value) {
        std::memcpy(bytes.data()+offset, &value, sizeof value);
    }

    size_t numRegs() const override {
        return allocator.numRegs();
    }

    void resurect(Assembler::RegisterHandle handle) override {
        TODO();
    }

    void trap() override {
        // mc.pushBack(0xCD);
        mc.pushBack(0xCC);
        // mc.hlt();
    }
};