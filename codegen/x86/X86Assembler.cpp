#include <variant>
#include <iostream>
#include <fstream>
#include "X86Assembler.h"

#include "utils/utils.h"
#include "utils/code_gen.h"
#include "utils/arch.h"

// TODO mov cache

void insertBytes(vector<u8>& target, const vector<u8>& source, size_t offset) {
    target.insert(target.begin()+offset, source.begin(), source.end());
}

void removeBytes(vector<u8>& target, size_t offset, size_t amount) {
    target.erase(target.begin()+offset, target.begin()+offset+amount);
}

struct CodeGroup {
    vector<u8>& bytes;
    map<string, Label>& labels;
    map<string, vector<Label>>& spaces;

    explicit CodeGroup(vector<u8>& bytes, map<string, Label>& labels, map<string, vector<Label>>& spaces) : bytes(
            bytes), labels(labels), spaces(spaces) {}

    void insertBytes(const vector<u8>& source, size_t offset) {
        ::insertBytes(bytes, source, offset);

        for (auto& label : labels | views::values) {
            if (label.index >= (long) offset) {
                label.index += source.size();
            }
        }
        for (auto& spacs : spaces | views::values) {
            for (auto& space: spacs) {
                if (space.index >= (long) offset) {
                    space.index += source.size();
                }
            }
        }
    }

    void removeBytes(size_t amout, size_t offset) {
        ::removeBytes(bytes, offset, amout);

        for (auto& label : labels | views::values) {
            if (label.index >= (long) offset) {
                label.index -= amout;
            }
        }
        for (auto& spacs : spaces | views::values) {
            for (auto& space: spacs) {
                if (space.index >= (long) offset) {
                    space.index -= amout;
                }
            }
        }
    }
};

u8 roundToNs(u8 n) {
    if (n <= 8) {
        return 1;
    }
    if (n <= 16) {
        return 2;
    }
    if (n <= 32) {
        return 4;
    }
    return 8;
}

template<typename T>
void rawWrite(u8* data, T value) {
    auto ptr = reinterpret_cast<u8*>(&value);

    for (auto i = 0u; i < sizeof(T); i++) {
        data[i] = ptr[i];
    }
}

Result<void> linkRelative(u8* data, const map<string , vector<Label>>& missing, const map<string, Label>& labels) {
    // println("SPACES {}", missing);
    // println("LABELS {}", labels);

    for (const auto& [name, spaces] : missing) {
        for (const auto& space : spaces) {
            if (space.type != Label::Type::JUMP) continue;
            if (!labels.contains(name)) {
                println("[WARNING] label \"{}\" not found (this can either be bug or its never referenced)", name);
                // return FAILF("label \"{}\" not found", mName);
                continue;
            }

            const auto& tag = labels.at(name);

            unsigned char convSize = roundToNs(static_cast<u8>(countl_zero(static_cast<u32>(tag.size)) - 24)); // converting u8 to u32 (24 additional zeros)
            if (convSize > space.size) return FAIL("symbol is dataSize of {} bytes, but space is dataSize of {} bytes tag: {}", convSize, space.size, name);
            long offset = static_cast<long>(tag.index)- (static_cast<long>(space.index) + static_cast<long>(space.size));

 /*           if (space.type == Label::Type::GOT) {
                offset += space.data*8;
            }*/

            // println("[LINK] offset: {} {} {} {}", offset, space.index, space.size, tag.index);

            rawWrite(data+space.index, bit_cast<u32>(static_cast<int>(offset)));
        }
    }

    return {};
}


void X86Assembler::generateRet() {
    bindReturn(allocateLabel());
    mc.leave();
    mc.writeSimple(SimpleX64Instruction::ret);
}

void X86Assembler::generateRet(RegisterHandle value) {
    if constexpr (arch::isUnixLike()) {
        if (retSize <= REG_SIZE) {
            movHandleToReg(x86::Rax, value);
        } else if (retSize <= REG_SIZE*2) {
            assert(allocator.isStack(value));

            mc.readMem(x86::Rax, x86::Rsp, allocator.getStackOffset(value), REG_SIZE);
            mc.readMem(x86::Rdx, x86::Rsp, allocator.getStackOffset(value)+8, allocator.sizeOf(value)-REG_SIZE);
        } else {
            assert(allocator.isStack(value));

            // WHY ARE WE WRITING TO RDI ?????????
            // FIXME bcs we allocated the reg and never freed it OR didnt save it to stack to lower reg pressure
            // FIXME preserve it onto stack
            mc.movReg(x86::Rax, x86::Rdi);

            // rcx chosen just bcs its caller saved reg
            mc.garbageMemCpy(x86::Rax, x86::Rsp, allocator.stackSize(value), x86::Rcx, static_cast<int>(allocator.getStackOffset(value)));
        }
    } else if constexpr (arch::isWindows()) {
        if (retSize <= REG_SIZE) {
            movHandleToReg(x86::Rax, value);
        } else {
            assert(allocator.isStack(value));

            // WHY ARE WE WRITING TO RDI ?????????
            // FIXME bcs we allocated the reg and never freed it OR didnt save it to stack to lower reg pressure
            // FIXME preserve it onto stack
            mc.movReg(x86::Rax, x86::Rcx);

            // rcx chosen just bcs its caller saved reg
            mc.garbageMemCpy(x86::Rax, x86::Rsp, allocator.stackSize(value), x86::Rcx, static_cast<int>(allocator.getStackOffset(value)));
        }
    } else {
        static_assert("unsuported arch");
    }

    generateRet();
}

void X86Assembler::jmpLabelTrue(RegisterHandle cond, size_t label) {
    withRegs([&](x86::X64Register condReg) {
        mc.makeIsTrue(condReg);
    }, cond);
    writeJmp(CmpType::Equal, label);
}

void X86Assembler::jmpLabelFalse(RegisterHandle cond, size_t label) {
    withRegs([&](x86::X64Register condReg) {
        mc.makeIsTrue(condReg);
    }, cond);
    writeJmp(CmpType::NotEqual, label);
}

void X86Assembler::jmp(size_t label) {
    writeJmp(label);
}

void X86Assembler::createLabel(size_t mName) {
    this->bindJmp(mName);
}

void X86Assembler::print() const {}

void X86Assembler::freeRegister(Assembler::RegisterHandle rt) {
    allocator.freeHandle(rt);
}

X86Assembler::Assembler::RegisterHandle X86Assembler::allocateRegister(size_t size) {
    return allocator.allocateAny(size);
}

void X86Assembler::readMem(Assembler::RegisterHandle tgt, Assembler::RegisterHandle obj, size_t offset, size_t amount) {
    // mov tgt, [obj+offset]
    withRegs([&](x86::X64Register tgtReg, x86::X64Register objReg) {
        if (amount > 8) {
            assert(allocator.isStack(tgt));

            mc.garbageMemCpy(x86::Rsp, objReg, amount, tgtReg, static_cast<int>(offset), static_cast<int>(allocator.getStackOffset(tgt)));
            return;
        }

        mc.readMem(tgtReg, objReg, offset, amount);

        // write back
        movRegToHandle(tgt, tgtReg);
    }, tgt, obj);
}

void X86Assembler::writeMem(Assembler::RegisterHandle obj, Assembler::RegisterHandle value, size_t offset, size_t amount) {
    // mov [obj+offset], value
    withRegs([&](x86::X64Register objReg, x86::X64Register valueReg) {
        if (amount > 8) {
            assert(allocator.isStack(value));

            mc.garbageMemCpy(objReg, x86::Rsp, amount, valueReg, static_cast<int>(allocator.getStackOffset(value)), static_cast<int>(offset));
            return;
        }

        mc.writeMem(objReg, valueReg, offset, amount);
    }, obj, value);
}

size_t X86Assembler::preserveCalleeRegs(const std::function<x86::X64Register::SaveType(x86::X64Register)>& save) {
    // println("labels before: {}", labels);
    vector<u8> temp;
    X86mc tempAsm(temp);

    size_t stackSize = allocator.stackSize();

    auto managedStack = getUniqueBoundLabel(LABEL_STACK_BEGIN).offset;

    auto acumulatedStackSize = stackSize;
    auto clobberedRegs = allocator.clobberedRegs();

    // generate preservation code
    for (const auto& reg : clobberedRegs) {
        if (save(reg) != x86::X64Register::SaveType::Callee) continue;

        tempAsm.writeStack(static_cast<int>(acumulatedStackSize), reg);
        acumulatedStackSize += REG_SIZE;
        allocator.forceReserveStack(REG_SIZE);
    }

    insertBytes(temp, managedStack);
    temp.clear();

    // generate restoration code
    forEachBound([&](auto& it) {
        if (it.type != LABEL_TYPE_RETURN) return;

        temp.clear();

        auto acumulatedStackSize2 = stackSize;
        for (const auto& reg : clobberedRegs) {
            if (save(reg) != x86::X64Register::SaveType::Callee) continue;

            tempAsm.readStack(static_cast<int>(acumulatedStackSize2), reg);

            acumulatedStackSize2 += REG_SIZE;
        }

        insertBytes(temp, it.offset);
        temp.clear();
    });

    return acumulatedStackSize;
}

void X86Assembler::movInt(Assembler::RegisterHandle dest, u64 value, bool isSigned, size_t offsetBytes) {
    auto movToReg = [&](x86::X64Register reg) {
        mc.movFast(reg, value);
    };

    if (allocator.isStack(dest)) {
        withTempReg([&](x86::X64Register reg) -> void {
            movToReg(reg);

            movRegToStack(dest, static_cast<int>(offsetBytes), reg);
        }, {});
    }
    else {
        assert(offsetBytes == 0);
        movToReg(allocator.getReg(dest));
    }
}

void X86Assembler::movReg(Assembler::RegisterHandle dest, Assembler::RegisterHandle src, size_t destOffsetBytes, size_t srcOffsetBytes, size_t amount) {
    if (src == dest && destOffsetBytes == 0 && srcOffsetBytes == 0 && amount == REG_SIZE) {
        return;
    }
    if (amount == 0) {
        return;
    }

    if (!allocator.isStack(dest) && !allocator.isStack(src)) { // Reg2Reg
        auto srcReg = allocator.getReg(src);
        auto dstReg = allocator.getReg(dest);

        if (srcOffsetBytes == 0 && destOffsetBytes == 0) {
            movRegToReg(dstReg, srcReg, amount);
        } else {
            withTempReg([&](x86::X64Register tmp) {
                // FIXME
                // copy value reg to tmp
                mc.movReg(tmp, srcReg);
                // strip of low bits
                mc.shiftRImm(tmp, srcOffsetBytes*8);
                // strip of high bits
                mc.shiftLImm(tmp, 64-amount*8);
                // move value to proper place
                mc.shiftRImm(tmp, (64-amount*8)-destOffsetBytes*8);
                // or with dst
                mc.writeRegInst(X64Instruction::Or, dstReg, tmp);
            }, array{srcReg, dstReg});
        }
        return;
    }
    if (allocator.isStack(dest) && !allocator.isStack(src)) { // Reg2Stack
        assert(srcOffsetBytes == 0);
        movRegToStack(dest, destOffsetBytes, allocator.getReg(src));
        return;
    }
    if (!allocator.isStack(dest) && allocator.isStack(src)) { // Stack2Reg
        assert(destOffsetBytes == 0);
        movStackToReg(allocator.getReg(dest), src, srcOffsetBytes);
        return;
    }

    assert(isAligned(amount, REG_SIZE));

    withTempReg([&](x86::X64Register a){
        mc.garbageMemCpy(x86::Rsp, x86::Rsp, amount, a, static_cast<int>(allocator.getStackOffset(src) + srcOffsetBytes), static_cast<int>(allocator.getStackOffset(dest) + destOffsetBytes));
    }, {});
}

void X86Assembler::divInt(Assembler::RegisterHandle dest, Assembler::RegisterHandle left, Assembler::RegisterHandle right) {
    withSpecificReg(x86::Rax, [&] {
        withSpecificReg(x86::Rdx, [&] {
            withSpecificReg(x86::Rcx, [&] {
                movHandleToReg(x86::Rcx, right);
                movHandleToReg(x86::Rax, left);
                mc.cqo();
                mc.signedDivide(x86::Rcx);
                movRegToHandle(dest, x86::Rax);
            }, {dest});
        }, {dest});
    }, {dest});
}

void X86Assembler::modInt(RegisterHandle dest, RegisterHandle left, RegisterHandle right) {
    withSpecificReg(x86::Rax, [&] {
        withSpecificReg(x86::Rdx, [&] {
            withSpecificReg(x86::Rcx, [&] {
                movHandleToReg(x86::Rcx, right);
                movHandleToReg(x86::Rax, left);
                mc.cqo();
                mc.signedDivide(x86::Rcx);
                movRegToHandle(dest, x86::Rdx);
            }, {dest});
        }, {dest});
    }, {dest});
}

void X86Assembler::shlInt(Assembler::RegisterHandle dest, Assembler::RegisterHandle left, Assembler::RegisterHandle right) {
    movReg(dest, left);

    withSpecificReg(x86::Rcx, [&] {
        movHandleToReg(x86::Rcx, right);
        withRegs([&](x86::X64Register dstReg) {
            mc.shiftLeftByCl(dstReg);
            movRegToHandle(dest, dstReg);
        }, dest);
    }, {dest});
}

void X86Assembler::shrInt(Assembler::RegisterHandle dest, Assembler::RegisterHandle left, Assembler::RegisterHandle right) {
    movReg(dest, left);

    withSpecificReg(x86::Rcx, [&] {
        movHandleToReg(x86::Rcx, right);
        withRegs([&](x86::X64Register dstReg) {
            mc.shiftRightByCl(dstReg);
            movRegToHandle(dest, dstReg);
        }, dest);
    }, {dest});
}

void X86Assembler::nop() {
    // mc.nop();
}

void X86Assembler::garbageMemCpy(x86::X64Register ptrReg, size_t stackOffset, size_t stackSize) {
    // FIXME assert(isAligned(stackSize, REG_SIZE));

    withTempReg([&](x86::X64Register tmp) {
        mc.garbageMemCpy(ptrReg, x86::Rsp, stackSize, tmp, static_cast<int>(stackOffset));
    }, {&ptrReg, 1});
}

size_t X86Assembler::writeStack(size_t offset, Assembler::RegisterHandle handle) {
    if (not allocator.isStack(handle)) {
        auto r = allocator.getReg(handle);

        mc.writeStack(offset, r);
        return X86Assembler::REG_SIZE;
    }

    auto amount = allocator.stackSize(handle);
    auto srcOffset = allocator.getStackOffset(handle);
    // garbageMemCpy(x86::Rsp, srcOffset, amount); // FIXME WTF is this
    garbageMemCpy(x86::Rsp, offset, x86::Rsp, srcOffset, amount);
    return amount;
}

X86Assembler::X86Assembler(span<size_t> argSizes, size_t retSize) : mc(bytes), argSizes(collectVec(argSizes)), retSize(retSize) {
    mc.nop();
    mc.nop();
    mc.nop();
    mc.nop();
    mc.nop();

    if constexpr (arch::isUnixLike()) {
        initializeSYSV();
    } else if (arch::isWindows()) {
        initializeFastCall();
    } else {
        static_assert("unsuported");
    }
}

void X86Assembler::withSpecificReg(const x86::X64Register& reg, const function<void()>& callback, std::initializer_list<size_t> exclude) {
    bool isIdk = false;
    for (auto r : exclude) {
        if (!allocator.isStack(r) && allocator.getReg(r) == reg) {
            isIdk = true;
            break;
        }
    }

    // GOOD we can aquire it
    if (!allocator.isAcquired(reg)) {
        allocator.acquireSpecific(reg);
        callback();
        allocator.freeReg(reg);
        return;
    }

    // still good we can move it to another reg
    if (allocator.hasFreeReg()) {
        auto saved = allocator.acquireAny();
        mc.movReg(saved, reg);
        callback();
        if (!isIdk) mc.movReg(reg, saved);
        allocator.freeReg(saved);
        return;
    }

    // bad :( we need to move it to stack
    auto slot = allocateStack(REG_SIZE);
    auto offset = allocator.getStackOffset(slot);
    mc.writeStack(offset, reg);
    callback();
    if (!isIdk) mc.readStack(offset, reg);
    allocator.freeHandle(slot);
}

void X86Assembler::movHandleToReg(const x86::X64Register& dst, Assembler::RegisterHandle src) {
    if (allocator.isStack(src)) {
        movStackToReg(dst, src);
    }
    else {
        movRegToReg(dst, allocator.getReg(src), allocator.sizeOf(src));
    }
}

void X86Assembler::movToArgRegVIPLCallingConvention(const x86::X64Register& dst, Assembler::RegisterHandle src, const map<x86::X64Register, size_t>& preserved, bool moveAsIs) {
    if (not allocator.isStack(src)) {
        auto r = allocator.getReg(src);

        // is reg in stack?
        if (preserved.contains(r)) {
            mc.readStack(allocator.getStackOffset(preserved.at(r)), dst);
        }
        else {
            mc.movReg(dst, r);
        }
        return;
    }

    // move ptr to
    if (moveAsIs) {
        mc.getStackPtr(dst, allocator.getStackOffset(src));
    }

    if (allocator.stackSize(src) > 8) {
        mc.getStackPtr(dst, allocator.getStackOffset(src));
    } else {
        movStackToReg(dst, src);
    }
}

void X86Assembler::patchStackSize(size_t stackSize) {
    forEachLabel([&](SlotLabel& it) {
        if (it.type != LABEL_TYPE_STACK_SIZE) return;

        patchLabel<i32>(it, stackSize);
    });
}

void X86Assembler::instructionNumberHint(size_t id) {
/*    bool err = false;
    auto str = to_string(id);
    auto num = stringToLong(str, err, 16);

    // TODO use one line instruction like mov [Rsp+40], 0x21
    mc.push(x86::Rax);
    mc.mov(x86::Rax, num);
    mc.pop(x86::Rax);*/
}

size_t X86Assembler::dumpToStack(x86::X64Register reg) {
    auto handle = allocator.allocateStack(REG_SIZE);
    mc.writeStack(allocator.getStackOffset(handle), reg);

    return handle;
}

x86::X64Register X86Assembler::idk(size_t handle, set<x86::X64Register>& clobered, vector<pair<x86::X64Register, optional<size_t>>>& toRestore) {
    if (!allocator.isStack(handle)) {
        auto tmp = allocator.getReg(handle);
        clobered.insert(tmp);

        return tmp;
    }

    auto [reg, isClobered] = allocator.allocateEvenClobered(clobered);

    // ignore newly allocated reg from being allocated again
    clobered.insert(reg);

    if (isClobered) {
        // if we allocated used register persist it onto stack
        auto hand = dumpToStack(reg);
        toRestore.emplace_back(reg, hand);
    } else {
        toRestore.emplace_back(reg, nullopt);
    }

    // move stack variable to reg
    movHandleToReg(reg, handle);

    return reg;
}

x86::X64Register X86Assembler::idk2(set<x86::X64Register>& clobered, vector<pair<x86::X64Register, optional<size_t>>>& toRestore) {
    auto [reg, isClobered] = allocator.allocateEvenClobered(clobered);

    if (isClobered) {
        auto hand = dumpToStack(reg);
        toRestore.emplace_back(reg, hand);
    } else {
        toRestore.emplace_back(reg, nullopt);
    }

    return reg;
}

void X86Assembler::restoreRegState(vector<pair<x86::X64Register, optional<size_t>>>& regs) {
    for (const auto& [originalReg, stackHandle] : regs) {
        if (not stackHandle.has_value()) {
            allocator.freeReg(originalReg);
        }
        else {
            mc.readStack(allocator.getStackOffset(*stackHandle), originalReg);
            allocator.freeHandle(*stackHandle);
        }
    }
}

void X86Assembler::movRegToHandle(size_t dst, x86::X64Register src) {
    if (allocator.isStack(dst)) {
        movRegToStack(dst, 0, src);
    }
    else {
        auto dstReg = allocator.getReg(dst);
        auto size = allocator.sizeOf(dst);
        movRegToReg(dstReg, src, size);
    }
}

void X86Assembler::threeWayWrapper(size_t tgt, size_t lhs, size_t rhs, auto fun) {
    withRegs([&](x86::X64Register tReg, x86::X64Register lReg, x86::X64Register rReg) {
        fun(tReg, lReg, rReg);

        // flush result to original destination
        movRegToHandle(tgt, tReg);
    }, tgt, lhs, rhs);
}

void X86Assembler::twoWayWrapper(size_t tgt, size_t lhs, size_t rhs, auto fun) {
    threeWayWrapper(tgt, lhs, rhs, [&](x86::X64Register tReg, x86::X64Register lReg, x86::X64Register rReg) {
        mc.movReg(tReg, lReg);
        fun(tReg, rReg);
    });
}


void X86Assembler::withSavedCallRegs(span<const x86::X64Register> exclude, span<const x86::X64Register> excluceRestore, const std::function<x86::X64Register::SaveType(const x86::X64Register&)>& convention, const std::function<void(const map<x86::X64Register, size_t>&)>& callback) {
    auto saved = allocator.saveCallRegs(exclude, convention);

    for (const auto& [reg, stackHandle]: saved) {
        mc.writeStack(allocator.getStackOffset(stackHandle), reg);
    }

    callback(saved);

    for (const auto& [reg, stackHandle]: saved) {
        if (std::ranges::find(excluceRestore, reg) != excluceRestore.end()) continue;
        mc.readStack(allocator.getStackOffset(stackHandle), reg);
    }

    // free allocated stack
    allocator.restoreCallRegs(saved);
}

unique_ptr<X86Assembler::Assembler> X86Assembler::create(span<size_t> argSizes, size_t retSize) {
    return make_unique<X86Assembler>(argSizes, retSize);
}

vector<X86Assembler::Assembler::RegisterHandle> X86Assembler::getArgHandles() {
    return argHandles;
}

void X86Assembler::f64ToInt(RegisterHandle dest, RegisterHandle value) {
    withRegs([&](x86::X64Register dst, x86::X64Register src) {
        mc.f64ToI64(dst, src);

        // flush result to original destination
        movRegToHandle(dest, dst);
    }, dest, value);
}

void X86Assembler::writeJmp(size_t id) {
    auto space = mc.writeJmp((u32)0);
    requestJmpLabel(id, space);
}

void X86Assembler::writeJmp(CmpType jmp, size_t id) {
    auto space = mc.writeJmp(jmp, (u32)0);
    requestJmpLabel(id, space);
}

/*void X86Assembler::putAbsolute(string_view name, ImmSpace space) {
    putLabel(name, space, Label::Type::ABSOLUTE1);
}*/

/*void X86Assembler::putRelative(string_view name, ImmSpace space) {
    putLabel(name, space, Label::Type::JUMP);
}*/

string X86Assembler::nextReturnLabel() {
    return stringify("return-{}", returnCounter++);
}

void X86Assembler::int2f64(RegisterHandle dest, RegisterHandle value) {
    withRegs([&](x86::X64Register dst, x86::X64Register src) {
        auto srcSize = allocator.sizeOf(value);

        if (srcSize == 1) {
            mc.movsx8(src, src);
        }
        else if (srcSize == 2) {
            mc.movsx16(src, src);
        }
        else if (srcSize == 4) {
            mc.movsx32(src, src);
        }
        else {
            assert(srcSize == 8);
        }

        mc.i64ToF64(dst, src);

        // flush result to original destination
        movRegToHandle(dest, dst);
    }, dest, value);
}

void X86Assembler::arithmeticInt(BinaryOp op, Assembler::RegisterHandle tgt, Assembler::RegisterHandle lhs, Assembler::RegisterHandle rhs) {
    if (op == BinaryOp::DIV) {
        return divInt(tgt, lhs, rhs);
    }
    if (op == BinaryOp::MOD) {
        return modInt(tgt, lhs, rhs);
    }
    if (op == BinaryOp::SHL) {
        return shlInt(tgt, lhs, rhs);
    }
    if (op == BinaryOp::SHR) {
        return shrInt(tgt, lhs, rhs);
    }
    if (op == BinaryOp::EQ || op == BinaryOp::GT || op == BinaryOp::LS || op == BinaryOp::GE || op == BinaryOp::LE || op == BinaryOp::NEQ) {
        threeWayWrapper(tgt, lhs, rhs, [&](x86::X64Register tReg, x86::X64Register lReg, x86::X64Register rReg) {
            switch (op) {
                case BinaryOp::EQ:
                    mc.cmpI(CmpType::Equal, tReg, lReg, rReg);
                    break;
                case BinaryOp::NEQ:
                    mc.cmpI(CmpType::NotEqual, tReg, lReg, rReg);
                    break;
                case BinaryOp::GT:
                    mc.cmpI(CmpType::Greater, tReg, lReg, rReg);
                    break;
                case BinaryOp::LS:
                    mc.cmpI(CmpType::Less, tReg, lReg, rReg);
                    break;
                case BinaryOp::GE:
                    mc.cmpI(CmpType::GreaterOrEqual, tReg, lReg, rReg);
                    break;
                case BinaryOp::LE:
                    mc.cmpI(CmpType::LessOrEqual, tReg, lReg, rReg);
                    break;
                default:
                    PANIC();
            }
        });
        return;
    }

    auto doSimpleInst2 = [&](X64Instruction op, RegisterHandle acum, RegisterHandle value) {
        if (allocator.isStack(acum) && allocator.isStack(value)) { // we need temporary
            withRegs([&](x86::X64Register val) {
                mc.writeMemRegInst(op, x86::Rsp, allocator.getStackOffset(acum), val);
            }, value);
        } else if (allocator.isStack(acum) && !allocator.isStack(value)) {
            mc.writeMemRegInst(op, x86::Rsp, allocator.getStackOffset(acum), allocator.getReg(value));
        } else if (!allocator.isStack(acum) && allocator.isStack(value)) {
            mc.writeRegMemInst(op, allocator.getReg(acum), x86::Rsp, allocator.getStackOffset(value));
        } else {
            mc.writeRegInst(op, allocator.getReg(acum), allocator.getReg(value));
        }
    };

    auto doSimpleInst = [&](X64Instruction op, RegisterHandle dst, RegisterHandle lhs, RegisterHandle rhs) {
        // mov tmp, rhs
        // add lhs, tmp
        // or
        // add lhs, rhs
        if (dst == lhs) { // is the same
            doSimpleInst2(op, lhs, rhs);
        } else {
            // mov dst, lhs
            // add dst, rhs
            if (not allocator.isStack(dst)) {
                movReg(dst, lhs);
                doSimpleInst2(op, dst, rhs);
            } else {
                // mov tmp, [rsp+lhs]
                // add tmp, [rsp+rhs]
                // mov [rs+dst], tmp
                withTempReg([&](x86::X64Register tmp) {
                    movHandleToReg(tmp, lhs);
                    doSimpleInst2(op, allocator.regToHandle(tmp), rhs);
                    movRegToHandle(dst, tmp);
                }, {});
            }
        }
    };

    switch (op) {
        case BinaryOp::ADD:
            doSimpleInst(X64Instruction::add, tgt, lhs, rhs);
            break;
        case BinaryOp::SUB:
            doSimpleInst(X64Instruction::sub, tgt, lhs, rhs);
            break;
        case BinaryOp::MUL:
            twoWayWrapper(tgt, lhs, rhs, [&](x86::X64Register lhs, x86::X64Register rhs) {
                mc.writeRegInst(SusX64Instruction::imul, lhs, rhs);
            });
            break;
        case BinaryOp::OR:
            doSimpleInst(X64Instruction::Or, tgt, lhs, rhs);
        break;
        case BinaryOp::AND:
            doSimpleInst(X64Instruction::And, tgt, lhs, rhs);
        break;
        case BinaryOp::XOR:
            doSimpleInst(X64Instruction::xorI, tgt, lhs, rhs);
        break;
/*        case BinaryOp::NEGATE:
            doSimpleInst(X64Instruction::xorI, tgt, lhs, rhs);*/
        break;
        case BinaryOp::DIV:
        case BinaryOp::MOD:
        case BinaryOp::SHR:
        case BinaryOp::SHL:
        case BinaryOp::EQ:
        case BinaryOp::NEQ:
        case BinaryOp::GT:
        case BinaryOp::LS:
        case BinaryOp::GE:
        case BinaryOp::LE:
            PANIC();
    }
}

void X86Assembler::arithmeticFloat(Assembler::BinaryOp op, Assembler::FloatingPointType type, Assembler::RegisterHandle tgt, Assembler::RegisterHandle lhs, Assembler::RegisterHandle rhs) {
    auto is64Bit = type == Assembler::FloatingPointType::Double;

    threeWayWrapper(tgt, lhs, rhs, [&](x86::X64Register dst, x86::X64Register lhs, x86::X64Register rhs) {
        switch (op) {
            case BinaryOp::ADD:
                mc.addFloat(dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::SUB:
                mc.subFloat(dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::MUL:
                mc.mulFloat(dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::DIV:
                mc.divFloat(dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::EQ:
                mc.floatCompare(CmpType::Equal, dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::LS:
                mc.floatCompare(CmpType::Above, dst, rhs, lhs, is64Bit);
                break;
            case BinaryOp::GT:
                mc.floatCompare(CmpType::Above, dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::GE:
                mc.floatCompare(CmpType::NotBellow, dst, lhs, rhs, is64Bit);
                break;
            case BinaryOp::LE:
                mc.floatCompare(CmpType::NotBellow, dst, rhs, lhs, is64Bit);
                break;
            case BinaryOp::NEQ:
                mc.floatCompare(CmpType::NotEqual, dst, rhs, lhs, is64Bit);
                break;
            case BinaryOp::OR:
            case BinaryOp::AND:
            case BinaryOp::XOR:
            case BinaryOp::MOD:
            case BinaryOp::SHR:
            case BinaryOp::SHL:
                PANIC("unsuported arithmetic op");
        }
    });
}

void X86Assembler::garbageMemCpy(x86::X64Register dst, size_t dstOffset, x86::X64Register src, size_t srcOffset, size_t stackSize) {
    // FIXME assert(isAligned(stackSize, REG_SIZE));

    array<x86::X64Register, 2> ignore{dst, src};

    withTempReg([&](x86::X64Register tmp) {
        mc.garbageMemCpy(dst, src, stackSize, tmp, srcOffset, dstOffset);
    }, ignore);
}

void X86Assembler::addressOf(Assembler::RegisterHandle tgt, Assembler::RegisterHandle obj) {
    assert(allocator.isStack(obj));
    auto offset = allocator.getStackOffset(obj);

    withRegs([&](auto reg) {
        mc.getStackPtr(reg, offset);
        movRegToHandle(tgt, reg);
    }, tgt);
}

void X86Assembler::movRegToReg(const x86::X64Register& dst, const x86::X64Register& src, size_t size) {
    mc.movReg(dst, src);
    if (size == 1) {
        mc.writeMovZX8(dst, dst);
    }
    else if (size == 2) {
        mc.writeMovZX16(dst, dst);
    }
    // FIXME maybe even zx 32bit
}

void X86Assembler::movRegToStack(size_t handle, int _offset, const x86::X64Register& src) {
    auto size = allocator.stackSize(handle);
    auto offset = allocator.getStackOffset(handle)+_offset;

    mc.writeStackAmount(offset, src, size);
}

void X86Assembler::movStackToReg(const x86::X64Register& dst, size_t adrHandle, int srcOffset) {
    assert(allocator.isStack(adrHandle));
    auto size = allocator.stackSize(adrHandle);
    auto offset = allocator.getStackOffset(adrHandle)+srcOffset;

    if (size == 1) {
        mc.read1(dst, x86::Rsp, offset);
        mc.writeMovZX8(dst, dst);
    }
    else if (size == 2) {
        mc.read2(dst, x86::Rsp, offset);
        mc.writeMovZX16(dst, dst);
    }
    else if (size == 4) {
        mc.read4(dst, x86::Rsp, offset);
    }
    else if (size == 8) {
        mc.readStack(offset, dst);
    }
    else {
        // println("[WARN] trying to move {} from stack to reg -- moving only REG_SIZE", size);
        mc.readStack(offset, dst);
    }
}

void X86Assembler::freeHandles(span<const size_t> handles) {
    for (auto handle : handles) {
        allocator.freeHandle(handle);
    }
}

void X86Assembler::freeHandles(initializer_list<size_t> handles) {
    freeHandles({handles.begin(), handles.size()});
}

size_t X86Assembler::addressOf(size_t stackHandle) {
    auto dst = allocator.allocateAny(8);
    addressOf(dst, stackHandle);

    return dst;
}

void X86Assembler::signExtend(Assembler::RegisterHandle dst, Assembler::RegisterHandle src) {
    auto srcSize = allocator.sizeOf(src);
    auto dstSize = allocator.sizeOf(dst);
    (void)dstSize;
    assert(dstSize >= srcSize);

    withRegs([&](x86::X64Register dstReg, x86::X64Register srcReg) {
        // FIXME
        if (srcSize == 1) {
            mc.movsx8(dstReg, srcReg);
        } else if (srcSize == 2) {
            mc.movsx16(dstReg, srcReg);
        } else if (srcSize == 4) {
            mc.movsx32(dstReg, srcReg);
        } else {
        }

        if (allocator.isStack(dst)) {
            mc.writeStackAmount(allocator.getStackOffset(dst), dstReg, srcSize);
        }
    }, dst, src);
}

void X86Assembler::initializeSYSV() {
    const x86::X64Register TMP_REG = x86::Rax;
    const auto STACK_ARGS_BASE = 16;

    mc.push(x86::Rbp);
    mc.movReg(x86::Rbp, x86::Rsp);

    auto label = mc.subImm32(x86::Rsp, 0x0);
    requestLabel(allocateLabel(), label, LABEL_TYPE_STACK_SIZE, BaseType::ABSOLUTE_4);

    bindRawLabel(allocateLabel(), LABEL_STACK_BEGIN);

    size_t allocatedArgs = 0;
    size_t stackOffset = 0;

    // return cant fit into reg
    if (retSize > REG_SIZE*2) {
        assert(isAligned(retSize, REG_SIZE));
        // reserve Rdi for return ptr
        allocator.acquireSpecific(x86::SYSV_REGS[allocatedArgs]);
        allocatedArgs += 1;
    }

    for (auto argSize : argSizes) {
        if (argSize <= REG_SIZE && allocatedArgs < x86::SYSV_REGS.size()) {
            argHandles.push_back(allocator.acquireSpecific(x86::SYSV_REGS[allocatedArgs]));
            allocatedArgs += 1;
        } else if (argSize <= REG_SIZE*2 && allocatedArgs <= x86::SYSV_REGS.size()-2) {
            auto stack = allocator.allocateStack(REG_SIZE*2);

            mc.writeStackAmount(allocator.getStackOffset(stack), x86::SYSV_REGS[allocatedArgs], REG_SIZE);
            mc.writeStackAmount(allocator.getStackOffset(stack)+REG_SIZE, x86::SYSV_REGS[allocatedArgs+1], argSize-REG_SIZE);

            argHandles.push_back(stack);

            allocatedArgs += 2;
        } else {
            auto aligned = align(argSize, REG_SIZE);
            auto stack = allocateStack(aligned);

            mc.garbageMemCpy(x86::Rsp, x86::Rbp, aligned, TMP_REG, STACK_ARGS_BASE+stackOffset, allocator.getStackOffset(stack));

            argHandles.push_back(stack);

            stackOffset += aligned;
        }
    }
}

void X86Assembler::initializeFastCall() {
    const std::array<x86::X64Register, 4> FAST_REGS{x86::Rcx, x86::Rdx, x86::R8, x86::R9};
    const x86::X64Register TMP_REG = x86::Rax;
    const auto STACK_ARGS_BASE = 16;
    const auto STACK_RESERVED = REG_SIZE*FAST_REGS.size();

    mc.frameInit();

    auto label = mc.subImm32(x86::Rsp, 0x0);
    requestLabel(allocateLabel(), label, LABEL_TYPE_STACK_SIZE, BaseType::ABSOLUTE_4);

    bindRawLabel(allocateLabel(), LABEL_STACK_BEGIN);

    size_t allocatedArgs = 0;
    size_t stackOffset = 0;

    // return cant fit into reg
    if (retSize > REG_SIZE) {
        assert(isAligned(retSize, REG_SIZE));
        // reserve Rdi for return ptr
        allocator.acquireSpecific(FAST_REGS[allocatedArgs]);
        allocatedArgs += 1;
    }

    for (const auto [argId, argSize] : argSizes | views::enumerate) {
        if (argSize <= REG_SIZE && allocatedArgs < FAST_REGS.size()) { // value is pased trough reg
            argHandles.push_back(allocator.acquireSpecific(FAST_REGS[allocatedArgs]));
            allocatedArgs += 1;
        } else if (allocatedArgs < FAST_REGS.size()) { // value passed by reference
            // FIXME for now just copy the value onto our stack
            auto reg = FAST_REGS[allocatedArgs];
            assert(isAligned(argSize, 8));

            auto stackHandle = allocator.allocateStack(argSize);
            argHandles.push_back(stackHandle);

            mc.garbageMemCpy(x86::Rsp, reg, argSize, TMP_REG, 0, allocator.getStackOffset(stackHandle));

            allocatedArgs += 1;
        } else { // value is passed through stack
            auto aligned = align(argSize, REG_SIZE);

            auto stack = allocateStack(aligned);
            argHandles.push_back(stack);


            // if somehow 0..3rd argument is passed over stack, its locations is special spot
            size_t dynamicOffset = STACK_RESERVED;
            if ((size_t)argId < FAST_REGS.size() && argSize <= 8) {
                dynamicOffset = argId*REG_SIZE;
            }

            mc.garbageMemCpy(x86::Rsp, x86::Rbp, aligned, TMP_REG, STACK_ARGS_BASE+dynamicOffset+stackOffset, allocator.getStackOffset(stack));

            stackOffset += aligned;
        }
    }
}

void X86Assembler::f32ToF64(Assembler::RegisterHandle dest, Assembler::RegisterHandle value) {
    withRegs([&](x86::X64Register dst, x86::X64Register src) {
        mc.floatToDouble(dst, src);

        // flush result to original destination
        movRegToHandle(dest, dst);
    }, dest, value);
}

void X86Assembler::f64ToF32(Assembler::RegisterHandle dest, Assembler::RegisterHandle value) {
    withRegs([&](x86::X64Register dst, x86::X64Register src) {
        mc.doubleToFloat(dst, src);

        // flush result to original destination
        movRegToHandle(dest, dst);
    }, dest, value);
}

Arg X86Assembler::handleToArg(size_t handle) {
    if (allocator.isStack(handle)) {
        return Arg::StackValue(allocator.getStackOffset(handle), allocator.sizeOf(handle));
    } else {
        return Arg::Reg(allocator.getReg(handle));
    }
}

size_t X86Assembler::calculateStackSizeFastCall(span<const RegisterHandle> args) {
    size_t stackSize = 0;
    size_t regsUsed = 0;

    for (auto arg : args) {
        if (allocator.sizeOf(arg) <= 8 && regsUsed < 4) {
            continue;
        }
        regsUsed++; // well this is windows for ya if you organice ur args badly u will lose regs :))
        stackSize += align(allocator.sizeOf(arg), 8);
    }

    return stackSize;
}

void X86Assembler::invokeScuffedSYSV(Arg func, span<Arg> args, optional<Arg> ret) {
    vector<x86::X64Register> exclude;
    vector<x86::X64Register> excludeRestore;

    if (ret.has_value() && ret->type == Arg::REGISTER) {
        bool contains = false;
        for (auto arg : args) {
            // if ret reg is the same as any arg reg, preserve it but dont restore it
            if (arg.type == Arg::REGISTER && arg.reg == ret->reg) {
                contains = true;
                break;
            }
        }
        if (contains) {
            excludeRestore.push_back(ret->reg);
        } else {
            exclude.push_back(ret->reg);
        }
    }

    withSavedCallRegs(exclude, excludeRestore, x86::sysVSave, [&](const auto& saved) {
        mc.invokeScuffedSYSV2(func, args, ret, saved, [&](auto dst, auto sym, auto imm) {
            generateArgMove(dst, sym, imm);
        });
    });
}

void X86Assembler::invokeScuffedFastCall(Arg func, span<Arg> args, optional<Arg> ret) {
    vector<x86::X64Register> exclude;
    vector<x86::X64Register> excludeRestore;

    if (ret.has_value() && ret->type == Arg::REGISTER) {
        bool contains = false;
        for (auto arg : args) {
            // if ret reg is the same as any arg reg, preserve it but dont restore it
            if (arg.type == Arg::REGISTER && arg.reg == ret->reg) {
                contains = true;
                break;
            }
        }
        if (contains) {
            excludeRestore.push_back(ret->reg);
        } else {
            exclude.push_back(ret->reg);
        }
    }

    withSavedCallRegs(exclude, excludeRestore, x86::fastCallSave, [&](const auto& saved){
        mc.invokeScuffedFastCall(func, args, ret, saved, [&](auto dst, auto sym, auto imm) {
                                     generateArgMove(dst, sym, imm);
                                 }, [&](auto amount){ return allocator.getStackOffset(allocator.allocateStack(amount)); });
    });
}

void X86Assembler::chadCall(Arg fun, span<Arg> args, optional<Arg> ret) {
    if constexpr (arch::isUnixLike()) {
        invokeScuffedSYSV(fun, args, ret);
    } else if (arch::isWindows()) {
        invokeScuffedFastCall(fun, args, ret);
    } else {
        static_assert("unsuported platform");
    }
}

CmpType toCmpType(JumpCondType it) {
    switch (it) {
        case JumpCondType::EQUALS: return CmpType::Equal;
        case JumpCondType::NOT_EQUALS: return CmpType::NotEqual;
        case JumpCondType::GREATER: return CmpType::Greater;
        case JumpCondType::GREATER_OR_EQUAL: return CmpType::GreaterOrEqual;
        case JumpCondType::LESS: return CmpType::Less;
        case JumpCondType::LESS_OR_EQUAL: return CmpType::LessOrEqual;
    }
    PANIC()
}

void X86Assembler::jmpCond(size_t label, JumpCondType type, RegisterHandle lhs, RegisterHandle rhs) {
    withRegs([&](x86::X64Register a, x86::X64Register b) {
        mc.writeRegInst(X64Instruction::cmp, a, b);
    }, lhs, rhs);
    writeJmp(toCmpType(type), label);
}

size_t X86Assembler::preserveCalleeRegs() {
    if constexpr (arch::isUnixLike()) {
        return  preserveCalleeRegs(x86::sysVSave);
    } else if (arch::isWindows()) {
        return  preserveCalleeRegs(x86::fastCallSave);
    } else {
        static_assert("unsuported platform");
    }
}

CmpType X86Assembler::toCmpType(JumpCondType it) {
    switch (it) {
        case JumpCondType::EQUALS: return CmpType::Equal;
        case JumpCondType::NOT_EQUALS: return CmpType::NotEqual;
        case JumpCondType::GREATER: return CmpType::Greater;
        case JumpCondType::GREATER_OR_EQUAL: return CmpType::GreaterOrEqual;
        case JumpCondType::LESS: return CmpType::Less;
        case JumpCondType::LESS_OR_EQUAL: return CmpType::LessOrEqual;
    }
    PANIC()
}

CmpType X86Assembler::toCmpType2(JumpCondType it) {
    switch (it) {
        case JumpCondType::EQUALS: return CmpType::Equal;
        case JumpCondType::NOT_EQUALS: return CmpType::NotEqual;
        case JumpCondType::GREATER: return CmpType::Above;
        case JumpCondType::GREATER_OR_EQUAL: return CmpType::AboveEqual;
        case JumpCondType::LESS: return CmpType::Bellow;
        case JumpCondType::LESS_OR_EQUAL: return CmpType::BellowOrEqual;
    }
    PANIC()
}