#include <fstream>
#include "X86mc.h"
#include "definitions.h"
#include "Register.h"

void X86mc::writeModRM(const x86::X64Register &dest, const x86::X64Register &src, bool isRegDirect, bool isNoOffset, u8 orFlag) {
    u8 base = 0xC0; // IDK WHY dis

    if (!isRegDirect && !isNoOffset)
        base = 0x40;
    else if (isNoOffset)
        base = 0x0;

    if (orFlag < 4) base = static_cast<u8>(orFlag << 6_u8);

    // cout << "aaa " << hex << (int )base << endl;

    base |= static_cast<u8>(dest.getEncoding() << 3_u8);
    base |= src.getEncoding();

    // cout << "aaa " << hex << (int )base << endl;

    pushBack(base);
}

void X86mc::writeModMR(const x86::X64Register &dest, const x86::X64Register &src, bool isRegDirect, bool isNoOffset) {
    u8 base = 0xC0; // IDK WHY dis

    if (!isRegDirect && !isNoOffset)
        base = 0x40;
    else if (isNoOffset)
        base = 0x0;

    base |= dest.getEncoding();
    base |= static_cast<u8>(src.getEncoding() << 3);

    pushBack(base);
}

void X86mc::pushBack(u8 value) {
    // cout << hex << (int )value << endl;
    bytes.push_back(value);
}

void X86mc::writeRegInst(SusX64Instruction inst, const x86::X64Register &dest, const x86::X64Register &src) {
    writeRex(true, dest.isExt(), src.isExt());

    writeSusInst(inst);

    writeModRM(dest, src);
}

void X86mc::writeRegInst(X64Instruction inst, const x86::X64Register &dest, const x86::X64Register &src) {
    writeRex(true, dest.isExt(), src.isExt());
    pushBack(static_cast<u8>(inst));
    writeModRM(dest, src);
}

void X86mc::writeRex(bool immediate64, bool extDest, bool extSrc, bool extIndex) {
    u8 value = 0b01000000;

    value |= extSrc; // 1
    value |= extIndex << 1; // 2
    value |= extDest << 2; // 4
    value |= immediate64 << 3; // 8

    pushBack(value);
}

void X86mc::writeMRCode(X64Instruction inst) {
    pushBack((u8)inst-2);
}

void X86mc::writeRMCode(X64Instruction inst) {
    pushBack((u8)inst);
}

void X86mc::setIfFlag(x86::X64Register dest, CmpType type) {
    pushBack((u8)X64Instruction::setCC);
    pushBack(0x90 | (u8)type);
    writeRegInst(SusX64Instruction::movzx8, dest, dest);
}

void X86mc::movReg(const x86::X64Register& dest, const x86::X64Register& src) {
    if (dest == src) return;
    writeRegInst(X64Instruction::mov, dest, src);
}

ImmSpace X86mc::writeStack(int byteOffset, const x86::X64Register& reg) {
    return writePtr(x86::Rsp, reg, byteOffset);
}

u8 X86mc::applyRegs(unsigned char value, const x86::X64Register& dest, const x86::X64Register& src) {
    value = applyDestReg(value, dest);
    value = applySrcReg(value, src);

    return value;
}

ImmSpace X86mc::writeJmp(CmpType jmp, unsigned char offset) {
    pushBack((u8)X64Instruction::baseJmp+(u8)jmp);

    return writeImmValue<u8>(offset);
}

void X86mc::writeMovF(const x86::X64Register& dest, double value) {
    mov(dest, std::bit_cast<size_t>(value));
}

u8 X86mc::applyRegExt(unsigned char value, const x86::X64Register& dest, const x86::X64Register& src) {
    value = applyDestExt(value, dest);
    value = applySrcExt(value, src);

    return value;
}

u8 X86mc::applyDestExt(unsigned char value, const x86::X64Register& reg) {
    if (reg.isExt()) {
        value |= 1;
    }

    return value;
}

u8 X86mc::applySrcExt(unsigned char value, const x86::X64Register& reg) {
    if (reg.isExt()) {
        value |= 1 << 2;
    }

    return value;
}

ImmSpace X86mc::currentImm(size_t sizeBytes) const {
    return ImmSpace(bytes.size(), sizeBytes);
}

ImmSpace X86mc::writeJmp(unsigned char offset) {
    pushBack(0xeb);

    return writeImmValue<u8>(offset);
}

void X86mc::cmov(CmpType jmpType, const x86::X64Register& dest, const x86::X64Register& src) {
    writeRex(true, dest.isExt(), src.isExt());
    pushBack(0x0f);
    pushBack(0x40|(u8)jmpType);
    writeModRM(dest, src);
}

void X86mc::pop(const x86::X64Register& dest) {
    if (dest.isExt()) writeRex(false, false, true);

    pushBack(applyDestReg(0x58, dest));
}

void X86mc::cmpI(CmpType type, const x86::X64Register& dest, const x86::X64Register& left, const x86::X64Register& right) {
    writeRegInst(X64Instruction::cmp, left, right);
    setCC(dest, type);
    writeMovZX8(dest, dest);
}

void X86mc::push(const x86::X64Register& src) {
    if (src.isExt()) writeRex(false, false, true);

    pushBack(applyDestReg(0x50, src));
}

void X86mc::makeIsTrue(const x86::X64Register& reg) {
    // 48 83 f8 01             cmp    rax, 0x1
    // 49 83 f8 01             cmp    r8,  0x1
    pushBack(applyDestExt(0x48, reg));
    pushBack(0x83);
    pushBack(applyDestReg(0xf8, reg));
    pushBack(0x1);
}

void X86mc::writeSimple(SimpleX64Instruction inst) {
    pushBack(static_cast<u8>(inst));
}

void X86mc::writeSimpleRex(SimpleX64Instruction inst) {
    writeRex();
    pushBack(static_cast<unsigned char>(inst));
}

void X86mc::call(const x86::X64Register& reg) {
    if (reg.isExt()) writeRex(false, false, true);
    pushBack(0xff);
    pushBack(applyDestReg(0xd0, reg));
}

u8 X86mc::applyDestReg(unsigned char value, const x86::X64Register& reg) {
    return value | reg.getEncoding();
}

u8 X86mc::applySrcReg(unsigned char value, const x86::X64Register& reg) {
    return value | reg.getEncoding() << 3;
}

void X86mc::writeBytes(unsigned char* data, size_t amount) {
    for (auto i = 0u; i < amount; i++) {
        pushBack(data[i]);
    }
}

ImmSpace X86mc::mov(const x86::X64Register& dest, size_t value) {
    u8 a = applyDestExt(0x48, dest);
    u8 b = applyDestReg(0xb8, dest);

    pushBack(a);
    pushBack(b);

    return writeImmValue(value);
}

ImmSpace X86mc::writePtr(const x86::X64Register& dst, const x86::X64Register& src, int offset, SibScale scale) {
    writeRex(true, src.isExt(), dst.isExt());
    writeMRCode(X64Instruction::mov);

    return someOffsetStuffForMov(src, dst, offset);
}

// FIXME src dest is swaped
ImmSpace X86mc::readPtr(const x86::X64Register& dst, const x86::X64Register& src, int offset, SibScale scale) {
    writeRex(true, dst.isExt(), src.isExt());
    writeRMCode(X64Instruction::mov);

    return someOffsetStuffForMov(dst, src, offset);
}

ImmSpace X86mc::readStack(int byteOffset, const x86::X64Register& tgt) {
    return readPtr(tgt, x86::Rsp, byteOffset);
}

void X86mc::writeMovZX8(const x86::X64Register& dest, const x86::X64Register& src) {
    writeRex(true, dest.isExt(), src.isExt());
    writeSusInst(SusX64Instruction::movzx8);
    writeModRM(dest, src);
}

void X86mc::writeSusInst(SusX64Instruction inst) {
    u8* ptr = reinterpret_cast<unsigned char*>(&inst);

    for (auto i = 0; i < 8; i++) {
        if (*(ptr+i) == 0) break;

        pushBack(*(ptr+i));
    }
}

void X86mc::setCC(const x86::X64Register& dest, CmpType type) {
    writeRex(false, false, dest.isExt());
    pushBack(0x0f);
    pushBack(0x90 | static_cast<u8>(type));
    writeModMR(dest, x86::Zero); // Rax bcs its bit representation is 0
}

ImmSpace X86mc::addImm32(const x86::X64Register& dest, int value) {
    writeRex(true, false, dest.isExt());
    pushBack(0x81);
    writeModRM(x86::Rax, dest);
    return writeImmValue(value);
}

ImmSpace X86mc::subImm32(const x86::X64Register& dest, i32 value) {
    writeRex(true, false, dest.isExt());
    pushBack(0x81);
    // NOTE /5, /4 in mod rm specifies dest bytes :D
    writeModRM(x86::Rbp, dest);
    return writeImmValue(value);
}

void X86mc::signedDivide(const x86::X64Register& divisor) {
    writeRex(true, false, divisor.isExt());
    pushBack(0xF7);
    writeModRM(x86::Rdi, divisor);
}

void X86mc::cqo() {
    writeRex(true);
    pushBack(0x99);
}

ImmSpace X86mc::writeJmp(u32 offset) {
    pushBack(0xE9);
    return writeImmValue(offset);
}

ImmSpace X86mc::writeJmp(CmpType jmp, u32 offset) {
    pushBack(0x0F);
    pushBack(0x80 | static_cast<u8>(jmp));

    return writeImmValue(offset);
}

void X86mc::shiftLeftByCl(const x86::X64Register& dest) {
    writeRex(true, false, dest.isExt());
    pushBack(0xD3);
    pushBack(0xE0 | dest.getEncoding());
}

void X86mc::shiftRightByCl(const x86::X64Register& dest) {
    writeRex(true,false, dest.isExt());
    pushBack(0xD3);
    pushBack(0xF8 | dest.getEncoding());
}

void X86mc::writeSIB(SibScale scale, const x86::X64Register& base, const x86::X64Register& index) {
    u8 scalePart = static_cast<u8>(scale) << 6;
    u8 indexPart = index.getEncoding() << 3;
    u8 basePart = base.getEncoding();

    u8 finalByte = scalePart | basePart | indexPart;
    pushBack(finalByte);
}

// FIXME this doesnt work
void X86mc::callMC(size_t address) {
    writeRex(true);
    pushBack(0xff);
    writeModRM(x86::Three, x86::Rcx, true, true, true);
    writeImmValue(address);
}

ImmSpace X86mc::someOffsetStuffForMov(x86::X64Register dst, x86::X64Register src, int32_t offset) {
    auto srcEncoding = src.getEncoding();

    bool is32BitOffset = offset > 127 || offset < -128;
    bool isRspEncoding = srcEncoding == x86::Rsp.getEncoding();

    // rbp encoding + rm 0b00 => rip indexing ... we don't want that, so we will do 8b imm
    bool isRbpEncoding = srcEncoding == x86::Rbp.getEncoding();

    ModRmType type;
    if (offset == 0 && !isRbpEncoding) {
        type = ModRmType::DEREF;
    } else if (!is32BitOffset) {
        type = ModRmType::DEREF_8;
    } else {
        type = ModRmType::DEREF_32;
    }

    writeModRm(type, dst, src);

    if (isRspEncoding) writeSIB(SibScale::One, x86::Rsp, x86::Rsp);

    switch (type) {
        case ModRmType::DEREF:
            return {0,0};
            break;
        case ModRmType::DEREF_8:
            return writeImmValue(static_cast<char>(offset));
            break;
        case ModRmType::DEREF_32:
            return writeImmValue(offset);
            break;
        case ModRmType::NO_DEREF:
            PANIC();
    }
    PANIC()
}

void X86mc::leave() {
    writeSimple(SimpleX64Instruction::leave);
}

void X86mc::garbageMemCpy(const x86::X64Register& dest, const x86::X64Register& src, size_t amountBytes, const x86::X64Register& tmp, int srcOffset, int dstOffset) {
    if (amountBytes == 1) {
        read1(tmp, src, srcOffset);
        write1(dest, tmp, dstOffset);
        return;
    }
    if (amountBytes == 2 || amountBytes == 4) {
        TODO();
    }

    assert(isAligned(amountBytes, 8));

    for (auto offset = 0u; offset < amountBytes; offset += 8) {
        readPtr(tmp, src, offset + srcOffset);
        writePtr(dest, tmp, offset+dstOffset);
    }
}

ImmSpace X86mc::lea(const x86::X64Register& dest, const x86::X64Register& value, int32_t offset) {
    writeRex(true, dest.isExt(), value.isExt());
    pushBack(0x8D);

    if (value == x86::Rsp) {
        pushBack(0b10 << 6 | dest.getEncoding() << 3 | 0b100); // FIXME isnt this encoded RSP? => this line is the same is in else?
        pushBack(0x24); // this is SIB right? :)
    }
    else {
        pushBack(0b10 << 6 | dest.getEncoding() << 3 | value.getEncoding());
    }
    return writeImmValue(offset);
}

ImmSpace X86mc::leaRip(const x86::X64Register& dest, int32_t offset) {
    writeRex(true, dest.isExt());
    pushBack(0x8D);

    pushBack(dest.getEncoding() << 3 | 0b101);

    return writeImmValue(offset);
}

void X86mc::nop() {
    writeSimple(SimpleX64Instruction::nop);
}

void X86mc::getStackPtr(const x86::X64Register& reg, int stackOffset) {
    lea(reg, x86::Rsp, stackOffset);
    // mov(reg, stackOffset);
    // writeRegInst(X64Instruction::add, reg, x86::Rsp);
}

void X86mc::ret() {
    writeSimple(SimpleX64Instruction::ret);
}

void X86mc::addFloat(const x86::X64Register& dst, const x86::X64Register& lhs, x86::X64Register rhs, bool is64Bit) {
    movq(0, lhs, is64Bit);
    movq(1, rhs, is64Bit);
    addsd(0, 1, is64Bit);
    movq(dst, 0, is64Bit);
}

void X86mc::subFloat(const x86::X64Register& dst, const x86::X64Register& lhs, x86::X64Register rhs, bool is64Bit) {
    movq(0, lhs, is64Bit);
    movq(1, rhs, is64Bit);
    subsd(0, 1, is64Bit);
    movq(dst, 0, is64Bit);
}

void X86mc::mulFloat(const x86::X64Register& dst, const x86::X64Register& lhs, x86::X64Register rhs, bool is64Bit) {
    movq(0, lhs, is64Bit);
    movq(1, rhs, is64Bit);
    mulsd(0, 1, is64Bit);
    movq(dst, 0, is64Bit);
}

void X86mc::divFloat(const x86::X64Register& dst, const x86::X64Register& lhs, x86::X64Register rhs, bool is64Bit) {
    movq(0, lhs, is64Bit);
    movq(1, rhs, is64Bit);
    divsd(0, 1, is64Bit);
    movq(dst, 0, is64Bit);
}

void X86mc::movq(unsigned char dstFpu, x86::X64Register src, bool is64Bit) {
    // 66 48 0f 6e c0          movq   xmm0, rax
    // 66 49 0f 6e c0          movq   xmm0, r15
    // lower 3 bits src
    // upper 3 bits dst

    pushBack(0x66);
    writeRex(is64Bit, false, src.isExt());
    pushBack(0x0F);
    pushBack(0x6E);
    pushBack(0xC0 | src.getEncoding() | (dstFpu << 3));
}

void X86mc::movq(x86::X64Register dst, unsigned char srcFpu, bool is64Bit) {
    // 66 48 0f 7e c0          movq   rax,xmm0
    // 66 48 0f 7e c8          movq   rax,xmm1
    // lower 3 bits dNPst
    // upper 3 bits src

    pushBack(0x66);
    writeRex(is64Bit, false, dst.isExt());
    pushBack(0x0F);
    pushBack(0x7E);
    pushBack(0xC0 | dst.getEncoding() | (srcFpu << 3));
}

void X86mc::addsd(unsigned char dstFpu, unsigned char srcFpu, bool is64Bit) {
    // f2 0f 58 c0             addsd  xmm0,xmm0
    // lower 3 bits src
    // upper 3 bits dst

    pushBack(0xF2);
    pushBack(0x0F);
    pushBack(0x58);
    pushBack(0xC0 | (dstFpu << 3) | srcFpu);
}

void X86mc::subsd(unsigned char dstFpu, unsigned char srcFpu, bool is64Bit) {
    // f2 0f 5c c0             subsd  xmm0,xmm0
    // lower 3 bits src
    // upper 3 bits dst

    pushBack(0xF2 | !is64Bit);
    pushBack(0x0F);
    pushBack(0x5C);
    pushBack(0xC0 | (dstFpu << 3) | srcFpu);
}

void X86mc::mulsd(unsigned char dstFpu, unsigned char srcFpu, bool is64Bit) {
    // f2 0f 59 c0             mulsd  xmm0,xmm0
    // lower 3 bits src
    // upper 3 bits dst

    pushBack(0xF2 | !is64Bit);
    pushBack(0x0F);
    pushBack(0x59);
    pushBack(0xC0 | (dstFpu << 3) | srcFpu);
}

void X86mc::divsd(unsigned char dstFpu, unsigned char srcFpu, bool is64Bit) {
    // f2 0f 5e c0             divsd  xmm0,xmm0
    // lower 3 bits src
    // upper 3 bits dst

    pushBack(0xF2 | !is64Bit);
    pushBack(0x0F);
    pushBack(0x5E);
    pushBack(0xC0 | (dstFpu << 3) | srcFpu);
}
