#include <vector>
#include <array>
#include "gen64/X86mc.h"
#include "utils/utils.h"
#include "utils/code_gen.h"
#include "gen64/elf.h"

using namespace std;

void idk() {
    unsigned char opcodes[] = {
            0x48, 0xc7, 0xc0, 0x3c, 0x00, 0x00, 0x00, 0x48,
            0xc7, 0xc7, 0x45, 0x0, 0x0, 0x0, 0x0f, 0x05,
    };

    ElfBuilder builder;
    builder.type = ElfBuilder::Type::EXEC;
    builder.entryPoint = 0x400078;
    builder.isa = ElfBuilder::Isa::amd64;

    auto prog = builder.putProgram();
    prog->type = ElfPRogram::Type::LOAD;
    prog->pageFlags = ElfPRogram::FLAG_R | ElfPRogram::FLAG_X;
    prog->virtAddress = 0x400078;
    prog->data = opcodes;

    builder.build();

    auto filePath = writeBytesToTempFile(builder.bytes, "a");
    println("{}", filePath);
}

void lmao2() {
    unsigned char opcodes[] = {
            0x49, 0xba, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
            0xaa, 0xaa, 0x48, 0xc7, 0xc7, 0x55, 0x00, 0x00,
            0x00, 0x41, 0xff, 0xd2, 0x48, 0xc7, 0xc7, 0x77,
            0x00, 0x00, 0x00, 0x41, 0xff, 0xd2, 0x48, 0xc7,
            0xc7, 0x55, 0x00, 0x00, 0x00, 0x41, 0xff, 0xd2,
            0x48, 0xc7, 0xc7, 0x0a, 0x00, 0x00, 0x00, 0x41,
            0xff, 0xd2, 0x48, 0xc7, 0xc0, 0x3c, 0x00, 0x00,
            0x00, 0x48, 0xc7, 0xc7, 0x45, 0x00, 0x00, 0x00,
            0x0f, 0x05,
    };

    ElfBuilder builder;
    builder.type = ElfBuilder::Type::REL;
    builder.isa = ElfBuilder::Isa::amd64;
    builder.stringSectionId = 4;

    // ???????????????
    {
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::Null;
        sec->nameId = 6;
    }

    // .text
    {
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::ProgramBits;
        sec->nameId = 1;
        sec->data = opcodes;
        sec->flags = ElfSection::FLAG_ALLOC | ElfSection::FLAG_EXECUTABLE;
        sec->align = 1;
    }

    // .symtab
    {
        static ElfSymbol pepa[] = {
                ElfSymbol{8, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Local, ElfSymbol::Other::Default, 0, 0, 0},  // ?????????
                ElfSymbol{1, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, 0, 0, 0}, // putchar
                ElfSymbol{9, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, 1, 0, 0} // _start
        };

        auto sec = builder.putSection();
        sec->type = ElfSection::Type::SYM_TABLE;
        sec->nameId = 7;
        sec->link = 3; // string section
        sec->data = {(u8*)pepa, sizeof pepa};
        sec->entrySize = sizeof(ElfSymbol);
        sec->info = 1;
        sec->align = 1;
    }

    // .strtab
    {
        static u8 pepa[] = "\0putchar\0_start\0";
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::STRING_TABLE;
        sec->nameId = 15;
        sec->data = {pepa, sizeof pepa};
        sec->align = 1;
    }

    // .shstrtab
    {
        static u8 pepa[] = "\0.text\0.symtab\0.strtab\0.shstrtab\0.rela.text\0";
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::STRING_TABLE;
        sec->nameId = 23;
        sec->data = {pepa, sizeof pepa};
        sec->align = 1;
    }
    // .rela.text
    {
        static ElfReAllocation pepa[] = {
                ElfReAllocation{2, 1, ElfReAllocation::Type::R_X86_64_64, 0} // give me putchar
        };
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::REALOCATION_ADENTS;
        sec->nameId = 33;
        sec->data = {(u8*)pepa, sizeof pepa};
        sec->align = 8;
        sec->link = 2; // .symtab
        sec->info = 1; // .text
        sec->entrySize = sizeof(ElfReAllocation);
    }


    builder.build();

    auto filePath = writeBytesToTempFile(builder.bytes, "b");
    println("{}", filePath);
}

void lmao3() {
    unsigned char opcodes[] = {
            0x58, 0x48, 0x8b, 0x34, 0x24, 0x48, 0x8d, 0x7c,
            0xc4, 0x01, 0x50, 0x54, 0x48, 0x83, 0xe4, 0xf0,
            0x49, 0xba, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb,
            0xbb, 0xbb, 0x41, 0xff, 0xd2, 0x49, 0xba, 0xaa,
            0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x48,
            0xc7, 0xc7, 0x55, 0x00, 0x00, 0x00, 0x41, 0xff,
            0xd2, 0x48, 0xc7, 0xc7, 0x77, 0x00, 0x00, 0x00,
            0x41, 0xff, 0xd2, 0x48, 0xc7, 0xc7, 0x55, 0x00,
            0x00, 0x00, 0x41, 0xff, 0xd2, 0x48, 0xc7, 0xc7,
            0x0a, 0x00, 0x00, 0x00, 0x41, 0xff, 0xd2, 0x48,
            0xc7, 0xc0, 0x3c, 0x00, 0x00, 0x00, 0x48, 0xc7,
            0xc7, 0x16, 0x00, 0x00, 0x00, 0x0f, 0x05,
    };

    ElfBuilder builder;
    builder.type = ElfBuilder::Type::REL;
    builder.isa = ElfBuilder::Isa::amd64;
    builder.stringSectionId = 4;

    // ???????????????
    {
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::Null;
        sec->nameId = 6;
    }

    // .text
    {
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::ProgramBits;
        sec->nameId = 1;
        sec->data = opcodes;
        sec->flags = ElfSection::FLAG_ALLOC | ElfSection::FLAG_EXECUTABLE;
        sec->align = 1;
    }

    // .symtab
    {
        static ElfSymbol pepa[] = {
                ElfSymbol{8, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Local, ElfSymbol::Other::Default, 0, 0, 0},  // ?????????
                ElfSymbol{1, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, 0, 0, 0}, // putchar
                ElfSymbol{9, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, 1, 0, 0}, // _start
                ElfSymbol{16, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, 0, 0, 0}, // __init_libc
        };

        auto sec = builder.putSection();
        sec->type = ElfSection::Type::SYM_TABLE;
        sec->nameId = 7;
        sec->link = 3; // string section
        sec->data = {(u8*)pepa, sizeof pepa};
        sec->entrySize = sizeof(ElfSymbol);
        sec->info = 1;
        sec->align = 1;
    }

    // .strtab
    {
        static u8 pepa[] = "\0putchar\0_start\0__init_libc\0";
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::STRING_TABLE;
        sec->nameId = 15;
        sec->data = {pepa, sizeof pepa};
        sec->align = 1;
    }

    // .shstrtab
    {
        static u8 pepa[] = "\0.text\0.symtab\0.strtab\0.shstrtab\0.rela.text\0";
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::STRING_TABLE;
        sec->nameId = 23;
        sec->data = {pepa, sizeof pepa};
        sec->align = 1;
    }
    // .rela.text
    {
        static ElfReAllocation pepa[] = {
                ElfReAllocation{31, ElfReAllocation::Type::R_X86_64_64, 1, 0}, // give me putchar
                ElfReAllocation{18, ElfReAllocation::Type::R_X86_64_64, 3, 0} // giv me __libc_start_main
        };
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::REALOCATION_ADENTS;
        sec->nameId = 33;
        sec->data = {(u8*)pepa, sizeof pepa};
        sec->align = 8;
        sec->link = 2; // .symtab
        sec->info = 1; // .text
        sec->entrySize = sizeof(ElfReAllocation);
    }


    builder.build();

    auto filePath = writeBytesToTempFile(builder.bytes, "b");
    println("{}", filePath);
}

void lmao() {
    unsigned char opcodes[] = {
            0x57, 0x48, 0xc7, 0xc0, 0x01, 0x00, 0x00, 0x00,
            0x48, 0xc7, 0xc7, 0x01, 0x00, 0x00, 0x00, 0x48,
            0x89, 0xe6, 0x48, 0xc7, 0xc2, 0x01, 0x00, 0x00,
            0x00, 0x0f, 0x05, 0x5f, 0xc3,
    };

    ElfBuilder builder;
    builder.type = ElfBuilder::Type::REL;
    builder.isa = ElfBuilder::Isa::amd64;
    builder.stringSectionId = 4;

    // ???????????????
    {
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::Null;
        sec->nameId = 6;
    }

    // .text
    {
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::ProgramBits;
        sec->nameId = 1;
        sec->data = opcodes;
        sec->flags = ElfSection::FLAG_ALLOC | ElfSection::FLAG_EXECUTABLE;
        sec->align = 1;
    }

    // .symtab
    {
        static ElfSymbol pepa[] = {ElfSymbol{8, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Local, ElfSymbol::Other::Default, 0, 0, 0}, ElfSymbol{1, (u8)ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, 1, 0, 0}};

        auto sec = builder.putSection();
        sec->type = ElfSection::Type::SYM_TABLE;
        sec->nameId = 7;
        sec->link = 3; // string section
        sec->data = {(u8*)pepa, sizeof pepa};
        sec->entrySize = sizeof(ElfSymbol);
        sec->info = 1;
        sec->align = 1;
    }

    // .strtab
    {
        static u8 pepa[] = "\nputchar\0";
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::STRING_TABLE;
        sec->nameId = 15;
        sec->data = {pepa, sizeof pepa};
        sec->align = 1;
    }

    // .shstrtab
    {
        static u8 pepa[] = "\0.text\0.symtab\0.strtab\0.shstrtab\0";
        auto sec = builder.putSection();
        sec->type = ElfSection::Type::STRING_TABLE;
        sec->nameId = 23;
        sec->data = {pepa, sizeof pepa};
        sec->align = 1;
    }


    builder.build();

    auto filePath = writeBytesToTempFile(builder.bytes);
    println("{}", filePath);
}

struct SimpleTarget {
    X64Register reg;
    long offset; // -1 is


};

typedef long (*pepa123)(long a);

int main() {
    vector<u8> bytes;

    X86mc mc(bytes);

/*    mc.push(X64Register::Rax);
    mc.pop(X64Register::Rax);
    mc.writeMemRead(X64Register::Rax, X64Register::Rcx, 0x10);
    mc.writeMemRead(X64Register::Rax, X64Register::Rdi, 0x10);
    mc.writeMemRead(X64Register::R8, X64Register::Rdi, 0x10);
    mc.writeMemRead(X64Register::R15, X64Register::R15, 0x10);
    mc.writeMemRead(X64Register::Rax, X64Register::R15, 0x10);
    mc.readPtr(X64Register::Rax, X64Register::R8, 0x10);*/
/*    mc.writeMemWrite(X64Register::Rax, X64Register::R10, 0x10);
    mc.writeMemWrite(X64Register::Rax, X64Register::Rsp, 0x10);
    mc.writeMemWrite(X64Register::R15, X64Register::R15, 0x10);
    mc.writeMemWrite(X64Register::Rax, X64Register::Rax, 0x10);
    mc.writeMemWrite(X64Register::Rsp, X64Register::R11, 0x10);
    mc.writePtr(X64Register::Rax, X64Register::R12, 0x10);
    mc.writeMemRead(X64Register::R8, X64Register::Rsp, 0x0);
    mc.writeMemRead(X64Register::R9, X64Register::Rsp, 0x30);
    mc.writeMemRead(X64Register::R9, X64Register::Rsp, 0x20);
    mc.writeMemRead(X64Register::R9, X64Register::Rsp, 0x28);
    mc.readPtr(X64Register::R9, X64Register::R9, 0x28);*/
    // mc.call(0x55cbdb5a568e);
    //mc.call(0x603000000c30);
    // mc.lea(X64Register::Rax, X64Register::Rax, 0x12121212);
    // mc.lea(X64Register::R15, X64Register::R15, 0x12121212);
    // mc.lea(X64Register::R15, X64Register::Rsp, 0x12121212);
    // size_t idk[] = {8u, 8u};
    // bytes = generateCall(span(idk, 2), 16, 0);

/*    mc.readStack(0x88, X64Register::Rax);
    mc.readStack(0x88, X64Register::Rcx);
    mc.readStack(0x88, X64Register::Rdx);
    mc.readStack(0x88, X64Register::R13);
    mc.writeStack(0x88, X64Register::Rax);
    mc.writeStack(0x88, X64Register::Rcx);
    mc.writeStack(0x88, X64Register::Rdx);
    mc.writeStack(0x88, X64Register::R13);*/
/*    mc.writePtr(X64Register::Rax, X64Register::Rdx, 0x88);
    mc.writePtr(X64Register::R15, X64Register::Rdi, 0x88);
    mc.writePtr(X64Register::R15, X64Register::Rdi, 0x8);
    mc.writeStack(0x88, X64Register::Rdx);
    mc.writeStack(0x8, X64Register::Rdi);

    mc.readPtr(X64Register::Rax, X64Register::Rdx, 0x88);
    mc.readPtr(X64Register::R15, X64Register::Rdi, 0x88);
    mc.readPtr(X64Register::R15, X64Register::Rdi, 0x8);
    mc.readStack(0x88, X64Register::Rdx);
    mc.readStack(0x8, X64Register::Rdi);
    mc.setCC(X64Register::Rsi, CmpType::Equal);
    mc.setCC(X64Register::Rdi, CmpType::Equal);
    mc.setCC(X64Register::Rax, CmpType::Equal);
    mc.setCC(X64Register::Rcx, CmpType::Equal);*/

/*    for (auto a : X64Register::ALL_REGS) {
        for (auto b : X64Register::ALL_REGS) {
            mc.read1(a, b, 33);
            mc.read2(a, b, 33);
            mc.read4(a, b, 33);
            mc.writeMovZX8(a, b);
        }
    }
    for (auto b : X64Register::ALL_REGS) {
        // mc.readStack(110, b);
    }*/
 /*   for (auto b : X64Register::ALL_REGS) {
        mc.relativeRead(b, 33);
        mc.relativeWrite(b, 33);
    }*/
    // mc.floatCompare(CmpType::Less, X64Register::Rax, X64Register::Rax, X64Register::Rax);
    // mc.writeStack(0x88, X64Register::Rax);
    //
    // clemc.writeStack(0x88, X64Register::R15);
    // mc.readStack(512, X64Register::R15);
    //mc.readStack(-512, X64Register::Rdi);

    // lmao();
    // lmao3();

    vector<Arg> argz{Arg::Reg(X64Register::R13)};

  /*  for (auto reg : X64Register::ALL_REGS) {
        mc.push(reg, 32);
        mc.pop(reg, 32);
    }*/

    mc.addImm32(X64Register::Rdi, 42);
    mc.movReg(X64Register::Rax, X64Register::Rdi);
    mc.ret();

    auto exec = cg::allocateJIT(bytes.size());
    std::memcpy(exec, bytes.data(), bytes.size());
    cg::makeRX(exec, bytes.size());
    auto res = (pepa123) exec;

    std::cout << res(13) << std::endl;

    // mc.writeRegMemInst(X64Instruction::Call, X64Register::Two, X64Register::R15, 32);
    mc.writeRegRipInst(X64Instruction::Call, X64Register::Two, 0xFFFF);


    // mc.invokeScuffedSYSV(Arg::Imm(0), argz, Arg::StackValue(32, 16), map<X64Register, size_t>{}, [](auto a, auto b) {});
    // mc.shiftRImm(X64Register::R12, 12);
    // mc.shiftLImm(X64Register::R12, 12);
    /*mc.movq(X64Register::R15, 0);
    mc.movq(X64Register::R15, 4, false);
    mc.comisd(1,5);
    mc.comisd(1,5, false);*/
    // mc.writeMemRegInst(X64Instruction::add, X64Register::Rsp, 0x10, X64Register::R15);
    // mc.writeRegMemInst(X64Instruction::add, X64Register::R15, X64Register::Rsp, 0x10);
    // mc.lea(X64Register::Rdi, X64Register::Rsi, 20);
    // mc.lea(X64Register::R15, X64Register::Rsi, 20);
    // mc.elfMagic();
    auto filePath = writeBytesToTempFile(bytes);
    println("{}", filePath);
    // system(stringify("readelf -a {}", filePath).c_str());
    // system(stringify("chmod +x {}", filePath).c_str());
    // system(stringify("exec {}", filePath).c_str());

    system(stringify("objdump -bbinary -mi386:x86-64:intel -D {}", filePath).c_str());
}