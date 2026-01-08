#pragma once
#include "../utils/utils.h"

struct ElfPRogram {
    enum Type : u32 {
        Null = 0,
        LOAD = 1,
    };

    constexpr static u32 FLAG_R = 1;
    constexpr static u32 FLAG_W = 2;
    constexpr static u32 FLAG_X = 4;

    span <u8> data;
    Type type = Type::Null;
    u32 pageFlags = 0;
    u64 virtAddress = 0;
    u64 physAddress = 0;
    u64 alignment = 0;
};

struct ElfReAllocation {
    enum Type : u32 {
        R_X86_64_NONE = 0,
        R_X86_64_64 = 1, // S + A
        R_AMD64_PC32 = 2, // S + A - P => SOURCE+ADEND - IP
    };

    u64 address;
    u32 type;
    u32 symbol;
    i64 addend;
};

struct ElfSymbol {
    enum Type : u8 {
        None = 0,
        Object = 1,
        Func = 2,
        Section = 3,
    };

    enum Bind : u8 {
        Local = 0 << 4,
        Global = 1 << 4,
    };

    enum Other : u8 {
        Default = 0,
        Internal = 1,
        Hidden = 2,
        Protected = 3
    };

    u32 st_name;
    // Type type = Type::None;
    // Bind bind = Bind::Local;
    u8 info;
    Other st_other = Other::Default;
    u16 st_shndx = 0; // index of relevant section
    u64 st_value = 0; // offset
    u64 st_size = 0; // idk
};

struct ElfSection {
    enum Type : u32 {
        Null = 0,
        ProgramBits = 1,
        SYM_TABLE = 2,
        STRING_TABLE = 3,
        REALOCATION_ADENTS = 4,
    };

    constexpr static u64 FLAG_WRITABLE = 1;
    constexpr static u64 FLAG_ALLOC = 2;
    constexpr static u64 FLAG_EXECUTABLE = 4;
    constexpr static u64 FLAG_STRINGS = 0x20;

    u32 nameId = 0;
    Type type = Type::Null;
    u64 flags = 0;
    u64 virtAddress = 0;
    span <u8> data;
    u32 link = 0;
    u32 info = 0;
    u64 align = 0;
    u64 entrySize = 0;

    // mine
    u16 SECTION_ID;
};

struct Relocation {
    u64 offset;

};

struct ElfBuilder {
    enum Type : u16 {
        NONE = 0,
        REL = 1,
        EXEC = 2,
        DYN = 3
    };

    enum Isa : u16 {
        None = 0,
        amd64 = 0x3E
    };

    ElfBuilder() {
        // ???????????????
        {
            auto sec = this->putSection();
            sec->type = ElfSection::Type::Null;
            sec->nameId = 0;
        }
    }

    vector <u8> bytes;
    vector <unique_ptr<ElfPRogram>> programs;
    vector <unique_ptr<ElfSection>> sections;
    vector <string> sectionNames;

    Type type = Type::NONE;
    Isa isa = Isa::None;
    u32 version = 1;
    u64 entryPoint = 0;
    u32 flags = 0;
    u16 stringSectionId = 0;

    void pushBack(u8 v) {
        bytes.push_back(v);
    }

    template<class T>
    void writeImmValue(T value) {
        auto ptr = (u8 * ) & value;
        for (auto i = 0u; i < sizeof(T); i++) {
            pushBack(ptr[i]);
        }
    }

    ElfPRogram *putProgram() {
        programs.push_back(make_unique<ElfPRogram>());
        return programs.back().get();
    }

    ElfSection *putSection() {
        sections.push_back(make_unique<ElfSection>());
        sections.back().get()->SECTION_ID = sections.size() - 1;
        return sections.back().get();
    }

    void writeProgramHeader() {
        TODO();
    }

    void build() {
        writeFileHeader();
        writeElfHeader();
        for (const auto &program: programs) {
            writeProgram(program.get());
        }
        for (const auto &section: sections) {
            writeSection(section.get());
        }
        copyData();
    }

    void writeFileHeader() {
        bool is64Bit = true;
        bool isLitleEndian = true;

        pushBack(0x7F);
        pushBack('E');
        pushBack('L');
        pushBack('F');

        pushBack(is64Bit ? 2 : 1); // is64bit?
        pushBack(isLitleEndian ? 1 : 2);
        pushBack(1); // ELF version
        pushBack(0x0); // OS abi 3=linux
        for (auto _i: views::iota(0u, 8u)) { // padding
            (void) _i;
            pushBack(0);
        }
    }

    constexpr static size_t PROGRAM_SIZE = 56;
    constexpr static size_t SECTION_SIZE = 64;
    constexpr static size_t ELF_HEADER_SIZE = 0x40;

    void writeElfHeader() {
        writeImmValue<u16>(type); // type 2=executable
        writeImmValue<u16>(isa); // ISA, 0x3E=x86_64
        // 20B
        writeImmValue<u32>(version); // ELF version
        // 24B
        writeImmValue<u64>(entryPoint); // entry point address
        writeImmValue<u64>(programs.empty() ? 0 : ELF_HEADER_SIZE); // offset to program header from file start
        writeImmValue<u64>(
                ELF_HEADER_SIZE + programs.size() * PROGRAM_SIZE); // offset to section header from file start
        // 48B
        writeImmValue<u32>(flags); // FLAGS
        // 52B
        writeImmValue<u16>(ELF_HEADER_SIZE); // ELF header size
        writeImmValue<u16>(programs.empty() ? 0 : PROGRAM_SIZE); // size of program header
        writeImmValue<u16>(programs.size()); // number of entries in program header
        writeImmValue<u16>(sections.empty() ? 0 : SECTION_SIZE); // size of section header
        writeImmValue<u16>(sections.size()); // number of entries in section header
        writeImmValue<u16>(stringSectionId); // section header index for strings
    }

    void writeSection(ElfSection *section) {
        writeImmValue<u32>(section->nameId);
        writeImmValue<u32>(section->type);
        writeImmValue<u64>(section->flags);
        writeImmValue<u64>(section->virtAddress);
        writeImmValue<u64>(0); // offset of data in file
        writeImmValue<u64>(section->data.size()); // size of section in file
        writeImmValue<u32>(section->link);
        writeImmValue<u32>(section->info);
        writeImmValue<u64>(section->align);
        writeImmValue<u64>(section->entrySize);
        // 8*8 = 64
    }

    void writeProgram(ElfPRogram *program) {
        writeImmValue<u32>(program->type); // type, 1=loadable
        writeImmValue<u32>(program->pageFlags); //flags 1=X, 2=2W, 4=R

        writeImmValue<u64>(0); // offset of actual data in file
        writeImmValue<u64>(program->virtAddress); // virtual address of segment
        writeImmValue<u64>(program->physAddress); // physical address of segment
        writeImmValue<u64>(program->data.size()); // size of segment in file
        writeImmValue<u64>(program->data.size()); // size of segment in memory
        writeImmValue<u64>(program->alignment); // aligment, 0|1 = none
        // 7*8 = 56
    }

    u64 *getProgramPtr(size_t id) {
        return (u64 * )(((bytes.data() + ELF_HEADER_SIZE) + (id * PROGRAM_SIZE)) + 8);
    }

    u64 *getSectionPtr(size_t id) {
        return (u64 * )((bytes.data() + ELF_HEADER_SIZE) + (programs.size() * PROGRAM_SIZE) + (id * SECTION_SIZE) + 24);
    }

    u64 currentOffset() {
        return bytes.size();
    }

    void putSectionNames() {
        for (auto name: sectionNames) {
            bytes.insert(bytes.end(), name.begin(), name.end());
            bytes.push_back(0);
        }
    }

    void copyData() {
        for (const auto &[i, program]: programs | views::enumerate) {
            *getProgramPtr(i) = currentOffset();
            bytes.insert(bytes.end(), program->data.begin(), program->data.end());
        }
        for (const auto &[i, section]: sections | views::enumerate) {
            *getSectionPtr(i) = (section->data.size() == 0) ? 0 : currentOffset();
            bytes.insert(bytes.end(), section->data.begin(), section->data.end());
        }
    }

    void elfMagic() {
        // 16B
        writeImmValue<u16>(0x2); // type 2=executable
        writeImmValue<u16>(0x3E); // ISA, 0x3E=x86_64
        // 20B
        writeImmValue<u32>(1); // ELF version
        // 24B
        writeImmValue<u64>(0x400078); // entry point address
        writeImmValue<u64>(0x40); // offset to program header from file start
        writeImmValue<u64>(0x0); // offset to section header from file start
        // 48B
        writeImmValue<u32>(0x0); // FLAGS
        // 52B
        writeImmValue<u16>(0x40); // ELF header size
        writeImmValue<u16>(0x38); // size of program header
        writeImmValue<u16>(0x1); // number of entries in program header
        writeImmValue<u16>(0x0); // size of section header
        writeImmValue<u16>(0x0); // number of entries in section header
        writeImmValue<u16>(0x0); // section header index for strings
        // 64B


        // program header
        writeImmValue<u32>(0x1); // type, 1=loadable
        writeImmValue<u32>(0x1 | 0x4); //flags 1=X, 2=2W, 4=R
        writeImmValue<u64>(0x78); // offset of actual data in file
        writeImmValue<u64>(0x400078); // virtual address of segment
        writeImmValue<u64>(0x0); // physical address of segment
        writeImmValue<u64>(16); // size of segment in file
        writeImmValue<u64>(16); // size of segment in memory
        writeImmValue<u64>(0x1000); // aligment, 0|1 = none
        // 56B

        // bytes
        unsigned char opcodes[] = {
                0x48, 0xc7, 0xc0, 0x3c, 0x00, 0x00, 0x00, 0x48,
                0xc7, 0xc7, 0x45, 0x0, 0x0, 0x0, 0x0f, 0x05,
        };
        for (auto i = 0u; i < 16; i++) {
            pushBack(opcodes[i]);
        }
        /*   for (auto i = 0u; i < 16000; i++) {
               pushBack(0xF4);
           }*/
    }

    struct StringsBuilder {
        std::vector <u8> table;
        std::map <std::string, size_t> lookup;

        StringsBuilder() {
            table.push_back(0);
        }

        size_t getString(std::string_view name) {
            std::string ss(name);
            if (lookup.contains(ss)) return lookup[ss];

            auto idex = table.size();

            table.insert(table.end(), name.begin(), name.end());
            table.push_back(0);

            lookup[ss] = idex;

            return idex;
        }

        void bind(ElfSection *sec) {
            sec->type = ElfSection::Type::STRING_TABLE;
            sec->data = {table.data(), table.size()};
            sec->align = 1;
        }
    };

    struct SymbolBuilder {
        std::vector<ElfSymbol> symbols;
        bool isFreezed = false;

        SymbolBuilder() {
            symbols.push_back(ElfSymbol{0, (u8) ElfSymbol::Type::None | ElfSymbol::Bind::Local, ElfSymbol::Other::Default, 0, 0, 0});
        }

        void putGlobalSymbol(u32 name, u16 sectionId, u32 offset) {
            assert(not isFreezed);
            auto sym = ElfSymbol{(u32) name, (u8) ElfSymbol::Type::None | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, sectionId, offset, 0};

            symbols.push_back(sym);
        }

        void putLocalSection(u32 name, u16 sectionId) {
            assert(not isFreezed);
            auto sym = ElfSymbol{(u32) name, (u8) ElfSymbol::Type::Section | ElfSymbol::Bind::Global, ElfSymbol::Other::Default, sectionId, 0, 0};

            symbols.push_back(sym);
        }

        void bind(ElfSection *sec, u32 stringsSection) {
            sec->type = ElfSection::Type::SYM_TABLE;
            sec->link = stringsSection;

            sec->data = {(u8 *) symbols.data(), symbols.size() * sizeof(ElfSymbol)};
            sec->entrySize = sizeof(ElfSymbol);
            sec->info = 1; // index of first non local symbol
            sec->align = 1;
        }

        void freeze() {
            isFreezed = true;
        }

        std::optional <u32> getSymbolIdByNameId(size_t nameId) {
            assert(isFreezed);

            for (auto i = 0ul; i < symbols.size(); i++) {
                auto sym = symbols[i];
                if (sym.st_name == nameId) return i;
            }

            return {};
        }
    };

    class BytesBuilder {
        std::vector<u8> bytes;
      public:
        size_t append(std::span<u8> data) {
            auto idex = bytes.size();
            bytes.insert(bytes.end(), data.begin(), data.end());
            return idex;
        }

        void bind(ElfSection *sec, size_t align, size_t flags1) {
            sec->type = ElfSection::Type::ProgramBits;
            sec->data = bytes;
            sec->align = align;
            sec->flags = flags1;
        }
    };

    struct RelocationBuilder {
        vector <ElfReAllocation> relocation;

        RelocationBuilder() {

        }

        void addRealocation(size_t offset, ElfReAllocation::Type type1, u32 symbolId, i64 adend) {
            relocation.push_back(ElfReAllocation{offset, type1, symbolId, adend});
        }

        void bind(ElfSection *sec, u16 namesSectionId, u16 bytesSectionId) {
            sec->type = ElfSection::Type::REALOCATION_ADENTS;
            sec->data = {(u8 *) relocation.data(), relocation.size() * sizeof(ElfReAllocation)};
            sec->align = 8;
            sec->link = namesSectionId;
            sec->info = bytesSectionId;
            sec->entrySize = sizeof(ElfReAllocation);
        }
    };
};

inline void stupidElf1() {
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
    prog->alignment = 0x1000;

    builder.build();
}