#pragma once

#include "elf.h"

template<typename K, typename V>
V getOr(std::map<K, V>& m, K key, V def) {
	if (m.contains(key)) {
		return m.at(key);
	} else {
		return def;
	}
}

// TODO set params? visibility? extern? ...
struct SymbolManager {
	enum class Type {
		FUNCTION,
		STRING,

		UNKNOWN
	};

	std::map<std::string, size_t> nameToId;
	std::map<size_t, std::string> idToName;
	std::map<size_t, Type> symbolType;

	size_t getId(std::string name) {
		if (not nameToId.contains(name)) {
			auto id = nameToId.size();
			nameToId[name] = id;
			idToName[id] = name;
		}

		return nameToId[name];
	}

	std::string getName(size_t id) {
		assert(idToName.contains(id));

		return idToName.at(id);
	}

	void setType(size_t id, Type type) {	
		symbolType[id] = type;
	}

	Type getType(size_t id) {
		if (not symbolType.contains(id)) return Type::UNKNOWN;
		return symbolType[id];
	}

	std::vector<size_t> symbols() {
		std::vector<size_t> res;

		for (auto [id, name] : idToName) {
			res.push_back(id);
		}

		return res;
	}

	void dump() {
		println("# symbol manager");
		for (auto [name, id] : nameToId) {
			auto type = getOr(symbolType, id, Type::UNKNOWN);
			println("- {} - {} - {}", id, name, (int)type);
		}
	}
};

struct SectionBuilder {
	SymbolManager& symbols;
	size_t offset = 0;
	std::map<size_t, std::vector<size_t>> labels; // id : offset[]
    std::map<size_t, size_t> bounds;              // id : offset
    std::vector<u8> bytes;

	void appendBytes(std::span<u8> _bytes) {
        bytes.insert(bytes.end(), _bytes.begin(), _bytes.end());
    }

    void appendCString(std::string_view s) {
    	for (auto c : s) {
    		bytes.push_back(c);
    	}
    	bytes.push_back(0);
    }

    void setEditOffset(size_t offset) {
    	this->offset = offset;
    }

    size_t getEditsOffset() {
    	return offset;
    }

    void requestLabel(size_t offset, size_t id) {
    	labels[id].push_back(this->offset+offset);
    }

    void requestLabel(size_t offset, std::string name) {
		requestLabel(offset, symbols.getId(name));
    }

    void bindLabel(size_t offset, size_t id) {
    	bounds[id] = this->offset+offset;
    }

    void bindLabel(size_t offset, std::string name) {
    	bindLabel(offset, symbols.getId(name));
    }

    void bindCurrent(size_t id) {
    	println("BINDING UwU");
    	bounds[id] = bytes.size();
    }

    size_t bindCurrent(std::string name) {
    	auto id = symbols.getId(name);
    	bindCurrent(id);

    	return id;
    }

    size_t getOffset(size_t id) {
    	assert(bounds.contains(id));
    	return bounds[id];
    }

    size_t getSize() {
    	return bytes.size();
    }

    auto withOffset(auto fn) {
    	auto oldOffset = getEditsOffset();
    	setEditOffset(getSize());

    	fn();

    	setEditOffset(oldOffset);
    }
};

struct ObjectBuilder {
	enum class Type {
		CODE,
		RDATA,
		DATA
	};

	SymbolManager* symbols = new SymbolManager();
	std::map<Type, SectionBuilder*> sections;

	SectionBuilder* getSection(Type type) {
		if (sections.contains(type)) return sections[type];
		auto b = new SectionBuilder(*symbols);
		sections.emplace(type, b);

		return b;
	}

	std::vector<u8> emitElf() {
	    auto codeSection = getSection(Type::CODE);
	    auto readSection = getSection(Type::RDATA);

	    ElfBuilder builder;
	    builder.type = ElfBuilder::Type::REL;
	    builder.isa = ElfBuilder::Isa::amd64;

	    auto sectionNamesSection = builder.putSection();
	    builder.stringSectionId = sectionNamesSection->SECTION_ID;
	    ElfBuilder::StringsBuilder sectionNames;
	    sectionNamesSection->nameId = sectionNames.getString(".shstrtab");

	    auto symbolNamesSection = builder.putSection();
	    ElfBuilder::StringsBuilder symbolNames;
	    symbolNamesSection->nameId = sectionNames.getString(".strtab");

	    // .text
	    ElfBuilder::BytesBuilder text;
	    text.append(codeSection->bytes);
	    auto textSection = builder.putSection();
	    textSection->nameId = sectionNames.getString(".text");
	    text.bind(textSection, 1, ElfSection::FLAG_ALLOC | ElfSection::FLAG_EXECUTABLE);

	    // .rodata
	    ElfBuilder::BytesBuilder rodata;
	    rodata.append(readSection->bytes);
	    auto rodataSection = builder.putSection();
	    rodataSection->nameId = sectionNames.getString(".rodata");
	    rodata.bind(rodataSection, 1, ElfSection::FLAG_ALLOC);
	

		// .symtab
	    ElfBuilder::SymbolBuilder syms;
	    std::set<size_t> didPut;

		//// GUUUD ////
		println("AAAAAAAAA {}", codeSection->bounds.size());

	    for (auto [labelId, offset] : codeSection->bounds) {
	    	didPut.insert(labelId);
	        auto nameId = symbolNames.getString(symbols->getName(labelId));
	        syms.putGlobalSymbol(nameId, textSection->SECTION_ID, offset);
	    }

	    // bind .rodata
	    syms.putLocalSection(symbolNames.getString(".rodata"), rodataSection->SECTION_ID);

	    // add missing functions
	    for (auto [labelId, _offsets] : codeSection->labels) {
	    	if (symbols->getType(labelId) != SymbolManager::Type::FUNCTION) continue;
	    	if (didPut.contains(labelId)) continue;
	    	didPut.insert(labelId);
	        auto nameId = symbolNames.getString(symbols->getName(labelId));
	        syms.putGlobalSymbol(nameId, 0, 0);
	    }

	    for (auto labelId : symbols->symbols()) {
	    	if (symbols->getType(labelId) != SymbolManager::Type::FUNCTION) continue;
	    	if (didPut.contains(labelId)) continue;
	    	didPut.insert(labelId);
	    	auto nameId = symbolNames.getString(symbols->getName(labelId));
	        syms.putGlobalSymbol(nameId, 0, 0);
	    }

	    syms.freeze();

	    auto symbolSection = builder.putSection();
	    symbolSection->nameId = sectionNames.getString(".symtab");
	    syms.bind(symbolSection, symbolNamesSection->SECTION_ID);

	    // .rela.text
	    ElfBuilder::RelocationBuilder ralocations;
	    auto roDataSymbolId = UNWRAP(syms.getSymbolIdByNameId(symbolNames.getString(".rodata")));

	    for (auto [labelId, offsets]: codeSection->labels) {
	        auto type = symbols->getType(labelId);
	        if (type == SymbolManager::Type::FUNCTION) {
	            auto id = symbolNames.getString(symbols->getName(labelId));
	            auto symId = syms.getSymbolIdByNameId(id);

	            assert(symId.has_value());
	            assert(not offsets.empty());

	            for (auto offset: offsets) {
	            	println("requesting label {} / {}", offset, *symId);
	                ralocations.addRealocation(offset, ElfReAllocation::Type::R_AMD64_PC32, *symId, -4);
	            }
	        }
	        else if (type == SymbolManager::Type::STRING) {
	            auto labelOffset = readSection->getOffset(labelId);
	            for (auto offset: offsets) {
	                ralocations.addRealocation(offset, ElfReAllocation::Type::R_AMD64_PC32, roDataSymbolId, labelOffset-4);
	            }
	        } else {
	        	PANIC("mrda {}", symbols->getName(labelId));
	        }
	    }
	    auto realocationSection = builder.putSection();
	    realocationSection->nameId = sectionNames.getString(".rela.text");

	    ralocations.bind(realocationSection, symbolSection->SECTION_ID, textSection->SECTION_ID);

	    // initilize strings
	    symbolNames.bind(symbolNamesSection);
	    sectionNames.bind(sectionNamesSection);

	    // generate elf

	    builder.build();

	    return builder.bytes;
	}

	void emitElf(std::string path) {
		auto bytes = emitElf();
		writeBytesToFile(path, bytes);
	}
};