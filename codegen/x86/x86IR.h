#pragma once

#include "codegen/ControlFlowGraph.h"
#include "codegen/IRInstruction.h"
#include "codegen/IRInstructions.h"
#include "codegen/SSARegisterHandle.h"
#include "utils/utils.h"
#include "x86_insts.h"
#include "gen64/definitions.h"
#include <chrono>
#include <cstddef>
#include "utils/Logger.h"

struct BaseRegister {
    virtual ~BaseRegister() = default;

    template<typename T>
    T* as() {
        return dynamic_cast<T*>(this);
    }

    template<typename T>
    bool is() {
        return as<T>() != nullptr;
    }

    virtual std::string toString() const = 0;
};

struct Physical : BaseRegister {
    x86::X64Register id;
    Physical(x86::X64Register id) : id(id) {}

    std::string toString() const override {
        return id.toString();
    }
};

struct Virtual : BaseRegister {
    size_t id;
    Virtual(size_t id) : id(id) {}

    std::string toString() const override {
        return stringify("v{}", id);
    }
};

/*struct Immediate: BaseRegister {};

struct StackRef: BaseRegister {};

struct RipRef: BaseRegister {};*/

struct X86Instruction : Debuggable {
    std::vector<BaseRegister*> uses;
    std::vector<BaseRegister*> defs;
    std::vector<BaseRegister*> kills;
    X86Instruction* prev;
    X86Instruction* next;
    long orderId = 0;

    bool hasDefVirt(size_t id) {
        for (auto def: defs) {
            auto virt = def->as<Virtual>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    void rewriteDef(Virtual* old, Virtual* newReg) {
        for (auto& def : defs) {
            if (def->is<Virtual>() && def->as<Virtual>()->id == old->id) def = newReg;
        }
    }

    void rewriteUse(Virtual* old, Virtual* newReg) {
        for (auto& def : uses) {
            if (def->is<Virtual>() && def->as<Virtual>()->id == old->id) def = newReg;
        }
    }

    bool hasUseVirt(size_t id) {
        for (auto use: uses) {
            auto virt = use->as<Virtual>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    bool hasDefPhy(x86::X64Register id) {
        for (auto def: defs) {
            auto virt = def->as<Physical>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    template<typename T>
    bool is()  {
        return dynamic_cast<T*>(this) != nullptr;
    }
};

struct RET : X86Instruction {
    DEBUG_INFO2(RET)
    RET(std::initializer_list<BaseRegister*> uses) {
        this->uses = uses;
    }
};

struct FAKE_DEF : X86Instruction {
    DEBUG_INFO2(FAKE_DEF)
    FAKE_DEF(BaseRegister* def) {
        this->defs.push_back(def);
    }
};

struct FAKE_USE : X86Instruction {
    DEBUG_INFO2(FAKE_USE)
    FAKE_USE(BaseRegister* def) {
        this->uses.push_back(def);
    }
};

struct CALLRIP : X86Instruction {
    DEBUG_INFO2(CALLRIP)
    CALLRIP(std::initializer_list<BaseRegister*> defs, std::initializer_list<BaseRegister*> uses, std::initializer_list<BaseRegister*> kills, size_t id) {
        this->defs = defs;
        this->kills = kills;
        this->uses = uses;
    }

    CALLRIP(std::span<BaseRegister*> defs, std::span<BaseRegister*> uses, std::span<BaseRegister*> kills, size_t id) {
        this->defs = std::vector<BaseRegister*>{defs.begin(), defs.end()};
        this->kills = std::vector<BaseRegister*>{kills.begin(), kills.end()};
        this->uses = std::vector<BaseRegister*>{uses.begin(), uses.end()};
    }
};

struct CALLREG : X86Instruction {
    DEBUG_INFO2(CALLREG)

    BaseRegister* reg = nullptr;

    CALLREG(std::initializer_list<BaseRegister*> defs, std::initializer_list<BaseRegister*> uses, std::initializer_list<BaseRegister*> kills, BaseRegister* reg): reg(reg) {
        this->defs = defs;
        this->kills = kills;
        this->uses = uses;
        this->uses.push_back(reg);
    }

    CALLREG(std::span<BaseRegister*> defs, std::span<BaseRegister*> uses, std::span<BaseRegister*> kills, BaseRegister* reg): reg(reg) {
        this->defs = std::vector<BaseRegister*>{defs.begin(), defs.end()};
        this->kills = std::vector<BaseRegister*>{kills.begin(), kills.end()};
        this->uses = std::vector<BaseRegister*>{uses.begin(), uses.end()};
        this->uses.push_back(reg);
    }
};

struct MOVRIP : X86Instruction {
    DEBUG_INFO2(MOVRIP)
    size_t id;
    MOVRIP(BaseRegister* dst, size_t id): id(id) {
        this->defs.push_back(dst);
    }
};

struct MOV : X86Instruction {
    DEBUG_INFO2(MOV)
    MOV(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct MOVIMM : X86Instruction {
    DEBUG_INFO2(MOVIMM)
    uint64_t value;

    MOVIMM(BaseRegister* lhs, uint64_t value): value(value) {
        this->defs.push_back(lhs);
    }
};

struct MUL : X86Instruction {
    DEBUG_INFO2(MUL)
    MUL(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct SUB : X86Instruction {
    DEBUG_INFO2(SUB)
    SUB(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct ADD : X86Instruction {
    DEBUG_INFO2(ADD)
    ADD(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct XOR : X86Instruction {
    DEBUG_INFO2(XOR)
    XOR(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct AND : X86Instruction {
    DEBUG_INFO2(AND)
    AND(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct OR: X86Instruction {
    DEBUG_INFO2(OR)
    OR(BaseRegister* lhs, BaseRegister* rhs) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct ADDIMM : X86Instruction {
    DEBUG_INFO2(ADDIMM)

    long imm;

    ADDIMM(BaseRegister* lhs, long imm): imm(imm) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
    }
};

struct SUBIMM : X86Instruction {
    DEBUG_INFO2(SUBIMM)

    long imm;

    SUBIMM(BaseRegister* lhs, long imm): imm(imm) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
    }
};

struct Block;

struct JMP : X86Instruction {
    DEBUG_INFO2(JMP)
    Block* tgt;
    JMP(Block* tgt): tgt(tgt) {}
};

struct CMP : X86Instruction {
    DEBUG_INFO2(CMP)
    CMP(BaseRegister* lhs, BaseRegister* rhs) {
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct TEST : X86Instruction {
    DEBUG_INFO2(TEST)
    TEST(BaseRegister* lhs, BaseRegister* rhs) {
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct CQO : X86Instruction {
    DEBUG_INFO2(CQO)
    CQO() {
        this->uses.push_back(new Physical(x86::Rax));

        this->defs.push_back(new Physical(x86::Rdx));
        this->defs.push_back(new Physical(x86::Rax));
    }
};

struct IDIV : X86Instruction {
    DEBUG_INFO2(IDIV)
    IDIV(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rdx));
        this->uses.push_back(new Physical(x86::Rax));

        this->defs.push_back(new Physical(x86::Rdx));
        this->defs.push_back(new Physical(x86::Rax));
    }
};

struct SHL : X86Instruction {
    DEBUG_INFO2(SHL)
    SHL(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rcx));

        this->defs.push_back(reg);
    }
};

struct SHR : X86Instruction {
    DEBUG_INFO2(SHR)
    SHR(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rcx));

        this->defs.push_back(reg);
    }
};

struct ASR : X86Instruction {
    DEBUG_INFO2(ASR)
    ASR(BaseRegister* reg) {
        this->uses.push_back(reg);

        this->uses.push_back(new Physical(x86::Rcx));

        this->defs.push_back(reg);
    }
};

struct SETZ : X86Instruction {
    DEBUG_INFO2(SETZ)
    SETZ(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct SETNZ : X86Instruction {
    DEBUG_INFO2(SETNZ)
    SETNZ(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct SETLE : X86Instruction {
    DEBUG_INFO2(SETLE)
    SETLE(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct SETLS: X86Instruction {
    DEBUG_INFO2(SETLS)
    SETLS(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct SETGT: X86Instruction {
    DEBUG_INFO2(SETGT)
    SETGT(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct SETGE: X86Instruction {
    DEBUG_INFO2(SETGE)
    SETGE(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct INT3: X86Instruction {
    DEBUG_INFO2(INT3)
};

struct HLT: X86Instruction {
    DEBUG_INFO2(HLT)
};

struct CMPIMM : X86Instruction {
    DEBUG_INFO2(CMPIMM)

    long imm;

    CMPIMM(BaseRegister* lhs, long imm): imm(imm) {
        this->uses.push_back(lhs);
    }
};

struct LOADMEM : X86Instruction {
    DEBUG_INFO2(LOADMEM)

    long offset;

    LOADMEM(BaseRegister* dst, BaseRegister* src, long offset): offset(offset) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct STOREMEM : X86Instruction {
    DEBUG_INFO2(STOREMEM)

    long offset;

    STOREMEM(BaseRegister* dst, BaseRegister* src, long offset): offset(offset) {
        this->uses.push_back(dst);
        this->uses.push_back(src);
    }
};

struct StackSlot {
    size_t id = 999'999;
};

struct STORESTACK : X86Instruction {
    DEBUG_INFO2(STORESTACK)

    StackSlot slot;
    long offset;

    STORESTACK(BaseRegister* value, StackSlot slot, long offset): slot(slot), offset(offset) {
        this->uses.push_back(value);
    }
};

struct LOADSTACK : X86Instruction {
    DEBUG_INFO2(LOADSTACK)

    StackSlot slot;
    long offset;
    size_t size;

    LOADSTACK(BaseRegister* dst, StackSlot slot, long offset, size_t size): slot(slot), offset(offset), size(size) {
        this->defs.push_back(dst);
    }
};

struct LOADRIP : X86Instruction {
    DEBUG_INFO2(LOADRIP)
    LOADRIP(BaseRegister* dst, long offset) {
        this->defs.push_back(dst);
    }
};

struct LEASTACK : X86Instruction {
    DEBUG_INFO2(LEASTACK)

    StackSlot slot;

    LEASTACK(BaseRegister* dst, StackSlot slot): slot(slot) {
        this->defs.push_back(dst);
    }
};

struct MOVMEMIMM : X86Instruction {
    DEBUG_INFO2(MOVMEMIMM)

    long imm;

    MOVMEMIMM(BaseRegister* dst, BaseRegister* src, long imm): imm(imm) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct JZ : X86Instruction {
    DEBUG_INFO2(JZ)
    Block* tgt;
    JZ(Block* tgt): tgt(tgt) {}
};

struct Block {
    size_t id = 0;
    std::string tag;
    X86Instruction* first = nullptr;
    X86Instruction* last = nullptr;
    std::set<Block*> incoming;
    std::set<Block*> outgoing;

    X86Instruction* insertPoint = nullptr;

    size_t size = 0;

    struct Iterator {
        X86Instruction* next;

        void operator++() {
            next = next->next;
        }

        X86Instruction*& operator*() {
            return next;
        }

        bool operator==(const Iterator& other) const {
            return this->next == other.next;
        }
    };

    struct RevIterator {
        X86Instruction* next;

        void operator++() {
            next = next->prev;
        }

        X86Instruction*& operator*() {
            return next;
        }

        bool operator==(const RevIterator& other) const {
            return this->next == other.next;
        }
    };

    struct Iter {
        X86Instruction* start;

        Iterator begin() {
            return Iterator{start};
        }

        Iterator end() {
            return Iterator{nullptr};
        }
    };

    struct RevIter {
        X86Instruction* start;

        RevIterator begin() {
            return RevIterator{start};
        }

        RevIterator end() {
            return RevIterator{nullptr};
        }
    };

    Iter iterator() {
        return Iter{this->first};
    }

    void insertBefore(X86Instruction* subj, X86Instruction* self) {
        if (subj == first) first = self;
        if (last == nullptr) last = self;

        auto before = subj == nullptr ? nullptr : subj->prev;
        auto after = subj;

        insertBetweene(self, before, after);
    }

    static constexpr long STEP = 1024*1024;

    void insertBetweene(X86Instruction* self, X86Instruction* before, X86Instruction* after) {
        if (before != nullptr) before->next = self;
        if (after != nullptr) after->prev = self;

        self->prev = before;
        self->next = after;

        if (before == nullptr && after == nullptr) {
            self->orderId = 0;
        } else if (before == nullptr) {
            self->orderId = after->orderId-STEP;
        } else if (after == nullptr) {
            self->orderId = before->orderId+STEP;
        } else {
            self->orderId = (before->orderId+after->orderId)/2;
        }

        size += 1;
    }

    void insertAfter(X86Instruction* subj, X86Instruction* self) {
        if (first == nullptr) first = self;
        if (last == subj) last = self;

        auto before = subj;
        auto after = subj == nullptr ? nullptr : subj->next;

        insertBetweene(self, before, after);
    }

    void erase(X86Instruction* inst) {
        assert(inst != nullptr);
        auto before = inst->prev;
        auto after = inst->next;

        if (before != nullptr) before->next = after;
        if (after != nullptr) after->prev = before;

        if (first == inst) first = after;
        if (last == inst) last = before;

        size -= 1;
    }

    template<typename T, typename... Args>
    void push(Args&&... args) {
        auto self = new T(args...);

        insertAfter(insertPoint, self);

        insertPoint = self;
    }

    void addTarget(Block* b) {
        b->incoming.insert(this);
        this->outgoing.insert(b);
    }
};

static void printBlockInst(X86Instruction* inst, std::ostream& ss) {
    if (inst->defs.size() == 1) {
        ss << inst->defs.front()->toString();
    } else {
        ss << "[";
        for (auto def: inst->defs) {
            ss << def->toString() << ",";
        }
        ss << "]";
    }

    ss << " := ";

    ss << inst->className();
    ss << " [";
    for (auto i = 0ul; i < inst->uses.size(); i++) {
        ss << inst->uses[i]->toString();
        if (i != inst->uses.size()-1) ss << ",";
    }
    ss << "]";
}


static void printBlock(Block* b) {
    for (auto inst: b->iterator()) {
        if (inst->defs.size() == 1) {
            std::cout << inst->defs.front()->toString();
        } else {
            std::cout << "[";
            for (auto def: inst->defs) {
                std::cout << def->toString() << ",";
            }
            std::cout << "]";
        }

        std::cout << " := ";

        std::cout << inst->className();
        std::cout << " [";
        for (auto i = 0ul; i < inst->uses.size(); i++) {
            std::cout << inst->uses[i]->toString();
            if (i != inst->uses.size()-1) std::cout << ",";
        }
        std::cout << "]";
        std::cout << std::endl;
    }
}

#define MIN_BY(a, b, op) (a op < b op) ? a : b
#define MAX_BY(a, b, op) (a op > b op) ? a : b

struct Range {
    X86Instruction* first;
    X86Instruction* last;

    Range merge(Range other) {
        auto start = MIN_BY(this->first, other.first, ->orderId);
        auto end = MAX_BY(this->last, other.last, ->orderId);

        return Range{start, end};
    }

    static bool intersects(Range other, Range self) {
        return (other.first->orderId >= self.first->orderId && other.first->orderId <= self.last->orderId) || (other.last->orderId >= self.first->orderId && other.last->orderId <= self.last->orderId);
    }

    bool intersects(Range other) {
        return Range::intersects(other, *this) || Range::intersects(*this, other);
    }
};

struct RangeSet {
    std::vector<Range> ranges;

    void add(Range r) {
        ranges.push_back(r);
    }

    bool intersects(Range other) {
        for (auto r: ranges) {
            if (r.intersects(other)) return true;
        }
        return false;
    }
};

struct GraphColoring {
    std::vector<size_t> registers;
    std::map<size_t, x86::X64Register> allocated;
    std::map<size_t, std::set<size_t>> interference;
    size_t regCount = 16;
    size_t regCounter = 0;
    std::shared_ptr<Logger> logger;

    GraphColoring(std::shared_ptr<Logger> logger): logger(logger) {

    }

    size_t interferenceCount(size_t reg) {
        return this->interference[reg].size();
    }

    size_t getInterferenceMaxNeighbour(size_t reg) {
        auto max = interferenceCount(reg);
        auto worst = reg;

        for (auto other : interference[reg]) {
            if (other >= 50'000) continue; // HACK
            auto otherInt = interferenceCount(other);
            if (otherInt > max) {
                max = otherInt;
                worst = other;
            }
        }

        return worst;
    }

    static GraphColoring create(std::shared_ptr<Logger> logger, std::map<size_t, std::set<size_t>> interf) {
        GraphColoring self{logger};

        std::vector<size_t> regs;
        for (auto i : interf) {
            regs.push_back(i.first);
        }

        self.interference = interf;
        self.registers = regs;

        return self;
    }

    void colorReg(size_t id, x86::X64Register color) {
        allocated.emplace(id, color);
    }

    bool doesRegInterfere(size_t self, x86::X64Register phyReg) {
        for (auto inte: interference[self]) {
            if (!allocated.contains(inte)) continue;
            if (allocated.at(inte) == phyReg) return true;
        }
        return false;
    };

    bool regAlloc(size_t i) {
        if (i == registers.size()) {
            logger->DEBUG("[graph-color] SUCESSS!!!!");
            return true;
        }

        auto self = registers[i];

        if (allocated.contains(self)) {
            if (regAlloc(i + 1)) return true;
        } else {
            size_t sucesses = 0;
            for (auto j = 0ul; j < regCount; j++) {
                if (j == x86::Rsp.getEncoding() || j == x86::Rbp.getEncoding()) continue;
                if (doesRegInterfere(self, x86::fromRaw(j))) continue;
                sucesses += 1;

                assert(!allocated.contains(self));
                allocated.emplace(self, x86::fromRaw(j));
                logger->DEBUG("[graph-color] allocating self: {}", self);
                if (regAlloc(i + 1)) return true;
                allocated.erase(self);
            }

            if (sucesses == 0) {
                logger->DEBUG("[graph-color] FAILED to allocate ID: {}", self);
            }
        }

        return false;
    }

    size_t getUncoloredReg() {
        return registers[regCounter++];
    }

    std::optional<size_t> findColorForReg(size_t reg) {
        for (auto j = 0ul; j < regCount; j++) {
            if (j == x86::Rsp.getEncoding() || j == x86::Rbp.getEncoding()) continue;
            if (doesRegInterfere(reg, x86::fromRaw(j))) continue;

            assert(!allocated.contains(reg));
            allocated.emplace(reg, x86::fromRaw(j));
            logger->DEBUG("[graph-color] allocating self: {}", reg);

            return reg;
        }

        return std::nullopt;
    }

    bool hasUncoloredReg() {
        return regCounter < registers.size();
    }

    std::optional<size_t> regAllocFast() {
        while (hasUncoloredReg()) {
            auto reg = getUncoloredReg();
            if (this->allocated.contains(reg)) continue;
            auto color = findColorForReg(reg);
            if (not color.has_value()) return reg; // failed allocation
        }

        return std::nullopt;
    }

    std::string toGraphViz() {
        std::string buffer;

        buffer += "uniform graph {\n";
        for (auto [self, others] : this->interference) {
            for (auto other : others) {
                buffer += stringify("{} -- {}\n", self, other);
                buffer += "\n";
            }
        }

        buffer += "}";

        return buffer;
    }
};

struct Graph {
    std::vector<Block*> blocks;
    Block* root = nullptr;

    size_t virtCounter = 0;
    std::vector<size_t> stackSizes;
    std::shared_ptr<Logger> logger;

    Graph(std::shared_ptr<Logger> logger): logger(logger) {
    }

    size_t totalStackSize() const {
        size_t acu = 0;
        for (auto size : stackSizes) {
            acu += size;
        }

        return acu;
    }

    std::map<size_t, size_t> stackOffsets() const {
        std::map<size_t, size_t> offsets;
        size_t acu = 0;
        for (auto [i, size] : stackSizes | views::enumerate) {
            offsets[i] = acu;
            acu += size;
        }

        return offsets;
    }

    // live ranges
    std::map<Block*, std::map<size_t, Range>> virtualRanges;
    std::map<Block*, std::map<x86::X64Register, RangeSet>> physicalRanges;

    std::string tag;

    GraphColoring registerAllocate(std::function<std::string(size_t)> getPath) {
        size_t spillCounter = 0;

        while (true) {
            this->calcLiveRanges();

            auto gc = this->buildAllocatorGraph();

            auto res = gc.regAllocFast();
            if (not res.has_value()) return gc;
            logger->DEBUG("[graph-color] FAILED to color {}", *res);

            auto toSpill = gc.getInterferenceMaxNeighbour(*res);

            logger->DEBUG("[graph-color] spilling {}", toSpill);
            this->spill(new Virtual{toSpill});

            Graph::GVEmit ee1;
            emitGV2(ee1);
            writeBytesToFile(getPath(spillCounter), ee1.ss.str());

            spillCounter += 1;
        }
    }

    size_t allocVirtId() {
        return virtCounter++;
    }

    Virtual* allocaVirtReg() {
        return new Virtual{allocVirtId()};
    }

    size_t allocStack(size_t size, size_t alignment) {
        stackSizes.push_back(size);
        return stackSizes.size()-1;
    }

    StackSlot allocStackSlot(size_t size, size_t alignment) {
        return StackSlot{allocStack(size, alignment)};
    }

    void prune() {
        std::set<size_t> used;

        for (auto b : blocks) {
            for (auto inst : b->iterator()) {
                for (auto use : inst->uses) {
                    if (auto v = use->as<Virtual>(); v) {
                        used.insert(v->id);
                    }
                }
            }
        }

        auto canDelete = [&](std::span<BaseRegister*> defs) {
            if (defs.empty()) return false;

            for (auto reg : defs) {
                if (auto v = reg->as<Virtual>(); v) {
                    if (used.contains(v->id)) return false;
                } else {
                    return false;
                }
            }
            return true;
        };

        for (auto b : blocks) {
            std::set<X86Instruction*> toDelete;
            for (auto inst : b->iterator()) {
                if (canDelete(inst->defs)) {
                    toDelete.insert(inst);
                }
            }
            for (auto inst : toDelete) b->erase(inst);
        }
    }

    void print() {
        for (auto [id, block] : blocks | views::enumerate) {
            println("BLOCK {}", id);
            printBlock(block);
        }
    }

    void printLiveRange() {
        for (auto [id, block] : blocks | views::enumerate) {
            println("BLOCK {} range 0..<{}", id, block->size-1);
            for (auto range : virtualRanges[block]) {
                println("  v{} = {}..{}", range.first, range.second.first->orderId, range.second.last->orderId);
            }
        }
    }

    X86Instruction* findDefVirt(Block* block, X86Instruction* start, size_t virtId) {
        for (auto inst : Block::RevIter{start->prev}) {
            if (inst->hasDefVirt(virtId)) return inst;
        }
        return nullptr;
    }


    X86Instruction* findDefPhy(Block* block, X86Instruction* start, x86::X64Register phyId) {
        for (auto inst : Block::RevIter{start->prev}) {
            if (inst->hasDefPhy(phyId)) return inst;
        }
        return nullptr;
    }

    void markLiveVirt(Block* b, X86Instruction* start, X86Instruction* endIncl, size_t virtId) {
        if (virtualRanges[b].contains(virtId)) {
            virtualRanges[b][virtId] = virtualRanges[b][virtId].merge(Range{start, endIncl});
        } else {
            virtualRanges[b][virtId] = Range{start, endIncl};
        }
    }

    void markLivePhy(Block* b, X86Instruction* start, X86Instruction* endIncl, x86::X64Register phyId) {
        if (physicalRanges[b].contains(phyId)) {
            physicalRanges[b][phyId].add(Range{start, endIncl});
        } else {
            physicalRanges[b][phyId].add(Range{start, endIncl});
        }
    }

    bool virtVirtInt(size_t id, size_t other) {
        for (auto block: blocks) {
            auto x = virtualRanges[block];
            if (!x.contains(id) || !x.contains(other)) continue;
            auto a = virtualRanges[block][id];
            auto b = virtualRanges[block][other];
            if (a.intersects(b)) return true;
        }
        return false;
    }

    bool virtPhyInt(size_t id, x86::X64Register other) {
        for (auto block: blocks) {
            auto x = virtualRanges[block];
            auto x1 = physicalRanges[block];
            if (!x.contains(id) || !x1.contains(other)) continue;
            auto a = virtualRanges[block][id];
            auto b = physicalRanges[block][other];
            if (b.intersects(a)) return true;
        }
        return false;
    }


    void findDefVirtRec(Block* block, X86Instruction* foundUse, size_t virtId, std::set<Block*>& visited) {
        if (visited.contains(block)) return;
        visited.insert(block);

        if (foundUse == nullptr) foundUse = block->last;

        auto foundDef = findDefVirt(block, foundUse, virtId);
        if (foundDef != nullptr) {
            markLiveVirt(block, foundDef, foundUse, virtId);
        } else {
            markLiveVirt(block, block->first, foundUse, virtId);// implicitly mark live range, search in incoming

            for (auto inc: block->incoming) {
                findDefVirtRec(inc, nullptr, virtId, visited);
            }
        }

        visited.erase(block);
    }

    void calcLiveRanges() {
        this->virtualRanges.clear();
        this->physicalRanges.clear();
        for (auto block: blocks) {
            for (auto inst : block->iterator()) {
                for (auto use: inst->uses) {
                    auto v = use->as<Virtual>();
                    if (v == nullptr) continue;
                    std::set<Block*> visited;
                    findDefVirtRec(block, inst, v->id, visited);
                }

                for (auto use: inst->uses) {
                    auto v = use->as<Physical>();
                    if (v == nullptr) continue;
                    auto def = findDefPhy(block, inst, v->id);
                    if (def == nullptr) {
                        PANIC("definition for value {} not found", v->id.toString());
                    }
                    assert(def != nullptr);
                    markLivePhy(block, def, inst, v->id);
                }

                for (auto use: inst->kills) {
                    auto v = use->as<Physical>();
                    if (v == nullptr) continue;
                    markLivePhy(block, inst, inst, v->id);
                }
            }
        }
    }

    std::set<size_t> getVirtIds() {
        std::set<size_t> res;

        for (const auto& [block, ranges]: virtualRanges) {
            for (auto [vId, stuff]: ranges) res.insert(vId);
        }

        return res;
    }

    std::set<x86::X64Register> getPhyIds() {
        std::set<x86::X64Register> res;

        for (const auto& [block, ranges]: physicalRanges) {
            for (auto [vId, stuff]: ranges) res.insert(vId);
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
    void spill(Virtual* regId) {
        auto stackSlot = allocStackSlot(8,8);

        for (auto block : this->blocks) {
            for (auto inst : block->iterator()) {
                if (inst->hasDefVirt(regId->id)) {
                    auto tmp = allocaVirtReg();
                    block->insertAfter(inst, new STORESTACK(tmp, stackSlot, 0));
                    inst->rewriteDef(regId, tmp);
                }
                if (inst->hasUseVirt(regId->id)) {
                    auto tmp = allocaVirtReg();
                    block->insertBefore(inst, new LOADSTACK(tmp, stackSlot, 0, 8));
                    inst->rewriteUse(regId, tmp);
                }
                assert(!inst->hasDefVirt(regId->id));
                assert(!inst->hasUseVirt(regId->id));
            }
        }
    }

    GraphColoring buildAllocatorGraph() {
        std::set<size_t> virtIds = getVirtIds();
        std::set<x86::X64Register> phyIds = getPhyIds();

        std::map<size_t, std::set<size_t>> virtToVirt;
        std::map<size_t, std::set<x86::X64Register>> virtToPhy;

        for (auto block: blocks) {
            for (auto rangeA : virtualRanges[block]) {
                for (auto rangeB : virtualRanges[block]) {
                    if (rangeA.second.intersects(rangeB.second)) {
                        virtToVirt[rangeB.first].insert(rangeA.first);
                        virtToVirt[rangeA.first].insert(rangeB.first);
                    }
                }
            }
        }

        for (auto self : virtIds) {
            for (auto other : phyIds) {
                auto ints = virtPhyInt(self, other);
                if (ints) {
                    virtToPhy[self].insert(other);
                }
            }
        }

        size_t physVirtBase = 50'000;
        std::map<size_t, std::set<size_t>> virtToVirt2 = virtToVirt;
        for (auto [virt, physs] : virtToPhy) {
            for (auto phys : physs) {
                virtToVirt2[virt].insert(physVirtBase+phys.getRawValue());
                virtToVirt2[physVirtBase+phys.getRawValue()].insert(virt);
            }
        }

        GraphColoring gc = GraphColoring::create(logger, virtToVirt2);
        for (auto [virt, physs] : virtToPhy) {
            for (auto phys : physs) {
                gc.colorReg(physVirtBase+phys.getRawValue(), phys);
            }
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

        void appendLine() {
            ss << std::endl;
        }
    };

    void emitGVBlock(GVEmit& e, Block* b, size_t id) {
        e.appendLine("subgraph cluster_{} {", id);

        e.ident([&] {
            e.appendLine("label = \"process #1\";");
            e.appendLine("style=filled;");
            e.appendLine("color=lightgrey;");
            e.appendLine("node [style=filled,color=white];");
            e.appendLine("edge[rank=same];");
            e.appendLine();

            for (auto i = 0ul; i < b->size-1; i++) {
                e.appendLine("b{}i{} -> b{}i{}", id, i, id, i+1);
            }
            e.appendLine();

            for (auto inst : b->iterator()) {
                e.appendLine("b{}i{} [shape=rect]", id, inst->orderId);
                std::stringstream ss;
                printBlockInst(inst, ss);
                e.appendLine("b{}i{} [label=\"{}\"]", id, inst->orderId, ss.str());
            }
        });

        e.appendLine("}");
    }

    void emitGVBlock2(GVEmit& e, Block* b, size_t id) {
        e.appendLine("b{} [", id);

        e.ident([&] {
            e.appendLine(R"(xlabel="B{}", label = "{)", id);

            for (auto inst : b->iterator()) {
                std::stringstream ss;
                printBlockInst(inst, ss);
                e.appendLine("<i{}>{}", inst->orderId, ss.str());

                if (inst->next != nullptr) e.appendLine("|");
            }
            e.appendLine("}\";");
        });

        e.appendLine("]");
    }

    void emitGV2(GVEmit& e) {
        e.appendLine("digraph G {");
        e.appendLine("node [shape=Mrecord, fontsize=66];");
        e.appendLine("dummy0 -> b0:i0;");
        e.appendLine("dummy0 -> v1;");

        e.ident([&] {
            // blocks
            for (auto item: blocks) {
                emitGVBlock2(e, item, item->id);
            }

            // block edges
            for (auto src: blocks) {
                for (auto dst : src->outgoing) {
                    e.appendLine("b{}:i{} -> b{}:i{} [tailport=s, headport=n];", src->id, src->last->orderId, dst->id, dst->first->orderId);
                }
            }

            // virt regs
            std::optional<std::size_t> lastId;
            e.appendLine("subgraph {");
            for (auto block : blocks) {
                for (auto [regId, range] : virtualRanges[block]) {
                    e.appendLine("v{} [shape=oval]", regId);
                    auto color = (range.first->hasDefVirt(regId)) ? "green"sv : "red"sv;
                    auto xd = false;
                    e.appendLine("v{} -> b{}:i{} [color={}, constraint={}, concentrate=false];", regId, block->id, range.first->orderId, color, (xd ? "true" : "false"));
                    e.appendLine("v{} -> b{}:i{} [color=blue, constraint={}, concentrate=false];", regId, block->id, range.last->orderId, (xd ? "true" : "false"));
                    e.appendLine("v{} -> b{} [style=invis, constraint=true, concentrate=false];", regId, block->id);
                    if (lastId.has_value()) {
                        e.appendLine("v{} -> v{} [style=invis];", *lastId, regId);
                    }
                    lastId = regId;
                }
            }
            e.appendLine("}");

            // phy regs
            e.appendLine("subgraph {");
            for (auto block : blocks) {
                for (auto [regId, rangeSet] : physicalRanges[block]) {
                    // e.appendLine("{} [shape=oval]", regId.toString());
                    for (auto range : rangeSet.ranges) {
                        // e.appendLine("b{}:i{} -> b{}:i{} [color=purple] [label={}, tailport=w];", block->id, range.first->orderId, block->id, range.last->orderId, regId.toString());
                    }
                }
            }
            e.appendLine("}");
        });

        e.appendLine("}");
    }


    void emitGV(GVEmit& e) {
        e.appendLine("digraph G {");

        e.ident([&] {
            // blocks
            for (auto item: blocks) {
                emitGVBlock(e, item, item->id);
            }

            // block edges
            for (auto src: blocks) {
                for (auto dst : src->outgoing) {
                    e.appendLine("b{}i{} -> b{}i{};", src->id, src->size-1, dst->id, 0);
                }
            }

            // regs
            for (auto block : blocks) {
                for (auto [regId, range] : virtualRanges[block]) {
                    e.appendLine("v{} -> b{}i{} [color=red];", regId, block->id, range.first->orderId);
                    e.appendLine("v{} -> b{}i{} [color=red];", regId, block->id, range.last->orderId);
                }
            }
        });

        e.appendLine("}");
    }

    std::vector<size_t> linearized;
};

//   XXX XXXXXXX XXX
//1: XX
//2:
//3:             XX
//4:     XXXX

#define CHCK(n, type) if (type<CTX>* n = inst.template cst<type>(); n)

struct BasePattern {
    virtual ~BasePattern() = default;

    template<class T>
    T* as() {
        return dynamic_cast<T*>(this);
    }
};

struct SimplePattern : BasePattern {
    SimplePattern(const char* base, std::vector<BasePattern*> params) : base(base), params(params) {}
    const char* base;
    std::vector<BasePattern*> params;
};

struct SimpleWildPattern : BasePattern {
    SimpleWildPattern(const char* base) : base(base) {}
    const char* base;
};

struct WildCardPattern : BasePattern {};

struct BinOpPattern : BasePattern {
    BinOpPattern(
        Assembler::BinaryOp op,
        Assembler::BaseDataType type,
        BasePattern* lhs,
        BasePattern* rhs) : op(op), type(type), lhs(lhs), rhs(rhs) {}
    Assembler::BinaryOp op;
    Assembler::BaseDataType type;
    BasePattern* lhs;
    BasePattern* rhs;
};

template<typename T, typename... Args>
BasePattern* makePattern(Args... arg) {
    return new SimplePattern{typeid(T).name(), std::vector<typename First<Args...>::TYPE>{arg...}};
}

template<typename T>
BasePattern* makePattern() {
    return new SimplePattern{typeid(T).name(), std::vector<BasePattern*>{}};
}

template<typename T>
BasePattern* makeWildPattern() {
    return new SimpleWildPattern{typeid(T).name()};
}

template<Assembler::BinaryOp OP, Assembler::BaseDataType TYP>
BasePattern* makeBin(BasePattern* lhs, BasePattern* rhs) {
    return new BinOpPattern{OP, TYP, lhs, rhs};
}

static BasePattern* WILD = new WildCardPattern{};

/*
args := alloca 8
_ := store_mem args, a0
ret := alloca 8
run := mov_rip
hand := mov_rip
_ := call [run, hand, args, ret]
 */

template<typename CTX>
struct Lower {
    std::map<SSARegisterHandle, size_t> virtRegs;
    std::map<void*, StackSlot> allocaSlots;
    std::string name;
    Graph g;

    Lower(std::shared_ptr<Logger> logger): g(logger) {

    }


    StackSlot allocateStack(size_t size, size_t alignment) {
        // FIXME this is retarded
        return g.allocStackSlot(size, alignment);
    }

    struct MatchedInstructions {
        IRInstruction<CTX>* inst = nullptr;
        std::vector<MatchedInstructions*> params;

        template<typename... Args>
        MatchedInstructions& operator[](Args... args) {
            MatchedInstructions* self = this;
            for (auto arg: {args...}) {
                self = self->params[arg];
            }
            return *self;
        }

        IRInstruction<CTX>* operator->() {
            return inst;
        }

        IRInstruction<CTX>* operator*() {
            return inst;
        }
    };

    template<template<typename> typename T, typename... Args>
    BasePattern* makePattern(Args... arg) {
        return ::makePattern<T<CTX>, Args...>(std::forward<Args>(arg)...);
    }

    template<template<typename> typename T>
    BasePattern* makePattern() {
        return ::makePattern<T<CTX>>();
    }

    bool matchPattern(ControlFlowGraph<CTX>& cfg, IRInstruction<CTX>* inst, BasePattern* pattern, std::vector<IRInstruction<CTX>*>& wildcards, std::vector<IRInstruction<CTX>*>& consumed, MatchedInstructions* matched) {
        matched->inst = inst;
        if (auto bin = pattern->as<BinOpPattern>(); bin) {
            instructions::BinaryInstruction<CTX>* bbin = inst->template cst<instructions::BinaryInstruction<CTX>>();
            if (bbin == nullptr) return false;
            if (bbin->op != bin->op) return false;
            if (bbin->type != bin->type) return false;
            consumed.push_back(inst);
            auto l = new MatchedInstructions{};
            auto r = new MatchedInstructions{};
            matched->params.push_back(l);
            matched->params.push_back(r);
            return matchPattern(cfg, cfg.resolveInstruction(bbin->getSrc(0)), bin->lhs, wildcards, consumed, l) && matchPattern(cfg, cfg.resolveInstruction(bbin->getSrc(1)), bin->rhs, wildcards, consumed, r);
        } else if (auto wild = pattern->as<WildCardPattern>(); wild) {
            wildcards.push_back(inst);
            return true;
        } else if (auto sim = pattern->as<SimplePattern>(); sim) {
            if (typeid(*inst).name() != sim->base) return false;

            for (auto i = 0ul; i < sim->params.size(); i++) {
                auto ii = new MatchedInstructions{};
                auto didMatch = matchPattern(cfg, cfg.resolveInstruction(inst->getSrc(i)), sim->params[i], wildcards, consumed, ii);
                matched->params.push_back(ii);
                if (!didMatch) return false;
            }
            consumed.push_back(inst);
            return true;
        } else if (auto sim1 = pattern->as<SimpleWildPattern>(); sim1) {
            if (typeid(*inst).name() != sim1->base) return false;

            for (auto i = 0ul; i < inst->srcCount(); i++) {
                auto ii = new MatchedInstructions{};
                auto didMatch = matchPattern(cfg, cfg.resolveInstruction(inst->getSrc(i)), WILD, wildcards, consumed, ii);
                matched->params.push_back(ii);
                if (!didMatch) return false;
            }
            consumed.push_back(inst);
            return true;
        }
        PANIC();
    }

    void insertPrologueEpilogue(std::set<x86::X64Register> regs) {
        std::map<x86::X64Register, StackSlot> ss;
        for (auto reg : regs) {
            auto saveType = x86::sysVSave(reg);
            if (saveType != x86::X64Register::SaveType::Callee) continue;

            ss[reg] = this->allocateStack(8, 8);
            g.root->insertBefore(g.root->first, new STORESTACK(new Physical(reg), ss[reg], 0));
        }

        for (auto block : g.blocks) {
            for (auto inst : block->iterator()) {
                if (inst->is<RET>()) {
                    for (auto reg : regs) {
                        auto saveType = x86::sysVSave(reg);
                        if (saveType != x86::X64Register::SaveType::Callee) continue;

                        block->insertBefore(inst, new LOADSTACK(new Physical(reg), ss[reg], 0, 8));
                    }
                }
            }
        }
    }


    // template<typename CTX>
    struct RewriteRule {
        std::function<void(Lower& l, Lower::MatchedInstructions inst, Block* block)> rewrite;
        BasePattern* pattern;
    };

    static RewriteRule* makeRewrite(BasePattern* pat, std::function<void(Lower& l, Lower::MatchedInstructions inst, Block* block)> rev) {
        return new RewriteRule{rev, pat};
    }

    void doConsuming(
        ControlFlowGraph<CTX>& cfg,
        std::set<IRInstruction<CTX>*>& globalConsumed,
        std::span<RewriteRule*> patterns,
        IRInstruction<CTX>* inst,
        Block* block) {
        if (globalConsumed.contains(inst)) return;

        for (auto pattern: patterns) {
            std::vector<IRInstruction<CTX>*> wildcards;
            std::vector<IRInstruction<CTX>*> consumed;
            MatchedInstructions matched;
            auto didMatch = matchPattern(cfg, inst, pattern->pattern, wildcards, consumed, &matched);
            if (!didMatch) continue;
            globalConsumed.insert(inst);

            // block->insertPoint = 0;
            pattern->rewrite(*this, matched, block);

            // for (auto con : consumed) globalConsumed.insert(con);

            // for (auto wild: wildcards) doConsuming(cfg, globalConsumed, patterns, wild, block);

            return;
        }

        PANIC("failed to match inst {}", inst->name);
    }

    std::map<BlockId, Block*> blocks;
    Block* getBlockForId(BlockId id) {
        if (not blocks.contains(id)) {
            blocks.emplace(id, new Block(blocks.size()));
        }

        return blocks.at(id);
    }

    void matchBlock(ControlFlowGraph<CTX>& cfg, CodeBlock<CTX>& block, std::span<RewriteRule*> patterns) {
        std::set<IRInstruction<CTX>*> globalConsumed;

        auto bb = getBlockForId(block.id());
        for (auto b : block.getTargets()) {
            bb->addTarget(getBlockForId(b));
        }

        for (auto i = 0ul; i < block.instructions.size(); i++) {
            auto cur = block.instructions[i].get();

            doConsuming(cfg, globalConsumed, patterns, cur, bb);
            // bb->insertPoint = 0;
        }

        g.blocks.push_back(bb);
    }

    Virtual* getReg(SSARegisterHandle hand) {
        assert(hand.isValid());
        if (!virtRegs.contains(hand)) {
            virtRegs[hand] = g.allocVirtId();
        }

        return new Virtual{virtRegs[hand]};
    }

    Virtual* allocReg() {
        return g.allocaVirtReg();
    }

    Virtual* getReg(IRInstruction<CTX>* hand) {
        return getReg(hand->target);
    }

    long getImm(IRInstruction<CTX>* inst) {
        if (auto intLit = inst->template cst<instructions::IntLiteral>(); intLit) {
            return intLit->value;
        }
        TODO()
    }

    void emitLoadAddImm(Lower<CTX>& l, MatchedInstructions inst, Block* block) {
        block->push<MOVMEMIMM>(l.getReg(inst->target), l.getReg(inst[0, 0]->target), l.getImm(inst[0, 1]->target));
    }

    void emitMul(Lower<CTX>& l, MatchedInstructions inst, Block* block) {
        block->push<MOV>(l.getReg(inst->target), l.getReg(inst[0]));
        block->push<MUL>(l.getReg(inst->target), l.getReg(inst[1]));
    }

    void matchCFG(ControlFlowGraph<CTX>& cfg, std::span<RewriteRule*> patterns) {
        cfg.forEachBlock([&](CodeBlock<CTX>& block) {
            matchBlock(cfg, block, patterns);
        });
    }

    void emitCall(std::vector<SSARegisterHandle> ret, std::span<IRInstruction<CTX>*> argz, Block* block, size_t id) {
        std::vector<x86::X64Register> argRegs{x86::Rdi, x86::Rsi, x86::Rdx, x86::Rcx, x86::R8, x86::R9};


        std::vector<x86::X64Register> kills{x86::Rax, x86::R10, x86::R11, x86::R9, x86::R8, x86::Rcx, x86::Rdx, x86::Rsi, x86::Rdi};
        std::vector<x86::X64Register> uses;
        std::vector<x86::X64Register> defs;

        for (auto i = 0ul; i < argz.size(); i++) {
            block->push<MOV>(new Physical(argRegs[i]), getReg(argz[i]));
            uses.push_back(kills.back());
            kills.pop_back();
        }

        assert(ret.size() <= 2);

        if (ret.size() == 2) {
            defs.push_back(x86::Rax);
            defs.push_back(x86::Rdx);
        } else if (ret.size() == 1) {
            defs.push_back(x86::Rax);
        }

        auto callReg = allocReg();

        std::vector<BaseRegister*> killsR;
        std::vector<BaseRegister*> usesR;
        std::vector<BaseRegister*> defsR;

        for (auto it: kills) killsR.push_back(new Physical(it));
        for (auto it: uses) usesR.push_back(new Physical(it));
        for (auto it: defs) defsR.push_back(new Physical(it));

        // code gen
        block->push<MOVRIP>(callReg, id);

        block->push<CALLREG>(defsR, usesR, killsR, callReg);

        if (ret.size() == 2) {
            block->push<MOV>(getReg(ret[0]), new Physical(x86::Rax));
            block->push<MOV>(getReg(ret[1]), new Physical(x86::Rdx));
        } else if (ret.size() == 1) {
            block->push<MOV>(getReg(ret[0]), new Physical(x86::Rax));
        }
    }

    struct RewriteBuilder {
        std::vector<RewriteRule*> rules;

        RewriteBuilder& makeRewrite(BasePattern* p, std::function<void(Lower& l, Lower::MatchedInstructions inst, Block* block)> rev) {
            rules.push_back(Lower::makeRewrite(p, rev));
            return *this;
        }
    };

    void genLiveRanges() {
        g.print();
        g.calcLiveRanges();
        println("=== X86 GRAPH ===");
        g.print();
        println("=== RANGES ===");
        g.printLiveRange();
        println("=== END X86 GRAPH ===");
    }

    void lowerIR(ControlFlowGraph<CTX>& cfg) {
        using OP = Assembler::BinaryOp;
        using TYP = Assembler::BaseDataType;
        auto argCounter = 0;
        RewriteBuilder rewrites;

        rewrites
            .makeRewrite(
                makePattern<instructions::Branch>(makeBin<OP::EQ, TYP::I64>(WILD, makePattern<instructions::IntLiteral>())),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMPIMM>(l.getReg(inst[0, 0]->target), l.getImm(*inst[0, 1]));
                    block->push<JZ>(l.getBlockForId(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlockForId(inst->branchTargets()[1]));
                })

            .makeRewrite(
                makePattern<instructions::Branch>(makeBin<OP::EQ, TYP::I64>(WILD, WILD)),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(l.getReg(inst[0, 0]->target), l.getReg(inst[0, 1]->target));
                    block->push<JZ>(l.getBlockForId(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlockForId(inst->branchTargets()[1]));
                })

            .makeRewrite(
                makePattern<instructions::Branch>(WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMPIMM>(l.getReg(*inst[0]), 1);
                    block->push<JZ>(l.getBlockForId(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlockForId(inst->branchTargets()[1]));
                })

            // cmp *, *
            // jmpe true
            // jmp false
            // makePattern<instructions::Branch<CTX>>(makeBin<OP::EQ, TYP::I64>(WILD,WILD)),

            // mov rax, *
            // ret
            .makeRewrite(
                makePattern<instructions::Return>(WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(new Physical(x86::Rax), getReg(*inst[0]));
                    block->push<RET>(std::initializer_list<BaseRegister*>{new Physical(x86::Rax)});
                })

            .makeRewrite(
                makePattern<instructions::ReturnCompound>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(new Physical(x86::Rax), getReg(*inst[0]));
                    block->push<MOV>(new Physical(x86::Rdx), getReg(*inst[1]));
                    block->push<RET>(std::initializer_list<BaseRegister*>{new Physical(x86::Rax), new Physical(x86::Rdx)});
                })

            // mov rax, imm
            // ret
            .makeRewrite(
                makePattern<instructions::Return>(makePattern<instructions::IntLiteral>()),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) { TODO() })


            // mov rax, [*+imm]
            .makeRewrite(
                makePattern<instructions::PointerLoad>(makeBin<OP::ADD, TYP::I64>(WILD, makePattern<instructions::IntLiteral>())),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOVMEMIMM>(getReg(*inst), getReg(*inst[0][0]), getImm(*inst[0][1]));
                })

            .makeRewrite(
                makePattern<instructions::PointerLoad>(makePattern<instructions::AllocaPtr>()),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    auto pLoad = inst->template cst<instructions::PointerLoad>();
                    block->push<LOADSTACK>(getReg(*inst), allocaSlots[inst[0].inst], pLoad->offset, cfg.getSize(pLoad->target));
                })

            .makeRewrite(
                makePattern<instructions::PointerStore>(makePattern<instructions::AllocaPtr>(), WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<STORESTACK>(getReg(*inst[1]), allocaSlots[inst[0].inst], inst->template cst<instructions::PointerStore>()->offset);
                })

            .makeRewrite(
                makePattern<instructions::PointerLoad>(WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<LOADMEM>(getReg(*inst), getReg(*inst[0]), inst->template cst<instructions::PointerLoad>()->offset);
                })

            // mov rax, [*]
            .makeRewrite(
                makePattern<instructions::PointerStore>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<STOREMEM>(getReg(*inst[0]), getReg(*inst[1]), inst->template cst<instructions::PointerStore>()->offset);
                })

            .makeRewrite(
                makeBin<OP::MUL, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MUL>(getReg(*inst), getReg(*inst[1]));
                })


            .makeRewrite(
                makeBin<OP::ADD, TYP::I64>(WILD, makePattern<instructions::IntLiteral>()),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<ADDIMM>(getReg(*inst), getImm(*inst[1]));
                })

            .makeRewrite(
                makeBin<OP::SUB, TYP::I64>(WILD, makePattern<instructions::IntLiteral>()),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<SUBIMM>(getReg(*inst), getImm(*inst[1]));
                })

            .makeRewrite(
                makeBin<OP::ADD, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<ADD>(getReg(*inst), getReg(*inst[1]));
                })


            .makeRewrite(
                makeBin<OP::SUB, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<SUB>(getReg(*inst), getReg(*inst[1]));
                })

            // mov rax, imm
            .makeRewrite(
                makePattern<instructions::IntLiteral>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOVIMM>(getReg(*inst), getImm(*inst));
                })


            // ret
            .makeRewrite(
                makePattern<instructions::VoidReturn>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<RET>(std::initializer_list<BaseRegister*>{});
                })

            // rax
            .makeRewrite(
                makePattern<instructions::Arg>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    auto reg = x86::SYSV_REGS[argCounter++];
                    block->push<FAKE_DEF>(new Physical(reg));
                    block->push<MOV>(getReg(*inst), new Physical(reg));
                })

            .makeRewrite(
                makePattern<instructions::AllocaPtr>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    auto alloc = allocateStack(inst->template cst<instructions::AllocaPtr>()->size, 8);
                    block->push<LEASTACK>(getReg(*inst), alloc);
                    allocaSlots[inst.inst] = alloc;
                })

            .makeRewrite(
                makeBin<OP::EQ, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETZ>(getReg(*inst));
                })

            .makeRewrite(
                makeBin<OP::NEQ, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETNZ>(getReg(*inst));
                })

            .makeRewrite(
                makeBin<OP::LE, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLE>(getReg(*inst));
                })
            .makeRewrite(
                makeBin<OP::GE, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETGE>(getReg(*inst));
                })

            .makeRewrite(
                makePattern<x86::inst::MovRip>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOVRIP>(getReg(*inst), inst->template cst<x86::inst::MovRip>()->id);
                })

            .makeRewrite(
                makePattern<instructions::Assign>(WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                })

            .makeRewrite(
                makePattern<instructions::BoolLiteral>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOVIMM>(getReg(*inst), (int)inst->template cst<instructions::BoolLiteral>()->value);
                })

            .makeRewrite(
                makePattern<instructions::CharLiteral>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOVIMM>(getReg(*inst), (int)inst->template cst<instructions::CharLiteral>()->value);
                })

            .makeRewrite(
                makePattern<instructions::Jump>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<JMP>(getBlockForId(inst->branchTargets()[0]));
                })

            .makeRewrite(
                makeBin<OP::LS, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLS>(getReg(*inst));
                })
            .makeRewrite(
                makeBin<OP::GT, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETGT>(getReg(*inst));
                })
            .makeRewrite(
                makeBin<OP::LE, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLE>(getReg(*inst));
                })
            .makeRewrite(
                makeBin<OP::AND, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<AND>(getReg(*inst), getReg(*inst[1]));
                })
            .makeRewrite(
                makeBin<OP::OR, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<OR>(getReg(*inst), getReg(*inst[1]));
                })
            .makeRewrite(
                makeBin<OP::XOR, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<XOR>(getReg(*inst), getReg(*inst[1]));
                })
            .makeRewrite(
                makeBin<OP::SHL, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MOV>(new Physical(x86::Rcx), getReg(*inst[1]));
                    block->push<SHL>(getReg(*inst));
                })
            .makeRewrite(
                makeBin<OP::SHR, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MOV>(new Physical(x86::Rcx), getReg(*inst[1]));
                    block->push<ASR>(getReg(*inst));
                })
            .makeRewrite(
                makeBin<OP::DIV, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(new Physical(x86::Rax), getReg(*inst[0]));
                    block->push<CQO>();
                    block->push<IDIV>(getReg(*inst[1]));
                    block->push<FAKE_USE>(new Physical(x86::Rdx));
                    block->push<MOV>(getReg(*inst), new Physical(x86::Rax));
                })
            .makeRewrite(
                makeBin<OP::MOD, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(new Physical(x86::Rax), getReg(*inst[0]));
                    block->push<CQO>();
                    block->push<IDIV>(getReg(*inst[1]));
                    block->push<FAKE_USE>(new Physical(x86::Rax));
                    block->push<MOV>(getReg(*inst), new Physical(x86::Rdx));
                })

            .makeRewrite(
                makeWildPattern<x86::inst::CallRIP<CTX>>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    auto insts = inst.params | views::transform([&](auto& it) { return it->inst; }) | ranges::to<vector>();
                    std::vector<SSARegisterHandle> xd;
                    if (inst->target.isValid()) xd.push_back(inst->target);
                    l.emitCall(xd, insts, block, inst->template cst<x86::inst::CallRIP>()->id);
                })

            .makeRewrite(
                makeWildPattern<x86::inst::CallRIP2<CTX>>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    auto insts = inst.params | views::transform([&](auto& it) { return it->inst; }) | ranges::to<vector>();

                    auto pepa = inst.inst->template cst<x86::inst::CallRIP2<CTX>>();
                    l.emitCall(pepa->results, insts, block, inst->template cst<x86::inst::CallRIP2>()->id);
                })

            .makeRewrite(
                makePattern<instructions::BoolNot>(WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<XOR>(getReg(*inst), getReg(*inst));
                    block->push<TEST>(getReg(*inst[0]), getReg(*inst[0]));
                    block->push<SETZ>(getReg(*inst));
                })

            .makeRewrite(
                makeWildPattern<instructions::BitExtract<CTX>>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    TODO()
                })

            .makeRewrite(
                makeWildPattern<instructions::Builtin<CTX>>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    if (inst->template cst<instructions::Builtin>()->type == "trap") {
                        block->push<INT3>();
                    } else {
                        TODO()
                    }
                })

        .makeRewrite(
            makeWildPattern<instructions::Dummy<CTX>>(),
            [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
            });

        matchCFG(cfg, rewrites.rules);
    }
};
