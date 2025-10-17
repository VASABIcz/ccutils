#pragma once

#include "codegen/ControlFlowGraph.h"
#include "codegen/IRInstruction.h"
#include "codegen/IRInstructions.h"
#include "utils/utils.h"
#include "x86_insts.h"

constexpr int RDI = 0;
constexpr int RSI = 1;
constexpr int RDX = 2;
constexpr int RCX = 3;
constexpr int RAX = 4;
constexpr int RSP = 5;
constexpr int RBP = 6;
constexpr int RBX = 7;
constexpr int R8 = 8;
constexpr int R9 = 9;
constexpr int R10 = 10;
constexpr int R11 = 11;
constexpr int R12 = 12;
constexpr int R13 = 13;
constexpr int R14 = 14;
constexpr int R15 = 15;

#define MEMP(x) \
    if (id == x) return #x;

static std::string toName(size_t id) {
    MEMP(RDI)
    MEMP(RSI)
    MEMP(RDX)
    MEMP(RCX)
    MEMP(RAX)
    MEMP(RSP)
    MEMP(RBP)
    MEMP(R8)
    MEMP(R9)
    MEMP(R10)
    MEMP(R11)

    return "£ASDASDASD";
}
#undef MEMP

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
    size_t id;
    Physical(size_t id) : id(id) {}

    std::string toString() const override {
        return toName(id);
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

    bool hasDefVirt(size_t id) {
        for (auto def: defs) {
            auto virt = def->as<Virtual>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
    }

    bool hasDefPhy(size_t id) {
        for (auto def: defs) {
            auto virt = def->as<Physical>();
            if (virt == nullptr) continue;
            if (virt->id == id) return true;
        }
        return false;
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

struct CALLRIP : X86Instruction {
    DEBUG_INFO2(CALLRIP)
    CALLRIP(std::initializer_list<BaseRegister*> defs, std::initializer_list<BaseRegister*> uses, std::initializer_list<BaseRegister*> kills, size_t id) {
        this->defs = defs;
        this->kills = kills;
        this->uses = uses;
    }

    CALLRIP(std::span<BaseRegister*> defs, std::span<BaseRegister*> uses, std::span<BaseRegister*> kills) {
        this->defs = std::vector<BaseRegister*>{defs.begin(), defs.end()};
        this->kills = std::vector<BaseRegister*>{kills.begin(), kills.end()};
        this->uses = std::vector<BaseRegister*>{uses.begin(), uses.end()};
    }
};

struct MOVRIP : X86Instruction {
    DEBUG_INFO2(MOVRIP)
    MOVRIP(BaseRegister* dst, size_t id) {
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

struct LEA : X86Instruction {
    DEBUG_INFO2(LEA)
};

struct MOVIMM : X86Instruction {
    DEBUG_INFO2(MOVIMM)
    MOVIMM(BaseRegister* lhs, uint64_t value) {
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

struct ADDIMM : X86Instruction {
    DEBUG_INFO2(ADDIMM)
    ADDIMM(BaseRegister* lhs, long imm) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
    }
};

struct SUBIMM : X86Instruction {
    DEBUG_INFO2(SUBIMM)
    SUBIMM(BaseRegister* lhs, long imm) {
        this->defs.push_back(lhs);
        this->uses.push_back(lhs);
    }
};

struct Block;

struct JMP : X86Instruction {
    DEBUG_INFO2(JMP)
    JMP(Block*) {}
};

struct CMP : X86Instruction {
    DEBUG_INFO2(CMP)
    CMP(BaseRegister* lhs, BaseRegister* rhs) {
        this->uses.push_back(lhs);
        this->uses.push_back(rhs);
    }
};

struct SETZ : X86Instruction {
    DEBUG_INFO2(SETZ)
    SETZ(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct SETLE : X86Instruction {
    DEBUG_INFO2(SETLE)
    SETLE(BaseRegister* dst) {
        this->defs.push_back(dst);
    }
};

struct CMPIMM : X86Instruction {
    DEBUG_INFO2(CMPIMM)
    CMPIMM(BaseRegister* lhs, long imm) {
        this->uses.push_back(lhs);
    }
};

struct LOADMEM : X86Instruction {
    DEBUG_INFO2(LOADMEM)
    LOADMEM(BaseRegister* dst, BaseRegister* src) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct STOREMEM : X86Instruction {
    DEBUG_INFO2(STOREMEM)
    STOREMEM(BaseRegister* dst, BaseRegister* src) {
        this->uses.push_back(src);
    }
};

struct MOVMEMOFFSET : X86Instruction {
    DEBUG_INFO2(MOVMEMOFFSET)
    MOVMEMOFFSET(BaseRegister* dst, BaseRegister* src, long offset) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct StackSlot {
    size_t id;
};

struct STORESTACK : X86Instruction {
    DEBUG_INFO2(STORESTACK)
    STORESTACK(BaseRegister* value, StackSlot stackSlot, long offset) {
        this->uses.push_back(value);
    }
};

struct LOADSTACK : X86Instruction {
    DEBUG_INFO2(LOADSTACK)
    LOADSTACK(BaseRegister* dst, StackSlot stackSlot, long offset) {
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
    LEASTACK(BaseRegister* dst, StackSlot slot) {
        this->defs.push_back(dst);
    }
};

struct MOVMEMIMM : X86Instruction {
    DEBUG_INFO2(MOVMEMIMM)
    MOVMEMIMM(BaseRegister* dst, BaseRegister* src, long imm) {
        this->defs.push_back(dst);
        this->uses.push_back(src);
    }
};

struct JZ : X86Instruction {
    DEBUG_INFO2(JZ)
    JZ(Block*) {}
};

struct Block {
    std::string tag;
    std::vector<X86Instruction*> insts;
    std::set<Block*> incoming;
    std::set<Block*> outgoing;

    size_t insertPoint = 0;

    template<typename T, typename... Args>
    void push(Args&&... args) {
        insts.insert(insts.begin() + insertPoint, new T(args...));
        insertPoint += 1;
    }

    void addTarget(Block* b) {
        b->incoming.insert(this);
        outgoing.insert(b);
    }
};

static void printBlock(Block* b) {
    for (auto inst: b->insts) {
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

struct Range {
    long start, end_ex;

    size_t endInclusive() const {
        return end_ex;
    }

    size_t endExclusive() const {
        return end_ex;
    }

    Range merge(Range other) {
        auto start = std::min(this->start, other.start);
        auto end = std::max(this->end_ex, other.end_ex);

        return Range{start, end};
    }

    bool intersects(Range other) {
        return (other.start >= this->start && other.start < this->end_ex) || (other.end_ex > this->start && other.end_ex <= this->end_ex);
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

struct Graph {
    std::vector<Block*> blocks;

    std::map<Block*, std::map<size_t, Range>> virtualRanges;
    std::map<Block*, std::map<size_t, RangeSet>> physicalRanges;

    long findDefVirt(Block* block, size_t start, size_t virtId) {
        for (long i = start - 1; i >= 0; i--) {
            if (block->insts[i]->hasDefVirt(virtId)) return i;
        }
        return -1;
    }


    long findDefPhy(Block* block, size_t start, size_t phyId) {
        for (long i = start - 1; i >= 0; i--) {
            if (block->insts[i]->hasDefPhy(phyId)) return i;
        }
        return -1;
    }

    void markLiveVirt(Block* b, long start, long endIncl, size_t virtId) {
        if (virtualRanges[b].contains(virtId)) {
            virtualRanges[b][virtId] = virtualRanges[b][virtId].merge(Range{start, endIncl + 1});
        } else {
            virtualRanges[b][virtId] = Range{start, endIncl + 1};
        }
    }

    void markLivePhy(Block* b, long start, long endIncl, size_t phyId) {
        if (physicalRanges[b].contains(phyId)) {
            physicalRanges[b][phyId].add(Range{start, endIncl + 1});
        } else {
            physicalRanges[b][phyId].add(Range{start, endIncl + 1});
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

    bool virtPhyInt(size_t id, size_t other) {
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


    void findDefVirtRec(Block* block, long foundUse, size_t virtId, std::set<Block*>& visited) {
        if (visited.contains(block)) return;
        visited.insert(block);

        if (foundUse == -1) foundUse = block->insts.size() - 1;// -1 means last inst in BB

        auto foundDef = findDefVirt(block, foundUse, virtId);
        if (foundDef != -1) {
            markLiveVirt(block, foundDef, foundUse, virtId);
        } else {
            markLiveVirt(block, 0, foundUse, virtId);// implicitly mark live range, search in incoming

            for (auto inc: block->incoming) {
                findDefVirtRec(inc, -1, virtId, visited);
            }
        }

        visited.erase(block);
    }

    void calcLiveRanges() {
        for (auto block: blocks) {
            for (auto i = 0ul; i < block->insts.size(); i++) {
                auto inst = block->insts[i];

                for (auto use: inst->uses) {
                    auto v = use->as<Virtual>();
                    if (v == nullptr) continue;
                    std::set<Block*> visited;
                    findDefVirtRec(block, i, v->id, visited);
                }

                for (auto use: inst->uses) {
                    auto v = use->as<Physical>();
                    if (v == nullptr) continue;
                    auto def = findDefPhy(block, i, v->id);
                    assert(def != -1);
                    markLivePhy(block, def, i, v->id);
                }

                for (auto use: inst->kills) {
                    auto v = use->as<Physical>();
                    if (v == nullptr) continue;
                    markLivePhy(block, i, i, v->id);
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

    std::set<size_t> getPhyIds() {
        std::set<size_t> res;

        for (const auto& [block, ranges]: physicalRanges) {
            for (auto [vId, stuff]: ranges) res.insert(vId);
        }

        return res;
    }

    std::vector<size_t> linearized;
};

struct GraphColoring {
    std::vector<size_t> registers;
    std::map<size_t, size_t> allocated;
    std::map<size_t, std::set<size_t>> interference;
    size_t regCount = 32;

    static GraphColoring create(std::map<size_t, std::set<size_t>> interf) {
        GraphColoring self;

        std::vector<size_t> regs;
        for (auto i: interf) {
            regs.push_back(i.first);
        }

        self.interference = interf;
        self.registers = regs;

        return self;
    }

    void colorReg(size_t id, size_t color) {
        allocated[id] = color;
    }

    bool regAlloc(size_t i) {
        if (i == registers.size()) {
            std::cout << "SUCESSS!!!!" << std::endl;
            return true;
        }

        auto self = registers[i];

        auto doesRegInterfere = [&](size_t reg) -> bool {
            for (auto inte: interference[self]) {
                if (!allocated.contains(inte)) continue;
                if (allocated[inte] == reg) return true;
            }
            return false;
        };


        if (allocated.contains(self)) {
            if (regAlloc(i + 1)) return true;
        } else {
            size_t sucesses = 0;
            for (auto j = 0ul; j < regCount; j++) {
                if (doesRegInterfere(j)) continue;
                sucesses += 1;

                assert(!allocated.contains(self));
                allocated[self] = j;
                std::cout << "allocating self: " << self << std::endl;
                if (regAlloc(i + 1)) return true;
                allocated.erase(self);
            }

            if (sucesses == 0) {
                std::cout << "FAILED to allocate ID=" << self << std::endl;
            }
        }

        return false;
    }
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
    size_t virtCounter = 0;
    std::map<size_t, size_t> allocas;// stackSlot - size bytes

    StackSlot allocateStack(size_t size, size_t alignment) {
        return StackSlot{0};
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
            // std::cout << "MRDAAAAAAAAAAAAA???? " << typeid(*inst).name() << " VS? " << sim->base << std::endl;
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
            // std::cout << "MRDAAAAAAAAAAAAA???? " << typeid(*inst).name() << " VS? " << sim->base << std::endl;
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
            println("DID MATCH INST: {}", inst->name);

            // block->insertPoint = 0;
            pattern->rewrite(*this, matched, block);

            // for (auto con : consumed) globalConsumed.insert(con);

            for (auto wild: wildcards) doConsuming(cfg, globalConsumed, patterns, wild, block);

            return;
        }

        println("failed to match inst {}", inst->name);
    }

    void matchBlock(ControlFlowGraph<CTX>& cfg, CodeBlock<CTX>& block, std::span<RewriteRule*> patterns) {
        std::set<IRInstruction<CTX>*> globalConsumed;

        auto bb = new Block{};

        for (auto i = 0ul; i < block.instructions.size(); i++) {
            auto cur = block.instructions[i].get();

            doConsuming(cfg, globalConsumed, patterns, cur, bb);
            // bb->insertPoint = 0;
        }

        std::cout << "BLOOOOOOOOOOOOOK" << std::endl;
        printBlock(bb);
    }

    Virtual* getReg(SSARegisterHandle hand) {
        if (!virtRegs.contains(hand)) {
            virtRegs[hand] = virtCounter++;
        }

        return new Virtual{virtRegs[hand]};
    }

    Virtual* getReg(IRInstruction<CTX>* hand) {
        return getReg(hand->target);
    }

    Block* getBlock(BlockId) {
        return nullptr;
    }

    long getImm(IRInstruction<CTX>* inst) {
        if (auto intLit = inst->template cst<instructions::IntLiteral>(); intLit) {
            return intLit->value;
        }
        TODO()
    }

    void emit(Lower<CTX>& l, MatchedInstructions inst, Block* block) {
        block->push<CMPIMM>(l.getReg(inst[0, 0]->target), l.getImm(inst[0, 1]));
        block->push<JZ>(l.getBlock(inst->branchTargets()[0]));
        block->push<JMP>(l.getBlock(inst->branchTargets()[1]));
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

    void callDynamic(Block* b) {
        b->push<STORESTACK>(new Virtual(0), 0xF);

        b->push<LOADRIP>(new Physical(RDI), 0x0);// runtime
        b->push<LOADRIP>(new Physical(RSI), 0xF);// function handle

        b->push<LEASTACK>(new Physical(RCX), 0xF); // args - buffer
        b->push<LEASTACK>(new Physical(RDX), 0x10);// ret - buffer

        b->push<CALL>(std::initializer_list<BaseRegister*>{}, std::initializer_list<BaseRegister*>{new Physical(RDI), new Physical(RSI), new Physical(RCX), new Physical(RDX)}, std::initializer_list<BaseRegister*>{});

        b->push<LOADSTACK>(new Virtual(1), 0x10);
    }

    void emitCall(std::optional<SSARegisterHandle> ret, std::span<IRInstruction<CTX>*> argz, Block* block, size_t id) {
        std::vector<int> argRegs{RDI, RSI, RCX, RDX, R8, R9};


        std::vector<int> kills{RAX, R10, R11, R9, R8, RDX, RCX, RSI, RDI};
        std::vector<int> uses;
        std::vector<int> defs;

        for (auto i = 0ul; i < argz.size(); i++) {
            block->push<MOV>(new Physical(argRegs[i]), getReg(argz[i]));
            uses.push_back(kills.back());
            kills.pop_back();
        }

        if (ret.has_value()) {
            defs.push_back(RAX);
        }

        std::vector<BaseRegister*> killsR;
        std::vector<BaseRegister*> usesR;
        std::vector<BaseRegister*> defsR;

        for (auto it: kills) killsR.push_back(new Physical(it));
        for (auto it: uses) usesR.push_back(new Physical(it));
        for (auto it: defs) defsR.push_back(new Physical(it));

        // code gen

        block->push<CALLRIP>(defsR, usesR, killsR, id);

        if (ret.has_value()) {
            block->push<MOV>(getReg(*ret), new Physical(RAX));
        }
    }

    struct RewriteBuilder {
        std::vector<RewriteRule*> rules;

        RewriteBuilder& makeRewrite(BasePattern* p, std::function<void(Lower& l, Lower::MatchedInstructions inst, Block* block)> rev) {
            rules.push_back(Lower::makeRewrite(p, rev));
            return *this;
        }
    };

    void lowerIR(ControlFlowGraph<CTX>& cfg) {
        using OP = Assembler::BinaryOp;
        using TYP = Assembler::BaseDataType;

        RewriteBuilder rewrites;

        rewrites
            .makeRewrite(
                makePattern<instructions::Branch>(makeBin<OP::EQ, TYP::I64>(WILD, makePattern<instructions::IntLiteral>())),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMPIMM>(l.getReg(inst[0, 0]->target), l.getImm(*inst[0, 1]));
                    block->push<JZ>(l.getBlock(inst->branchTargets()[0]));
                    block->push<JMP>(l.getBlock(inst->branchTargets()[1]));
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
                    block->push<MOV>(new Physical(RAX), getReg(*inst[0]));
                    block->push<RET>(std::initializer_list<BaseRegister*>{new Physical(RAX)});
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

            // mov rax, [*]
            .makeRewrite(
                makePattern<instructions::PointerLoad>(WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<LOADMEM>(getReg(*inst), getReg(*inst[0]));
                })

            // mov rax, [*]
            .makeRewrite(
                makePattern<instructions::PointerStore>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<STOREMEM>(getReg(*inst[0]), getReg(*inst[1]));
                })

            .makeRewrite(
                makeBin<OP::MUL, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<MUL>(getReg(*inst), getReg(*inst[1]));
                })


            .makeRewrite(
                makeBin<OP::SUB, TYP::I64>(WILD, makePattern<instructions::IntLiteral>()),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOV>(getReg(*inst), getReg(*inst[0]));
                    block->push<SUB>(getReg(*inst), getReg(*inst[1]));
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
                    block->push<MOV>(getReg(*inst), new Physical(RDI));
                })

            .makeRewrite(
                makePattern<instructions::AllocaPtr>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<LEASTACK>(getReg(*inst), allocateStack(inst->template cst<instructions::AllocaPtr>()->size, 8));
                })

            .makeRewrite(
                makeBin<OP::EQ, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETZ>(getReg(*inst));
                })

            .makeRewrite(
                makeBin<OP::LE, TYP::I64>(WILD, WILD),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<CMP>(getReg(*inst[0]), getReg(*inst[1]));
                    block->push<SETLE>(getReg(*inst));
                })

            .makeRewrite(
                makePattern<x86::inst::MovRip>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    block->push<MOVRIP>(getReg(*inst), inst->template cst<x86::inst::MovRip>()->id);
                })

            .makeRewrite(
                makeWildPattern<x86::inst::CallRIP<CTX>>(),
                [&](Lower& l, Lower::MatchedInstructions inst, Block* block) {
                    auto insts = inst.params | views::transform([&](auto& it) { return it->inst; }) | ranges::to<vector>();
                    l.emitCall(inst->target, insts, block);
                });

        matchCFG(cfg, rewrites.rules);
    }
    // poop poop
};