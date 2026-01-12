#pragma once
#include "X86Instruction.hpp"
#include <set>

struct Block {
    size_t id = 0;
    std::string tag;
    X86Instruction* first = nullptr;
    X86Instruction* last = nullptr;
    std::set<Block*> incoming;
    std::set<Block*> outgoing;

    X86Instruction* insertPoint = nullptr;
    Graph* graph = nullptr;

    size_t size = 0;

    void appender(auto&& fn) {
        auto cpy = insertPoint;
        fn();
        insertPoint = cpy;
    }

    struct Iterator {
        X86Instruction* next;

        void operator++() { next = next->getNext(); }

        X86Instruction*& operator*() { return next; }

        bool operator==(const Iterator& other) const { return this->next == other.next; }
    };

    struct RevIterator {
        X86Instruction* next;

        void operator++() { next = next->getPrev(); }

        X86Instruction*& operator*() { return next; }

        bool operator==(const RevIterator& other) const { return this->next == other.next; }
    };

    struct Iter {
        X86Instruction* start;

        Iterator begin() { return Iterator{start}; }

        Iterator end() { return Iterator{nullptr}; }
    };

    struct RevIter {
        X86Instruction* start;

        RevIterator begin() { return RevIterator{start}; }

        RevIterator end() { return RevIterator{nullptr}; }
    };

    Iter iterator() { return Iter{this->first}; }

    void insertBefore(X86Instruction* subj, X86Instruction* self) {
        if (subj == first) first = self;
        if (last == nullptr) last = self;

        auto before = subj == nullptr ? nullptr : subj->prev;
        auto after = subj;

        insertBetweene(self, before, after);
    }

    static constexpr long STEP = 1024 * 1024;

    void insertBetweene(X86Instruction* self, X86Instruction* before, X86Instruction* after) {
        if (before != nullptr) before->next = self;
        if (after != nullptr) after->prev = self;

        self->prev = before;
        self->next = after;

        if (before == nullptr && after == nullptr) {
            self->orderId = 0;
        } else if (before == nullptr) {
            self->orderId = after->orderId - STEP;
        } else if (after == nullptr) {
            self->orderId = before->orderId + STEP;
        } else {
            self->orderId = (before->orderId + after->orderId) / 2;
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
        self->block = this;

        insertAfter(insertPoint, self);

        insertPoint = self;
    }

    void addTarget(Block* b) {
        b->incoming.insert(this);
        this->outgoing.insert(b);
    }
};
