#pragma once
#include "X86Instruction.hpp"
#include <vector>


#define MIN_BY(a, b, op) (a op < b op) ? a : b
#define MAX_BY(a, b, op) (a op > b op) ? a : b

struct Range {
    X86Instruction* first;
    X86Instruction* last;

    size_t getLength() {
        size_t i = 1;
        auto cur = first;
        while (cur != last) {
            cur = cur->next;
            i++;
        }

        return i;
    }

    Range merge(Range other) {
        auto start = MIN_BY(this->first, other.first, ->orderId);
        auto end = MAX_BY(this->last, other.last, ->orderId);

        return Range{start, end};
    }

    static bool intersects(Range other, Range self) {
        return (other.first->orderId >= self.first->orderId && other.first->orderId <= self.last->orderId) || (other.last->orderId >= self.first->orderId && other.last->orderId <= self.last->orderId);
    }

    bool intersects(Range other) { return Range::intersects(other, *this) || Range::intersects(*this, other); }
};

struct ListNode {
    ListNode* next = nullptr;
    Range range;
};

struct List {
    ListNode* root = nullptr;

    struct Iter {
        ListNode* n;

        Range& operator*() const {
            return n->range;
        }

        void operator++() {
            n = n->next;
        }

        bool operator==(const Iter& other) const { return this->n == other.n; }
    };

    auto begin() {
        return Iter{root};
    }

    auto end() {
        return Iter{nullptr};
    }

    void add(Range r) {
        root = new ListNode(root, r);
    }

    void forEach(auto fn) {
        auto cur = root;
        while (cur) {
            auto n = cur->next;
            fn(cur);
            cur = n;
        }
    }

    void remove(ListNode* node, ListNode* prev) {
        if (prev == nullptr) {
            root = node->next;
        } else {
            prev->next = node->next;
        }
        delete node;
    }

    void addRang(Range newRange) {
        ListNode* prev = nullptr;
        forEach([&](ListNode* it) {
            auto r1 = it->range;
            if (it->range.intersects(newRange)) {
                newRange = r1.merge(newRange);
                remove(it, prev);
            }
            prev = it;
        });
        add(newRange);
    }
};

struct RangeSet {
    List list;

    void add(Range r) {
        list.addRang(r);
    }

    bool intersects(Range other) {
        for (auto r : list) {
            if (r.intersects(other)) return true;
        }
        return false;
    }

    bool intersects(RangeSet other) {
        for (auto a : list) {
            for (auto b : other.list) {
                if (a.intersects(b)) return true;
            }
        }
        return false;
    }

    std::optional<Range> getStart(X86Instruction* inst) {
        for (auto range : list) {
            if (range.first == inst) return range;
        }

        return std::nullopt;
    }
};