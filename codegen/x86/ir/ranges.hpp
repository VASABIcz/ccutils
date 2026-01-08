#pragma once
#include "X86Instruction.hpp"
#include <vector>


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

    bool intersects(Range other) { return Range::intersects(other, *this) || Range::intersects(*this, other); }
};

struct RangeSet {
    std::vector<Range> ranges;

    void add(Range r) { ranges.push_back(r); }

    bool intersects(Range other) {
        for (auto r: ranges) {
            if (r.intersects(other)) return true;
        }
        return false;
    }
};