#pragma once

#include <map>
#include <vector>
#include <optional>
#include <ranges>
#include <cassert>

#include "utils/utils.h"
#include "SSARegisterHandle.h"

struct LiveRanges {
    std::map<SSARegisterHandle, std::vector<bool>> currentLiveRanges;

    // make register live range contiguous ... this is not optimall but its simple
    void clumpLiveRanges() {
        for (auto& range : currentLiveRanges) {
            auto& set = range.second;
            auto min = set.size(), max = 0ul;
            for (auto i = 0UL; i < set.size(); i++) {
                if (!set[i]) continue;
                if (i < min) min = i;
                if (i > max) max = i;
            }

            for (auto s = min; s <= max; s++) {
                set[s] = true;
            }
        }
    }

    std::map<SSARegisterHandle, size_t> calcStarts() {
        std::map<SSARegisterHandle, size_t> useCount;

        for (auto [reg, ranges] : currentLiveRanges) {
            auto acu = 0;
            for (auto isAlive : ranges) {
                if (isAlive) break;
                acu += 1;
            }
            useCount[reg] = acu;
        }

        return useCount;
    }

    std::map<SSARegisterHandle, size_t> calcLength() {
        std::map<SSARegisterHandle, size_t> useCount;

        for (auto [reg, ranges] : currentLiveRanges) {
            auto acu = 0;
            for (auto isAlive : ranges) {
                if (isAlive) acu += 1;
            }
            useCount[reg] = acu;
        }

        return useCount;
    }

    static void printRange(SSARegisterHandle reg, std::vector<bool>& range) {
        vector<string> subranges;

        std::optional<size_t> start = std::nullopt;
        for (auto [index, v]: range | std::ranges::views::enumerate) {
            if (!start.has_value() && v) {
                start = index;
                continue;
            }
            if (not v && start.has_value()) {
                subranges.push_back(stringify("{}..{}", *start, index - 1));
                start = nullopt;
            }
        }
        if (start.has_value() && not range.empty()) {
            subranges.push_back(stringify("{}..{}", *start, range.size() - 1));
            start = nullopt;
        }

        println("{} {} {}", stringify(range, {StringifyCtx{"", "", ""}}), reg, subranges);
    }

    void doPrintLiveRanges() {
        for (auto& range: currentLiveRanges) {
            printRange(range.first, range.second);
        }
    }

    bool contains(SSARegisterHandle handle) const {
        return currentLiveRanges.contains(handle);
    }

    size_t length() const {
        return currentLiveRanges.begin()->second.size();
    }

    void addRange(SSARegisterHandle handle) {
        assert(not contains(handle));

        currentLiveRanges[handle] = std::vector<bool>(length());
    }

    size_t regCount() const {
        return currentLiveRanges.size();
    }

    void appendRange(SSARegisterHandle target, size_t start, bool value) {
        appendRange(target, start, start, value);
    }

    void appendRange(SSARegisterHandle target, size_t start, size_t end, bool value) {
        for (auto i = start; i <= end; i++) {
            currentLiveRanges[target][i] = value;
        }
    }

    std::vector<SSARegisterHandle> regs() const {
        std::vector<SSARegisterHandle> ret;
        for (auto item : currentLiveRanges) {
            ret.push_back(item.first);
        }

        return ret;
    }

    bool isAlive(SSARegisterHandle reg, size_t i) {
        return currentLiveRanges[reg][i];
    }

    bool isAnyAlive(SSARegisterHandle reg, std::pair<size_t, size_t> range) const {
        assert(range.first >= 0);
        assert(range.second < length());
        assert(range.second >= range.first);
        for (auto i = range.first; i <= range.second; i++) {
            if (currentLiveRanges.at(reg)[i]) return true;
        }
        return false;
    }

    bool isAliveSafe(SSARegisterHandle reg, long i) {
        if (i < 0) return false;
        if (i >= (long)length()) return false;
        return currentLiveRanges[reg][i];
    }

    std::set<SSARegisterHandle> intersections(std::pair<size_t, size_t> range) const {
        std::set<SSARegisterHandle> regz;
        for (auto reg : regs()) {
            if (isAnyAlive(reg, range)) regz.insert(reg);
        }
        return regz;
    }

    std::pair<size_t, size_t> firstLiveRange(SSARegisterHandle reg) {
        for (auto i = 0UL; i < length(); i++) {
            bool isALive = currentLiveRanges[reg][i];
            if (!isALive) continue;
            auto j = i;
            while (isALive && j < length()) {
                isALive = currentLiveRanges[reg][j];
                j++;
            }
            return {i, j-1};
        }
        PANIC();
    }
};