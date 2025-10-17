#pragma once

#include <map>
#include <vector>
#include <optional>
#include <ranges>
#include <cassert>

#include "utils/utils.h"
#include "SSARegisterHandle.h"

struct LiveRange {
    SSARegisterHandle ssaReg;
    std::vector<bool> ranges;
    std::set<size_t> allocated;
    size_t parrent = -1;
};

class LiveRanges {
    std::vector<LiveRange> ranges;
    std::map<SSARegisterHandle, size_t> regIndex;
    size_t _length = 0;
  public:
    LiveRanges() {

    }

    explicit LiveRanges(std::map<SSARegisterHandle, std::vector<bool>> rengs) {
        for (const auto& [reg, frend] : rengs) {
            regIndex[reg] = ranges.size();
            ranges.push_back(LiveRange{reg, frend, {}});
        }
        _length = (*rengs.begin()).second.size();
    }

    SSARegisterHandle getSSA(size_t interval) {
        return ranges[interval].ssaReg;
    }

    size_t getInterval(SSARegisterHandle reg, size_t idex) {
        auto ints = intersections2({idex, idex});
        std::optional<size_t> found;
        for (auto intt : ints) {
            if (ranges[intt].ssaReg == reg) {
                if (found.has_value()) PANIC();
                found = intt;
            }
        }
        if (not found.has_value()) PANIC();
        return *found;
    }

    std::set<size_t> getColored(SSARegisterHandle reg) const {
        return ranges[regIndex.at(reg)].allocated;
    }

    std::set<size_t> getColored(size_t reg) const {
        return ranges[reg].allocated;
    }

    std::optional<size_t> getColoredOne(size_t reg) const {
        if (ranges[reg].allocated.empty()) {
            return {};
        } else if (ranges[reg].allocated.size() == 1) {
            return *ranges[reg].allocated.begin();
        } else {
            PANIC();
        }
    }

    size_t split(size_t src, size_t idex) {
        auto buf = std::vector<bool>(length());
        auto& src1 = ranges[src].ranges;

        for (auto i = idex; i < length(); i++) {
            buf[i] = src1[i];
            src1[i] = false;
        }

        ranges.push_back(LiveRange{ranges[src].ssaReg, buf, {}, src});

        return ranges.size()-1;
    }

    // make register live range contiguous ... this is not optimall but its simple
    void clumpLiveRanges() {
        for (auto& range : ranges) {
            auto& set = range.ranges;
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

        for (auto range : ranges) {
            auto acu = 0;
            for (auto isAlive : range.ranges) {
                if (isAlive) break;
                acu += 1;
            }
            useCount[range.ssaReg] = acu;
        }

        return useCount;
    }

    std::map<size_t, size_t> calcStarts2() {
        std::map<size_t, size_t> useCount;

        for (auto i = 0ul; i < ranges.size(); i++) {
            auto acu = 0;
            for (auto isAlive : ranges[i].ranges) {
                if (isAlive) break;
                acu += 1;
            }
            useCount[i] = acu;
        }

        return useCount;
    }

    std::map<SSARegisterHandle, size_t> calcLength() {
        std::map<SSARegisterHandle, size_t> useCount;

        for (auto range : ranges) {
            auto acu = 0;
            for (auto isAlive : range.ranges) {
                if (isAlive) acu += 1;
            }
            useCount[range.ssaReg] = acu;
        }

        return useCount;
    }

    void printRange(SSARegisterHandle reg) {
        printRange(reg.toString(), rangeForReg(reg));
    }

    void printRange(size_t reg) {
        printRange(">_<", ranges[reg].ranges);
    }

    static void printRange(std::string label, std::vector<bool>& range) {
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

        println("{} {} {}", stringify(range, {StringifyCtx{"", "", ""}}), label, subranges);
    }

    void doPrintLiveRanges() {
        for (auto& range : ranges) {
            printRange(range.ssaReg.toString(), range.ranges);
        }
    }

    bool contains(SSARegisterHandle handle) const {
        return regIndex.contains(handle);
    }

    size_t length() const {
        return _length;
    }

    size_t addRange(SSARegisterHandle handle) {
        assert(not handle.isValid() || not contains(handle));

        regIndex[handle] = ranges.size();
        ranges.push_back(LiveRange{handle, std::vector<bool>(length()), {}});

        return ranges.size()-1;
    }

    void collorReg(SSARegisterHandle hand, size_t reg) {
        ranges[regIndex[hand]].allocated.insert(reg);
    }

    void collorReg(size_t hand, size_t reg) {
        ranges[hand].allocated.insert(reg);
    }

    size_t regCount() const {
        return ranges.size();
    }

    std::vector<bool>& rangeForReg(SSARegisterHandle reg) {
        return ranges[regIndex[reg]].ranges;
    }

    std::vector<bool>& rangeForReg(size_t reg) {
        return ranges[reg].ranges;
    }

    const std::vector<bool>& rangeForReg(SSARegisterHandle reg) const {
        return ranges[regIndex.at(reg)].ranges;
    }

    void appendRange(SSARegisterHandle target, size_t start, bool value) {
        appendRange(target, start, start, value);
    }

    void appendRange(SSARegisterHandle target, size_t start, size_t end, bool value) {
        for (auto i = start; i <= end; i++) {
            rangeForReg(target)[i] = value;
        }
    }

    void appendRange(size_t target, size_t start, size_t end, bool value) {
        for (auto i = start; i <= end; i++) {
            ranges[target].ranges[i] = value;
        }
    }

    void appendRange(size_t target, size_t start, bool value) {
        appendRange(target, start, start, value);
    }

    std::vector<SSARegisterHandle> regs() const {
        std::vector<SSARegisterHandle> ret;
        for (auto item : regIndex) {
            ret.push_back(item.first);
        }

        return ret;
    }

    std::vector<size_t> intervals() const {
        std::vector<size_t> ret;
        for (auto i = 0ul; i < ranges.size(); i++) {
            ret.push_back(i);
        }

        return ret;
    }

    bool isAlive(SSARegisterHandle reg, size_t i) {
        return rangeForReg(reg)[i];
    }

    bool isAlive(size_t reg, size_t i) {
        return ranges[reg].ranges[i];
    }

    bool isAnyAlive(SSARegisterHandle reg, std::pair<size_t, size_t> range) const {
        // assert(range.first >= 0);
        assert(range.second < length());
        assert(range.second >= range.first);
        for (auto i = range.first; i <= range.second; i++) {
            if (rangeForReg(reg)[i]) return true;
        }
        return false;
    }

    bool isAnyAlive(size_t reg, std::pair<size_t, size_t> range) const {
        // assert(range.first >= 0);
        assert(range.second < length());
        assert(range.second >= range.first);
        for (auto i = range.first; i <= range.second; i++) {
            if (ranges[reg].ranges[i]) return true;
        }
        return false;
    }

    bool isAliveSafe(SSARegisterHandle reg, long i) {
        if (i < 0) return false;
        if (i >= (long)length()) return false;
        return rangeForReg(reg)[i];
    }

    bool isAliveSafe(size_t reg, long i) {
        if (i < 0) return false;
        if (i >= (long)length()) return false;
        return ranges[reg].ranges[i];
    }

    std::set<SSARegisterHandle> intersections(std::pair<size_t, size_t> range) const {
        std::set<SSARegisterHandle> regz;
        for (auto reg : regs()) {
            if (isAnyAlive(reg, range)) regz.insert(reg);
        }
        return regz;
    }

    std::set<size_t> intersections2(std::pair<size_t, size_t> range) const {
        std::set<size_t> regz;
        for (auto i = 0ul; i < ranges.size(); i++) {
            if (isAnyAlive(i, range)) regz.insert(i);
        }
        return regz;
    }

    std::pair<size_t, size_t> firstLiveRange(size_t reg) {
        for (auto i = 0UL; i < length(); i++) {
            bool isALive = ranges[reg].ranges[i];
            if (!isALive) continue;
            auto j = i;
            while (isALive && j < length()) {
                isALive = ranges[reg].ranges[j];
                j++;
            }
            return {i, j-1};
        }
        PANIC();
    }

    std::pair<size_t, size_t> firstLiveRange(SSARegisterHandle reg) {
        return firstLiveRange(regIndex[reg]);
    }
};