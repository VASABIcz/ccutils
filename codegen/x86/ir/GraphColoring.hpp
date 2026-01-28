#pragma once
#include "forward.hpp"
#include "gen64/definitions.h"
#include "Registers.hpp"

struct GraphColoring {
    std::vector<VHAND> registers;
    std::map<VHAND, x86::X64Register> allocated;
    std::map<VHAND, std::set<VHAND>> interference;
    size_t regCount = 16;
    size_t regCounter = 0;
    std::shared_ptr<Logger> logger;

    GraphColoring(std::shared_ptr<Logger> logger) : logger(logger) {}

    size_t interferenceCount(VHAND reg) { return this->interference[reg].size(); }

    VHAND getInterferenceMaxNeighbour(VHAND reg) {
        auto max = interferenceCount(reg);
        auto worst = reg;

        for (auto other: interference[reg]) {
            if (!other->canSpill()) continue;
            auto otherInt = interferenceCount(other);
            if (otherInt > max) {
                max = otherInt;
                worst = other;
            }
        }

        return worst;
    }

    static GraphColoring create(std::shared_ptr<Logger> logger, std::map<VHAND, std::set<VHAND>> interf) {
        GraphColoring self{logger};

        std::vector<VHAND> regs;
        for (auto i: interf) {
            regs.push_back(i.first);
        }

        self.interference = interf;
        self.registers = regs;

        return self;
    }

    void colorReg(VHAND id, x86::X64Register color) { allocated.emplace(id, color); }

    bool doesRegInterfere(VHAND self, x86::X64Register phyReg) {
        for (auto inte: interference[self]) {
            if (!allocated.contains(inte)) continue;
            if (allocated.at(inte) == phyReg) return true;
        }
        return false;
    };

    VHAND getUncoloredReg() { return registers[regCounter++]; }

    std::optional<VHAND> findColorForReg(VHAND reg) {
        for (auto j = 0ul; j < regCount; j++) {
            if (j == x86::Rsp.getEncoding() || j == x86::Rbp.getEncoding()) continue;
            if (doesRegInterfere(reg, x86::fromRaw(j))) continue;

            assert(!allocated.contains(reg));
            allocated.emplace(reg, x86::fromRaw(j));
            logger->DEBUG("[graph-color] allocating self: {}", reg->toString());

            return reg;
        }

        return std::nullopt;
    }

    bool hasUncoloredReg() { return regCounter < registers.size(); }

    std::optional<VHAND> regAllocFast() {
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
        for (auto [self, others]: this->interference) {
            for (auto other: others) {
                buffer += stringify("{} -- {}\n", self, other);
                buffer += "\n";
            }
        }

        buffer += "}";

        return buffer;
    }
};