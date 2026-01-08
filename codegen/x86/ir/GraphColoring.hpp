#pragma once

struct GraphColoring {
    std::vector<size_t> registers;
    std::map<size_t, x86::X64Register> allocated;
    std::map<size_t, std::set<size_t>> interference;
    size_t regCount = 16;
    size_t regCounter = 0;
    std::shared_ptr<Logger> logger;

    GraphColoring(std::shared_ptr<Logger> logger) : logger(logger) {}

    size_t interferenceCount(size_t reg) { return this->interference[reg].size(); }

    size_t getInterferenceMaxNeighbour(size_t reg) {
        auto max = interferenceCount(reg);
        auto worst = reg;

        for (auto other: interference[reg]) {
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
        for (auto i: interf) {
            regs.push_back(i.first);
        }

        self.interference = interf;
        self.registers = regs;

        return self;
    }

    void colorReg(size_t id, x86::X64Register color) { allocated.emplace(id, color); }

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

    size_t getUncoloredReg() { return registers[regCounter++]; }

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

    bool hasUncoloredReg() { return regCounter < registers.size(); }

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