#pragma once

#include "codegen/ControlFlowGraph.h"
#include "codegen/IRInstruction.h"
#include "codegen/IRInstructions.h"
#include "codegen/SSARegisterHandle.h"
#include "gen64/definitions.h"
#include "utils/Logger.h"
#include "utils/Variant.h"
#include "utils/utils.h"
#include "codegen/InstructionMatcher.hpp"
#include "utils/BetterSpan.h"
#include <chrono>
#include <cstddef>

inline void formatDst(X86Instruction* inst, std::ostream& ss) {
    if (inst->defCount() == 0) {
        ss << "()";
    } else if (inst->defCount() == 1) {
        ss << inst->getDef(0)->toString();
    } else {
        ss << "(";
        for (auto i = 0u; i < inst->defCount(); i++) {
            ss << inst->getDef(i)->toString() << ",";
        }
        ss << ")";
    }
}

inline void printBlockInst(X86Instruction* inst, std::ostream& ss) {
    std::stringstream buffer;
    formatDst(inst, buffer);
    if (buffer.str().size() < 3) {
        auto size = buffer.str().size();
        repeat1(3-size) {
            buffer << "⠀";
        }
    }

    ss << buffer.str();

    ss << "⠀= ";

    ss << inst->className();

    ss << " ";

    for (auto i = 0ul; i < inst->useCount(); i++) {
        ss << inst->getUse(i)->toString();
        if (i != inst->useCount() - 1) ss << ",";
    }
    ss << "⠀⠀⠀⠀⠀";
}

static void printBlock(Block* b) {
    for (auto inst: b->iterator()) {
        if (inst->defCount() == 1) {
            std::cout << inst->getDef(0)->toString();
        } else {
            std::cout << "[";
            for (auto i = 0u; i < inst->defCount(); i++) {
                std::cout << inst->getDef(i)->toString() << ",";
            }
            std::cout << "]";
        }

        std::cout << " = ";

        std::cout << inst->className();
        std::cout << " [";
        for (auto i = 0ul; i < inst->useCount(); i++) {
            std::cout << inst->getUse(i)->toString();
            if (i != inst->useCount() - 1) std::cout << ",";
        }
        std::cout << "]";
        std::cout << std::endl;
    }
}