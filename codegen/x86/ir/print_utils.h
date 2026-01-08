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
        if (i != inst->uses.size() - 1) ss << ",";
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
            if (i != inst->uses.size() - 1) std::cout << ",";
        }
        std::cout << "]";
        std::cout << std::endl;
    }
}