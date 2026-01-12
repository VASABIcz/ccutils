#include "Graph.h"
#include "gen64/definitions.h"

BaseRegister* X86Instruction::PHY(x86::X64Register reg) {
    return getGraph()->getReg(reg);
}

Graph* X86Instruction::getGraph() {
    return block->graph;
}