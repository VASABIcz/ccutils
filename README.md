## C++ utilities for (mainly) building compilers


## NI-GEN project

### goals:
- IR for representing x86 (similar to LLVM mir)
- lowering pass from CFG SSA IR into x86 IR
- live range pass on x86 IR
- build interference graph
- graph coloring register allocation
- spilling/spliting strategy / x86 IR rewriting
- register allocator needs to support precollored values (calling convention, div)

### simplifications:
 - only 64bit regs for simplicity
 - no FPU regs for simplicity

---


### structure:
codegen
 - "Assembler" interface
   - x86 - works well
   - Bytecode - VIPL only
   - arm64 - unmaintained
 - "VIIR" SSA IR
 - utils for converting to "VIIR"

parsing
  - utils for parsing

lexing:
  - utils for tokenizing

gen64:
  - utils for emitting x86-64 machine code

utilities
  - not so optimal but convenient helpers
  - formated printing, error handling, macros
