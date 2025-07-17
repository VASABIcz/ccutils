C++ utilities for (mainly) building compilers


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