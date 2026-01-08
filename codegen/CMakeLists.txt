

add_library(vi_codegen OBJECT
        SSARegister.h
        IRInstructions.h
        CodeBlock.h
        IRGen.h
        ControlFlowGraph.h
        forward.h
        CodeGen.h
        IRInstruction.h
        SSARegisterHandle.h
        IRGenCtx.h
        SSARegisterHandle.cpp
        Assembler.h
        StackAllocator.h
        BinaryOutput.cpp
        BinaryOutput.h
        optimizations.h
        allocators/Allocator.h
        allocators/BetterAllocator.h
        LiveRanges.h
        BlockId.h
        CodeGenImpl.h
        JumpCondType.h
        CFGPatcher.h
        InstructionMatcher.hpp
)

add_library(vi_codegen_x86 OBJECT
        $<TARGET_OBJECTS:vi_codegen>
        $<TARGET_OBJECTS:gen64>
        x86/X86Assembler.h
        x86/X86Assembler.cpp
        x86/RegAlloc.h
        x86/RegAlloc.cpp
        x86/x86_insts.h
        x86/emit_x86.h
        x86/spillStage.h
        x86/decomposeStage.h
)

target_link_libraries(vi_codegen PRIVATE ccutils)
target_link_libraries(vi_codegen_x86 PRIVATE ccutils)

set(LLVM_TARGETS_TO_BUILD "X86")

find_package(LLVM REQUIRED CONFIG)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

# Set your project compile flags.
# E.g. if using the C++ header files
# you will need to enable C++11 support
# for your compiler.

include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

# Now build our tools
add_executable(llvm_toy llvm/llvm_toy.cpp
        allocators/SimpleAllocator.h
        allocators/NewAllocator.h
        allocators/OtherAllocator.h)
# Find the libraries that correspond to the LLVM components
# that we wish to use
llvm_map_components_to_libnames(llvm_libs support core irreader)

# Link against LLVM libraries
# target_link_libraries(llvm_toy ${llvm_libs})
target_link_libraries(llvm_toy LLVM)

add_executable(x86_ir x86/x86IR.cpp)
target_link_libraries(x86_ir PRIVATE ccutils)