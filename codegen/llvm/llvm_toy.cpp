#include <iostream>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/JITSymbol.h>
#include <llvm/ExecutionEngine/Orc/Core.h>
#include <llvm/ExecutionEngine/Orc/ExecutionUtils.h>
#include <llvm/ExecutionEngine/Orc/IRCompileLayer.h>
#include <llvm/ExecutionEngine/Orc/RTDyldObjectLinkingLayer.h>
#include <llvm/ExecutionEngine/SectionMemoryManager.h>
#include "llvm/ExecutionEngine/Orc/JITTargetMachineBuilder.h"
#include "llvm/ExecutionEngine/Orc/CompileUtils.h"
#include "llvm/ExecutionEngine/Orc/AbsoluteSymbols.h"
#include "llvm/Target/TargetIntrinsicInfo.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/Error.h>
#include <map>
#include <memory>

static llvm::ExitOnError ExitOnErr;
static std::unique_ptr<llvm::LLVMContext> TheContext;
static std::unique_ptr<llvm::IRBuilder<>> Builder;
static std::unique_ptr<llvm::Module> TheModule;
auto EPC = llvm::orc::SelfExecutorProcessControl::Create();
std::unique_ptr<llvm::orc::ExecutionSession> ES = std::make_unique<llvm::orc::ExecutionSession>(std::move(*EPC));;
llvm::orc::JITDylib& TheLib = ES->createBareJITDylib("<main>");
llvm::orc::RTDyldObjectLinkingLayer ObjectLayer{*ES, [] { return std::make_unique<llvm::SectionMemoryManager>(); }};
std::unique_ptr<llvm::orc::JITTargetMachineBuilder> JTMB;
llvm::DataLayout LAYUOUT;
std::unique_ptr<llvm::orc::IRCompileLayer> TheLayout;

int toJit() {
    std::cout << "CALLED FROM JIT :3" << std::endl;
    return 69;
}

void setupLlvm() {
    JTMB = std::make_unique<llvm::orc::JITTargetMachineBuilder>(ES->getExecutorProcessControl().getTargetTriple());


    if (JTMB->getTargetTriple().isOSBinFormatCOFF()) {
        ObjectLayer.setOverrideObjectFlagsWithResponsibilityFlags(true);
        ObjectLayer.setAutoClaimResponsibilityForObjectSymbols(true);
    }

    TheLib.addGenerator(cantFail(llvm::orc::DynamicLibrarySearchGenerator::GetForCurrentProcess(LAYUOUT.getGlobalPrefix())));

    llvm::orc::SymbolMap Symbols;
    llvm::orc::MangleAndInterner Mangle(*ES, LAYUOUT);
    // llvm::JITEvaluatedSymbol(llvm::pointerToJITTargetAddress(&toJit), llvm::JITSymbolFlags::Exported)
    Symbols.insert({Mangle("uwujit"), llvm::orc::ExecutorSymbolDef::fromPtr(&toJit)});

    ExitOnErr(TheLib.define(llvm::orc::absoluteSymbols(Symbols)));

    LAYUOUT = ExitOnErr(JTMB->getDefaultDataLayoutForTarget());
    TheLayout = std::make_unique<llvm::orc::IRCompileLayer>(*ES, ObjectLayer, std::make_unique<llvm::orc::ConcurrentIRCompiler>(std::move(*JTMB)));

    TheContext = std::make_unique<llvm::LLVMContext>();
    TheModule = std::make_unique<llvm::Module>("my cool jit", *TheContext);

    // Create a new builder for the module.
    Builder = std::make_unique<llvm::IRBuilder<>>(*TheContext);
}

int main() {
    LLVMInitializeAllTargets();
    LLVMInitializeNativeTarget();
    LLVMInitializeNativeAsmPrinter();
    LLVMInitializeNativeAsmParser();
    setupLlvm();

    auto ft = llvm::FunctionType::get(llvm::Type::getInt32Ty(*TheContext), false);
    auto TheFunction = llvm::Function::Create(ft, llvm::Function::ExternalLinkage, "test", TheModule.get());
    auto uwujit = llvm::Function::Create(ft, llvm::Function::ExternalLinkage, "uwujit", TheModule.get());
    auto* BB = llvm::BasicBlock::Create(*TheContext, "entry", TheFunction);
    Builder->SetInsertPoint(BB);

    auto* a = llvm::ConstantInt::get(*TheContext, llvm::APInt(32, 3));
    auto* b = llvm::ConstantInt::get(*TheContext, llvm::APInt(32, 69));
    auto* c = Builder->CreateCall(uwujit);

    auto* ret = Builder->CreateAdd(a, b);
    auto* ret1 = Builder->CreateAdd(ret, c);

    Builder->CreateRet(ret1);

    // llvm::verifyFunction(*TheFunction, &llvm::errs());

    TheModule->print(llvm::errs(), nullptr);

    auto RT = TheLib.createResourceTracker();

    auto TSM = llvm::orc::ThreadSafeModule(std::move(TheModule), std::move(TheContext));

    ExitOnErr(TheLayout->add(TheLib, std::move(TSM)));

    auto sym = ES->lookup({&TheLib}, "test");
    assert(sym && "Function not found");


    auto ptr = sym->getAddress().toPtr<int(*)()>();


    std::cout << "LOL " << ptr() << std::endl;
}