#pragma once
#include <memory>

/// usage
/// #define PDO_VISITOR MyVisitor
/// #define PDO_BASE MyDefaultBase

#define STRUCT_BEGIN(nejm) struct Data {
#define STRUCT_END(nejm) } data; nejm(Data data): data(std::move(data)) {} void visit(ASTVisitor& it) override { it.invoke(*this); } const std::string_view className() const override { return #nejm; }

#define STRUCT_BASE(nejm, fields) struct Data fields data; nejm(Data data): data(std::move(data)) {}

#define STRUCT_MAKE(nejm, fields, ...) struct _Data_##nejm fields; struct nejm: _Data_##nejm __VA_OPT__(,) __VA_ARGS__ { using Data = _Data_##nejm; nejm(_Data_##nejm _data): _Data_##nejm(std::move(_data)) {} void visit(PDO_VISITOR& it) override { it.invoke(*this); } const std::string_view className() const override { return #nejm; }

#define STRUCT_BASIC(nejm, fields) STRUCT_MAKE(nejm, fields, PDO_BASE)
#define EXPR_BASE(nejm, fields) STRUCT_MAKE(nejm, fields, Expression)
#define DECL_BASE(nejm, fields) STRUCT_MAKE(nejm, fields, Decl)
#define STATEMENT_BASE(nejm, fields) STRUCT_MAKE(nejm, fields, Statement)

template<typename T, typename... ARGS>
std::unique_ptr<T> makeStuff(ARGS&&... args) {
    return make_unique<T>(typename T::Data(std::forward<ARGS>(args)...));
}

template<typename T, typename... ARGS>
T* makeStuff2(ARGS&&... args) {
    return new T(typename T::Data(args...));
}