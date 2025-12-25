#pragma once
#include "utils/utils.h"

enum class JumpCondType {
    EQUALS,
    NOT_EQUALS,
    GREATER,
    GREATER_OR_EQUAL,
    LESS,
    LESS_OR_EQUAL
};

inline std::string_view toString1(JumpCondType type) {
    switch (type) {
        case JumpCondType::EQUALS: return "==";
        case JumpCondType::NOT_EQUALS: return "!=";
        case JumpCondType::GREATER: return ">";
        case JumpCondType::GREATER_OR_EQUAL: return ">=";
        case JumpCondType::LESS: return "<";
        case JumpCondType::LESS_OR_EQUAL: return "<=";
        default: PANIC();
    }
}

inline JumpCondType negateType(JumpCondType type) {
    switch (type) {
        case JumpCondType::EQUALS: return JumpCondType::NOT_EQUALS;
        case JumpCondType::NOT_EQUALS: return JumpCondType::EQUALS;
        case JumpCondType::GREATER: return JumpCondType::LESS_OR_EQUAL;
        case JumpCondType::GREATER_OR_EQUAL: return JumpCondType::LESS;
        case JumpCondType::LESS: return JumpCondType::GREATER_OR_EQUAL;
        case JumpCondType::LESS_OR_EQUAL: return JumpCondType::GREATER;
        default:PANIC();
    }
}