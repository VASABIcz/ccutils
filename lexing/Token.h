#pragma once
#include <string_view>
#include "Position.h"

using namespace std;

template<typename T>
class Token {
public:
    string_view content;
    Position position;
    T type;

    explicit Token(Position position, string_view content, T type) : content(content), position(position), type(type) {
    }

    explicit Token(T type, string_view content) : content(content), position(Position{0,0,0}), type(type) {
    
    }

    explicit Token(T type) : content(""), position(Position{0,0,0}), type(type) {
    
    }

    string toString() const {
        return stringify("Token{ content: {}, type: {}, pos: {} }", content, type, position.index);
    }
};