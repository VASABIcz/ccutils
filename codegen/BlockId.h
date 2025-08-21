#pragma once

#include <cstdio>
#include <string>
#include "utils/stringify.h"

struct BlockId {
    size_t id;

    [[nodiscard]] std::string toString() const {
        return stringify("@{}", id);
    }

    auto operator<=>(const BlockId&) const = default;
};