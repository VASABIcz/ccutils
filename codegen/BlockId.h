#pragma once

#include <cstdio>
#include <string>
#include "utils/stringify.h"

class BlockId {
    size_t id;
    explicit BlockId(size_t id): id(id) {}
  public:
    static BlockId raw(size_t id) {
        return BlockId{id};
    }

    static BlockId invalid() {
        return BlockId{static_cast<size_t>(-1)};
    }

    [[nodiscard]] std::string toString() const {
        return stringify("@{}", id);
    }

    auto operator<=>(const BlockId&) const = default;
};