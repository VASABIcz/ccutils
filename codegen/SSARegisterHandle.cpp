#include "SSARegisterHandle.h"
#include "../utils/stringify.h"
#include "../utils/utils.h"

SSARegisterHandle::SSARegisterHandle(size_t regId, bool valid) : registerId(regId), mValid(valid) {}

SSARegisterHandle::SSARegisterHandle() : registerId(0), mValid(false) {}

bool SSARegisterHandle::operator<(const SSARegisterHandle& other) const {
    return registerId < other.registerId;
}

bool SSARegisterHandle::operator==(const SSARegisterHandle& other) const {
    return other.registerId == registerId;
}

SSARegisterHandle SSARegisterHandle::invalid() {
    // NOTE
    // this should be enough to fix doble counting of registers
    // visit src passes even invalid reg handle the consumer of that doesn't check that, so it peers like it uses reg 0:0
    return {SIZET_MAX, false};
}

SSARegisterHandle SSARegisterHandle::valid(size_t regId) {
    return {regId, true};
}

bool SSARegisterHandle::isValid() const {
    return mValid;
}

string SSARegisterHandle::toString() const {
    if (!isValid()) {
        return "_";
    }
    return stringify("#{}", registerId);
}

string SSARegisterHandle::toTextString() const {
    if (!isValid()) {
        return " _ ";
    }
    return stringify("#{}", registerId);
}
