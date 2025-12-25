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

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    size_t h;
    size_t s;
    size_t v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

string SSARegisterHandle::toString() const {
    if (!isValid()) {
        return "_";
    }
    std::stringstream s;
    auto h = hashInt(registerId+31);
    auto [r,g,b] = HsvToRgb(HsvColor{(h >> 1) & 0xFF, ((h >> 32) & 0xFF) | 0xF, 255});
    putColor(s, r, g, b);
    s << "#" << registerId;
    resetColor(s);
    return s.str();
}

string SSARegisterHandle::toTextString() const {
    if (!isValid()) {
        return " _ ";
    }
    std::stringstream s;
    auto h = hashInt(registerId+31);
    putColor(s, (h >> 0) & 0xFF, (h >> 8) & 0xFF, (h >> 16) & 0xFF);
    s << "#" << registerId;
    resetColor(s);
    return s.str();
}
