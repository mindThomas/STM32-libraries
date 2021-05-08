#pragma once

#include <cstdint>
#include <vector>

namespace lspc {

class Serializable
{
public:
    virtual std::vector<uint8_t> serialize() const = 0;
    virtual uint8_t              type() const      = 0;
};

} // namespace lspc