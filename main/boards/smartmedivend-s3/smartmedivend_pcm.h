#pragma once

#include <cstdint>
#include <limits>

namespace smv {
// Preserve the original Xiaozhi NoAudioCodec 32-bit I2S -> signed PCM16 scaling.
// INMP441 capture uses 32-bit standard-I2S slots. No unverified auto-gain,
// high-pass or noise gate: the server receives the original microphone signal.
inline int16_t MicToPcm16(int32_t raw) {
    const int32_t value = raw >> 12;
    if (value > std::numeric_limits<int16_t>::max())
        return std::numeric_limits<int16_t>::max();
    if (value < std::numeric_limits<int16_t>::min())
        return std::numeric_limits<int16_t>::min();
    return static_cast<int16_t>(value);
}
}  // namespace smv
