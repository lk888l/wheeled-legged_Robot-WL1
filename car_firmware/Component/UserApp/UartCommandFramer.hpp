#pragma once

#include <cstdint>
#include "etl/string.h"
#include "etl/string_view.h"

namespace app {

// Legacy remotes send one complete command per UART idle event. New clients
// use @<command>\n: the marker makes split numeric tokens unambiguous, even
// when the old protocol remains enabled. Payload capacity excludes framing.
class UartCommandFramer final {
public:
    template<typename Emit>
    void feed(etl::string_view bytes, Emit emit, uint32_t tick = 0U)
    {
        const auto finish = [&] {
            if (!discard_ && !payload_.empty()) { emit(etl::string_view(payload_)); }
            payload_.clear();
            framed_ = discard_ = false;
        };
        for (const unsigned char byte : bytes) {
            if (byte == '@') {
                payload_.clear();
                framed_ = true;
                discard_ = false;
                frame_started_ = tick;
            } else if (byte == '\n' || byte == '\r' || byte == '\0') {
                finish();
            } else if (!discard_) {
                if ((byte < 0x20U && byte != '\t') || byte > 0x7EU || payload_.full()) {
                    payload_.clear();
                    discard_ = true;
                } else {
                    payload_.push_back(static_cast<char>(byte));
                }
            }
        }
        if (!framed_) { finish(); }
    }

    // A UART error/overflow must not turn a surviving suffix into a command.
    // Framed input waits for a fresh marker/ending. Legacy input discards the
    // next idle block, then accepts complete legacy frames again.
    void discard_partial() { payload_.clear(); discard_ = true; }

    void expire(uint32_t tick, uint32_t timeout)
    {
        if (framed_ && static_cast<uint32_t>(tick - frame_started_) >= timeout) {
            discard_partial();
        }
    }

private:
    etl::string<32U> payload_{};
    bool framed_{};
    bool discard_{};
    uint32_t frame_started_{};
};

} // namespace app
