#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include "TelemetryFrame.hpp"
#include "test_check.hpp"

int main()
{
    struct Guarded {
        uint32_t before{0xA5A5A5A5U};
        std::array<uint8_t, 32U> payload{};
        uint32_t after{0x5A5A5A5AU};
    } buffer;
    buffer.payload.fill(0xFFU);
    CHECK(radio_frame::encode_telemetry({1.0F, -2.5F, 0.0F, 70.0F}, buffer.payload));
    CHECK(std::strcmp(reinterpret_cast<char*>(buffer.payload.data()), " 1.0 -2.5 0.0 70.0") == 0);
    CHECK(buffer.payload.back() == 0U);
    CHECK(!radio_frame::encode_telemetry({1.0e30F, 2.0F, 3.0F, 4.0F}, buffer.payload));
    CHECK(std::all_of(buffer.payload.begin(), buffer.payload.end(), [](uint8_t b) { return b == 0U; }));
    CHECK(!radio_frame::encode_telemetry({std::numeric_limits<float>::infinity(), 0, 0, 0}, buffer.payload));
    CHECK(!radio_frame::encode_telemetry({std::numeric_limits<float>::quiet_NaN(), 0, 0, 0}, buffer.payload));
    CHECK(buffer.before == 0xA5A5A5A5U && buffer.after == 0x5A5A5A5AU);
}
