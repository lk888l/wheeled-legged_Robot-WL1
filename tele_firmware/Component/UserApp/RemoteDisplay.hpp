#pragma once

#include "U8g2Oled.hpp"

struct RemoteDisplayState {
    float speed;
    float turn;
    float leg_height_mm;
    float roll_degrees;
    bool leg_locked;
    bool roll_locked;
};

class RemoteDisplay {
public:
    RemoteDisplay();

    bool initialize();
    void render(const RemoteDisplayState& state);

private:
    U8g2Oled oled_;
};
