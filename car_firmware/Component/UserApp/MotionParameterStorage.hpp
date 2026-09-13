#pragma once

#include "MotionParameterJournal.hpp"

namespace MotionSettings {
bool loadFromFlash(Parameters& parameters);
// Caller must hold the control inhibit for the entire operation; no ISR calls.
SaveResult saveToFlash(const Parameters& parameters, bool recycle = false);
} // namespace MotionSettings
