#include "MotionParameterJournal.hpp"

// This object is never linked into the application. Objcopy extracts only this
// section, then places it at the journal base in the factory HEX/BIN.
struct FactoryFlash;
using FactoryJournal = MotionSettings::ParameterJournal<FactoryFlash>;
static_assert(FactoryJournal::record_bytes == 84U);
static_assert(MotionSettings::valid(MotionSettings::Parameters{}));

[[gnu::used, gnu::section(".motion_defaults")]]
constexpr auto factory_defaults = FactoryJournal::makeRecord(MotionSettings::Parameters{});
