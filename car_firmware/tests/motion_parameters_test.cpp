#include "MotionParameterCommands.hpp"
#include "MotionParameterJournal.hpp"
#include "MotionStorageInterlock.hpp"
#include "CtrlAlgorithm/BalanceStartupGate.hpp"
#include "CtrlAlgorithm/PID.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace MS = MotionSettings;

static void require(bool condition, const char* scenario)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", scenario);
        std::exit(EXIT_FAILURE);
    }
}

static bool same(const MS::Parameters& a, const MS::Parameters& b)
{
    return MS::sameParameters(a, b);
}

// A small NOR flash enforces alignment, bounds and one-way programming. Faults
// are injected at each programmed word, including the commit marker.
struct FakeFlash {
    static constexpr std::size_t capacity_bytes = 84 * 3;
    std::array<std::uint32_t, capacity_bytes / 4> words;
    int writes = 0, erases = 0, fail_after = -1;
    bool fail_erase = false, corrupt_write = false;
    bool awaiting_yield = false;

    FakeFlash() { words.fill(0xFFFFFFFFU); }
    std::uint32_t readWord(std::size_t offset)
    {
        require(offset % 4 == 0 && offset < capacity_bytes, "read stays within aligned journal words");
        return words[offset / 4];
    }
    bool programWord(std::size_t offset, std::uint32_t value)
    {
        require(!awaiting_yield, "every programmed word yields before another write");
        require(offset % 4 == 0 && offset < capacity_bytes, "write stays within aligned journal words");
        auto& word = words[offset / 4];
        require(word == 0xFFFFFFFFU, "never overwrite an occupied or torn word");
        if (fail_after == 0) return false;
        if (fail_after > 0) --fail_after;
        ++writes;
        word &= corrupt_write ? value ^ 1U : value;
        awaiting_yield = true;
        return true;
    }
    void yieldAfterProgram() { awaiting_yield = false; }
    bool erase()
    {
        ++erases;
        if (fail_erase) return false;
        words.fill(0xFFFFFFFFU);
        return true;
    }
};
using Journal = MS::ParameterJournal<FakeFlash>;
static_assert(Journal::record_bytes == 84);

static void testCommandsAndCompensation()
{
    MS::Parameters p;
    require(p.minimum_pitch_bias == 9.5F && p.angle.kp == 75.35F &&
            p.motor_deadzone == 0U, "preserve compiled calibration, gains and motor dead zone");
    for (const auto name : {"anglepid", "velocitypid", "differpid", "rollpid", "legpid"}) {
        auto edited = p;
        require(MS::applyTuning(edited, name, "  -p   1.25\r\n"), "P accepts whitespace and CRLF");
        require(MS::applyTuning(edited, name, "-i -0.23"), "negative integral coefficients remain supported");
        require(MS::applyTuning(edited, name, "-d 4e-2"), "D accepts exponent notation");
        const auto gains = std::string_view(name) == "anglepid" ? edited.angle :
            std::string_view(name) == "velocitypid" ? edited.velocity :
            std::string_view(name) == "differpid" ? edited.difference : edited.roll;
        require(gains.kp == 1.25F && gains.ki == -0.23F && gains.kd == 0.04F, "PID options map to distinct gains");
        for (const auto bad : {"", "-p", "-", "-x 1", "-p 1x", "-p nan", "-i inf", "-d -inf",
                               "-p 1e999", "-p 1 2", "-p 1\nsave", "-p2 5"}) {
            const auto before = edited;
            require(!MS::applyTuning(edited, name, bad), "reject malformed or nonfinite tuning");
            require(same(before, edited), "invalid command has no partial side effects");
        }
    }
    require(MS::applyTuning(p, "rollpid", "-p 0.3") && p.roll.ki == -0.4F, "roll P no longer overwrites I");
    require(MS::applyTuning(p, "anglebias", "10.5") && p.minimum_pitch_bias == 10.5F, "calibrate minimum-height bias");
    require(!MS::applyTuning(p, "anglebias", "nan"), "nonfinite bias is rejected");
    require(MS::applyTuning(p, "legheight", "99") && p.leg_height == 78.5F, "save bounded leg height");
    require(MS::applyTuning(p, "legheight", "0") && p.leg_height == 44.5F, "bound low leg height");
    require(MS::applyTuning(p, "target_roll", "-2") && p.roll_target == -2, "roll posture is tunable");
    require(MS::applyTuning(p, "deadzone", "75") && p.motor_deadzone == 75U,
            "shared motor dead zone is tunable");
    for (const auto bad : {"-1", "1001", "1.5", "50junk", "50 60"}) {
        const auto before_deadzone = p;
        require(!MS::applyTuning(p, "deadzone", bad), "reject invalid motor dead zone");
        require(same(before_deadzone, p), "invalid dead zone has no side effects");
    }
    require(!MS::applyTuning(p, "R", "0 0 0 61.5"), "motion frames use their separate transient path");
    require(MS::parseSaveMode("") == MS::SaveMode::append &&
            MS::parseSaveMode(" all\r\n") == MS::SaveMode::append &&
            MS::parseSaveMode("recycle") == MS::SaveMode::recycle &&
            MS::parseSaveMode("all junk") == MS::SaveMode::invalid, "save syntax is exact");
    for (const auto line : {"save", "save\n", "save\r\n", "  save  \r\n", "save all\r\n"}) {
        std::string_view text = line;
        require(MS::takeToken(text) == "save" && MS::parseSaveMode(text) == MS::SaveMode::append,
                "bare serial/radio save commands tolerate line endings");
    }
    for (unsigned n = 0; n <= 340; ++n) {
        const float h = 44.5F + 0.1F * static_cast<float>(n);
        require(std::fabs(MS::effectiveAngleKp(75.35F, h) - (0.3F * h + 56.9F)) < 0.00002F,
                "default compensation preserves original effective Kp across travel");
        require(std::fabs(MS::effectiveAngleKp(80.0F, h) - MS::effectiveAngleKp(75.35F, h) - 4.65F) < 0.00002F,
                "tuning shifts the entire height curve without accumulating changes");
    }
    require(MS::applyTuning(p, "anglepid", "-p 80"), "edit midpoint gain");
    const auto before = MS::encode(p);
    for (unsigned n = 0; n < 1000; ++n) {
        const float h = n % 2 ? 44.5F : 78.5F;
        require(std::fabs(MS::effectiveAngleKp(p.angle.kp, h) - (n % 2 ? 74.9F : 85.1F)) < 0.00002F,
                "changed baseline remains effective after repeated height changes");
    }
    require(before == MS::encode(p), "derived Kp is never written back into tunings");
    PID derivative_only(0, 0, 2, -100, 100, -100, 100);
    require(derivative_only.updateIncremental(0, 1) == -2, "leg D-only tuning reaches incremental controller");
}

static void testJournal()
{
    FakeFlash flash;
    Journal journal(flash);
    MS::Parameters first, restored;
    require(!journal.load(restored) && same(restored, first), "empty flash leaves compiled defaults intact");
    require(journal.save(first) == MS::SaveResult::saved && flash.erases == 0, "first save only programs an empty slot");
    const auto persisted = flash;
    const int initial_writes = flash.writes;
    require(journal.save(first) == MS::SaveResult::unchanged && flash.writes == initial_writes, "unchanged settings do not wear flash");

    MS::Parameters second{10.5F, {80, 0.2F, 55}, {0.04F, 0.007F, 0.001F},
        {1.5F, 0.0008F, 0.3F}, {0.2F, -0.3F, 0.01F}, 61.5F, -1.5F};
    second.motor_deadzone = 72U;
    require(journal.save(second) == MS::SaveResult::saved, "save all command-tunable fields together");
    require(Journal(flash).load(restored) && same(second, restored), "new boot restores every saved field exactly");
    // Flip each header/payload/CRC/commit word and fall back to the prior record.
    for (std::size_t word = 0; word < Journal::Record{}.size(); ++word) {
        auto damaged = flash;
        damaged.words[Journal::Record{}.size() + word] ^= 1U;
        require(Journal(damaged).load(restored) && same(first, restored), "corrupted latest record falls back atomically");
    }
    for (int cut = 0; cut < static_cast<int>(Journal::Record{}.size()); ++cut) {
        auto interrupted = persisted;
        interrupted.fail_after = cut;
        require(Journal(interrupted).save(second) == MS::SaveResult::io_error, "injected power loss is reported");
        require(Journal(interrupted).load(restored) && same(first, restored), "power loss before commit preserves prior settings");
        interrupted.fail_after = -1;
        require(Journal(interrupted).save(second) == MS::SaveResult::saved, "retry skips partially programmed slot");
        require(Journal(interrupted).load(restored) && same(second, restored), "retry produces a complete restorable set");
    }
    auto bad_write = persisted;
    bad_write.corrupt_write = true;
    require(Journal(bad_write).save(second) == MS::SaveResult::io_error, "readback catches programming corruption");
    require(Journal(bad_write).load(restored) && same(first, restored), "failed verification never commits bad data");

    auto incompatible = persisted;
    Journal::Record unsupported{};
    std::copy_n(incompatible.words.begin(), unsupported.size(), unsupported.begin());
    unsupported[1] = Journal::version + 1;
    unsupported[Journal::crc_index] = Journal::crc(unsupported);
    std::copy(unsupported.begin(), unsupported.end(), incompatible.words.begin());
    require(!Journal(incompatible).load(restored), "unknown schema is rejected even with valid CRC");
    unsupported[1] = Journal::version;
    unsupported[Journal::header_words] = 0x7FC00000U;
    unsupported[Journal::crc_index] = Journal::crc(unsupported);
    std::copy(unsupported.begin(), unsupported.end(), incompatible.words.begin());
    require(!Journal(incompatible).load(restored), "CRC-valid nonfinite payload is rejected");
    auto invalid = second;
    invalid.velocity.kp = std::numeric_limits<float>::infinity();
    const auto before_invalid = flash.words;
    require(journal.save(invalid) == MS::SaveResult::invalid && flash.words == before_invalid, "invalid save never modifies flash");
    invalid = second;
    invalid.leg_height = 100;
    require(journal.save(invalid) == MS::SaveResult::invalid, "out-of-bounds stored posture is rejected");
    invalid = second;
    invalid.motor_deadzone = MS::maximum_motor_deadzone + 1U;
    require(journal.save(invalid) == MS::SaveResult::invalid, "out-of-bounds motor dead zone is rejected");

    auto third = second;
    third.angle.kp = 81;
    require(journal.save(third) == MS::SaveResult::saved, "use final journal slot");
    auto fourth = third;
    fourth.minimum_pitch_bias = 11;
    const auto full = flash.words;
    require(journal.save(fourth) == MS::SaveResult::full && flash.words == full && flash.erases == 0,
            "full journal cannot silently erase the last known-good settings");
    require(journal.save(third, true) == MS::SaveResult::unchanged && flash.erases == 0, "same settings need no recycle even when full");
    auto failed_erase = flash;
    failed_erase.fail_erase = true;
    require(Journal(failed_erase).save(fourth, true) == MS::SaveResult::io_error, "failed maintenance erase is reported");
    auto lost_recycle = flash;
    lost_recycle.fail_after = 1;
    require(Journal(lost_recycle).save(fourth, true) == MS::SaveResult::io_error && !Journal(lost_recycle).load(restored),
            "interrupted explicit recycle has documented defaults fallback");
    require(journal.save(fourth, true) == MS::SaveResult::saved && flash.erases == 1, "explicit recycle reclaims full journal");
    require(Journal(flash).load(restored) && same(fourth, restored), "recycled journal restores newest settings");
}

static void testLegacyJournal()
{
    FakeFlash flash;
    // Construct the original on-device v1 schema explicitly, including its
    // old 15-float payload/count and 84-byte stride.
    Journal::Record legacy{};
    legacy[0] = 0x574C3150U;
    legacy[1] = 1U;
    legacy[2] = 15U;
    legacy[3] = 7U;
    MS::Parameters original;
    original.angle.kp = 80;
    original.minimum_pitch_bias = 10.5F;
    const auto words = MS::encode(original);
    std::copy(words.begin(), words.end(), legacy.begin() + 4);
    legacy[19] = Journal::crc(legacy);
    legacy[20] = 0x434F4D54U;
    std::copy(legacy.begin(), legacy.end(), flash.words.begin());
    MS::Parameters restored;
    require(Journal(flash).load(restored) && same(original, restored) &&
            restored.motor_deadzone == MS::default_motor_deadzone,
            "v1 restores midpoint mode, gains and the compiled dead-zone default");
    require(Journal(flash).save(original) == MS::SaveResult::unchanged && flash.writes == 0,
            "reading legacy settings does not force a migration write");

    FakeFlash v2_flash;
    auto v2 = legacy;
    v2[1] = 2U;
    v2[2] = MS::parameter_count | Journal::manual_kp_flag;
    v2[Journal::crc_index] = Journal::crc(v2);
    std::copy(v2.begin(), v2.end(), v2_flash.words.begin());
    auto expected_v2 = original;
    expected_v2.angle_kp_auto = false;
    require(Journal(v2_flash).load(restored) && same(expected_v2, restored) &&
            restored.motor_deadzone == MS::default_motor_deadzone,
            "v2 restores its Kp mode and the compiled dead-zone default");

    auto manual = original;
    manual.angle_kp_auto = false;
    manual.motor_deadzone = 72U;
    require(Journal(flash).save(manual) == MS::SaveResult::saved && flash.erases == 0,
            "mode and dead-zone changes append v3 without erasing legacy record");
    require(flash.words[21] == Journal::magic &&
            (flash.words[22] & Journal::version_mask) == Journal::version &&
            (flash.words[22] >> Journal::deadzone_shift) == 72U,
            "v3 keeps the v1 physical record stride and packs the motor dead zone");
    require(Journal(flash).load(restored) && same(manual, restored),
            "manual mode and motor dead zone survive a new boot");
    auto damaged = flash;
    damaged.words[41] = 0xFFFFFFFFU;
    require(Journal(damaged).load(restored) && same(original, restored), "torn v3 falls back to v1");
    Journal::Record invalid{};
    std::copy_n(flash.words.begin() + 21, invalid.size(), invalid.begin());
    invalid[2] |= 1U << 17;
    invalid[19] = Journal::crc(invalid);
    std::copy(invalid.begin(), invalid.end(), damaged.words.begin() + 21);
    require(Journal(damaged).load(restored) && same(original, restored), "unknown v3 flags rejected despite valid CRC");
    require(Journal(flash).save(original) == MS::SaveResult::saved &&
            Journal(flash).load(restored) && restored.angle_kp_auto, "returning to auto also persists");
}

static void testStorageInterlock()
{
    MotionStorageInterlock interlock;
    require(interlock.canRun(), "normal automatic startup is unchanged");
    require(interlock.begin(true, 250, -120, false), "append allowed with active control and nonzero PWM");
    require(interlock.busy() && interlock.canRun() && !interlock.consumeReset(), "append never disarms or resets PID");
    require(!interlock.begin(false, 0, 0, false), "append still excludes overlapping writers");
    interlock.finish();
    require(interlock.canRun() && !interlock.consumeReset(), "append completion does not restart the gate");
    require(!interlock.begin(true, 0, 0) && !interlock.begin(false, 1, 0) && !interlock.begin(false, 0, -1),
            "armed control or nonzero PWM blocks flash writes");
    require(!interlock.busy() && !interlock.consumeReset(), "rejected save does not disturb running control");
    BalanceStartupGate gate;
    for (unsigned i = 0; i < 49; ++i) gate.update(true, 0, 0, 0, 0, 0);
    require(interlock.begin(false, 0, 0) && !interlock.canRun(), "save locks out arming before any flash operation");
    require(!interlock.begin(false, 0, 0), "concurrent save is rejected");
    interlock.finish(); // Save completed entirely between two control ticks.
    if (interlock.consumeReset()) gate.reset();
    for (unsigned i = 0; i < 49; ++i)
        require(!gate.update(true, 0, 0, 0, 0, 0), "short save still requires a full fresh startup interval");
    require(gate.update(true, 0, 0, 0, 0, 0), "normal arming resumes after 500 ms");
    interlock.setEnabled(false);
    require(!interlock.canRun() && interlock.consumeReset() && interlock.consumeReset(), "control off keeps resetting startup gate");
    require(interlock.begin(false, 0, 0) && interlock.consumeReset(), "saving while explicitly disabled is supported");
    interlock.finish();
    require(!interlock.canRun(), "save does not undo control off");
    interlock.setEnabled(true);
    require(interlock.canRun() && interlock.consumeReset(), "control on requests normal startup from reset");
}

int main()
{
    testCommandsAndCompensation();
    testJournal();
    testLegacyJournal();
    testStorageInterlock();
    std::puts("PASS: motion commands, height gains, flash recovery/readback/recycling and storage interlock");
}
