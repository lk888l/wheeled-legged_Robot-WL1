#pragma once

#include "MotionParameters.hpp"
#include <algorithm>

namespace MotionSettings {

enum class SaveResult { saved, unchanged, full, invalid, io_error, busy };

// A record becomes valid only after its final commit word has been programmed.
// Never reuse a partially programmed slot. Normal saves never erase old records.
template<class Flash>
class ParameterJournal {
public:
    static constexpr std::uint32_t magic = 0x574C3150U; // WL1P
    // Keep the 84-byte stride so existing version-1 records remain readable.
    // Version 2 uses bit 16 of the field-count word for fixed/manual angle Kp.
    static constexpr std::uint32_t version = 2;
    static constexpr std::uint32_t manual_kp_flag = 1U << 16;
    static constexpr std::uint32_t committed = 0x434F4D54U;
    static constexpr std::size_t header_words = 4;
    static constexpr std::size_t crc_index = header_words + parameter_count;
    static constexpr std::size_t commit_index = crc_index + 1;
    using Record = std::array<std::uint32_t, commit_index + 1>;
    static constexpr std::size_t record_bytes = sizeof(Record);

    explicit ParameterJournal(Flash& flash) : flash_(flash) {}

    bool load(Parameters& parameters)
    {
        const auto state = scan();
        if (state.latest == no_slot) return false;
        parameters = parametersFrom(state.record);
        return true;
    }

    SaveResult save(const Parameters& parameters, bool recycle = false)
    {
        if (!valid(parameters)) return SaveResult::invalid;
        const auto state = scan();
        if (state.latest != no_slot && sameParameters(parametersFrom(state.record), parameters))
            return SaveResult::unchanged;
        auto slot = state.next;
        if (slot >= slotCount()) {
            if (!recycle) return SaveResult::full;
            // Explicit maintenance operation only: a power loss during recycling
            // can leave no valid record. Boot then keeps compiled defaults.
            if (!flash_.erase()) return SaveResult::io_error;
            slot = 0;
        }
        Record record{};
        record[0] = magic;
        record[1] = version;
        record[2] = parameter_count | (parameters.angle_kp_auto ? 0U : manual_kp_flag);
        record[3] = state.latest == no_slot ? 1U : state.record[3] + 1U;
        const auto words = encode(parameters);
        std::copy(words.begin(), words.end(), record.begin() + header_words);
        record[crc_index] = crc(record);
        record[commit_index] = committed;
        const auto offset = slot * record_bytes;
        for (std::size_t i = 0; i < commit_index; ++i) {
            if (!flash_.programWord(offset + i * 4, record[i])) return SaveResult::io_error;
            flash_.yieldAfterProgram();
        }
        // Verify all data before publishing the commit marker.
        auto readback = read(slot);
        if (!std::equal(record.begin(), record.begin() + commit_index, readback.begin()))
            return SaveResult::io_error;
        if (!flash_.programWord(offset + commit_index * 4, committed)) return SaveResult::io_error;
        flash_.yieldAfterProgram();
        readback = read(slot);
        return readback == record && validRecord(readback) ? SaveResult::saved : SaveResult::io_error;
    }

    static constexpr std::uint32_t crc(const Record& record) noexcept
    {
        // CRC-32/ISO-HDLC, explicitly little-endian words on host and target.
        std::uint32_t result = 0xFFFFFFFFU;
        for (std::size_t i = 0; i < crc_index; ++i) {
            for (unsigned byte = 0; byte < 4; ++byte) {
                result ^= (record[i] >> (byte * 8)) & 0xFFU;
                for (unsigned bit = 0; bit < 8; ++bit)
                    result = (result >> 1) ^ ((result & 1U) ? 0xEDB88320U : 0U);
            }
        }
        return ~result;
    }

private:
    static constexpr std::size_t no_slot = static_cast<std::size_t>(-1);
    struct Scan {
        std::size_t latest = no_slot;
        std::size_t next = 0;
        Record record{};
    };
    Flash& flash_;

    std::size_t slotCount() const { return flash_.capacity_bytes / record_bytes; }

    Record read(std::size_t slot)
    {
        Record record{};
        for (std::size_t i = 0; i < record.size(); ++i)
            record[i] = flash_.readWord(slot * record_bytes + i * 4);
        return record;
    }

    static Parameters parametersFrom(const Record& record)
    {
        ParameterWords words{};
        std::copy_n(record.begin() + header_words, parameter_count, words.begin());
        auto parameters = decode(words);
        parameters.angle_kp_auto = record[1] == 1U || (record[2] & manual_kp_flag) == 0U;
        return parameters;
    }

    static bool validRecord(const Record& record)
    {
        const bool supported = (record[1] == 1U && record[2] == parameter_count) ||
            (record[1] == version && (record[2] & ~manual_kp_flag) == parameter_count);
        return record[0] == magic && supported &&
            record[commit_index] == committed && record[crc_index] == crc(record) &&
            valid(parametersFrom(record));
    }

    Scan scan()
    {
        Scan state;
        for (std::size_t slot = 0; slot < slotCount(); ++slot) {
            const auto record = read(slot);
            if (!std::all_of(record.begin(), record.end(), [](auto word) { return word == 0xFFFFFFFFU; }))
                state.next = slot + 1;
            if (validRecord(record)) {
                state.latest = slot;
                state.record = record;
            }
        }
        return state;
    }
};

} // namespace MotionSettings
