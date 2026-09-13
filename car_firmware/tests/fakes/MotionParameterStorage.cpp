#include "MotionParameterStorage.hpp"
#include "PersistentFlash.hpp"

namespace MotionSettings {
bool loadFromFlash(Parameters& parameters)
{
    return ParameterJournal<fake_flash::Flash>(fake_flash::device).load(parameters);
}
SaveResult saveToFlash(const Parameters& parameters, bool recycle)
{
    return ParameterJournal<fake_flash::Flash>(fake_flash::device).save(parameters, recycle);
}
} // namespace MotionSettings
