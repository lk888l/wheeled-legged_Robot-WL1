#include <cstring>
#include <new>
#include "CtrlAlgorithm/PID.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp" // Header must be safe to include twice.
#include "test_check.hpp"

int main()
{
    // Different previous stack contents must not change the first motor result.
    for (unsigned char pattern : {0x00, 0x55, 0x7F, 0xFF}) {
        alignas(PID) unsigned char storage[sizeof(PID)];
        std::memset(storage, pattern, sizeof(storage));
        auto* pid = new (storage) PID(1.0F, 0.0F, 2.0F, -100.0F, 100.0F, -10.0F, 10.0F);
        CHECK(pid->update(10.0F, 3.0F) == 1.0F);
        CHECK(pid->update(10.0F, 4.0F) == 4.0F);
        pid->reset();
        CHECK(pid->update(10.0F, 3.0F) == 1.0F);
        pid->~PID();
    }
}
