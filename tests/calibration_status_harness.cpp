#include "calibration_status.h"

#include <cassert>
#include <cstdint>

using calibration_status::Status;

int main() {
    Status status;
    assert(status.state == calibration_status::IDLE);
    assert(status.error == calibration_status::ERROR_NONE);
    assert(status.operation == calibration_status::OP_NONE);
    assert(status.sequence == 0);

    assert(status.begin(calibration_status::OP_O2_ZERO, 0xFFFFFFF0u));
    status.updateElapsed(0x10u);
    assert(status.state == calibration_status::RUNNING);
    assert(status.elapsedMs == 32u);
    assert(!status.begin(calibration_status::OP_O2_SPAN, 0x20u));
    assert(status.sequence == 1);
    assert(status.operation == calibration_status::OP_O2_ZERO);
    assert(status.elapsedMs == 32u);

    status.finishO2(2, 0x30u, false);
    assert(status.state == calibration_status::FAILED);
    assert(status.error == calibration_status::ERROR_CURRENT_BELOW_MIN);
    assert(status.elapsedMs == 64u);

    assert(status.begin(calibration_status::OP_O2_SPAN, 0x40u));
    status.finishO2(0, 0x50u, true);
    assert(status.state == calibration_status::SUCCEEDED);
    assert(status.error == calibration_status::ERROR_NONE);
    assert((status.flags & calibration_status::FLAG_O2_COMMIT_COMPLETED) != 0);

    status.setConfigSavePending(true);
    assert(status.flags == (calibration_status::FLAG_O2_COMMIT_COMPLETED
                            | calibration_status::FLAG_CONFIG_SAVE_PENDING));
    status.setConfigSavePending(false);
    assert(status.flags == calibration_status::FLAG_O2_COMMIT_COMPLETED);

    assert(status.begin(calibration_status::OP_CO2_SPAN, 0x60u));
    status.finish(false, calibration_status::ERROR_UNSUPPORTED, 0x60u);
    assert(status.state == calibration_status::FAILED);
    assert(status.error == calibration_status::ERROR_UNSUPPORTED);
    assert(status.sequence == 3);

    assert(status.begin(calibration_status::OP_O2_ZERO, 0x70u));
    assert((status.flags & calibration_status::FLAG_O2_COMMIT_COMPLETED) == 0);
    status.finishO2(1, 0x71u, false);
    assert(status.state == calibration_status::FAILED);
    assert(status.error == calibration_status::ERROR_DID_NOT_STABILISE);

    const uint8_t libraryErrors[] = {1, 2, 3, 99};
    const uint16_t statusErrors[] = {
        calibration_status::ERROR_DID_NOT_STABILISE,
        calibration_status::ERROR_CURRENT_BELOW_MIN,
        calibration_status::ERROR_CURRENT_ABOVE_MAX,
        calibration_status::ERROR_UNKNOWN
    };
    for (unsigned i = 0; i < sizeof(libraryErrors) / sizeof(libraryErrors[0]); ++i) {
        Status mapped;
        assert(mapped.begin(calibration_status::OP_O2_ZERO, 100));
        mapped.finishO2(libraryErrors[i], 125, false);
        assert(mapped.state == calibration_status::FAILED);
        assert(mapped.error == statusErrors[i]);
        assert(mapped.elapsedMs == 25u);
        assert(mapped.flags == 0);
    }

    Status storageFailure;
    assert(storageFailure.begin(calibration_status::OP_O2_ZERO, 0));
    storageFailure.finishO2(0, 1, false);
    assert(storageFailure.state == calibration_status::FAILED);
    assert(storageFailure.error == calibration_status::ERROR_STORAGE_FAILURE);
    assert(storageFailure.flags == 0);

    return 0;
}
