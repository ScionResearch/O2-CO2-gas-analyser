#pragma once

#include <stdint.h>

namespace calibration_status {

enum : uint16_t {
    IDLE = 0,
    RUNNING = 1,
    SUCCEEDED = 2,
    FAILED = 3
};

enum : uint16_t {
    ERROR_NONE = 0,
    ERROR_DID_NOT_STABILISE = 1,
    ERROR_CURRENT_BELOW_MIN = 2,
    ERROR_CURRENT_ABOVE_MAX = 3,
    ERROR_UNKNOWN = 4,
    ERROR_STORAGE_FAILURE = 5,
    ERROR_UNSUPPORTED = 6
};

enum : uint16_t {
    OP_NONE = 0,
    OP_O2_ZERO = 1,
    OP_O2_SPAN = 2,
    OP_CO2_ZERO = 3,
    OP_CO2_SPAN = 4,
    OP_HEATER = 5
};

enum : uint16_t {
    FLAG_O2_COMMIT_COMPLETED = 1,
    FLAG_CONFIG_SAVE_PENDING = 2
};

class Status {
public:
    uint16_t state = IDLE;
    uint16_t error = ERROR_NONE;
    uint16_t operation = OP_NONE;
    uint16_t sequence = 0;
    uint16_t flags = 0;
    uint32_t elapsedMs = 0;

    bool isRunning() const {
        return state == RUNNING;
    }

    bool begin(uint16_t newOperation, uint32_t nowMs) {
        if (isRunning()) return false;
        ++sequence;
        state = RUNNING;
        error = ERROR_NONE;
        operation = newOperation;
        startedAtMs = nowMs;
        elapsedMs = 0;
        if (newOperation == OP_O2_ZERO || newOperation == OP_O2_SPAN) {
            flags &= static_cast<uint16_t>(~FLAG_O2_COMMIT_COMPLETED);
        }
        return true;
    }

    void updateElapsed(uint32_t nowMs) {
        if (isRunning()) elapsedMs = nowMs - startedAtMs;
    }

    void finish(bool succeeded, uint16_t failureError, uint32_t nowMs) {
        if (!isRunning()) return;
        updateElapsed(nowMs);
        state = succeeded ? SUCCEEDED : FAILED;
        error = succeeded ? static_cast<uint16_t>(ERROR_NONE) : failureError;
    }

    void finishO2(uint8_t libraryError, uint32_t nowMs, bool commitCompleted) {
        if (!isRunning()) return;
        if (libraryError != 0) {
            uint16_t mappedError = ERROR_UNKNOWN;
            if (libraryError == 1) mappedError = ERROR_DID_NOT_STABILISE;
            else if (libraryError == 2) mappedError = ERROR_CURRENT_BELOW_MIN;
            else if (libraryError == 3) mappedError = ERROR_CURRENT_ABOVE_MAX;
            finish(false, mappedError, nowMs);
            return;
        }
        if (!commitCompleted) {
            finish(false, ERROR_STORAGE_FAILURE, nowMs);
            return;
        }
        flags |= FLAG_O2_COMMIT_COMPLETED;
        finish(true, ERROR_NONE, nowMs);
    }

    void setConfigSavePending(bool pending) {
        if (pending) flags |= FLAG_CONFIG_SAVE_PENDING;
        else flags &= static_cast<uint16_t>(~FLAG_CONFIG_SAVE_PENDING);
    }

private:
    uint32_t startedAtMs = 0;
};

}
