#pragma once

#include <stddef.h>
#include <stdint.h>
#include "AutoCommand.h"
#include "AutoRuntimeConfig.h"
#include "AutoRuntimeIO.h"
#include "AutoTelemetry.h"

enum class AutoRunnerState : uint8_t {
    Idle,
    Running,
    Finished,
    Error,
    Cancelled
};

class AutoRunner {
public:
    AutoRunner();

    void begin(const AutoCommand* commands, size_t count);
    void begin(const AutoCommand* commands, size_t count, const AutoRuntimeConfig& config);
    void setConfig(const AutoRuntimeConfig& config);
    void update(const AutoRuntimeInput& input, const AutoRuntimeIO& io);
    void cancel(const AutoRuntimeIO& io);
    void reset();

    bool isRunning() const;
    bool isFinished() const;
    bool hasError() const;
    bool isCancelled() const;
    size_t activeIndex() const;
    Cmd activeCommandType() const;
    AutoTelemetry telemetry() const;
    AutoRunnerState state() const;

private:
    enum CommandResult : uint8_t {
        CommandContinue,
        CommandFinished,
        CommandFailed,
        CommandSkipped
    };

    const AutoCommand* commands_;
    size_t commandCount_;
    size_t activeIndex_;
    AutoRunnerState state_;
    AutoRuntimeConfig config_;
    bool commandStarted_;
    bool activeVision_;
    uint32_t commandStartMs_;
    AutoTelemetry telemetry_;

    void sanitizeConfig();
    void startCommand(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);
    CommandResult executeCommand(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);
    CommandResult executeDriveHoldYaw(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);
    CommandResult executeStrafeHoldYaw(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);
    CommandResult executeTurnToYaw(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);
    CommandResult executeVisionPickup(const AutoRuntimeInput& input, const AutoRuntimeIO& io);
    CommandResult executeWait(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);

    void finishActiveCommand(const AutoRuntimeIO& io, const AutoCommand& command);
    void finishRoutine(const AutoRuntimeIO& io);
    void skipActiveCommandAfterTimeout(const AutoRuntimeIO& io, const AutoCommand& command);
    void enterError(const AutoRuntimeIO& io, const char* message);
    void reportError(const AutoRuntimeIO& io, const char* message);
    void stopDrive(const AutoRuntimeIO& io) const;
    void stopLift(const AutoRuntimeIO& io) const;
    void cancelVisionIfActive(const AutoRuntimeIO& io);
    void stopDriveForCommandFinish(const AutoRuntimeIO& io, Cmd command) const;
    bool requireStopDrive(const AutoRuntimeIO& io);
    bool requireDriveMecanum(const AutoRuntimeIO& io);
    bool validateSafetyInput(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command);
    bool validateCommandValues(const AutoRuntimeIO& io, const AutoCommand& command);
    bool isTimedOut(const AutoRuntimeInput& input) const;
    uint32_t elapsedMs(const AutoRuntimeInput& input) const;
    void updateTelemetry(const AutoRuntimeInput& input, float yawErrorDeg);
};
