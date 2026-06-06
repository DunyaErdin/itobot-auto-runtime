#include "AutoRunner.h"
#include "AutoMath.h"

static const char* kNullCommandsError = "AutoRunner.begin received null commands";
static const char* kStopDriveRequiredError = "AutoRuntimeIO.stopDrive is required";
static const char* kDriveMecanumRequiredError = "AutoRuntimeIO.driveMecanum is required for drive/turn commands";
static const char* kSetIntakeRequiredError = "AutoRuntimeIO.setIntake is required for intake commands";
static const char* kSetLiftRequiredError = "AutoRuntimeIO.setLift is required for lift commands";
static const char* kDropRequiredError = "AutoRuntimeIO.drop is required for Drop command";
static const char* kCommandTimeoutError = "Auto command timed out";
static const char* kAbortRequestedError = "Auto safety abort requested";
static const char* kInvalidYawError = "AutoRuntimeInput yaw is invalid";
static const char* kStaleYawError = "AutoRuntimeInput yaw is stale";
static const char* kInvalidCommandValueError = "AutoCommand contains NaN or Inf";

AutoRunner::AutoRunner()
    : commands_(0),
      commandCount_(0),
      activeIndex_(0),
      state_(AutoRunnerState::Idle),
      config_(),
      commandStarted_(false),
      activeVision_(false),
      commandStartMs_(0),
      telemetry_() {}

void AutoRunner::begin(const AutoCommand* commands, size_t count) {
    begin(commands, count, config_);
}

void AutoRunner::begin(const AutoCommand* commands, size_t count, const AutoRuntimeConfig& config) {
    setConfig(config);
    commands_ = commands;
    commandCount_ = count;
    activeIndex_ = 0;
    commandStarted_ = false;
    activeVision_ = false;
    commandStartMs_ = 0;
    telemetry_ = AutoTelemetry();

    if (commands == 0 && count > 0) {
        state_ = AutoRunnerState::Error;
        telemetry_.error = true;
        telemetry_.lastError = kNullCommandsError;
        return;
    }

    if (count == 0) {
        state_ = AutoRunnerState::Finished;
        telemetry_.finished = true;
        return;
    }

    state_ = AutoRunnerState::Running;
    telemetry_.running = true;
    telemetry_.activeIndex = 0;
    telemetry_.commandCount = commandCount_;
    telemetry_.activeCommand = commands_[0].type;
}

void AutoRunner::setConfig(const AutoRuntimeConfig& config) {
    config_ = config;
    sanitizeConfig();
}

void AutoRunner::sanitizeConfig() {
    config_.turnToleranceDeg = config_.turnToleranceDeg < 0.0f ? -config_.turnToleranceDeg : config_.turnToleranceDeg;
    config_.maxTurnPower = clampFloat(config_.maxTurnPower < 0.0f ? -config_.maxTurnPower : config_.maxTurnPower, 0.0f, 1.0f);
    config_.minTurnPower = clampFloat(config_.minTurnPower < 0.0f ? -config_.minTurnPower : config_.minTurnPower, 0.0f, config_.maxTurnPower);
    config_.maxHeadingCorrection = clampFloat(config_.maxHeadingCorrection < 0.0f ? -config_.maxHeadingCorrection : config_.maxHeadingCorrection, 0.0f, 1.0f);
    if (config_.turnKp < 0.0f) config_.turnKp = -config_.turnKp;
    if (config_.headingHoldKp < 0.0f) config_.headingHoldKp = -config_.headingHoldKp;
}

void AutoRunner::update(const AutoRuntimeInput& input, const AutoRuntimeIO& io) {
    if (state_ != AutoRunnerState::Running) {
        updateTelemetry(input, telemetry_.yawErrorDeg);
        return;
    }

    if (!requireStopDrive(io)) return;

    size_t guard = 0;
    while (state_ == AutoRunnerState::Running && guard < commandCount_) {
        if (io.shouldAbort != 0 && io.shouldAbort()) {
            if (io.onSafetyEvent != 0) {
                io.onSafetyEvent(kAbortRequestedError);
            }
            enterError(io, kAbortRequestedError);
            return;
        }
        if (commands_ == 0) {
            enterError(io, kNullCommandsError);
            return;
        }
        if (activeIndex_ >= commandCount_) {
            finishRoutine(io);
            return;
        }

        const AutoCommand& command = commands_[activeIndex_];
        if (!validateCommandValues(io, command)) return;
        if (!validateSafetyInput(input, io, command)) return;
        if (!commandStarted_) {
            startCommand(input, io, command);
        }

        const CommandResult result = executeCommand(input, io, command);
        if (result == CommandContinue || result == CommandFailed) {
            return;
        }
        if (result == CommandFinished) {
            finishActiveCommand(io, command);
            guard++;
            continue;
        }
        if (result == CommandSkipped) {
            skipActiveCommandAfterTimeout(io, command);
            guard++;
            continue;
        }
    }

    if (state_ == AutoRunnerState::Running && guard >= commandCount_) {
        enterError(io, "AutoRunner instant-command guard tripped");
    }
}

void AutoRunner::cancel(const AutoRuntimeIO& io) {
    if (state_ == AutoRunnerState::Running) {
        cancelVisionIfActive(io);
        stopDrive(io);
        stopLift(io);
        state_ = AutoRunnerState::Cancelled;
        telemetry_.running = false;
        telemetry_.finished = false;
        telemetry_.error = false;
        telemetry_.cancelled = true;
    }
}

void AutoRunner::reset() {
    commands_ = 0;
    commandCount_ = 0;
    activeIndex_ = 0;
    state_ = AutoRunnerState::Idle;
    commandStarted_ = false;
    activeVision_ = false;
    commandStartMs_ = 0;
    telemetry_ = AutoTelemetry();
}

bool AutoRunner::isRunning() const {
    return state_ == AutoRunnerState::Running;
}

bool AutoRunner::isFinished() const {
    return state_ == AutoRunnerState::Finished;
}

bool AutoRunner::hasError() const {
    return state_ == AutoRunnerState::Error;
}

bool AutoRunner::isCancelled() const {
    return state_ == AutoRunnerState::Cancelled;
}

size_t AutoRunner::activeIndex() const {
    return activeIndex_;
}

Cmd AutoRunner::activeCommandType() const {
    if (commands_ == 0 || activeIndex_ >= commandCount_) return Cmd::Stop;
    return commands_[activeIndex_].type;
}

AutoTelemetry AutoRunner::telemetry() const {
    return telemetry_;
}

AutoRunnerState AutoRunner::state() const {
    return state_;
}

void AutoRunner::startCommand(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    commandStarted_ = true;
    commandStartMs_ = input.nowMs;
    activeVision_ = false;
    telemetry_.activeIndex = activeIndex_;
    telemetry_.commandCount = commandCount_;
    telemetry_.activeCommand = command.type;
    telemetry_.commandElapsedMs = 0;
    if (io.onCommandStart != 0) {
        io.onCommandStart(activeIndex_, command.type);
    }
}

AutoRunner::CommandResult AutoRunner::executeCommand(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    switch (command.type) {
        case Cmd::DriveHoldYaw:
            return executeDriveHoldYaw(input, io, command);
        case Cmd::StrafeHoldYaw:
            return executeStrafeHoldYaw(input, io, command);
        case Cmd::TurnToYaw:
            return executeTurnToYaw(input, io, command);
        case Cmd::VisionPickup:
            return executeVisionPickup(input, io);
        case Cmd::IntakeOn:
            if (io.setIntake == 0) {
                enterError(io, kSetIntakeRequiredError);
                return CommandFailed;
            }
            io.setIntake(1.0f);
            updateTelemetry(input, 0.0f);
            return CommandFinished;
        case Cmd::IntakeOff:
            if (io.setIntake == 0) {
                enterError(io, kSetIntakeRequiredError);
                return CommandFailed;
            }
            io.setIntake(0.0f);
            updateTelemetry(input, 0.0f);
            return CommandFinished;
        case Cmd::LiftOn:
            if (io.setLift == 0) {
                enterError(io, kSetLiftRequiredError);
                return CommandFailed;
            }
            io.setLift(1.0f);
            updateTelemetry(input, 0.0f);
            return CommandFinished;
        case Cmd::LiftOff:
            if (io.setLift == 0) {
                enterError(io, kSetLiftRequiredError);
                return CommandFailed;
            }
            io.setLift(0.0f);
            updateTelemetry(input, 0.0f);
            return CommandFinished;
        case Cmd::Drop:
            if (io.drop == 0) {
                enterError(io, kDropRequiredError);
                return CommandFailed;
            }
            io.drop();
            updateTelemetry(input, 0.0f);
            return CommandFinished;
        case Cmd::Wait:
            return executeWait(input, io, command);
        case Cmd::Stop:
            stopDrive(io);
            updateTelemetry(input, 0.0f);
            return CommandFinished;
    }
    enterError(io, "Unknown AutoCommand type");
    return CommandFailed;
}

AutoRunner::CommandResult AutoRunner::executeDriveHoldYaw(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    if (!requireDriveMecanum(io)) return CommandFailed;
    const uint32_t elapsed = elapsedMs(input);
    const float yawError = shortestAngleErrorDeg(command.b, input.yawDeg);
    updateTelemetry(input, yawError);
    if (elapsed >= command.durationMs) {
        return CommandFinished;
    }

    const float power = clampFloat(command.a, -1.0f, 1.0f);
    const float correction = clampFloat(yawError * config_.headingHoldKp, -config_.maxHeadingCorrection, config_.maxHeadingCorrection);
    io.driveMecanum(power, 0.0f, correction);
    return CommandContinue;
}

AutoRunner::CommandResult AutoRunner::executeStrafeHoldYaw(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    if (!requireDriveMecanum(io)) return CommandFailed;
    const uint32_t elapsed = elapsedMs(input);
    const float yawError = shortestAngleErrorDeg(command.b, input.yawDeg);
    updateTelemetry(input, yawError);
    if (elapsed >= command.durationMs) {
        return CommandFinished;
    }

    const float strafePower = clampFloat(command.a, -1.0f, 1.0f);
    const float correction = clampFloat(yawError * config_.headingHoldKp, -config_.maxHeadingCorrection, config_.maxHeadingCorrection);
    io.driveMecanum(0.0f, strafePower, correction);
    return CommandContinue;
}

AutoRunner::CommandResult AutoRunner::executeTurnToYaw(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    if (!requireDriveMecanum(io)) return CommandFailed;

    const float yawError = shortestAngleErrorDeg(command.a, input.yawDeg);
    updateTelemetry(input, yawError);
    if (fabsf(yawError) <= config_.turnToleranceDeg) {
        stopDrive(io);
        return CommandFinished;
    }

    if (isTimedOut(input)) {
        if (config_.stopOnTimeout) {
            enterError(io, kCommandTimeoutError);
            return CommandFailed;
        }
        reportError(io, kCommandTimeoutError);
        return CommandSkipped;
    }

    float omega = clampFloat(yawError * config_.turnKp, -config_.maxTurnPower, config_.maxTurnPower);
    if (fabsf(omega) < config_.minTurnPower) {
        omega = yawError >= 0.0f ? config_.minTurnPower : -config_.minTurnPower;
    }
    io.driveMecanum(0.0f, 0.0f, omega);
    return CommandContinue;
}

AutoRunner::CommandResult AutoRunner::executeVisionPickup(const AutoRuntimeInput& input, const AutoRuntimeIO& io) {
    updateTelemetry(input, 0.0f);
    if (!activeVision_) {
        activeVision_ = true;
        if (io.startVisionPickup != 0) {
            io.startVisionPickup();
        }
    }

    if (io.updateVisionPickup != 0) {
        io.updateVisionPickup();
    }

    if (!config_.visionOwnsDrive) {
        stopDrive(io);
    }

    if (input.visionPickupFinished) {
        activeVision_ = false;
        return CommandFinished;
    }

    if (isTimedOut(input)) {
        if (config_.stopOnTimeout) {
            enterError(io, kCommandTimeoutError);
            return CommandFailed;
        }
        reportError(io, kCommandTimeoutError);
        return CommandSkipped;
    }

    return CommandContinue;
}

AutoRunner::CommandResult AutoRunner::executeWait(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    stopDrive(io);
    updateTelemetry(input, 0.0f);
    return elapsedMs(input) >= command.durationMs ? CommandFinished : CommandContinue;
}

void AutoRunner::finishActiveCommand(const AutoRuntimeIO& io, const AutoCommand& command) {
    stopDriveForCommandFinish(io, command.type);
    if (io.onCommandFinish != 0) {
        io.onCommandFinish(activeIndex_, command.type);
    }

    commandStarted_ = false;
    activeVision_ = false;

    if (command.type == Cmd::Stop) {
        finishRoutine(io);
        return;
    }

    activeIndex_++;
    if (activeIndex_ >= commandCount_) {
        finishRoutine(io);
    } else {
        telemetry_.activeIndex = activeIndex_;
        telemetry_.activeCommand = commands_[activeIndex_].type;
    }
}

void AutoRunner::finishRoutine(const AutoRuntimeIO& io) {
    stopDrive(io);
    stopLift(io);
    state_ = AutoRunnerState::Finished;
    commandStarted_ = false;
    activeVision_ = false;
    telemetry_.running = false;
    telemetry_.finished = true;
    telemetry_.error = false;
    telemetry_.cancelled = false;
    telemetry_.routineProgress = 1.0f;
}

void AutoRunner::skipActiveCommandAfterTimeout(const AutoRuntimeIO& io, const AutoCommand& command) {
    cancelVisionIfActive(io);
    stopDriveForCommandFinish(io, command.type);
    stopLift(io);
    if (io.onCommandFinish != 0) {
        io.onCommandFinish(activeIndex_, command.type);
    }
    commandStarted_ = false;
    activeVision_ = false;
    activeIndex_++;
    if (activeIndex_ >= commandCount_) {
        finishRoutine(io);
    } else {
        telemetry_.activeIndex = activeIndex_;
        telemetry_.activeCommand = commands_[activeIndex_].type;
    }
}

void AutoRunner::enterError(const AutoRuntimeIO& io, const char* message) {
    cancelVisionIfActive(io);
    stopDrive(io);
    stopLift(io);
    state_ = AutoRunnerState::Error;
    commandStarted_ = false;
    telemetry_.running = false;
    telemetry_.finished = false;
    telemetry_.error = true;
    telemetry_.cancelled = false;
    telemetry_.lastError = message;
    reportError(io, message);
}

void AutoRunner::reportError(const AutoRuntimeIO& io, const char* message) {
    telemetry_.lastError = message;
    if (io.onError != 0) {
        io.onError(message);
    }
}

void AutoRunner::stopDrive(const AutoRuntimeIO& io) const {
    if (io.stopDrive != 0) {
        io.stopDrive();
    }
}

void AutoRunner::stopLift(const AutoRuntimeIO& io) const {
    if (io.setLift != 0) {
        io.setLift(0.0f);
    }
}

void AutoRunner::cancelVisionIfActive(const AutoRuntimeIO& io) {
    if (activeVision_ && io.cancelVisionPickup != 0) {
        io.cancelVisionPickup();
    }
    activeVision_ = false;
}

void AutoRunner::stopDriveForCommandFinish(const AutoRuntimeIO& io, Cmd command) const {
    if (command == Cmd::DriveHoldYaw
        || command == Cmd::StrafeHoldYaw
        || command == Cmd::TurnToYaw
        || command == Cmd::VisionPickup
        || command == Cmd::Wait
        || command == Cmd::Stop) {
        stopDrive(io);
    }
}

bool AutoRunner::requireStopDrive(const AutoRuntimeIO& io) {
    if (io.stopDrive != 0) return true;
    enterError(io, kStopDriveRequiredError);
    return false;
}

bool AutoRunner::requireDriveMecanum(const AutoRuntimeIO& io) {
    if (io.driveMecanum != 0) return true;
    enterError(io, kDriveMecanumRequiredError);
    return false;
}

bool AutoRunner::validateCommandValues(const AutoRuntimeIO& io, const AutoCommand& command) {
    if (!isFiniteFloat(command.a) || !isFiniteFloat(command.b)) {
        enterError(io, kInvalidCommandValueError);
        return false;
    }
    return true;
}

bool AutoRunner::validateSafetyInput(const AutoRuntimeInput& input, const AutoRuntimeIO& io, const AutoCommand& command) {
    const bool needsYaw = command.type == Cmd::DriveHoldYaw
        || command.type == Cmd::StrafeHoldYaw
        || command.type == Cmd::TurnToYaw;
    if (!needsYaw) return true;

    if (!input.yawValid || !isFiniteFloat(input.yawDeg)) {
        enterError(io, kInvalidYawError);
        return false;
    }

    if (config_.yawStaleTimeoutMs > 0
        && input.yawTimestampMs > 0
        && static_cast<uint32_t>(input.nowMs - input.yawTimestampMs) > config_.yawStaleTimeoutMs) {
        enterError(io, kStaleYawError);
        return false;
    }

    return true;
}

bool AutoRunner::isTimedOut(const AutoRuntimeInput& input) const {
    if (config_.commandTimeoutMs == 0) return false;
    return elapsedMs(input) >= config_.commandTimeoutMs;
}

uint32_t AutoRunner::elapsedMs(const AutoRuntimeInput& input) const {
    return static_cast<uint32_t>(input.nowMs - commandStartMs_);
}

void AutoRunner::updateTelemetry(const AutoRuntimeInput& input, float yawErrorDeg) {
    telemetry_.running = state_ == AutoRunnerState::Running;
    telemetry_.finished = state_ == AutoRunnerState::Finished;
    telemetry_.error = state_ == AutoRunnerState::Error;
    telemetry_.cancelled = state_ == AutoRunnerState::Cancelled;
    telemetry_.activeIndex = activeIndex_;
    telemetry_.commandCount = commandCount_;
    telemetry_.activeCommand = activeCommandType();
    telemetry_.commandElapsedMs = commandStarted_ ? elapsedMs(input) : 0;
    telemetry_.yawDeg = input.yawDeg;
    telemetry_.yawErrorDeg = yawErrorDeg;
    telemetry_.visionActive = activeVision_;
    telemetry_.routineProgress = commandCount_ > 0
        ? static_cast<float>(activeIndex_) / static_cast<float>(commandCount_)
        : (state_ == AutoRunnerState::Finished ? 1.0f : 0.0f);
}
