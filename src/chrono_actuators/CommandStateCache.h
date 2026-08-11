#ifndef CHRONO_ACTUATORS_COMMAND_STATE_CACHE_H
#define CHRONO_ACTUATORS_COMMAND_STATE_CACHE_H

#include <atomic>

#include "chrono_actuators/models/ActuatorModel.h"

namespace chrono {
namespace actuators {

/**
 * @brief Lock-free command/state handoff contract between async I/O and physics.
 *
 * Threading contract:
 * - Physics thread: non-blocking ReadLatestCommand() + non-blocking WriteState().
 * - I/O thread(s): non-blocking WriteCommand() + non-blocking ReadLatestState().
 * - No method may block waiting for other threads.
 *
 * This reference scaffold uses atomic snapshot exchange over trivially copyable
 * POD payloads. It provides architectural legibility now; more advanced cache
 * strategies (ring-buffer timestamps, sequence counters, diagnostics) can be
 * introduced later without changing call sites.
 */
class CommandStateCache {
  public:
    CommandStateCache();

    void WriteCommand(const model::ActuatorCommand& command) noexcept;
    model::ActuatorCommand ReadLatestCommand() const noexcept;

    void WriteState(const model::ActuatorState& state) noexcept;
    model::ActuatorState ReadLatestState() const noexcept;

  private:
    std::atomic<model::ActuatorCommand> command_;
    std::atomic<model::ActuatorState> state_;
};

inline CommandStateCache::CommandStateCache() : command_(model::ActuatorCommand{}), state_(model::ActuatorState{}) {}

inline void CommandStateCache::WriteCommand(const model::ActuatorCommand& command) noexcept {
    command_.store(command, std::memory_order_release);
}

inline model::ActuatorCommand CommandStateCache::ReadLatestCommand() const noexcept {
    return command_.load(std::memory_order_acquire);
}

inline void CommandStateCache::WriteState(const model::ActuatorState& state) noexcept {
    state_.store(state, std::memory_order_release);
}

inline model::ActuatorState CommandStateCache::ReadLatestState() const noexcept {
    return state_.load(std::memory_order_acquire);
}

}  // namespace actuators
}  // namespace chrono

#endif
