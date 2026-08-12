// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: chrono-actuators contributors
// =============================================================================
//
// Hardware/HIL edge: non-blocking command exchange + safety policy.
//
// Seqlock command exchange (Invariant C):
//   Writer:  increment seq (odd), write payload, increment seq (even).
//   Reader:  load seq, read payload, reload seq; retry if odd or changed.
//   Uses std::atomic<uint32_t> only — genuinely lock-free, no libatomic.
//
// Safety policy (all independently testable without Chrono or ROS):
//   1. Staleness watchdog  — zero effort after configurable timeout.
//   2. Engage gate         — must be explicitly engaged; defaults to off.
//   3. Ramp-in             — linear ramp from zero over configurable time.
//   4. Layered clamps      — model-level + independent output clamp.
//   5. Zero on destruct.
// =============================================================================

#ifndef CHRONO_ACTUATORS_CH_ACTUATOR_IO_H
#define CHRONO_ACTUATORS_CH_ACTUATOR_IO_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "chrono_actuators/models/ActuatorModel.h"
#include "chrono_actuators/models/Telemetry.h"

namespace chrono {
namespace actuators {

/// Simulation mode taxonomy.
/// Selects which binding is active and who owns the state.
enum class ActuatorMode : int {
    Sil      = 0,  ///< Software-in-loop: Chrono owns state; integrates dynamics; publishes state.
    Parallel = 1,  ///< Parallel: hardware owns state; Chrono shadows; observer correction applied.
    Hil      = 2   ///< Hardware-in-loop: hardware owns state; Chrono evaluates load effort from
                   ///  measured displacement/velocity only (no dynamics integration).
};

/// Safety policy parameters for ChActuatorIO.
struct ActuatorIOPolicy {
    double watchdog_timeout_s = 0.5;   ///< Zero effort after this many seconds without a fresh command.
    double ramp_time_s        = 0.1;   ///< Linear ramp duration on engage [s].
    double model_effort_clamp = 1e30;  ///< Model-level effort clamp [N] or [N·m].
    double output_effort_clamp= 1e30;  ///< Independent output clamp applied after model-level clamp.
};

/// Non-blocking command/telemetry exchange at the hardware edge.
///
/// The seqlock uses only std::atomic<uint32_t> (lock-free on all architectures)
/// instead of std::atomic<ActuatorCommand> (which exceeds 16 bytes and falls
/// back to a global mutex table — see docs/architecture.md §7).
///
/// Poll once per accepted step via ReadCommand().  The returned command is
/// then constant for the duration of that step (Invariant C).
class ChActuatorIO {
  public:
    explicit ChActuatorIO(const ActuatorIOPolicy& policy = ActuatorIOPolicy{})
        : policy_(policy)
        , seq_(0)
        , telem_seq_(0)
        , engaged_(false)
        , engage_time_(-1.0) {
        ActuatorCommand zero{};
        std::memcpy(cmd_buf_, &zero, sizeof(zero));
        ActuatorTelemetry tzero{};
        std::memcpy(telem_buf_, &tzero, sizeof(tzero));
    }

    ~ChActuatorIO() {
        ActuatorCommand zero{};
        WriteCommand(zero);
    }

    // -------------------------------------------------------------------------
    // Writer side (hardware/ROS thread)
    // -------------------------------------------------------------------------

    /// Write a new command from hardware/ROS.  Wait-free.
    ///
    /// Payload is copied via memcpy through an unsigned-char buffer to avoid
    /// the C++ data-race UB that would arise from concurrent access to a plain
    /// (non-atomic) struct even when guarded by fences alone.
    void WriteCommand(const ActuatorCommand& cmd) noexcept {
        uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);   // odd = write in progress
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(cmd_buf_, &cmd, sizeof(ActuatorCommand));
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_release);   // even = stable
        last_command_time_real_ = std::chrono::steady_clock::now();
    }

    // -------------------------------------------------------------------------
    // Reader side (physics thread)
    // -------------------------------------------------------------------------

    /// Read the latest command, applying safety policy.
    /// Call once per accepted step at the top of the step (Invariant C).
    ///
    /// @param sim_time  Current simulation time [s].
    /// @return  Safe command ready for model evaluation.
    ActuatorCommand ReadCommand(double sim_time) noexcept {
        ActuatorCommand raw = ReadRaw();

        // 1. Staleness watchdog
        const double age_s = WallAge();
        if (age_s > policy_.watchdog_timeout_s || last_command_time_real_ == TimePoint{}) {
            raw.effort  = 0.0;
            raw.enabled = false;
        }

        // 2. Engage gate
        if (!engaged_) {
            raw.effort  = 0.0;
            raw.enabled = false;
            return raw;
        }

        // 3. Ramp-in
        if (engage_time_ >= 0.0 && policy_.ramp_time_s > 0.0) {
            const double elapsed = sim_time - engage_time_;
            if (elapsed < policy_.ramp_time_s) {
                const double alpha = std::max(0.0, elapsed / policy_.ramp_time_s);
                raw.effort *= alpha;
            }
        }

        // 4. Model-level clamp
        raw.effort = std::clamp(raw.effort,
                                -policy_.model_effort_clamp,
                                 policy_.model_effort_clamp);

        return raw;
    }

    /// Apply output clamp (call after model ComputeEffort).
    double ApplyOutputClamp(double effort) const noexcept {
        return std::clamp(effort,
                          -policy_.output_effort_clamp,
                           policy_.output_effort_clamp);
    }

    // -------------------------------------------------------------------------
    // Engage / disengage
    // -------------------------------------------------------------------------

    /// Engage actuator.  Triggers ramp-in starting at sim_time.
    void Engage(double sim_time) noexcept {
        engaged_     = true;
        engage_time_ = sim_time;
    }

    /// Disengage actuator.  Immediately zeros effort.
    void Disengage() noexcept {
        engaged_     = false;
        engage_time_ = -1.0;
    }

    bool IsEngaged() const noexcept { return engaged_; }

    // -------------------------------------------------------------------------
    // Telemetry (physics writes, hardware/ROS reads)
    // -------------------------------------------------------------------------

    void WriteTelemetry(const ActuatorTelemetry& telem) noexcept {
        uint32_t s = telem_seq_.load(std::memory_order_relaxed);
        telem_seq_.store(s + 1, std::memory_order_release);
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(telem_buf_, &telem, sizeof(ActuatorTelemetry));
        std::atomic_thread_fence(std::memory_order_release);
        telem_seq_.store(s + 2, std::memory_order_release);
    }

    ActuatorTelemetry ReadTelemetry() const noexcept {
        ActuatorTelemetry t;
        uint32_t s0, s1;
        do {
            s0 = telem_seq_.load(std::memory_order_acquire);
            if (s0 & 1u)
                continue;
            std::memcpy(&t, telem_buf_, sizeof(ActuatorTelemetry));
            std::atomic_thread_fence(std::memory_order_acquire);
            s1 = telem_seq_.load(std::memory_order_acquire);
        } while (s0 != s1);
        return t;
    }

    const ActuatorIOPolicy& GetPolicy() const { return policy_; }

  private:
    using TimePoint = std::chrono::steady_clock::time_point;

    ActuatorCommand ReadRaw() const noexcept {
        ActuatorCommand out;
        uint32_t s0, s1;
        do {
            s0  = seq_.load(std::memory_order_acquire);
            if (s0 & 1u)
                continue;
            std::memcpy(&out, cmd_buf_, sizeof(ActuatorCommand));
            std::atomic_thread_fence(std::memory_order_acquire);
            s1  = seq_.load(std::memory_order_acquire);
        } while (s0 != s1);
        return out;
    }

    double WallAge() const noexcept {
        if (last_command_time_real_ == TimePoint{})
            return 1e30;
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - last_command_time_real_).count();
    }

    ActuatorIOPolicy policy_;

    // Seqlock for command (writer = hardware/ROS thread).
    // Payload stored in unsigned-char buffer so memcpy is the only access path
    // — avoiding the C++ data-race UB of concurrent access to a plain struct.
    std::atomic<uint32_t> seq_;
    alignas(ActuatorCommand) unsigned char cmd_buf_[sizeof(ActuatorCommand)]{};

    // Seqlock for telemetry (writer = physics thread).
    std::atomic<uint32_t> telem_seq_;
    alignas(ActuatorTelemetry) unsigned char telem_buf_[sizeof(ActuatorTelemetry)]{};

    bool   engaged_;
    double engage_time_;

    TimePoint last_command_time_real_{};
};

}  // namespace actuators
}  // namespace chrono

#endif  // CHRONO_ACTUATORS_CH_ACTUATOR_IO_H
