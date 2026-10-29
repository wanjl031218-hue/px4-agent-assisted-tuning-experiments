#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace px4_experiment
{
struct Point { double x{0}, y{0}, z{0}; };

struct Config
{
  std::string scenario{"single_axis"}, axis{"north"};
  double displacement{2}, altitude{3}, initial_hold{2}, hold{5};
  double radius{0.3}, speed_limit{0.3}, stable_duration{1};
  double takeoff_timeout{45}, response_timeout{30}, landing_timeout{60};
  double wall_timeout{180};

  void validate() const
  {
    if (scenario != "single_axis" && scenario != "hover") {
      throw std::invalid_argument("scenario must be single_axis or hover");
    }
    if (axis != "north" && axis != "east") {
      throw std::invalid_argument("axis must be north or east (local NED)");
    }
    for (double value : {altitude, radius, speed_limit, stable_duration,
                        takeoff_timeout, response_timeout, landing_timeout, wall_timeout}) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("heights, tolerances and timeouts must be finite and positive");
      }
    }
    if (!std::isfinite(displacement) || !std::isfinite(initial_hold) ||
        !std::isfinite(hold) || initial_hold < 0 || hold <= 0 ||
        (scenario == "single_axis" && std::abs(displacement) < 0.01)) {
      throw std::invalid_argument("invalid displacement or hold duration");
    }
    // Keep this teaching package within the current small X500 SITL experiments.
    if (altitude < 1 || altitude > 10 || std::abs(displacement) > 10 ||
        radius >= altitude ||
        (scenario == "single_axis" && radius >= std::abs(displacement)) ||
        wall_timeout <= hold + initial_hold + stable_duration) {
      throw std::invalid_argument("SITL limits: height 1..10m, displacement <=10m; check wall timeout");
    }
  }
};

struct Input
{
  double wall{0}, sim{0};
  bool clock_fresh{false}, position_fresh{false}, status_fresh{false}, land_fresh{false};
  bool position_valid{false}, velocity_valid{false}, preflight_ok{false};
  bool armed{false}, disarmed{false}, offboard{false}, auto_land{false}, failsafe{false};
  bool landed{false};
  Point position{};
  double speed{0}, heading{0};
  uint8_t xy_reset{0}, z_reset{0}, heading_reset{0};
  uint64_t land_sequence{0}, status_sequence{0};
};

enum class State { WAIT, PRIME, TAKEOFF, SETTLE, STEP, HOLD, LANDING, IDLE, RELEASED };

inline const char *name(State state)
{
  switch (state) {
    case State::WAIT: return "WAIT_FOR_START";
    case State::PRIME: return "PRIME_OFFBOARD";
    case State::TAKEOFF: return "TAKEOFF";
    case State::SETTLE: return "INITIAL_HOLD";
    case State::STEP: return "SINGLE_AXIS_RESPONSE";
    case State::HOLD: return "HOLD";
    case State::LANDING: return "LANDING";
    case State::IDLE: return "IDLE";
    case State::RELEASED: return "RELEASED";
  }
  return "UNKNOWN";
}

struct Actions
{
  bool stream{false}, mode{false}, arm{false}, land{false}, disarm{false};
  Point target{};
  double yaw{0};
  std::vector<std::string> events;
};

class Mission
{
public:
  explicit Mission(Config config) : _config(std::move(config)) { _config.validate(); }
  State state() const { return _state; }
  bool failed() const { return _failed; }
  bool complete() const { return _state == State::IDLE || _state == State::RELEASED; }
  bool succeeded() const { return _state == State::IDLE && !_failed && _task_done; }
  double hold_elapsed(const Input &in) const
  { return _state == State::HOLD ? in.sim - _stage_sim : 0; }

  Actions tick(const Input &in, bool abort = false)
  {
    Actions out;
    if (complete()) { return out; }
    // A simultaneous abort request must not override an existing pilot takeover.
    if (_state != State::WAIT && _state != State::LANDING && in.status_fresh) {
      if (in.failsafe || (_engaged && (!in.offboard || !in.armed))) {
        release(in, out, "PX4 failsafe or Offboard/armed state lost; no automatic reclaim");
        return out;
      }
    }
    if (abort) {
      _failed = true;
      if (_state == State::WAIT) { change(State::IDLE, in, out); }
      else if (_state != State::LANDING) { land(in, out, "operator abort requested"); }
    }
    if (complete()) { return out; }

    if (_state == State::WAIT) {
      if (ready(in)) {
        _home = in.position;
        _takeoff = {_home.x, _home.y, _home.z - _config.altitude};
        _final = _takeoff;
        if (_config.scenario == "single_axis") {
          if (_config.axis == "north") { _final.x += _config.displacement; }
          else { _final.y += _config.displacement; }
        }
        _yaw = in.heading;
        _xy_reset = in.xy_reset; _z_reset = in.z_reset; _heading_reset = in.heading_reset;
        _last_sim = in.sim;
        change(State::PRIME, in, out);
        out.events.push_back("home captured; takeoff target z=" + std::to_string(_takeoff.z));
      } else { return out; }
    }

    if (_state == State::LANDING) {
      handle_landing(in, out);
      return out;  // Never emit an Offboard heartbeat or trajectory during landing.
    }

    if (in.status_fresh && in.failsafe) {
      release(in, out, "PX4 failsafe active; leave control to PX4");
      return out;
    }
    if (_engaged && in.status_fresh && (!in.offboard || !in.armed)) {
      release(in, out, "Offboard/armed state lost or pilot takeover; no automatic reclaim");
      return out;
    }
    if (!in.clock_fresh || !in.position_fresh || !in.status_fresh ||
        !in.position_valid || !in.velocity_valid) {
      land(in, out, "clock/state stale or invalid");
      return out;
    }
    if (in.sim + 1e-6 < _last_sim) {
      land(in, out, "simulation clock moved backwards");
      return out;
    }
    _last_sim = in.sim;
    if (in.xy_reset != _xy_reset || in.z_reset != _z_reset || in.heading_reset != _heading_reset) {
      // Estimator resets can occur during startup or takeoff in SITL. Keep the
      // episode alive, but record a data-quality event for offline review.
      _xy_reset = in.xy_reset;
      _z_reset = in.z_reset;
      _heading_reset = in.heading_reset;
      out.events.push_back("DATA FLAG: estimator reset observed; mission continues, review this episode before tuning");
    }
    if (in.position.z < _home.z - 0.3) { _seen_airborne = true; }

    switch (_state) {
      case State::PRIME:
        if (in.armed) { release(in, out, "vehicle armed before this node requested arm"); }
        else if (in.wall - _stage_wall >= 1.2) {
          change(State::TAKEOFF, in, out);
          request_start(in, out);
        }
        break;
      case State::TAKEOFF:
        if (in.armed && in.offboard) { _engaged = true; }
        if (!_engaged && in.wall - _last_start_request >= 1.0) { request_start(in, out); }
        if (_engaged && stable(_takeoff, in)) { change(State::SETTLE, in, out); }
        else if (expired(in, _config.takeoff_timeout)) { land(in, out, "takeoff timeout"); }
        break;
      case State::SETTLE:
        if (!inside(_takeoff, in)) { _stable_since = -1; }
        else if (_stable_since < 0) { _stable_since = in.sim; }
        if (_stable_since >= 0 && in.sim - _stable_since >= _config.initial_hold) {
          change(_config.scenario == "single_axis" ? State::STEP : State::HOLD, in, out);
        } else if (expired(in, _config.response_timeout)) { land(in, out, "initial hold timeout"); }
        break;
      case State::STEP:
        if (stable(_final, in)) { change(State::HOLD, in, out); }
        else if (expired(in, _config.response_timeout)) { land(in, out, "single-axis response timeout"); }
        break;
      case State::HOLD:
        // Do not reset this measurement window when the vehicle drifts.
        if (in.sim - _stage_sim >= _config.hold) {
          _task_done = true;
          out.events.push_back("hold complete: " + std::to_string(in.sim - _stage_sim) + " sim seconds");
          land(in, out, "");
        } else if (in.wall - _stage_wall > _config.wall_timeout) {
          land(in, out, "hold wall-clock watchdog expired");
        }
        break;
      default: break;
    }

    out.stream = _state != State::LANDING && !complete();
    out.target = (_state == State::STEP || _state == State::HOLD) ? _final : _takeoff;
    out.yaw = _yaw;
    return out;
  }

private:
  bool ready(const Input &in) const
  {
    // Match the proven A/B mission start policy: preflight_ok is recorded for
    // later diagnosis, but it is not a ROS-side gate for a grounded SITL run.
    // PX4 remains the authority that accepts or rejects the arm command.
    return in.clock_fresh && in.position_fresh && in.status_fresh && in.land_fresh &&
      in.position_valid && in.velocity_valid && in.disarmed && in.landed &&
      !in.failsafe && std::isfinite(in.heading);
  }
  bool inside(const Point &target, const Input &in) const
  {
    const double dx = in.position.x - target.x, dy = in.position.y - target.y;
    const double dz = in.position.z - target.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) <= _config.radius && in.speed <= _config.speed_limit;
  }
  bool stable(const Point &target, const Input &in)
  {
    if (!inside(target, in)) { _stable_since = -1; return false; }
    if (_stable_since < 0) { _stable_since = in.sim; }
    return in.sim - _stable_since >= _config.stable_duration;
  }
  bool expired(const Input &in, double sim_limit) const
  { return in.sim - _stage_sim > sim_limit || in.wall - _stage_wall > _config.wall_timeout; }
  void change(State next, const Input &in, Actions &out)
  {
    _state = next; _stage_sim = in.sim; _stage_wall = in.wall; _stable_since = -1;
    out.events.push_back(std::string("state -> ") + name(next));
  }
  void request_start(const Input &in, Actions &out)
  {
    out.mode = !in.offboard; out.arm = !in.armed; _last_start_request = in.wall;
    out.events.push_back("requested Offboard/arm; awaiting status confirmation");
  }
  void land(const Input &in, Actions &out, const std::string &reason)
  {
    if (!reason.empty()) { _failed = true; out.events.push_back("ABORT: " + reason); }
    const bool start_request_pending = _last_start_request > -100;
    out.stream = out.mode = out.arm = out.disarm = false;
    change(State::LANDING, in, out);
    _landing_sequence = in.land_sequence;
    _landing_status_sequence = in.status_sequence;
    _last_land_request = in.wall;
    out.land = in.status_fresh && !in.failsafe && (in.armed || start_request_pending);
    out.events.push_back("landing handoff: stop Offboard heartbeat AND trajectory targets");
  }
  void release(const Input &in, Actions &out, const std::string &reason)
  {
    _failed = true;
    out.stream = out.mode = out.arm = out.land = out.disarm = false;
    out.events.push_back("RELEASE: " + reason);
    change(State::RELEASED, in, out);
  }
  void handle_landing(const Input &in, Actions &out)
  {
    if (in.status_fresh && in.failsafe) {
      release(in, out, "PX4 failsafe during landing; leave recovery to PX4");
      return;
    }
    if (in.status_fresh && in.armed && !in.auto_land && !in.offboard) {
      release(in, out, "landing mode changed/pilot takeover; no automatic reclaim");
      return;
    }
    if (in.status_fresh && in.land_fresh && in.landed &&
        in.land_sequence > _landing_sequence &&
        in.status_sequence > _landing_status_sequence &&
        (_seen_airborne || (_failed && in.wall - _stage_wall >= 2.0))) {
      if (in.disarmed) {
        change(State::IDLE, in, out);
        out.events.push_back(succeeded() ? "RESULT: SUCCESS (landed and disarmed confirmed)" : "RESULT: FAILED (ground state confirmed)");
        return;
      }
      if (in.armed && in.wall - _last_disarm_request >= 1.0) {
        out.disarm = true; _last_disarm_request = in.wall;
        out.events.push_back("landed confirmed; requesting normal disarm");
      }
    }
    if (in.status_fresh && in.armed) {
      if (!in.auto_land && in.wall - _stage_wall > 3.0) {
        release(in, out, "PX4 did not confirm Land mode; inspect PX4/QGC before continuing");
        return;
      }
      if (!in.auto_land && in.wall - _last_land_request >= 1.0) {
        out.land = true; _last_land_request = in.wall;
      }
    }
    const bool timed_out = (in.clock_fresh && in.sim - _stage_sim > _config.landing_timeout) ||
      in.wall - _stage_wall > _config.wall_timeout;
    if (timed_out && !_landing_timeout_reported) {
      _failed = true; _landing_timeout_reported = true;
      out.events.push_back("LANDING TIMEOUT: keep observing PX4; no forced disarm; inspect landed/status");
    }
  }

  Config _config;
  State _state{State::WAIT};
  Point _home{}, _takeoff{}, _final{};
  double _yaw{0}, _stage_sim{0}, _stage_wall{0}, _last_sim{0}, _stable_since{-1};
  double _last_start_request{-100}, _last_land_request{-100}, _last_disarm_request{-100};
  bool _engaged{false}, _seen_airborne{false}, _task_done{false}, _failed{false};
  bool _landing_timeout_reported{false};
  uint8_t _xy_reset{0}, _z_reset{0}, _heading_reset{0};
  uint64_t _landing_sequence{0}, _landing_status_sequence{0};
};
}  // namespace px4_experiment
