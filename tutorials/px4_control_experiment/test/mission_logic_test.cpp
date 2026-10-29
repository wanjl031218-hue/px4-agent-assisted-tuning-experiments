#include <iostream>
#include <limits>
#include <stdexcept>
#include "px4_control_experiment/mission_logic.hpp"

using namespace px4_experiment;

void check(bool condition, const char *reason)
{
  if (!condition) { throw std::runtime_error(reason); }
}

struct Fixture
{
  Config config;
  Mission mission;
  Input in;
  explicit Fixture(Config c = {}) : config(c), mission(c)
  {
    in.clock_fresh = in.position_fresh = in.status_fresh = in.land_fresh = true;
    in.position_valid = in.velocity_valid = in.preflight_ok = true;
    in.disarmed = in.landed = true;
    in.position = {10, -4, 0.2}; in.heading = 1.2;
    in.land_sequence = in.status_sequence = 1;
  }
  Actions advance(double sim_delta, double wall_delta = -1)
  {
    in.sim += sim_delta;
    in.wall += wall_delta < 0 ? sim_delta : wall_delta;
    ++in.status_sequence;
    return mission.tick(in);
  }
  void takeoff()
  {
    auto out = mission.tick(in);
    check(mission.state() == State::PRIME && out.stream && !out.arm, "must prime before arm");
    check(std::abs(out.target.z + 2.8) < 1e-9 && out.target.x == 10 && out.yaw == 1.2,
      "takeoff must preserve relative home and heading");
    out = advance(1.3);
    check(out.mode && out.arm && mission.state() == State::TAKEOFF, "must request mode/arm after priming");
    in.armed = in.offboard = true; in.disarmed = in.landed = false;
    in.position.z -= config.altitude; ++in.land_sequence;
    advance(0.1);
    advance(config.stable_duration + 0.01);
    check(mission.state() == State::SETTLE, "must observe stable takeoff");
    advance(0.1);
    advance(config.initial_hold + 0.01);
  }
  void finish_landing()
  {
    check(mission.state() == State::LANDING, "expected landing");
    in.offboard = false; in.auto_land = true;
    auto out = advance(1);
    check(!out.stream && !out.disarm, "never stream or disarm while airborne in Land");
    in.landed = true; ++in.land_sequence;
    out = advance(1);
    check(out.disarm && !out.stream && mission.state() == State::LANDING, "request disarm only after landed");
    in.armed = false; in.disarmed = true;
    out = advance(0.1);
    check(mission.state() == State::IDLE && !out.stream, "IDLE requires actual disarm feedback");
  }
};

int main()
{
  try {
    {
      Fixture f;
      f.in.clock_fresh = false;
      check(!f.mission.tick(f.in).stream && f.mission.state() == State::WAIT, "missing clock must prevent start");
      f.in.clock_fresh = true; f.in.preflight_ok = false;
      check(!f.mission.tick(f.in).arm && f.mission.state() == State::PRIME,
        "preflight flag is recorded but must not block SITL episode start");
    }
    {
      Fixture f;
      f.takeoff();
      check(f.mission.state() == State::STEP, "single-axis needs response phase");
      const auto target = f.advance(0.05).target;
      check(target.x == 12 && target.y == -4 && std::abs(target.z + 2.8) < 1e-9, "only north target may change");
      f.in.position = target;
      f.advance(0.1); f.advance(1.01);
      check(f.mission.state() == State::HOLD, "stable step must start hold");
      auto out = f.advance(5.01);
      check(out.land && !out.stream, "landing handoff must stop BOTH target/heartbeat publication immediately");
      f.finish_landing();
      check(f.mission.succeeded(), "complete single-axis flight must succeed");
      check(!f.advance(1).arm, "IDLE must never auto restart");
    }
    {
      Config c; c.axis = "east"; c.displacement = -2;
      Fixture f(c); f.takeoff();
      const auto out = f.advance(0.05);
      check(out.target.x == 10 && out.target.y == -6, "east axis and negative displacement must work");
    }
    {
      Config c; c.scenario = "hover"; c.hold = 30;
      Fixture f(c); f.takeoff();
      check(f.mission.state() == State::HOLD, "hover should skip STEP");
      f.advance(10, 80);  // Slow simulation: 80 wall seconds are not 30 simulation seconds.
      check(f.mission.state() == State::HOLD, "hover duration must use simulation time");
      f.in.position.x += 0.5;  // Drift must remain in the measurement, not reset the timer.
      f.advance(19.9, 1);
      check(f.mission.state() == State::HOLD, "must hold full 30 sim seconds");
      const auto out = f.advance(0.11, 1);
      check(out.land && !out.stream, "30s completes despite measured drift");
      f.finish_landing();
    }
    {
      Fixture f; f.takeoff();
      f.in.offboard = false;
      const auto out = f.mission.tick(f.in, true);
      check(f.mission.state() == State::RELEASED && !out.stream && !out.land && !out.arm,
        "pilot takeover must outrank abort and cannot be reclaimed");
    }
    {
      Fixture f; f.takeoff();
      f.in.sim = 0;
      const auto out = f.mission.tick(f.in);
      check(f.mission.state() == State::LANDING && !out.stream && f.mission.failed(), "clock reset must abort");
    }
    {
      Fixture f; f.takeoff();
      ++f.in.xy_reset;
      const auto reset_out = f.mission.tick(f.in);
      check(reset_out.stream && f.mission.state() == State::TAKEOFF,
        "estimator frame reset must be recorded without aborting the episode");
    }
    {
      Fixture f; f.takeoff();
      f.in.position_fresh = false;
      check(!f.mission.tick(f.in).stream && f.mission.failed(), "stale state must abort");
    }
    {
      Fixture f;
      f.mission.tick(f.in); f.advance(1.3);
      const auto timeout = f.advance(46);
      check(f.mission.state() == State::LANDING && f.mission.failed(), "denied arming must time out");
      check(!timeout.arm && !timeout.mode, "takeoff timeout must cancel queued arm/mode retries");
      check(f.mission.tick(f.in).disarm == false, "cannot treat cached ground flag as new landing feedback");
      ++f.in.land_sequence;
      f.advance(2.1);
      check(f.mission.state() == State::IDLE && !f.mission.succeeded(), "failed liftoff must not report success");
    }
    {
      Fixture f; f.takeoff(); f.mission.tick(f.in, true);
      f.in.offboard = false; f.in.auto_land = true;
      const auto out = f.advance(61);
      check(!out.stream && !out.disarm && f.mission.state() == State::LANDING,
        "landing timeout must never force disarm or falsely enter IDLE");
    }
    {
      Fixture f; f.takeoff(); f.mission.tick(f.in, true);
      f.in.offboard = false; f.in.landed = true; ++f.in.land_sequence;
      const auto out = f.advance(0.1);
      check(f.mission.state() == State::RELEASED && !out.disarm,
        "pilot takeover during landing must stop commands, including disarm");
    }
    {
      Config c; c.altitude = std::numeric_limits<double>::quiet_NaN();
      bool rejected = false;
      try { Mission m(c); } catch (const std::invalid_argument &) { rejected = true; }
      check(rejected, "NaN config must be rejected before flight");
    }
    std::cout << "PASS: start guards, both axes, full flight, sim-time hold, landing handoff, takeover, clock/frame resets, stale data and timeouts\n";
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
