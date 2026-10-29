#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

class AbMission final : public rclcpp::Node
{
public:
  AbMission()
  : Node("px4_ab_mission"),
    _state(State::WAIT_FOR_POSITION),
    _last_state_log(std::chrono::steady_clock::now())
  {
    // All offsets are expressed in PX4 local NED coordinates:
    // x = north, y = east, z = down.  z is negative above the home point.
    _a_x = declare_parameter<double>("a_x", 2.0);
    _a_y = declare_parameter<double>("a_y", 0.0);
    _b_x = declare_parameter<double>("b_x", 2.0);
    _b_y = declare_parameter<double>("b_y", 2.0);
    _altitude = declare_parameter<double>("altitude", 3.0);
    _task_seconds = declare_parameter<double>("task_seconds", 5.0);
    _acceptance_radius = declare_parameter<double>("acceptance_radius", 0.30);
    _max_stage_seconds = declare_parameter<double>("max_stage_seconds", 30.0);
    const char *user_home = std::getenv("HOME");
    _log_root = declare_parameter<std::string>(
      "log_root", std::string(user_home ? user_home : ".") + "/px4_experiments");
    if (_log_root.empty()) {
      throw std::invalid_argument("log_root must not be empty");
    }
    _state_started = _mission_started;
    open_logs();

    const auto sensor_qos = rclcpp::SensorDataQoS();

    _position_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", sensor_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        _position = *msg;
        _have_position = true;
      });

    _status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", sensor_qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        _status = *msg;
        _have_status = true;
      });

    _land_sub = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", sensor_qos,
      [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        _landed = msg->landed;
        _have_land_state = true;
      });
    _clock_sub = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", sensor_qos,
      [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
        _sim_time = static_cast<double>(msg->clock.sec) +
          static_cast<double>(msg->clock.nanosec) * 1e-9;
        _have_clock = true;
      });

    _offboard_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    _setpoint_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    _command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    // PX4 requires a continuous Offboard heartbeat and setpoint stream.
    _timer = create_wall_timer(50ms, [this]() { run_cycle(); });

    RCLCPP_INFO(
      get_logger(),
      "A/B mission build=2026-10-22-r9; ROS recording enabled; waiting for safe start conditions. "
      "Defaults: A=(%.1f, %.1f), B=(%.1f, %.1f), altitude=%.1f m, task=%.1f s",
      _a_x, _a_y, _b_x, _b_y, _altitude, _task_seconds);
    RCLCPP_INFO(get_logger(), "ROS recording root: %s", _run_path.string().c_str());
    record_event("state -> WAIT_FOR_POSITION");
  }

private:
  struct Point {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  enum class State {
    WAIT_FOR_POSITION,
    TAKEOFF,
    GO_TO_A,
    DO_TASK,
    GO_TO_B,
    LANDING,
    IDLE
  };

  static const char *state_name(State state)
  {
    switch (state) {
      case State::WAIT_FOR_POSITION: return "WAIT_FOR_POSITION";
      case State::TAKEOFF: return "TAKEOFF";
      case State::GO_TO_A: return "GO_TO_A";
      case State::DO_TASK: return "DO_TASK";
      case State::GO_TO_B: return "GO_TO_B";
      case State::LANDING: return "LANDING";
      case State::IDLE: return "IDLE";
      default: return "UNKNOWN";
    }
  }

  uint64_t timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  double elapsed_since_start() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - _mission_started).count();
  }

  void run_cycle()
  {
    if (_state == State::WAIT_FOR_POSITION) {
      if (!_have_position || !_position.xy_valid || !_position.z_valid ||
          !_have_status || !_have_land_state ||
          _status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED ||
          !_landed) {
        log_waiting_for_position();
        update_quality_flags();
        return;
      }

      _home = {_position.x, _position.y, _position.z};
      _takeoff = {_home.x, _home.y, _home.z - _altitude};
      _point_a = {_home.x + _a_x, _home.y + _a_y, _takeoff.z};
      _point_b = {_home.x + _b_x, _home.y + _b_y, _takeoff.z};
      _initial_xy_reset = _position.xy_reset_counter;
      _initial_z_reset = _position.z_reset_counter;
      _initial_heading_reset = _position.heading_reset_counter;
      _have_initial_reset = true;
      _setpoint_counter = 0;
      _offboard_command_sent = false;
      record_event("home captured; takeoff target z=" + format_number(_takeoff.z));
      transition_to(State::TAKEOFF);
    }

    if (_state == State::IDLE) {
      if (!_result_written) {
        write_result();
      }
      return;
    }

    observe_airborne_state();

    // NAV_LAND transfers trajectory ownership to PX4. Both external Offboard
    // targets and FlightModeManager publish the same uORB trajectory_setpoint;
    // continuing to publish the B-altitude target can overwrite descent targets
    // even after vehicle_status reports AUTO_LAND. Only monitor landing here.
    if (_state == State::LANDING) {
      handle_landing();
      update_quality_flags();
      write_sample();
      flush_logs_if_needed();
      return;
    }

    // Stream targets only for the Offboard-controlled part of the mission.
    publish_offboard_control_mode();

    Point target = current_target();
    publish_trajectory_setpoint(target);

    switch (_state) {
      case State::TAKEOFF:
        handle_takeoff();
        break;
      case State::GO_TO_A:
        handle_go_to_a();
        break;
      case State::DO_TASK:
        handle_do_task();
        break;
      case State::GO_TO_B:
        handle_go_to_b();
        break;
      case State::LANDING:
        handle_landing();
        break;
      case State::WAIT_FOR_POSITION:
      case State::IDLE:
        break;
    }
    update_quality_flags();
    write_sample();
    flush_logs_if_needed();
  }

  Point current_target() const
  {
    switch (_state) {
      case State::GO_TO_A:
      case State::DO_TASK:
        return _point_a;
      case State::GO_TO_B:
        return _point_b;
      case State::TAKEOFF:
      default:
        return _takeoff;
    }
  }

  void handle_takeoff()
  {
    // Give PX4 at least one second of valid setpoints before requesting Offboard.
    if (_setpoint_counter < 20) {
      ++_setpoint_counter;
      return;
    }

    if (!_offboard_command_sent) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      _offboard_command_sent = true;
      record_event("requested Offboard mode and arm");
      RCLCPP_INFO(get_logger(), "requested Offboard mode and arm");
    }

    if (target_reached(_takeoff) && offboard_and_armed()) {
      transition_to(State::GO_TO_A);
    } else if (stage_timed_out()) {
      fail_and_land("takeoff timed out");
    } else if (elapsed_in_state_seconds() > 5.0) {
      log_takeoff_waiting();
    }
  }

  void handle_go_to_a()
  {
    if (target_reached(_point_a)) {
      transition_to(State::DO_TASK);
    } else if (stage_timed_out()) {
      fail_and_land("A point timed out");
    }
  }

  void handle_do_task()
  {
    const double elapsed = elapsed_in_state_seconds();
    if (elapsed >= _task_seconds) {
      RCLCPP_INFO(get_logger(), "DO_TASK complete after %.1f seconds", elapsed);
      record_event("DO_TASK complete after " + format_number(elapsed) + " seconds");
      _task_completed = true;
      transition_to(State::GO_TO_B);
    } else if (stage_timed_out()) {
      fail_and_land("DO_TASK timed out");
    }
  }

  void handle_go_to_b()
  {
    if (target_reached(_point_b)) {
      RCLCPP_INFO(get_logger(), "B reached; requesting automatic landing");
      _seen_go_to_b = true;
      record_event("B reached; requesting automatic landing");
      transition_to(State::LANDING);
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      _land_command_sent = true;
    } else if (stage_timed_out()) {
      fail_and_land("B point timed out");
    }
  }

  void handle_landing()
  {
    if (!_land_command_sent) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      _land_command_sent = true;
      RCLCPP_INFO(get_logger(), "landing command sent");
    }

    // PX4 is the authority for landed detection. A local-position fallback
    // can report "near the ground" before PX4 has actually detected contact,
    // which makes PX4 reject the disarm command with "not landed".
    const bool topic_confirms_landing = _seen_airborne && _have_land_state && _landed;
    const bool failed_before_liftoff =
      _takeoff_failed && !_seen_airborne && _have_land_state && _landed;
    const bool status_confirms_disarm_after_landing =
      _seen_airborne && _have_status &&
      _status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
      elapsed_in_state_seconds() > 2.0;

    if (topic_confirms_landing || failed_before_liftoff || status_confirms_disarm_after_landing) {
      if (!_landing_confirmed) {
        _landing_confirmed = true;
        if (failed_before_liftoff) {
          RCLCPP_ERROR(
            get_logger(),
            "takeoff failed before liftoff; PX4 reports the vehicle is still landed");
        } else if (topic_confirms_landing) {
          RCLCPP_INFO(get_logger(), "landing detected by VehicleLandDetected");
          record_event("landing detected by VehicleLandDetected");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "PX4 status confirms disarmed after landing; VehicleLandDetected was not received");
        }
      }

      const auto now = std::chrono::steady_clock::now();
      const bool retry_disarm =
        !_disarm_command_sent ||
        std::chrono::duration<double>(now - _last_disarm_command).count() >= 1.0;

      if (retry_disarm &&
          (!_have_status ||
           _status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
        _disarm_command_sent = true;
        _last_disarm_command = now;
        record_event("requested disarm; waiting for PX4 confirmation");
        RCLCPP_INFO(get_logger(), "requested disarm; waiting for PX4 confirmation");
      }

      if (_have_status &&
          _status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
        transition_to(State::IDLE);
      } else if (elapsed_in_state_seconds() > 2.0) {
        log_waiting_for_disarm();
      }
    } else if (stage_timed_out()) {
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - _last_state_log).count() >= 2.0) {
        RCLCPP_ERROR(
          get_logger(),
          "landing timeout diagnostic: airborne=%s status_msg=%s arming_state=%u "
          "nav_state=%u land_msg=%s landed=%s; disarm withheld",
          _seen_airborne ? "yes" : "no", _have_status ? "yes" : "no",
          _have_status ? static_cast<unsigned>(_status.arming_state) : 0U,
          _have_status ? static_cast<unsigned>(_status.nav_state) : 0U,
          _have_land_state ? "yes" : "no", _landed ? "yes" : "no");
        _last_state_log = now;
      }
    }
  }

  bool target_reached(const Point &target)
  {
    if (!_have_position || !_position.xy_valid || !_position.z_valid) {
      _inside_target_since = {};
      return false;
    }

    const double dx = static_cast<double>(_position.x) - target.x;
    const double dy = static_cast<double>(_position.y) - target.y;
    const double dz = static_cast<double>(_position.z) - target.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance <= _acceptance_radius && speed() <= 0.30) {
      if (_inside_target_since.time_since_epoch().count() == 0) {
        _inside_target_since = std::chrono::steady_clock::now();
      }

      return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - _inside_target_since).count() >= 1.0;
    }

    _inside_target_since = {};
    return false;
  }

  double speed() const
  {
    return std::sqrt(
      static_cast<double>(_position.vx) * _position.vx +
      static_cast<double>(_position.vy) * _position.vy +
      static_cast<double>(_position.vz) * _position.vz);
  }

  void observe_airborne_state()
  {
    if (_seen_airborne || _state == State::WAIT_FOR_POSITION || !_have_position ||
        !_position.z_valid) {
      return;
    }

    // Local NED z becomes smaller when the vehicle climbs. Require a real
    // vertical displacement so a stale landed=true at the starting point
    // cannot be mistaken for a completed landing after a failed takeoff.
    if (_position.z < static_cast<float>(_home.z - 0.30)) {
      _seen_airborne = true;
      RCLCPP_INFO(get_logger(), "airborne state observed; landing confirmation is now armed");
    }
  }

  void log_takeoff_waiting()
  {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - _last_state_log).count() >= 2.0) {
      RCLCPP_WARN(
        get_logger(),
        "takeoff waiting: position=(%.2f, %.2f, %.2f) target_z=%.2f "
        "armed=%u nav_state=%u airborne=%s",
        static_cast<double>(_position.x), static_cast<double>(_position.y),
        static_cast<double>(_position.z), _takeoff.z,
        _have_status ? static_cast<unsigned>(_status.arming_state) : 0U,
        _have_status ? static_cast<unsigned>(_status.nav_state) : 0U,
        _seen_airborne ? "yes" : "no");
      _last_state_log = now;
    }
  }

  bool offboard_and_armed() const
  {
    return _have_status &&
           _status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
           _status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = timestamp_us();
    _offboard_mode_pub->publish(msg);
  }

  void publish_trajectory_setpoint(const Point &target)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {
      static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(target.z)};
    msg.yaw = 0.0F;
    msg.timestamp = timestamp_us();
    _setpoint_pub->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = timestamp_us();
    _command_pub->publish(msg);
  }

  void transition_to(State next)
  {
    if (_state == next) {
      return;
    }

    _state = next;
    _state_started = std::chrono::steady_clock::now();
    _inside_target_since = {};
    _land_command_sent = false;
    if (next != State::IDLE) {
      _landing_confirmed = false;
    }

    if (next == State::DO_TASK) {
      RCLCPP_INFO(get_logger(), "state -> DO_TASK: holding at A for %.1f seconds", _task_seconds);
      _seen_go_to_a = true;
    } else {
      RCLCPP_INFO(get_logger(), "state -> %s", state_name(next));
    }
    record_event(next == State::DO_TASK ?
      "state -> DO_TASK: holding at A for " + format_number(_task_seconds) + " seconds" :
      "state -> " + std::string(state_name(next)));

    if (next == State::LANDING) {
      RCLCPP_INFO(
        get_logger(),
        "landing handoff: stopping Offboard heartbeat and trajectory setpoints; "
        "PX4 owns automatic descent");
      record_event("landing handoff: stop Offboard heartbeat and trajectory targets");
    }

    if (next == State::IDLE && !_result_written) {
      write_result();
    }
  }

  double elapsed_in_state_seconds() const
  {
    return std::chrono::duration<double>(
             std::chrono::steady_clock::now() - _state_started).count();
  }

  bool stage_timed_out() const
  {
    return elapsed_in_state_seconds() > _max_stage_seconds;
  }

  void fail_and_land(const std::string &reason)
  {
    _takeoff_failed = (_state == State::TAKEOFF);
    record_event("ABORT: " + reason + "; switching to LANDING");
    RCLCPP_ERROR(get_logger(), "%s; switching to LANDING", reason.c_str());
    transition_to(State::LANDING);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    _land_command_sent = true;
  }

  static std::string format_number(double value)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
  }

  void record_event(const std::string &event)
  {
    if (!_events.is_open()) {
      return;
    }
    const auto state = std::string(state_name(_state));
    const auto elapsed = elapsed_since_start();
    _events << std::setprecision(15) << elapsed << ','
      << (_have_clock ? _sim_time : std::numeric_limits<double>::quiet_NaN()) << ','
      << state << ',' << std::quoted(event) << '\n';
    _agent_events << std::setprecision(15) << elapsed << ','
      << (_have_clock ? _sim_time : std::numeric_limits<double>::quiet_NaN()) << ','
      << state << ',' << std::quoted(event) << '\n';
  }

  void update_quality_flags()
  {
    if (_have_status && !_status.pre_flight_checks_pass) {
      _preflight_issue_seen = true;
    }
    if (_have_initial_reset &&
        (_position.xy_reset_counter != _initial_xy_reset ||
         _position.z_reset_counter != _initial_z_reset ||
         _position.heading_reset_counter != _initial_heading_reset)) {
      _estimator_reset_seen = true;
    }
  }

  Point sample_target() const
  {
    if (_state == State::TAKEOFF) {
      return _takeoff;
    }
    if (_state == State::GO_TO_A || _state == State::DO_TASK) {
      return _point_a;
    }
    if (_state == State::GO_TO_B) {
      return _point_b;
    }
    return {};
  }

  void write_sample()
  {
    if (_result_written || !_samples.is_open()) {
      return;
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const bool stream = _state != State::WAIT_FOR_POSITION && _state != State::LANDING &&
      _state != State::IDLE;
    const Point target = sample_target();
    const auto value = [nan](bool available, double data) { return available ? data : nan; };
    const double position_x = value(_have_position, _position.x);
    const double position_y = value(_have_position, _position.y);
    const double position_z = value(_have_position, _position.z);
    const double velocity_x = value(_have_position, _position.vx);
    const double velocity_y = value(_have_position, _position.vy);
    const double velocity_z = value(_have_position, _position.vz);
    const double current_speed = _have_position ? speed() : nan;
    _samples << std::setprecision(15) << elapsed_since_start() << ','
      << (_have_clock ? _sim_time : nan) << ','
      << timestamp_us() << ',' << (_have_position ? _position.timestamp : 0) << ','
      << (_have_position ? _position.timestamp_sample : 0) << ','
      << (_have_status ? _status.timestamp : 0) << ','
      << state_name(_state) << ',' << stream << ','
      << (stream ? target.x : nan) << ',' << (stream ? target.y : nan) << ','
      << (stream ? target.z : nan) << ',' << position_x << ',' << position_y << ','
      << position_z << ',' << velocity_x << ',' << velocity_y << ',' << velocity_z << ','
      << current_speed << ',' << value(_have_position, _position.heading) << ','
      << (_have_position && _position.xy_valid) << ','
      << (_have_position && _position.z_valid) << ','
      << (_have_position && _position.v_xy_valid) << ','
      << (_have_position && _position.v_z_valid) << ','
      << (_have_status ? static_cast<unsigned>(_status.arming_state) : 0U) << ','
      << (_have_status ? static_cast<unsigned>(_status.nav_state) : 0U) << ','
      << (_have_status && _status.failsafe) << ',' << (_have_land_state && _landed) << ','
      << (_have_status && _status.pre_flight_checks_pass) << ','
      << (_have_position ? static_cast<unsigned>(_position.xy_reset_counter) : 0U) << ','
      << (_have_position ? static_cast<unsigned>(_position.z_reset_counter) : 0U) << ','
      << (_have_position ? static_cast<unsigned>(_position.heading_reset_counter) : 0U) << '\n';
    _agent_samples << std::setprecision(15) << elapsed_since_start() << ','
      << (_have_clock ? _sim_time : nan) << ','
      << state_name(_state) << ',' << (stream ? target.x : nan) << ','
      << (stream ? target.y : nan) << ',' << (stream ? target.z : nan) << ','
      << position_x << ',' << position_y << ',' << position_z << ',' << velocity_x << ','
      << velocity_y << ',' << velocity_z << ',' << current_speed << ','
      << (_have_status ? static_cast<unsigned>(_status.arming_state) : 0U) << ','
      << (_have_status ? static_cast<unsigned>(_status.nav_state) : 0U) << ','
      << (_have_status && _status.failsafe) << ',' << (_have_land_state && _landed) << ','
      << (_have_position ? static_cast<unsigned>(_position.xy_reset_counter) : 0U) << ','
      << (_have_position ? static_cast<unsigned>(_position.z_reset_counter) : 0U) << ','
      << (_have_position ? static_cast<unsigned>(_position.heading_reset_counter) : 0U) << '\n';
  }

  void flush_logs_if_needed()
  {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - _last_flush).count() >= 1.0) {
      _samples.flush();
      _events.flush();
      _agent_samples.flush();
      _agent_events.flush();
      _last_flush = now;
    }
    if ((!_samples.good() || !_events.good() || !_agent_samples.good() || !_agent_events.good()) &&
        !_storage_failure_reported) {
      _storage_failure_reported = true;
      RCLCPP_ERROR(get_logger(), "recording failed; this run cannot be used for tuning");
    }
  }

  void write_result()
  {
    const bool success = _seen_airborne && _seen_go_to_a && _task_completed && _seen_go_to_b &&
      _landing_confirmed && _have_status &&
      _status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    const bool recording_ok = !_storage_failure_reported && _samples.good() && _events.good();
    const auto write_one = [&](const std::filesystem::path &path) {
      std::ofstream result(path / "result.txt");
      result << "build=2026-10-22-r9\n"
        << "scenario=ab_mission\nstate=IDLE\n"
        << "flight_success=" << (success ? "true" : "false") << '\n'
        << "recording_ok=" << (recording_ok ? "true" : "false") << '\n'
        << "preflight_issue_seen=" << (_preflight_issue_seen ? "true" : "false") << '\n'
        << "estimator_reset_seen=" << (_estimator_reset_seen ? "true" : "false") << '\n'
        << "data_quality=" << ((_preflight_issue_seen || _estimator_reset_seen || !recording_ok) ?
          "review_required" : "normal") << '\n'
        << "quality_review=pending\n";
      return result.good();
    };
    if (!write_one(_all_path) || !write_one(_agent_path)) {
      _storage_failure_reported = true;
    }
    _result_written = true;
    RCLCPP_INFO(get_logger(), "finished; state=IDLE success=%s; saved %s; node stays inactive until Ctrl+C",
      success ? "yes" : "no", _all_path.string().c_str());
    std::cout << "\n任务已完成：已回到 IDLE，飞机已落地并上锁。\n"
      << "现在请在本终端按 Ctrl+C 停止 ROS 任务节点。\n"
      << "随后切换到 PX4 SITL 的 pxh> 终端执行：\n"
      << "  logger off\n"
      << "  logger status\n"
      << std::flush;
  }

  void open_logs()
  {
    const auto id = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto run_name = std::string("ab_mission_") + std::to_string(id);
    _run_path = std::filesystem::path(_log_root) / run_name;
    _all_path = _run_path / ("allfile_" + run_name);
    _agent_path = _run_path / "agentdata";
    std::filesystem::create_directories(_all_path);
    std::filesystem::create_directories(_agent_path);
    std::ofstream manifest(_run_path / "run_manifest.txt");
    manifest << "scenario=ab_mission\nallfile=" << _all_path.string()
      << "\nagentdata=" << _agent_path.string() << '\n';
    _samples.open(_all_path / "samples.csv");
    _events.open(_all_path / "events.csv");
    _agent_samples.open(_agent_path / "samples.csv");
    _agent_events.open(_agent_path / "events.csv");
    if (!_samples || !_events || !_agent_samples || !_agent_events) {
      throw std::runtime_error("cannot open A/B experiment logs");
    }
    _samples << "wall_elapsed_s,sim_time_s,ros_time_us,position_timestamp_us,position_sample_us,status_timestamp_us,state,target_streaming,target_x,target_y,target_z,x,y,z,vx,vy,vz,speed,heading,xy_valid,z_valid,v_xy_valid,v_z_valid,arming_state,nav_state,failsafe,landed,preflight_ok,xy_reset,z_reset,heading_reset\n";
    _agent_samples << "wall_elapsed_s,sim_time_s,state,target_x,target_y,target_z,x,y,z,vx,vy,vz,speed,arming_state,nav_state,failsafe,landed,xy_reset,z_reset,heading_reset\n";
    _events << "wall_elapsed_s,sim_time_s,state,event\n";
    _agent_events << "wall_elapsed_s,sim_time_s,state,event\n";
    std::ofstream metadata(_all_path / "metadata.txt");
    metadata << "build=2026-10-22-r9\nscenario=ab_mission\nframe=PX4 local NED\n"
      << "position_unit=m\nvelocity_unit=m/s\nrecording=node receipt samples at 20Hz\n"
      << "sim_time=/clock; mission_timer=steady wall clock\nulog=preserved separately by PX4 logger\n"
      << "quality_review=pending\n";
    std::ofstream agent_metadata(_agent_path / "metadata.txt");
    agent_metadata << "build=2026-10-22-r9\nscenario=ab_mission\nframe=PX4 local NED\n"
      << "data_role=offline Agent analysis input\nparameter_values=external PX4 snapshot required\n";
    const auto write_parameters = [this](const std::filesystem::path &path) {
      std::ofstream params(path / "parameters.yaml");
      params << "px4_ab_mission:\n  ros__parameters:\n"
        << "    a_x: " << _a_x << "\n    a_y: " << _a_y
        << "\n    b_x: " << _b_x << "\n    b_y: " << _b_y
        << "\n    altitude: " << _altitude << "\n    task_seconds: " << _task_seconds
        << "\n    acceptance_radius: " << _acceptance_radius
        << "\n    max_stage_seconds: " << _max_stage_seconds << "\n";
      return params.good();
    };
    if (!metadata || !agent_metadata || !write_parameters(_all_path) || !write_parameters(_agent_path)) {
      throw std::runtime_error("cannot save A/B experiment metadata/configuration");
    }
    const auto write_allowlist = [](const std::filesystem::path &path) {
      std::ofstream output(path / "px4_selected_params.yaml");
      output << "version: px4-subject-allowlist-v0.1\n"
        << "source: external PX4 pxh snapshot required\n"
        << "value_status: this ROS node does not read PX4 parameters\n"
        << "parameters:\n"
        << "  - name: MPC_XY_P\n    role: regression_context\n"
        << "  - name: MPC_XY_VEL_P_ACC\n    role: regression_context\n"
        << "  - name: MPC_XY_VEL_I_ACC\n    role: regression_context\n"
        << "  - name: MPC_ACC_HOR_MAX\n    role: regression_context\n"
        << "  - name: MPC_XY_VEL_MAX\n    role: regression_context\n";
      return output.good();
    };
    if (!manifest || !write_allowlist(_all_path) || !write_allowlist(_agent_path)) {
      throw std::runtime_error("cannot save A/B run manifest or parameter allowlist");
    }
  }

  void log_waiting_for_position()
  {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - _last_state_log).count() >= 2.0) {
      RCLCPP_WARN(
        get_logger(),
        "waiting for start conditions: position_msg=%s xy_valid=%s z_valid=%s "
        "status_msg=%s arming_state=%u land_msg=%s landed=%s",
        _have_position ? "yes" : "no",
        (_have_position && _position.xy_valid) ? "yes" : "no",
        (_have_position && _position.z_valid) ? "yes" : "no",
        _have_status ? "yes" : "no",
        _have_status ? static_cast<unsigned>(_status.arming_state) : 0U,
        _have_land_state ? "yes" : "no",
        (_have_land_state && _landed) ? "yes" : "no");
      _last_state_log = now;
    }
  }

  void log_waiting_for_disarm()
  {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - _last_state_log).count() >= 2.0) {
      RCLCPP_WARN(
        get_logger(),
        "landing confirmed; waiting for PX4 disarm confirmation: "
        "arming_state=%u nav_state=%u landed=%s",
        _have_status ? static_cast<unsigned>(_status.arming_state) : 0U,
        _have_status ? static_cast<unsigned>(_status.nav_state) : 0U,
        (_have_land_state && _landed) ? "yes" : "no");
      _last_state_log = now;
    }
  }

  State _state;
  Point _home{};
  Point _takeoff{};
  Point _point_a{};
  Point _point_b{};

  double _a_x{2.0};
  double _a_y{0.0};
  double _b_x{2.0};
  double _b_y{2.0};
  double _altitude{3.0};
  double _task_seconds{5.0};
  double _acceptance_radius{0.30};
  double _max_stage_seconds{30.0};

  px4_msgs::msg::VehicleLocalPosition _position{};
  px4_msgs::msg::VehicleStatus _status{};
  double _sim_time{0.0};
  bool _have_position{false};
  bool _have_status{false};
  bool _have_land_state{false};
  bool _have_clock{false};
  bool _landed{false};
  bool _seen_airborne{false};
  bool _takeoff_failed{false};
  bool _landing_confirmed{false};
  bool _offboard_command_sent{false};
  bool _land_command_sent{false};
  bool _disarm_command_sent{false};
  unsigned _setpoint_counter{0};

  std::chrono::steady_clock::time_point _state_started{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point _mission_started{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point _inside_target_since{};
  std::chrono::steady_clock::time_point _last_state_log;
  std::chrono::steady_clock::time_point _last_disarm_command{};
  std::chrono::steady_clock::time_point _last_flush{std::chrono::steady_clock::now()};

  uint8_t _initial_xy_reset{0};
  uint8_t _initial_z_reset{0};
  uint8_t _initial_heading_reset{0};
  bool _have_initial_reset{false};
  bool _seen_go_to_a{false};
  bool _task_completed{false};
  bool _seen_go_to_b{false};
  bool _result_written{false};
  bool _storage_failure_reported{false};
  bool _preflight_issue_seen{false};
  bool _estimator_reset_seen{false};

  std::string _log_root;
  std::filesystem::path _run_path;
  std::filesystem::path _all_path;
  std::filesystem::path _agent_path;
  std::ofstream _samples;
  std::ofstream _events;
  std::ofstream _agent_samples;
  std::ofstream _agent_events;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_sub;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _clock_sub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _command_pub;
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AbMission>());
  rclcpp::shutdown();
  return 0;
}
