#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_control_experiment/mission_logic.hpp"

using namespace std::chrono_literals;
namespace logic = px4_experiment;
using Position = px4_msgs::msg::VehicleLocalPosition;
using Status = px4_msgs::msg::VehicleStatus;
using Command = px4_msgs::msg::VehicleCommand;

class ControlExperiment final : public rclcpp::Node
{
public:
  ControlExperiment() : Node("px4_control_experiment")
  {
    const bool sim_clock_for_ros = has_parameter("use_sim_time") ?
      get_parameter("use_sim_time").as_bool() : declare_parameter<bool>("use_sim_time", false);
    if (sim_clock_for_ros) {
      throw std::invalid_argument("leave use_sim_time=false; /clock is read separately for experiment timing");
    }
    _config.scenario = declare_parameter<std::string>("scenario", "single_axis");
    _config.axis = declare_parameter<std::string>("axis", "north");
    _config.displacement = declare_parameter<double>("displacement_m", 2.0);
    _config.altitude = declare_parameter<double>("altitude_m", 3.0);
    _config.initial_hold = declare_parameter<double>("initial_hold_s", 2.0);
    _config.hold = declare_parameter<double>("hold_s", 5.0);
    _config.radius = declare_parameter<double>("position_tolerance_m", 0.3);
    _config.speed_limit = declare_parameter<double>("speed_tolerance_mps", 0.3);
    _config.stable_duration = declare_parameter<double>("stable_duration_s", 1.0);
    _config.takeoff_timeout = declare_parameter<double>("takeoff_timeout_s", 45.0);
    _config.response_timeout = declare_parameter<double>("response_timeout_s", 30.0);
    _config.landing_timeout = declare_parameter<double>("landing_timeout_s", 60.0);
    _config.wall_timeout = declare_parameter<double>("wall_stage_timeout_s", 180.0);
    _message_timeout = declare_parameter<double>("message_timeout_s", 2.0);
    _position_topic = declare_parameter<std::string>("position_topic", "/fmu/out/vehicle_local_position_v1");
    _status_topic = declare_parameter<std::string>("status_topic", "/fmu/out/vehicle_status_v1");
    _land_topic = declare_parameter<std::string>("land_topic", "/fmu/out/vehicle_land_detected");
    _clock_topic = declare_parameter<std::string>("clock_topic", "/clock");
    const char *user_home = std::getenv("HOME");
    const auto default_root = std::string(user_home ? user_home : ".") + "/px4_experiments";
    _log_root = declare_parameter<std::string>("log_root", default_root);
    _target_system = declare_parameter<int>("target_system", 1);
    if (_target_system < 1 || _target_system > 255 ||
        !std::isfinite(_message_timeout) || _message_timeout <= 0 || _log_root.empty()) {
      throw std::invalid_argument("invalid target_system, message_timeout_s or log_root");
    }
    _mission = std::make_unique<logic::Mission>(_config);
    open_logs();

    const auto qos = rclcpp::SensorDataQoS();
    _position_sub = create_subscription<Position>(_position_topic, qos,
      [this](Position::SharedPtr msg) {
        _position = *msg; _position_wall = wall_time(); _have_position = true;
      });
    _status_sub = create_subscription<Status>(_status_topic, qos,
      [this](Status::SharedPtr msg) {
        _status = *msg; _status_wall = wall_time(); _have_status = true; ++_status_sequence;
      });
    _land_sub = create_subscription<px4_msgs::msg::VehicleLandDetected>(_land_topic, qos,
      [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        _landed = msg->landed; _land_timestamp = msg->timestamp;
        _land_wall = wall_time(); _have_land = true; ++_land_sequence;
      });
    _clock_sub = create_subscription<rosgraph_msgs::msg::Clock>(_clock_topic, qos,
      [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
        const double next_sim = static_cast<double>(msg->clock.sec) + msg->clock.nanosec * 1e-9;
        if (_have_clock && next_sim > _sim_time) { _clock_advanced = true; }
        if (!_have_clock || next_sim != _sim_time) { _clock_progress_wall = wall_time(); }
        _sim_time = next_sim;
        _clock_wall = wall_time(); _have_clock = true;
      });
    _mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 1);
    _target_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 1);
    _command_pub = create_publisher<Command>("/fmu/in/vehicle_command", 10);
    _abort_service = create_service<std_srvs::srv::Trigger>("~/abort",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = !_mission->complete();
        response->message = response->success ? "abort queued; monitor PX4 landing/disarm" : "already inactive";
        if (response->success) { _abort_requested = true; }
      });
    _fixed_parameters = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "experiment configuration is fixed; change YAML and restart after landing";
        return result;
      });
    _timer = create_wall_timer(50ms, [this]() { cycle(); });
    RCLCPP_INFO(get_logger(), "control_experiment build=2026-10-14-v3; scenario=%s axis=%s offset=%.2fm altitude=%.2fm hold=%.1f SIM seconds",
      _config.scenario.c_str(), _config.axis.c_str(), _config.displacement, _config.altitude, _config.hold);
    RCLCPP_INFO(get_logger(), "recording allfile: %s; agentdata: %s; waiting for fresh PX4 state AND Gazebo %s",
      _all_path.string().c_str(), _agent_path.string().c_str(), _clock_topic.c_str());
  }

private:
  double wall_time() const
  { return std::chrono::duration<double>(std::chrono::steady_clock::now() - _started).count(); }
  uint64_t timestamp_us() const
  { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000); }
  bool fresh(bool have, double received, double now) const
  { return have && now - received <= _message_timeout; }

  logic::Input input() const
  {
    logic::Input in;
    in.wall = wall_time(); in.sim = _sim_time;
    in.clock_fresh = _clock_advanced && fresh(_have_clock, _clock_wall, in.wall) &&
      in.wall - _clock_progress_wall <= _message_timeout;
    in.position_fresh = fresh(_have_position, _position_wall, in.wall);
    in.status_fresh = fresh(_have_status, _status_wall, in.wall);
    in.land_fresh = fresh(_have_land, _land_wall, in.wall);
    in.position = {_position.x, _position.y, _position.z};
    in.speed = std::sqrt(static_cast<double>(_position.vx) * _position.vx +
      static_cast<double>(_position.vy) * _position.vy + static_cast<double>(_position.vz) * _position.vz);
    in.position_valid = _have_position && _position.xy_valid && _position.z_valid &&
      std::isfinite(_position.x) && std::isfinite(_position.y) && std::isfinite(_position.z);
    in.velocity_valid = _have_position && _position.v_xy_valid && _position.v_z_valid && std::isfinite(in.speed);
    in.heading = _position.heading;
    in.preflight_ok = _status.pre_flight_checks_pass;
    in.armed = _have_status && _status.arming_state == Status::ARMING_STATE_ARMED;
    in.disarmed = _have_status && _status.arming_state == Status::ARMING_STATE_DISARMED;
    in.offboard = _status.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
    in.auto_land = _status.nav_state == Status::NAVIGATION_STATE_AUTO_LAND;
    in.failsafe = _status.failsafe;
    in.landed = _landed; in.land_sequence = _land_sequence;
    in.status_sequence = _status_sequence;
    in.xy_reset = _position.xy_reset_counter; in.z_reset = _position.z_reset_counter;
    in.heading_reset = _position.heading_reset_counter;
    return in;
  }

  void cycle()
  {
    const auto in = input();
    const auto actions = _mission->tick(in, _abort_requested);
    _abort_requested = false;
    // Check state BEFORE publishing. Land and released states never publish targets.
    if (actions.stream) { publish_target(actions); }
    if (actions.mode) { command(Command::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F); }
    if (actions.arm) { command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F); }
    if (actions.land) { command(Command::VEHICLE_CMD_NAV_LAND); }
    if (actions.disarm) { command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F); }
    for (const auto &event : actions.events) {
      if (event.rfind("DATA FLAG: estimator reset", 0) == 0) {
        _estimator_reset_seen = true;
      }
      _events << std::setprecision(15) << in.wall << ',' << in.sim << ','
        << logic::name(_mission->state()) << ',' << std::quoted(event) << '\n';
      _agent_events << std::setprecision(15) << in.wall << ',' << in.sim << ','
        << logic::name(_mission->state()) << ',' << std::quoted(event) << '\n';
      if (event.rfind("ABORT", 0) == 0 || event.rfind("RELEASE", 0) == 0 ||
          event.rfind("LANDING TIMEOUT", 0) == 0) {
        RCLCPP_ERROR(get_logger(), "%s", event.c_str());
      } else { RCLCPP_INFO(get_logger(), "%s", event.c_str()); }
    }
    write_sample(in, actions);
    if (_have_status && !in.preflight_ok) {
      _preflight_issue_seen = true;
    }
    if (in.wall - _last_log >= 2.0 && !_mission->complete()) {
      if (_mission->state() == logic::State::WAIT) {
        RCLCPP_WARN(get_logger(), "waiting: clock=%s position=%s valid=%s velocity=%s status=%s preflight=%s disarmed=%s land=%s landed=%s",
          yes(in.clock_fresh), yes(in.position_fresh), yes(in.position_valid), yes(in.velocity_valid),
          yes(in.status_fresh), yes(in.preflight_ok), yes(in.disarmed), yes(in.land_fresh), yes(in.landed));
      } else {
        RCLCPP_INFO(get_logger(), "%s sim=%.2f pos=(%.2f,%.2f,%.2f) speed=%.2f armed=%u nav=%u landed=%s hold=%.2f/%.2fs",
          logic::name(_mission->state()), in.sim, in.position.x, in.position.y, in.position.z,
          in.speed, static_cast<unsigned>(_status.arming_state), static_cast<unsigned>(_status.nav_state),
          yes(in.landed), _mission->hold_elapsed(in), _config.hold);
      }
      _last_log = in.wall;
    }
    if (in.wall - _last_flush >= 1.0 || !actions.events.empty()) {
      _samples.flush(); _events.flush(); _agent_samples.flush(); _agent_events.flush();
      _last_flush = in.wall;
    }
    if ((!_samples.good() || !_events.good() || !_agent_samples.good() || !_agent_events.good()) &&
        !_storage_failure_reported) {
      _storage_failure_reported = true;
      _abort_requested = true;
      RCLCPP_ERROR(get_logger(), "recording failed; abort queued (do not use this run for tuning)");
    }
    if (_mission->complete() && !_result_written) {
      const auto write_result = [this](const std::filesystem::path &path) {
        std::ofstream result(path / "result.txt");
        result << "build=2026-10-14-v3\nscenario=" << _config.scenario
          << "\nstate=" << logic::name(_mission->state())
          << "\nflight_success=" << (_mission->succeeded() ? "true" : "false")
          << "\nrecording_ok=" << (!_storage_failure_reported ? "true" : "false")
          << "\npreflight_issue_seen=" << (_preflight_issue_seen ? "true" : "false")
          << "\nestimator_reset_seen=" << (_estimator_reset_seen ? "true" : "false")
          << "\ndata_quality=" << ((_preflight_issue_seen || _estimator_reset_seen || _storage_failure_reported) ? "review_required" : "normal")
          << "\nquality_review=pending\n";
        return result.good();
      };
      if (!write_result(_all_path) || !write_result(_agent_path)) {
        _storage_failure_reported = true;
      }
      _result_written = true;
      RCLCPP_INFO(get_logger(), "finished; state=%s success=%s; saved %s; node stays inactive until Ctrl+C",
        logic::name(_mission->state()), yes(_mission->succeeded()), _all_path.string().c_str());
    }
  }

  static const char *yes(bool value) { return value ? "yes" : "no"; }

  void publish_target(const logic::Actions &actions)
  {
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = timestamp_us(); mode.position = true;
    _mode_pub->publish(mode);
    px4_msgs::msg::TrajectorySetpoint target{};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    target.timestamp = timestamp_us();
    target.position = {static_cast<float>(actions.target.x), static_cast<float>(actions.target.y),
      static_cast<float>(actions.target.z)};
    target.velocity = {nan, nan, nan};
    target.acceleration = {nan, nan, nan};
    target.jerk = {nan, nan, nan};
    target.yaw = static_cast<float>(actions.yaw); target.yawspeed = nan;
    _target_pub->publish(target);
  }

  void command(uint16_t id, float p1 = 0.0F, float p2 = 0.0F)
  {
    Command msg{};
    msg.timestamp = timestamp_us(); msg.command = id; msg.param1 = p1; msg.param2 = p2;
    msg.target_system = static_cast<uint8_t>(_target_system); msg.target_component = 1;
    msg.source_system = 1; msg.source_component = 1; msg.from_external = true;
    _command_pub->publish(msg);
  }

  void open_logs()
  {
    const auto id = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto run_name = _config.scenario + "_" + std::to_string(id);
    _run_path = std::filesystem::path(_log_root) / run_name;
    _all_path = _run_path / ("allfile_" + run_name);
    _agent_path = _run_path / "agentdata";
    if (!std::filesystem::create_directories(_all_path) ||
        !std::filesystem::create_directories(_agent_path)) {
      throw std::runtime_error("run directory already exists");
    }
    std::ofstream manifest(_run_path / "run_manifest.txt");
    manifest << "scenario=" << _config.scenario << '\n'
      << "allfile=" << _all_path.string() << '\n'
      << "agentdata=" << _agent_path.string() << '\n';
    if (!manifest) { throw std::runtime_error("cannot save run manifest"); }
    _samples.open(_all_path / "samples.csv");
    _events.open(_all_path / "events.csv");
    _agent_samples.open(_agent_path / "samples.csv");
    _agent_events.open(_agent_path / "events.csv");
    if (!_samples || !_events || !_agent_samples || !_agent_events) {
      throw std::runtime_error("cannot open experiment logs");
    }
    _samples << "wall_elapsed_s,sim_time_s,ros_time_us,position_timestamp_us,position_sample_us,status_timestamp_us,land_timestamp_us,"
      "state,target_streaming,target_x,target_y,target_z,target_yaw,x,y,z,vx,vy,vz,speed,heading,"
      "xy_valid,z_valid,v_xy_valid,v_z_valid,arming_state,nav_state,failsafe,landed,preflight_ok,"
      "clock_fresh,position_fresh,status_fresh,land_fresh,clock_age_s,position_age_s,status_age_s,land_age_s,"
      "xy_reset,z_reset,heading_reset,hold_elapsed_s\n";
    _agent_samples << "wall_elapsed_s,sim_time_s,state,target_x,target_y,target_z,x,y,z,vx,vy,vz,speed,"
      "arming_state,nav_state,failsafe,landed,preflight_ok,xy_reset,z_reset,heading_reset,hold_elapsed_s\n";
    _events << "wall_elapsed_s,sim_time_s,state,event\n";
    _agent_events << "wall_elapsed_s,sim_time_s,state,event\n";
    std::ofstream metadata(_all_path / "metadata.txt");
    metadata << "build=2026-10-14-v3\nframe=PX4 local NED\nposition_unit=m\nvelocity_unit=m/s\n"
      "timer=Gazebo /clock; independent of ROS system time\n"
      "wall_time=steady elapsed time; safety watchdog only\n"
      "PX4 DDS timestamp domain must be checked; do not assume identical to Gazebo clock\n"
      "recording=node receipt samples at wall 20Hz, not raw IMU or full control outputs\n"
      "quality_review=pending; preserve PX4 ULog separately\n"
      "agentdata=contains selected fields and parameter allowlist\n";
    std::ofstream agent_metadata(_agent_path / "metadata.txt");
    agent_metadata << "build=2026-10-14-v3\nframe=PX4 local NED\nposition_unit=m\nvelocity_unit=m/s\n"
      "data_role=offline Agent analysis input\n"
      "parameter_values=external PX4 snapshot required; this node does not read PX4 parameters\n";
    std::ofstream params(_all_path / "parameters.yaml");
    params << "px4_control_experiment:\n  ros__parameters:\n";
    for (const auto &parameter_name : list_parameters({}, 1).names) {
      const auto parameter = get_parameter(parameter_name);
      params << "    " << parameter_name << ": ";
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        params << std::quoted(parameter.as_string());
      } else { params << parameter.value_to_string(); }
      params << '\n';
    }
    std::ofstream agent_params(_agent_path / "parameters.yaml");
    agent_params << "px4_control_experiment:\n  ros__parameters:\n"
      << "    scenario: " << _config.scenario << '\n'
      << "    axis: " << _config.axis << '\n'
      << "    displacement_m: " << _config.displacement << '\n'
      << "    altitude_m: " << _config.altitude << '\n'
      << "    initial_hold_s: " << _config.initial_hold << '\n'
      << "    hold_s: " << _config.hold << '\n'
      << "    position_tolerance_m: " << _config.radius << '\n'
      << "    speed_tolerance_mps: " << _config.speed_limit << '\n'
      << "    stable_duration_s: " << _config.stable_duration << '\n';
    write_px4_allowlist(_agent_path / "px4_selected_params.yaml");
    write_px4_allowlist(_all_path / "px4_selected_params.yaml");
    if (!metadata || !agent_metadata || !params || !agent_params) {
      throw std::runtime_error("cannot save experiment metadata/configuration");
    }
  }

  void write_px4_allowlist(const std::filesystem::path &path) const
  {
    std::ofstream output(path);
    output << "version: px4-subject-allowlist-v0.1\n"
      << "source: external PX4 pxh snapshot required\n"
      << "value_status: this ROS node does not read PX4 parameters\n"
      << "parameters:\n";
    if (_config.scenario == "single_axis") {
      output << "  - name: MPC_XY_P\n    role: candidate\n"
        << "  - name: MPC_XY_VEL_P_ACC\n    role: candidate\n"
        << "  - name: MPC_XY_VEL_I_ACC\n    role: candidate\n"
        << "  - name: MPC_ACC_HOR_MAX\n    role: context\n"
        << "  - name: MPC_XY_VEL_MAX\n    role: context\n";
    } else {
      output << "  - name: MPC_THR_HOVER\n    role: candidate\n"
        << "  - name: MPC_Z_P\n    role: context\n"
        << "  - name: MPC_Z_VEL_P_ACC\n    role: candidate\n"
        << "  - name: MPC_Z_VEL_I_ACC\n    role: candidate\n"
        << "  - name: MPC_XY_VEL_I_ACC\n    role: conditional_candidate\n"
        << "  - name: MPC_XY_P\n    role: shared_context\n"
        << "  - name: MPC_XY_VEL_P_ACC\n    role: shared_context\n";
    }
    output << "quality_context:\n"
      << "  - SIM_BAT_ENABLE\n  - SIM_BAT_DRAIN\n"
      << "  - SIM_BAT_MIN_PCT\n  - COM_ARM_BAT_MIN\n";
  }

  void write_sample(const logic::Input &in, const logic::Actions &actions)
  {
    if (_result_written) { return; }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto age = [in, nan](bool have, double receipt) { return have ? in.wall - receipt : nan; };
    _samples << std::setprecision(15) << in.wall << ',' << (_have_clock ? in.sim : nan) << ','
      << timestamp_us() << ',' << _position.timestamp << ',' << _position.timestamp_sample << ','
      << _status.timestamp << ',' << _land_timestamp << ',' << logic::name(_mission->state()) << ','
      << actions.stream << ',' << (actions.stream ? actions.target.x : nan) << ','
      << (actions.stream ? actions.target.y : nan) << ',' << (actions.stream ? actions.target.z : nan) << ','
      << (actions.stream ? actions.yaw : nan) << ','
      << (_have_position ? _position.x : nan) << ',' << (_have_position ? _position.y : nan) << ','
      << (_have_position ? _position.z : nan) << ',' << (_have_position ? _position.vx : nan) << ','
      << (_have_position ? _position.vy : nan) << ',' << (_have_position ? _position.vz : nan) << ','
      << (_have_position ? in.speed : nan) << ',' << (_have_position ? in.heading : nan) << ','
      << _position.xy_valid << ',' << _position.z_valid << ',' << _position.v_xy_valid << ',' << _position.v_z_valid << ','
      << static_cast<unsigned>(_status.arming_state) << ',' << static_cast<unsigned>(_status.nav_state) << ','
      << in.failsafe << ',' << in.landed << ',' << in.preflight_ok << ','
      << in.clock_fresh << ',' << in.position_fresh << ',' << in.status_fresh << ',' << in.land_fresh << ','
      << age(_have_clock, _clock_wall) << ',' << age(_have_position, _position_wall) << ','
      << age(_have_status, _status_wall) << ',' << age(_have_land, _land_wall) << ','
      << static_cast<unsigned>(in.xy_reset) << ',' << static_cast<unsigned>(in.z_reset) << ','
      << static_cast<unsigned>(in.heading_reset) << ',' << _mission->hold_elapsed(in) << '\n';
    _agent_samples << std::setprecision(15) << in.wall << ',' << (_have_clock ? in.sim : nan) << ','
      << logic::name(_mission->state()) << ','
      << (actions.stream ? actions.target.x : nan) << ',' << (actions.stream ? actions.target.y : nan) << ','
      << (actions.stream ? actions.target.z : nan) << ','
      << (_have_position ? _position.x : nan) << ',' << (_have_position ? _position.y : nan) << ','
      << (_have_position ? _position.z : nan) << ',' << (_have_position ? _position.vx : nan) << ','
      << (_have_position ? _position.vy : nan) << ',' << (_have_position ? _position.vz : nan) << ','
      << (_have_position ? in.speed : nan) << ',' << static_cast<unsigned>(_status.arming_state) << ','
      << static_cast<unsigned>(_status.nav_state) << ',' << in.failsafe << ',' << in.landed << ','
      << in.preflight_ok << ',' << static_cast<unsigned>(in.xy_reset) << ','
      << static_cast<unsigned>(in.z_reset) << ',' << static_cast<unsigned>(in.heading_reset) << ','
      << _mission->hold_elapsed(in) << '\n';
  }

  const std::chrono::steady_clock::time_point _started{std::chrono::steady_clock::now()};
  logic::Config _config;
  std::unique_ptr<logic::Mission> _mission;
  Position _position{}; Status _status{};
  double _message_timeout{2}, _sim_time{0};
  double _position_wall{0}, _status_wall{0}, _land_wall{0}, _clock_wall{0}, _clock_progress_wall{0};
  double _last_log{0}, _last_flush{0};
  bool _have_position{false}, _have_status{false}, _have_land{false}, _have_clock{false};
  bool _clock_advanced{false}, _landed{false}, _abort_requested{false};
  bool _result_written{false}, _storage_failure_reported{false};
  bool _preflight_issue_seen{false}, _estimator_reset_seen{false};
  uint64_t _land_timestamp{0}, _land_sequence{0}, _status_sequence{0};
  int _target_system{1};
  std::string _position_topic, _status_topic, _land_topic, _clock_topic, _log_root;
  std::filesystem::path _run_path, _all_path, _agent_path;
  std::ofstream _samples, _events, _agent_samples, _agent_events;
  rclcpp::Subscription<Position>::SharedPtr _position_sub;
  rclcpp::Subscription<Status>::SharedPtr _status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_sub;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _clock_sub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _target_pub;
  rclcpp::Publisher<Command>::SharedPtr _command_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _abort_service;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _fixed_parameters;
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControlExperiment>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("px4_control_experiment"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
