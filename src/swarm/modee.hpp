#pragma once

#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/control/offboard_control_mode.hpp>
#include <px4_ros2/control/vehicle_command.hpp>
#include <px4_ros2/status/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;
using namespace px4_ros2::literals; // NOLINT

// Area structure definition
struct Area {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float altitude;
};

class SwarmOffboardNode : public rclcpp::Node
{
public:
  explicit SwarmOffboardNode(const std::string & node_name, bool enable_debug_output)
  : rclcpp::Node(node_name)
  {
    // 1. Parameter Declaration and Retrieval
    this->declare_parameter<int>("drone_id", 1);
    this->declare_parameter<int>("total_drones", 2);

    _drone_id = this->get_parameter("drone_id").as_int();
    _total_drones = this->get_parameter("total_drones").as_int();

    // 2. PX4-ROS 2 Component Initialization
    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _offboard_control_mode = std::make_shared<px4_ros2::OffboardControlMode>(*this);
    _vehicle_command = std::make_shared<px4_ros2::VehicleCommand>(*this);
    _vehicle_status = std::make_shared<px4_ros2::VehicleStatus>(*this);

    // 3. Swarm-specific setup
    if (_drone_id == 1)
        _spawn_offset = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    else if (_drone_id == 2)
        _spawn_offset = Eigen::Vector3f(0.0f, 20.0f, 0.0f);

    if (_drone_id == 1)
        _assigned_area = {0, 10, 0, 40, 5};
    else if (_drone_id == 2)
        _assigned_area = {10, 20, 0, 40, 5};
    else
        _assigned_area = {0, 20, 0, 40, 5};

    _home_position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    _state = State::PreFlight;
    generateLawnmowerWaypoints();

    // 4. Timer for main control loop (50Hz)
    _timer = this->create_wall_timer(
      20ms,
      std::bind(&SwarmOffboardNode::timer_callback, this)
    );
  }

private:
  // ROS 2 and PX4-ROS 2 Components
  rclcpp::TimerBase::SharedPtr _timer;
  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  std::shared_ptr<px4_ros2::OffboardControlMode> _offboard_control_mode;
  std::shared_ptr<px4_ros2::VehicleCommand> _vehicle_command;
  std::shared_ptr<px4_ros2::VehicleStatus> _vehicle_status;

  // Swarm and Mission State
  std::vector<Eigen::Vector3f> _waypoints;
  int _waypoint_index = 0;
  Eigen::Vector3f _home_position;
  Eigen::Vector3f _spawn_offset;
  Area _assigned_area;
  int _drone_id;
  int _total_drones;
  
  enum class State
  {
    PreFlight,
    Arming,
    OffboardMode,
    SettlingAtStart,
    ExecuteLawnmower,
    ReturnToHome,
    Descend,
    Finished
  } _state;

  Eigen::Vector3f _start_position_m;
  bool _start_position_set{false};
  int _offboard_setpoint_counter{0};
  static constexpr int kOffboardSetpointsBeforeArm = 100; // 2 seconds at 50Hz

  void generateLawnmowerWaypoints()
  {
    _waypoints.clear();
    float altitude = -_assigned_area.altitude;
    bool left_to_right = true;
    
    for (float y = _assigned_area.min_y; y <= _assigned_area.max_y; y += 2.0f)
    {
        float x = left_to_right ? _assigned_area.min_x : _assigned_area.max_x;
        _waypoints.emplace_back(x, y, altitude);
        
        // Alternate direction on next row
        left_to_right = !left_to_right;
    }
    
    // Ensure we end at min_x if needed
    if (!left_to_right) {
        _waypoints.emplace_back(_assigned_area.min_x, _assigned_area.max_y, altitude);
    }
    
    RCLCPP_INFO(this->get_logger(), "[Drone %d] 🧭 Generated %zu waypoints:", _drone_id, _waypoints.size());
    for (size_t i = 0; i < _waypoints.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  WP %zu: [%.2f, %.2f, %.2f]",
            i, _waypoints[i].x(), _waypoints[i].y(), _waypoints[i].z());
    }
  }

  bool positionReached(const Eigen::Vector3f & target_position_m) const
  {
    static constexpr float kPositionErrorThreshold = 0.5f; // [m]
    static constexpr float kVelocityErrorThreshold = 0.3f; // [m/s]
    const Eigen::Vector3f position_error_m = target_position_m -
      (_vehicle_local_position->positionNed() - _spawn_offset);
    return (position_error_m.norm() < kPositionErrorThreshold) &&
           (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
  }

  void arm()
  {
    RCLCPP_INFO(this->get_logger(), "[Drone %d] Sending arm command...", _drone_id);
    _vehicle_command->arm();
  }

  void setOffboardMode()
  {
    RCLCPP_INFO(this->get_logger(), "[Drone %d] Sending Offboard mode command...", _drone_id);
    _vehicle_command->setMode(px4_ros2::Mode::Offboard);
  }

  void timer_callback()
  {
    // Publish Offboard control mode at all times
    _offboard_control_mode->update(true, true, true, true);

    // Main state machine
    switch (_state) {
      case State::PreFlight: {
        // Publish a few setpoints before arming
        _goto_setpoint->update(_home_position);
        if (++_offboard_setpoint_counter >= kOffboardSetpointsBeforeArm) {
          _state = State::Arming;
        }
      }
      break;

      case State::Arming: {
        if (!_vehicle_status->isArmed()) {
          arm();
        } else {
          _state = State::OffboardMode;
        }
      }
      break;

      case State::OffboardMode: {
        if (!_vehicle_status->isOffboard()) {
          setOffboardMode();
        } else {
          _state = State::SettlingAtStart;
        }
      }
      break;

      case State::SettlingAtStart: {
        if (!_start_position_set) {
          _start_position_m = _vehicle_local_position->positionNed() + _spawn_offset;
          _start_position_set = true;
          RCLCPP_INFO(this->get_logger(), "[Drone %d] Starting position set to: [%.2f, %.2f, %.2f]",
            _drone_id, _start_position_m.x(), _start_position_m.y(), _start_position_m.z());
        }
        
        // just settling at the starting vehicle position
        _goto_setpoint->update(_start_position_m);
        if (positionReached(_start_position_m)) {
          RCLCPP_INFO(this->get_logger(), "[Drone %d] Reached start position. Starting mission.", _drone_id);
          _state = State::ExecuteLawnmower;
        }
      }
      break;

      case State::ExecuteLawnmower: {
        if (_waypoint_index < static_cast<int>(_waypoints.size())) {
          Eigen::Vector3f target_position = _waypoints[_waypoint_index];
          _goto_setpoint->update(target_position);
          if (positionReached(target_position)) {
            RCLCPP_INFO(this->get_logger(), "[Drone %d] Reached waypoint %d.", _drone_id, _waypoint_index);
            _waypoint_index++;
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "[Drone %d] Mission complete. Returning to home.", _drone_id);
          _state = State::ReturnToHome;
        }
      }
      break;

      case State::ReturnToHome: {
        Eigen::Vector3f target_position = _home_position;
        _goto_setpoint->update(target_position);
        if (positionReached(target_position)) {
          RCLCPP_INFO(this->get_logger(), "[Drone %d] Reached home position. Descending.", _drone_id);
          _state = State::Descend;
        }
      }
      break;

      case State::Descend: {
        Eigen::Vector3f target_position = Eigen::Vector3f(_home_position.x(), _home_position.y(), -0.5f);
        _goto_setpoint->update(target_position);
        if (positionReached(target_position)) {
          RCLCPP_INFO(this->get_logger(), "[Drone %d] Descended. Mission finished.", _drone_id);
          _state = State::Finished;
        }
      }
      break;

      case State::Finished: {
        // Keep sending setpoints to prevent mode switch
        _goto_setpoint->update(_start_position_m);
        // Optionally disarm and shutdown the node here if desired
      }
      break;
    }
  }
};
