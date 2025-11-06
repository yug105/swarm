#include "swarm_drone.hpp"
#include <functional>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>   


static const std::string kModeName = "swarm_lawnmower";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;
using std::placeholders::_1;

SwarmDrone::SwarmDrone(rclcpp::Node &node)
: ModeBase(node, kModeName),
  _node(node)
{
    // Parameters
    _node.declare_parameter<int>("drone_id", 1);
    _node.declare_parameter<int>("total_drones", 2);

    _drone_id = _node.get_parameter("drone_id").as_int();
    _total_drones = _node.get_parameter("total_drones").as_int();

    // PX4 interface objects MUST use mode context (*this)
    _local_pos = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _trajectory_sp = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _goto_sp = std::make_shared<px4_ros2::GotoSetpointType>(*this);


     

    if (_drone_id == 1)
        _spawn_offset = Eigen::Vector3f(0.0f, 0.0f, 0.0f);   // PX4_GZ_MODEL_POSE="0,20"
    else if (_drone_id == 2)
        _spawn_offset = Eigen::Vector3f(0.0f, 20.0f, 0.0f);    // PX4_GZ_MODEL_POSE="0,0"
 ;

    // --- Subscribe to own odometry with version suffix ---
    const std::string odom_topic = "fmu/out/vehicle_odometry";
    
    _odom_sub = _node.create_subscription<px4_msgs::msg::VehicleOdometry>(
        odom_topic,
        rclcpp::QoS(10).best_effort().durability_volatile(),
        std::bind(&SwarmDrone::localOdomCallback, this, _1)
    );
    // _traj_pub = _node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    //     "fmu/in/trajectory_setpoint", 10);
    RCLCPP_INFO(_node.get_logger(), 
        "[Drone %d] Subscribed to %s", _drone_id, odom_topic.c_str());

    // --- Subscribe to vehicle status for arming state with version suffix ---
    const std::string status_topic = "fmu/out/vehicle_status" + getMessageNameVersion<px4_msgs::msg::VehicleStatus>();
    
    _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic,
        rclcpp::QoS(10).best_effort().durability_volatile(),
        std::bind(&SwarmDrone::vehicleStatusCallback, this, _1)
    );
    
    RCLCPP_INFO(_node.get_logger(), 
        "[Drone %d] Subscribed to %s for arming monitoring", _drone_id, status_topic.c_str());

    // Create vehicle command publisher with version suffix
    const std::string cmd_topic = "fmu/in/vehicle_command";
    
    _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
        cmd_topic,
        10
    );
    
    RCLCPP_INFO(_node.get_logger(), 
        "[Drone %d] Publishing commands to %s", _drone_id, cmd_topic.c_str());

    // Prepare remote drone buffers
    _other_positions.resize(_total_drones + 1, Eigen::Vector3f::Zero());
    _other_last_update.resize(_total_drones + 1, _node.get_clock()->now());

    // Subscribe to other drones with version suffix
    for (int i = 1; i <= _total_drones; ++i)
    {
        if (i == _drone_id) continue;
        
        std::string topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_odometry";

        auto sub = _node.create_subscription<px4_msgs::msg::VehicleOdometry>(
            topic,
            rclcpp::QoS(10).best_effort().durability_volatile(),
            [this, i](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                otherOdomCallback(msg, i);
            }
        );

        _other_odom_subs.push_back(sub);
        RCLCPP_INFO(_node.get_logger(), "[Drone %d] Subscribed to %s", _drone_id, topic.c_str());
    }

    // Assign region
    if (_drone_id == 1)
        _assigned_area = {0, 10, 0, 40, 5};
    else if (_drone_id == 2)
        _assigned_area = {10, 20, 0, 40, 5};
    else
        _assigned_area = {0, 20, 0, 40, 5};

    
    _state = State::Start;

    RCLCPP_INFO(
        _node.get_logger(),
        "[Drone %d] Area X[%.1f, %.1f], Y[%.1f, %.1f], %zu waypoints generated",
        _drone_id,
        _assigned_area.min_x, _assigned_area.max_x,
        _assigned_area.min_y, _assigned_area.max_y,
        _waypoints.size()
    );

    // Create timer for monitoring and auto mode switching
    _arm_timer = _node.create_wall_timer(
        std::chrono::milliseconds(100),  // Check every 100ms
        std::bind(&SwarmDrone::autoArmSequence, this)
    );

    RCLCPP_INFO(_node.get_logger(), 
        "[Drone %d] 🤖 AUTO MODE-SWITCH ENABLED: Will switch to swarm mode 2s after arming", _drone_id);
}



// ---------------- Mode Hooks ----------------
void SwarmDrone::onActivate()
{ 
    _state = State::Start;
    _waypoint_index = 0;
    _home_set = false;
    generateLawnmowerWaypoints();
    RCLCPP_INFO(_node.get_logger(), "[Drone %d] ✅ Swarm lawnmower mode ACTIVATED!", _drone_id);
}

void SwarmDrone::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Drone %d] Swarm lawnmower mode deactivated", _drone_id);
}

// ---------------- Monitoring and Auto Mode Switch ----------------
void SwarmDrone::autoArmSequence()
{
    static bool init_message_shown = false;
    
    // Show initial message once when odometry is valid
    if (_current_pos.norm() > 0.1f && !init_message_shown) {
        RCLCPP_INFO(_node.get_logger(), 
            "[Drone %d] ✅ Odometry valid. System ready.", _drone_id);
        RCLCPP_WARN(_node.get_logger(),
            "[Drone %d] 📋 Just ARM the drone - mode will auto-switch!", _drone_id);
        RCLCPP_WARN(_node.get_logger(),
            "   Console: commander arm");
        init_message_shown = true;
    }
    
    // Monitor arming state and trigger mode switch
    static bool last_armed = false;
    static bool mode_switch_initiated = false;
    
    if (_armed != last_armed) {
        if (_armed) {
            RCLCPP_INFO(_node.get_logger(), "[Drone %d] 🚁 ARMED!", _drone_id);
            _arm_detected_time = _node.get_clock()->now();
            mode_switch_initiated = false;
        } else {
            RCLCPP_WARN(_node.get_logger(), "[Drone %d] 🔒 DISARMED!", _drone_id);
            mode_switch_initiated = false;
            _mode_switch_requested = false;
        }
        last_armed = _armed;
    }
    
    // Auto-switch to swarm mode 2 seconds after arming
    if (_armed && !mode_switch_initiated) {
        double time_since_arm = (_node.get_clock()->now() - _arm_detected_time).seconds();
        
        if (time_since_arm >= 2.0) {
            RCLCPP_INFO(_node.get_logger(), 
                "[Drone %d] ⏰ 2 seconds elapsed - requesting mode switch to swarm_lawnmower!", _drone_id);
            requestModeSwitch();
            mode_switch_initiated = true;
        } else if (!_mode_switch_requested) {
            static int countdown_shown = -1;
            int countdown = static_cast<int>(2.0 - time_since_arm) + 1;
            if (countdown != countdown_shown) {
                RCLCPP_INFO(_node.get_logger(), 
                    "[Drone %d] Auto mode-switch in %d...", _drone_id, countdown);
                countdown_shown = countdown;
            }
        }
    }
}

void SwarmDrone::requestModeSwitch()
{
    if (_mode_switch_requested) return;
    
    RCLCPP_INFO(_node.get_logger(), 
        "[Drone %d] 📡 Requesting switch to swarm_lawnmower mode...", _drone_id);
    
    // The mode is registered with PX4, we just need to request activation
    // PX4 will call onActivate() when the mode switch succeeds
    _mode_switch_requested = true;
}

void SwarmDrone::sendArmCommand(bool arm)
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
    cmd.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
    cmd.param1 = arm ? 1.0f : 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = NAN;
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = NAN;
    cmd.target_system = 0;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    _vehicle_command_pub->publish(cmd);
    
    RCLCPP_INFO(_node.get_logger(), 
        "[Drone %d] Sent %s command", _drone_id, arm ? "ARM" : "DISARM");
}


void SwarmDrone::sendVehicleCommand(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = NAN;
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = NAN;
    cmd.target_system = 0;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    _vehicle_command_pub->publish(cmd);
}

// ---------------- Setpoint Loop ----------------
void SwarmDrone::updateSetpoint(float dt_s)
{
    static int update_counter = 0;

    // ✅ Declare once at the top
    Eigen::Vector3f target_setpoint = _current_pos;
    px4_ros2::TrajectorySetpoint sp{};

    // Periodic logging
    if (++update_counter % 100 == 0) {
        RCLCPP_INFO(_node.get_logger(),
            "[%d] State=%s, Armed=%s, WP=%d/%zu, Pos=[%.2f,%.2f,%.2f]",
            _drone_id, stateName(_state).c_str(),
            _armed ? "YES" : "NO",
            _waypoint_index, _waypoints.size(),
            _current_pos.x(), _current_pos.y(), _current_pos.z());
    }

    // ❌ Don't proceed if not armed
    if (!_armed) {
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 3000,
            "[%d] Waiting for arming...", _drone_id);

        // sp.position_ned_m_x = target_setpoint.x();
        // sp.position_ned_m_y = target_setpoint.y();
        // sp.position_ned_m_z = target_setpoint.z();
        _goto_sp->update(target_setpoint);
        return;
    }

    // -------- START STATE --------
    if (_state == State::Start)
    {
        if (_current_pos.norm() < 0.01f) {
            RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                "[%d] Waiting for valid odometry...", _drone_id);
            // sp.position_ned_m_x = target_setpoint.x();
            // sp.position_ned_m_y = target_setpoint.y();
            // sp.position_ned_m_z = target_setpoint.z();
            _goto_sp->update(target_setpoint);
            return;
        }

        _home_position = _current_pos;
        _home_set = true;
        RCLCPP_INFO(_node.get_logger(), "[%d] 🏠 Home set at [%.2f, %.2f, %.2f]",
                    _drone_id, _home_position.x(), _home_position.y(), _home_position.z());

        bool all_drones_ready = true;
        {
            std::lock_guard<std::mutex> lock(_other_mutex);
            for (int i = 1; i <= _total_drones; ++i) {
                if (i == _drone_id) continue;
                if (!otherIsFresh(i)) {
                    all_drones_ready = false;
                    RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
                        "[%d] ⏳ Waiting for Drone %d to be ready...", _drone_id, i);
                    break;
                }
            }
        }

        if (all_drones_ready) {
            RCLCPP_INFO(_node.get_logger(),
                        "[%d] ✅ All drones ready! Starting mission...", _drone_id);
            switchToState(State::ExecuteLawnmower);
        }
    }

    // -------- COLLISION CHECK --------
    bool collision_detected = false;
    {
        std::lock_guard<std::mutex> lock(_other_mutex);
        for (int i = 1; i <= _total_drones; ++i)
        {
            if (i == _drone_id) continue;
            if (!otherIsFresh(i)) continue;

            float distance = (_other_positions[i] - _current_pos).norm();
            if (distance < COLLISION_DISTANCE_THRESHOLD)
            {
                collision_detected = true;
                if (_state != State::PauseForCollision) {
                    RCLCPP_WARN(_node.get_logger(),
                        "[%d] ⚠️ Collision risk with Drone %d (dist=%.2fm)! Pausing...",
                        _drone_id, i, distance);
                    switchToState(State::PauseForCollision);
                    _pause_start_time = _node.get_clock()->now();
                }
                break;
            }
        }
    }

    // -------- STATE MACHINE --------
    switch (_state)
    {
        case State::PauseForCollision:
            target_setpoint = _current_pos;
            if ((_node.get_clock()->now() - _pause_start_time).seconds() > PAUSE_DURATION) {
                if (!collision_detected) {
                    RCLCPP_INFO(_node.get_logger(), "[%d] ✅ Resuming mission", _drone_id);
                    switchToState(State::ExecuteLawnmower);
                } else {
                    RCLCPP_WARN(_node.get_logger(), "[%d] Still detecting collision...", _drone_id);
                    _pause_start_time = _node.get_clock()->now();
                }
            }
            break;

        case State::ExecuteLawnmower:
            if (_waypoint_index >= static_cast<int>(_waypoints.size())) {
                RCLCPP_INFO(_node.get_logger(), "[%d] ✅ All waypoints completed", _drone_id);
                switchToState(State::ReturnToHome);
                target_setpoint = _home_position;
                target_setpoint[2] = -_assigned_area.altitude;
            } else {
                target_setpoint = _waypoints[_waypoint_index];
                if (positionReached(target_setpoint)) {
                    _waypoint_index++;
                    RCLCPP_INFO(_node.get_logger(), "[%d] ✔ Reached waypoint %d/%zu",
                                _drone_id, _waypoint_index, _waypoints.size());
                }
            }
            break;

        case State::ReturnToHome:
            target_setpoint = _home_position;
            target_setpoint[2] = -_assigned_area.altitude;
            if (positionReached(target_setpoint)) {
                RCLCPP_INFO(_node.get_logger(), "[%d] 🏠 Reached home, descending", _drone_id);
                switchToState(State::Descend);
            }
            break;

        case State::Descend:
            target_setpoint = Eigen::Vector3f(_home_position.x(), _home_position.y(), -0.5f);
            if (positionReached(target_setpoint)) {
                RCLCPP_INFO(_node.get_logger(), "[%d] 🎉 Mission complete! Disarming...", _drone_id);
                sendArmCommand(false);
                switchToState(State::Finished);
            }
            break;

        case State::Finished:
            target_setpoint = _current_pos;
            break;

        case State::EmergencyClimb:
            target_setpoint = _current_pos;
            target_setpoint[2] = -(_assigned_area.altitude + EMERGENCY_CLIMB);
            if (positionReached(target_setpoint)) {
                RCLCPP_INFO(_node.get_logger(), "[%d] Emergency climb complete", _drone_id);
                switchToState(State::ExecuteLawnmower);
            }
            break;

        default:
            break;
    }

    // ✅ Always publish final setpoint once
    // sp.position_ned_m_x = target_setpoint.x();
    // sp.position_ned_m_y = target_setpoint.y();
    // sp.position_ned_m_z = target_setpoint.z();
    _goto_sp->update(target_setpoint);

    RCLCPP_DEBUG(_node.get_logger(), "[%d] Sent setpoint: [%.2f, %.2f, %.2f]",
                 _drone_id, target_setpoint.x(), target_setpoint.y(), target_setpoint.z());
}

// ---------------- Callbacks ----------------
void SwarmDrone::localOdomCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    try {
        //     RCLCPP_DEBUG(_node.get_logger(),
        // "[Drone %d] Odom update: [x=%.2f y=%.2f z=%.2f]",
        // _drone_id, msg->position[0], msg->position[1], msg->position[2]);

        _current_pos = {
            msg->position[0] + _spawn_offset.x(),
            msg->position[1] + _spawn_offset.y(),
            msg->position[2] + _spawn_offset.z()
        };
        _current_vel = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
        
        static bool first_odom = true;
        if (first_odom) {
            RCLCPP_INFO(_node.get_logger(), 
                "[Drone %d] First odometry received: [%.2f, %.2f, %.2f]",
                _drone_id, _current_pos.x(), _current_pos.y(), _current_pos.z());
            first_odom = false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(_node.get_logger(), 
            "[Drone %d] Error in localOdomCallback: %s", _drone_id, e.what());
    }
}

void SwarmDrone::vehicleStatusCallback(px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    try {
        bool was_armed = _armed;
        _armed = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        
        // Debug: Print first few messages
        static int debug_count = 0;
        if (debug_count++ < 3) {
            RCLCPP_INFO(_node.get_logger(), 
                "[Drone %d] vehicle_status: arming_state=%d, nav_state=%d", 
                _drone_id, msg->arming_state, msg->nav_state);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(_node.get_logger(), 
            "[Drone %d] Error in vehicleStatusCallback: %s", _drone_id, e.what());
    }
}

void SwarmDrone::otherOdomCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg, int id)
{
    try {
        std::lock_guard<std::mutex> lock(_other_mutex);
        Eigen::Vector3f offset = Eigen::Vector3f::Zero();
        if (id == 1) offset = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        else if (id == 2) offset = Eigen::Vector3f(0.0f, 20.0f, 0.0f);
    
        _other_positions[id] = {
            msg->position[0] + offset.x(),
            msg->position[1] + offset.y(),
            msg->position[2] + offset.z()
        };
        _other_last_update[id] = _node.get_clock()->now();
        
        static std::map<int, bool> first_other_odom;
        if (!first_other_odom[id]) {
            RCLCPP_INFO(_node.get_logger(), 
                "[Drone %d] 📡 Receiving odometry from Drone %d: [%.2f, %.2f, %.2f]",
                _drone_id, id, _other_positions[id].x(), _other_positions[id].y(), _other_positions[id].z());
            first_other_odom[id] = true;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(_node.get_logger(), 
            "[Drone %d] Error in otherOdomCallback for drone %d: %s", _drone_id, id, e.what());
    }
}

bool SwarmDrone::otherIsFresh(int idx) const
{
    return (_node.get_clock()->now() - _other_last_update[idx]).seconds()
            < COLLISION_UPDATE_TIMEOUT;
}

// ---------------- Waypoint Generator ----------------
void SwarmDrone::generateLawnmowerWaypoints()
{
    _waypoints.clear();
    float altitude = -_assigned_area.altitude;

    bool left_to_right = true;
    for (float y = _assigned_area.min_y; y <= _assigned_area.max_y; y += _strip_width)
    {
        float x = left_to_right ? _assigned_area.min_x : _assigned_area.max_x;
        _waypoints.emplace_back(x, y, altitude);
        left_to_right = !left_to_right;
    }
    
    if (!left_to_right) {
        _waypoints.emplace_back(_assigned_area.min_x, _assigned_area.max_y, altitude);
    }
    RCLCPP_INFO(_node.get_logger(), "[Drone %d] 🧭 Generated %zu waypoints:", _drone_id, _waypoints.size());
    for (size_t i = 0; i < _waypoints.size(); ++i) {
        RCLCPP_INFO(_node.get_logger(), "  WP %zu: [%.2f, %.2f, %.2f]",
            i, _waypoints[i].x(), _waypoints[i].y(), _waypoints[i].z());
    }
}

bool SwarmDrone::positionReached(const Eigen::Vector3f &goal) const
{
    float distance = (_current_pos - goal).norm();
    return distance < 1.5f;
}

void SwarmDrone::switchToState(State s)
{
    _state = s;
    RCLCPP_INFO(_node.get_logger(), "[%d] State → %s", _drone_id, stateName(s).c_str());
}

std::string SwarmDrone::stateName(State s) const
{
    switch (s) {
        case State::Start:             return "Start";
        case State::ExecuteLawnmower:  return "Execute";
        case State::PauseForCollision: return "Pause";
        case State::EmergencyClimb:    return "EmergencyClimb";
        case State::ReturnToHome:      return "Return";
        case State::Descend:           return "Descend";
        case State::Finished:          return "Finished";
    }
    return "Unknown";
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<SwarmDrone>>(
    kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}