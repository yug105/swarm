#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>
#include <map>

// Template function to get message version suffix
template <typename T>
std::string getMessageNameVersion() {
    if (T::MESSAGE_VERSION == 0) return "";
    return "_v" + std::to_string(T::MESSAGE_VERSION);
}

class SwarmDrone : public px4_ros2::ModeBase
{
public:
    explicit SwarmDrone(rclcpp::Node &node);
    
    // -------- Mode lifecycle from PX4 ---------
    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

private:
    enum class State {
        Start,
        ExecuteLawnmower,
        PauseForCollision,
        EmergencyClimb,
        ReturnToHome,
        Descend,
        Finished
    };

    struct Area {
        double min_x{0.0};
        double max_x{0.0};
        double min_y{0.0};
        double max_y{0.0};
        double altitude{0.0};
    };
    
    // ------- Callbacks ----------
    void localOdomCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicleStatusCallback(px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void otherOdomCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg, int other_id);
    
    // -------- Helpers ----------
    bool otherIsFresh(int idx) const;
    void generateLawnmowerWaypoints();
    bool positionReached(const Eigen::Vector3f &target) const;
    void switchToState(State state);
    std::string stateName(State s) const;
    
    // Monitoring and command methods
    void autoArmSequence();
    void sendArmCommand(bool arm);
    void sendVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void requestModeSwitch();
    
    // -------- Node ----------
    rclcpp::Node &_node;
    int _drone_id{1};
    int _total_drones{2};
    
    // PX4 Interfaces
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_pos;
    // std::shared_ptr<px4_ros2::GotoSetpointType> _goto_sp;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_sp;
    // new
    std::shared_ptr<px4_ros2::GotoSetpointType> _goto_sp;

    // -------- Subscribers --------
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr> _other_odom_subs;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
    
    // -------- Publishers --------
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _position_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
    
    // -------- Timers --------
    rclcpp::TimerBase::SharedPtr _arm_timer;
    
    // -------- Mission & navigation state --------
    State _state{State::Start};
    Area _assigned_area{};
    std::vector<Eigen::Vector3f> _waypoints;
    int _waypoint_index{0};
    Eigen::Vector3f _current_pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f _current_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f _home_position = Eigen::Vector3f::Zero();
    bool _home_set{false};
    
    // -------- Swarm tracking --------
    std::vector<Eigen::Vector3f> _other_positions;
    std::vector<rclcpp::Time> _other_last_update;
    mutable std::mutex _other_mutex;
    
    // -------- Field parameters --------
    double _field_min_x{0.0};
    double _field_max_x{20.0};
    double _field_min_y{0.0};
    double _field_max_y{40.0};
    double _strip_width{2.0};
    double _overlap_percentage{0.0};
    
    // -------- Collision constants --------
    static constexpr double COLLISION_DISTANCE_THRESHOLD = 3.0;
    static constexpr double COLLISION_UPDATE_TIMEOUT     = 2.0;
    static constexpr double PAUSE_DURATION               = 5.0;
    static constexpr double EMERGENCY_CLIMB              = 2.0;
    Eigen::Vector3f _spawn_offset = Eigen::Vector3f::Zero();
    rclcpp::Time _pause_start_time;
    bool _land_detected{false};
    bool _armed{false};
    
    // Auto-mode-switch members
    bool _mode_switch_requested{false};
    rclcpp::Time _arm_detected_time;
};