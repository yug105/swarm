/****************************************************************************
 * Two-drone lawn-mower pattern in PX4 Offboard mode
 ****************************************************************************/

 #include <px4_msgs/msg/offboard_control_mode.hpp>
 #include <px4_msgs/msg/trajectory_setpoint.hpp>
 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <chrono>
 #include <iostream>
 #include <string>
 #include <vector>
 #include <array>
 #include <cmath>
 
 using namespace std::chrono_literals;
 using namespace px4_msgs::msg;
 
 struct Waypoint {
   float x, y, z;
 };
 
 class LawnMowerDrone : public rclcpp::Node
 {
 public:
   LawnMowerDrone(std::string ns, std::array<float,3> start_offset, float field_length, float lane_width)
   : Node(ns + "_lawnmower"), namespace_(ns), offset_(start_offset), field_length_(field_length), lane_width_(lane_width)
   {
     std::string prefix = "/" + namespace_ + "/fmu/in/";
 
     offboard_control_mode_pub_ = create_publisher<OffboardControlMode>(prefix + "offboard_control_mode", 10);
     trajectory_setpoint_pub_   = create_publisher<TrajectorySetpoint>(prefix + "trajectory_setpoint", 10);
     vehicle_command_pub_       = create_publisher<VehicleCommand>(prefix + "vehicle_command", 10);
 
     generate_waypoints();
     RCLCPP_INFO(get_logger(), "[%s] Generated %zu waypoints", namespace_.c_str(), waypoints_.size());
 
     timer_ = create_wall_timer(100ms, std::bind(&LawnMowerDrone::timer_callback, this));
   }
 
 private:
   rclcpp::TimerBase::SharedPtr timer_;
   rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
   rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
   rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
 
   std::string namespace_;
   std::array<float,3> offset_;
   float field_length_;
   float lane_width_;
   uint64_t offboard_counter_{0};
   std::vector<Waypoint> waypoints_;
   int current_wp_{0};
 
   void timer_callback()
   {
     if (offboard_counter_ == 10) {
       publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
       publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
       RCLCPP_INFO(get_logger(), "[%s] Armed + OFFBOARD", namespace_.c_str());
     }
 
     publish_offboard_control_mode();
 
     if (current_wp_ < (int)waypoints_.size()) {
       auto &wp = waypoints_[current_wp_];
       publish_trajectory_setpoint(wp);
       if (reached_waypoint(wp)) {
         current_wp_++;
         RCLCPP_INFO(get_logger(), "[%s] → next waypoint %d/%zu", namespace_.c_str(), current_wp_, waypoints_.size());
       }
     } else {
       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "[%s] Mission finished", namespace_.c_str());
     }
 
     if (offboard_counter_ < 11) offboard_counter_++;
   }
 
   void publish_offboard_control_mode()
   {
     OffboardControlMode msg{};
     msg.position = true;
     msg.velocity = false;
     msg.acceleration = false;
     msg.attitude = false;
     msg.body_rate = false;
     msg.timestamp = get_clock()->now().nanoseconds() / 1000;
     offboard_control_mode_pub_->publish(msg);
   }
 
   void publish_trajectory_setpoint(const Waypoint &wp)
   {
     TrajectorySetpoint msg{};
     msg.position = {wp.x, wp.y, wp.z};
     msg.yaw = 0.0;
     msg.timestamp = get_clock()->now().nanoseconds() / 1000;
     trajectory_setpoint_pub_->publish(msg);
   }
 
   bool reached_waypoint(const Waypoint &wp)
   {
     // Simple time-based progression (simulate reach after fixed steps)
     static int counter = 0;
     counter++;
     return (counter % 50 == 0);  // roughly every 5 s at 10 Hz
   }
 
   void publish_vehicle_command(uint16_t command, float param1=0.0, float param2=0.0)
   {
     VehicleCommand msg{};
     msg.param1 = param1;
     msg.param2 = param2;
     msg.command = command;
     msg.target_system = 1;
     msg.target_component = 1;
     msg.source_system = 1;
     msg.source_component = 1;
     msg.from_external = true;
     msg.timestamp = get_clock()->now().nanoseconds() / 1000;
     vehicle_command_pub_->publish(msg);
   }
 
   void generate_waypoints()
   {
     const float x_start = offset_[0];
     const float y_start = offset_[1];
     const float z = offset_[2];
 
     const int lanes = std::ceil(field_length_ / lane_width_);
     bool left_to_right = true;
 
     for (int i = 0; i < lanes; ++i) {
       float y = y_start + i * lane_width_;
       if (left_to_right) {
         waypoints_.push_back({x_start, y, z});
         waypoints_.push_back({x_start + 20.0f, y, z});
       } else {
         waypoints_.push_back({x_start + 20.0f, y, z});
         waypoints_.push_back({x_start, y, z});
       }
       left_to_right = !left_to_right;
     }
   }
 };
 
 int main(int argc, char *argv[])
 {
   rclcpp::init(argc, argv);
   RCLCPP_INFO(rclcpp::get_logger("main"), "Starting 2-drone lawn-mower pattern...");
 
   auto drone1 = std::make_shared<LawnMowerDrone>("px4_1", std::array<float,3>{0.0, 0.0, -5.0}, 40.0, 5.0);
   auto drone2 = std::make_shared<LawnMowerDrone>("px4_2", std::array<float,3>{0.0, 20.0, -5.0}, 40.0, 5.0);
 
   rclcpp::executors::MultiThreadedExecutor exec;
   exec.add_node(drone1);
   exec.add_node(drone2);
   exec.spin();
 
   rclcpp::shutdown();
   return 0;
 }
 