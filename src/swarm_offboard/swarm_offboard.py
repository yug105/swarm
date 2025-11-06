#!/usr/bin/env python3
"""
swarm_lawnmower_offboard_fixed.py
Two-drone PX4 Offboard lawn-mower pattern following official PX4 ROS 2 guidelines.
Based on: https://docs.px4.io/main/en/ros/ros2_offboard_control.html
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition


class LawnMowerController:
    """Simple rectangular lawnmower pattern generator."""

    def __init__(self, origin, field_size=(20.0, 40.0), lane_spacing=5.0, altitude=-5.0):
        self.origin = np.array(origin, dtype=float)
        self.field_size = np.array(field_size, dtype=float)
        self.lane_spacing = lane_spacing
        self.altitude = altitude
        self.generate_waypoints()
        self.current_index = 0

    def generate_waypoints(self):
        """Generate proper lawnmower pattern with start and end points per lane."""
        x0, y0 = self.origin
        width, height = self.field_size
        wps = []
        left_to_right = True
        y = y0
        
        while y <= y0 + height:
            if left_to_right:
                wps.append(np.array([x0, y, self.altitude]))
                wps.append(np.array([x0 + width, y, self.altitude]))
            else:
                wps.append(np.array([x0 + width, y, self.altitude]))
                wps.append(np.array([x0, y, self.altitude]))
            
            left_to_right = not left_to_right
            y += self.lane_spacing
        
        self.waypoints = wps

    def current_target(self):
        return self.waypoints[self.current_index]

    def advance(self):
        """Advance to next waypoint. Returns True if more waypoints available."""
        if self.current_index < len(self.waypoints) - 1:
            self.current_index += 1
            return True
        return False

    def is_complete(self):
        """Check if mission is complete."""
        return self.current_index >= len(self.waypoints) - 1


class PX4LawnmowerNode(Node):
    """
    One drone Offboard controller following official PX4 ROS 2 pattern.
    Uses timer-based callbacks instead of threading.
    """

    def __init__(self, drone_id, start_offset):
        super().__init__(f"lawnmower_agent_{drone_id}")
        self.id = drone_id
        self.altitude = -5.0
        self.threshold = 0.8
        self.controller = LawnMowerController(start_offset, altitude=self.altitude)
        
        # Position tracking
        self.vehicle_local_position = VehicleLocalPosition()
        
        # State management - following official example pattern
        self.offboard_setpoint_counter = 0
        self.mission_state = "TAKEOFF"  # TAKEOFF -> MISSION -> COMPLETE
        # Increased warmup count to ensure both messages are published before switching
        self.warmup_count = 100  # 5 seconds at 20Hz

        # Configure QoS profile - matching official example
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publishers
        ns = f"/px4_{self.id}"
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f"{ns}/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback, qos_profile)

        # Create timer - 50ms (20Hz) to ensure 10+ Hz publishing rate
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info(f"[Drone {self.id}] Initialized with offset {start_offset}")

    def vehicle_local_position_callback(self, msg):
        """Callback for vehicle local position."""
        self.vehicle_local_position = msg

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f"[Drone {self.id}] Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f"[Drone {self.id}] Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"[Drone {self.id}] Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """
        Publish the offboard control mode heartbeat.
        For this example, only position control is active.
        Must be published at 10+ Hz.
        """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_current_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """
        Publish a trajectory setpoint.
        Must be published at 10+ Hz.
        """
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw  # [-PI:PI]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        """
        Main control loop callback - runs at 20Hz (50ms).
        CRITICAL: Must publish both OffboardControlMode AND TrajectorySetpoint at 10+ Hz.
        Pattern from official PX4 ROS 2 example.
        """
        # CRITICAL: Always publish both OffboardControlMode AND TrajectorySetpoint at 10+ Hz
        self.publish_offboard_control_heartbeat_signal()
        
        # Get current position (use origin if not received yet)
        if self.vehicle_local_position.x == 0.0 and self.vehicle_local_position.y == 0.0:
            # Use initial position estimate
            current_pos = np.array([self.controller.origin[0], 
                                   self.controller.origin[1], 
                                   self.vehicle_local_position.z])
        else:
            current_pos = np.array([
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ])
        
        # Determine target based on state
        if self.mission_state == "TAKEOFF":
            # Takeoff to origin at target altitude
            target = np.array([self.controller.origin[0], 
                             self.controller.origin[1], 
                             self.altitude])
        elif self.mission_state == "MISSION":
            # Execute lawnmower pattern
            target = self.controller.current_target()
        else:  # COMPLETE
            # Hold position at last waypoint
            target = self.controller.current_target()
        
        # CRITICAL: Always publish trajectory setpoint at 10+ Hz (even during warmup)
        self.publish_current_setpoint(target[0], target[1], target[2])
        
        # After warmup count, engage offboard mode and arm
        # Both messages must be published before switching
        if self.offboard_setpoint_counter == self.warmup_count:
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info(f"[Drone {self.id}] Offboard mode engaged after {self.warmup_count} setpoints")
        
        # State machine for mission execution
        if self.mission_state == "TAKEOFF":
            # Check if reached altitude (within 0.5m) after warmup
            if abs(current_pos[2] - self.altitude) < 0.5 and self.offboard_setpoint_counter > self.warmup_count + 20:
                self.mission_state = "MISSION"
                self.get_logger().info(f"[Drone {self.id}] Takeoff complete, starting mission")
        
        elif self.mission_state == "MISSION":
            # Check if reached waypoint (horizontal distance only)
            dist = np.linalg.norm(current_pos[:2] - target[:2])
            if dist < self.threshold:
                if self.controller.advance():
                    self.get_logger().info(
                        f"[Drone {self.id}] Waypoint {self.controller.current_index}/"
                        f"{len(self.controller.waypoints)-1}")
                else:
                    self.mission_state = "COMPLETE"
                    self.get_logger().info(f"[Drone {self.id}] Mission complete!")
        
        # Increment counter
        if self.offboard_setpoint_counter < self.warmup_count + 1:
            self.offboard_setpoint_counter += 1


def main():
    print('Starting offboard control node...')
    rclpy.init()

    # Configuration - two drones with different start positions
    num_drones = 2
    offsets = [(0.0, 0.0), (0.0, 25.0)]  # Separated by 25m in Y

    # Create drone nodes
    nodes = [PX4LawnmowerNode(i + 1, offsets[i]) for i in range(num_drones)]

    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=max(4, num_drones))
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nStopping swarm...")
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()