from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define your drone fleet here
    drones = [
        {"id": 1, "ns": "px4_1"},
        {"id": 2, "ns": "px4_2"},
    ]

    nodes = []

    for d in drones:
        nodes.append(
            Node(
                package="swarm_lawnmower",
                executable="swarm_node",
                name=f"swarm_lawnmower_node_{d['id']}",
                namespace=d["ns"],
                output="screen",
                emulate_tty=True,  # ✅ ensures colored + timestamped logs in terminal
                parameters=[{
                    "drone_id": d["id"],
                    "total_drones": len(drones)
                }],
                # These remaps ensure that each swarm node talks to its own PX4 instance
                remappings=[
                    ("fmu/out/vehicle_odometry", 
                     f"/{d['ns']}/fmu/out/vehicle_odometry"),
                    ("fmu/out/vehicle_status", 
                     f"/{d['ns']}/fmu/out/vehicle_status"),
                    ("fmu/in/vehicle_command", 
                     f"/{d['ns']}/fmu/in/vehicle_command"),
                    ("fmu/in/offboard_control_mode", 
                     f"/{d['ns']}/fmu/in/offboard_control_mode"),
                    ("fmu/in/trajectory_setpoint", 
                     f"/{d['ns']}/fmu/in/trajectory_setpoint"),
                ]
            )
        )

    return LaunchDescription(nodes)
