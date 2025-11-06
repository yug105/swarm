from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define your drone fleet here
    drones = [
        {'id': 1, 'ns': 'px4_1'},
        {'id': 2, 'ns': 'px4_2'}
    ]
    
    launch_description = LaunchDescription()
    
    for d in drones:
        launch_description.add_action(Node(
            package="swarm",
            executable="swarmlawn",
            name=f"swarm_node_{d['id']}",
            namespace=d['ns'],
            output="screen",
            emulate_tty=True,
            parameters=[{
                "drone_id": d['id'],
                "total_drones": len(drones),
                "use_sim_time": True
            }],
            remappings=[
                ("fmu/out/vehicle_odometry", f"/{d['ns']}/fmu/out/vehicle_odometry"),
                ("fmu/out/vehicle_status", f"/{d['ns']}/fmu/out/vehicle_status"),
                ("fmu/in/vehicle_command", f"/{d['ns']}/fmu/in/vehicle_command"),
                ("fmu/in/trajectory_setpoint", f"/{d['ns']}/fmu/in/trajectory_setpoint"),
            ]
        ))
    
    return launch_description
