import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command ##new
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    bringup_dir = get_package_share_directory('warehouse_world')

    world = os.path.join(bringup_dir, "worlds", "warehouse_storage.sdf")
    # --- TurtleBot3 path  --- #new
    urdf_path = '/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(bringup_dir, 'worlds'),
            os.path.join(bringup_dir, 'models'),
            str(Path(bringup_dir).parent.resolve())
        ])
    )

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s -v 4 ", world]}.items(),  #gazebo headless mode

    )

    # Launch ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'gz_bridge_ros.yaml'),
            'use_sim_time': use_sim_time}],
        output='screen'
    )

    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'robot_description': Command(['xacro ', urdf_path])
    }]
    ) #new

    spawn_tb3 = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'turtlebot3_burger',
        '-file', '/root/workspaces/turtlebot_ws/src/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
        '-x', '0.0', '-y', '0.0', '-z', '0.05', # spawn a bit above ground, let it settle by gravity
        '-R', '0.0', '-P', '0.0', '-Y', '0.0'
    ],
    output='screen'
    ) 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        gz_resource_path,
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher, #new  
        spawn_tb3,    #new  
    ])
