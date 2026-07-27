import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup     = get_package_share_directory('my_robot_bringup')

    urdf_path              = os.path.join(pkg_description, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path       = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')
    gazebo_config_path     = os.path.join(pkg_bringup, 'config', 'gazebo_bridge.yaml')
    controllers_config     = os.path.join(pkg_bringup, 'config', 'my_controllers.yaml')
    ekf_config             = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    world_path             = os.path.join(pkg_bringup, 'worlds', 'test_world.sdf')
    gz_launch              = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    scripts_dir = os.path.join(pkg_bringup, 'scripts')

    # Process xacro with absolute path to controllers YAML
    robot_description_xml = xacro.process_file(
        urdf_path,
        mappings={'controllers_config': controllers_config}
    ).toxml()

    # ── Spawn robot model node ──────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen'
    )

    # ── Controller Spawners (Deferred until after model is spawned) ─────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Trigger controller activation ONLY after the robot entity creation process exits
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                imu_broadcaster_spawner,
            ],
        )
    )

    return LaunchDescription([

        # ── Robot state publisher ─────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_xml,
                'use_sim_time': True,
            }]
        ),

        # ── Gazebo simulation ─────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'{world_path} -r'}.items()
        ),

        # ── Spawn robot model ─────────────────────────────────────────────────
        spawn_robot,

        # ── Controller spawners (event handled) ───────────────────────────────
        delayed_controller_spawners,

        # ── Gazebo ↔ ROS bridge ───────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': gazebo_config_path, 'use_sim_time': True}],
        ),

        # ── EKF: fused estimate (wheels + IMU) ────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}],
        ),

        # ── RViz ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
        ),

        # ── Odometry -> Path converter ────────────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'odom_to_path.py')],
            output='screen',
        ),

        # ── Trajectory recorder ───────────────────────────────────────────────
        ExecuteProcess(
            cmd=['python3', os.path.join(scripts_dir, 'plot_trajectories.py')],
            output='screen',
        ),
    ])