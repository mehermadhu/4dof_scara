from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("canopen_tests"),
                    "urdf/scara_controller",
                    "robot_controller.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("canopen_tests"), "config/scara_control", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "--controller-manager", "/controller_manager"],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

    gui_publisher_node = Node(
        package='my_gui_pkg',
        executable='gui_publisher',
        name='gui_publisher',
        output='screen',
    )
    
    slave_config = PathJoinSubstitution(
        [FindPackageShare("canopen_tests"), "config/scara_control", "cia402_slave.eds"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )
    
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_1",
            "slave_config": slave_config,
        }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_2",
            "slave_config": slave_config,
        }.items(),
    )

    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "4",
            "node_name": "slave_node_3",
            "slave_config": slave_config,
        }.items(),
    )

    master_node = Node(
        package="canopen_master_driver",
        executable="master_node",
        name="canopen_master",
        output="screen",
        parameters=[
            {"node_id": 1},
            {"baudrate": 500},
            {"sync_period": 1000},
            {"sync_window": 1000},
            {"error_behavior": {1: 0x00}},
            {"nmt_inhibit_time": 0},
            {"start": True},
            {"start_nodes": True},
            {"start_all_nodes": False},
            {"reset_all_nodes": False},
            {"stop_all_nodes": False},
            {"boot_time": 0},
            {"heartbeat_consumer": 1},
            {"consumer_heartbeat_time": 2000},
        ],
    )

    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        forward_position_controller_spawner,
        robot_state_publisher_node,
        slave_node_1,
        slave_node_2,
        slave_node_3,
        master_node,
        rviz_node,
        gui_publisher_node
    ]

    return LaunchDescription(nodes_to_start)
