from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare
from hal_hw_interface.launch import HalConfig, HalRTNode, HalUserNode, HalFiles


def generate_launch_description():
    arguments = [
        # hal_mgr launch args
        DeclareLaunchArgument(
            "hal_debug_output",
            default_value="true",
            description="Output HAL debug messages to screen (console).",
        ),
        DeclareLaunchArgument(
            "hal_debug_level",
            default_value="1",
            description="Set HAL debug level, 0-5.",
        ),
        # hal_hw_interface launch args
        DeclareLaunchArgument(
            "robot_description",
            description="Robot description from URDF.",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("rrbot_description"),
                    "urdf",
                    "rrbot.urdf.xacro",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "robot_controllers_param_file",
            description="Robot controller configuration param file.",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_bringup"),
                    "config",
                    "rrbot_controllers.yaml",
                ]
            ),
        ),
        # HAL configuration
        HalConfig(
            # hal_mgr args
            parameters=[
                dict(  # Individual keys
                    hal_debug_output=LaunchConfiguration("hal_debug_output"),
                    hal_debug_level=LaunchConfiguration("hal_debug_level"),
                ),
            ],
            output="both",
            emulate_tty=True,
            log_cmd=True,
            # HAL config
            actions=[
                HalRTNode(
                    name="hal_control_node",
                    package="hal_hw_interface",
                    component="hal_control_node",
                    parameters=[  # ROS parameters from various sources
                        # Individual keys
                        dict(
                            # Expanded robot description URDF
                            robot_description=Command(
                                [
                                    PathJoinSubstitution(
                                        [FindExecutable(name="xacro")]
                                    ),
                                    " ",
                                    LaunchConfiguration("robot_description"),
                                    " use_sim:=false",
                                    " use_fake_hardware:=false",
                                    " hardware_plugin:=",
                                    "hal_system_interface/HalSystemInterface",
                                ]
                            ),
                        ),
                        # Robot controllers yaml from launch params
                        LaunchConfiguration("robot_controllers_param_file"),
                    ],
                    log_cmd=True,
                    output="both",
                    emulate_tty=True,
                    # arguments=['--ros-args', '--log-level', 'debug'],
                    # prefix=['xterm -e gdb -ex run --args'],
                ),
                HalUserNode(
                    package="hw_device_mgr",
                    executable="hw_device_mgr",
                    additional_env=dict(USE_SIM="1"),
                    parameters=[
                        dict(  # One-off params
                            device_config_path=PathJoinSubstitution(
                                [
                                    FindPackageShare("x2_test_bench_demo"),
                                    "config",
                                    "device_config.yaml",
                                ]
                            ),
                            device_data_path=PathJoinSubstitution(
                                [
                                    FindPackageShare("x2_test_bench_demo"),
                                    "config",
                                    "sim_devices.yaml",
                                ]
                            ),
                        ),
                        PathJoinSubstitution(
                            [
                                FindPackageShare("x2_test_bench_demo"),
                                "config",
                                "hw_device_mgr.yaml",
                            ]
                        ),
                    ],
                    log_cmd=True,
                    output="both",
                    emulate_tty=True,
                    # arguments=['--ros-args', '--log-level', 'debug'],
                ),
                HalUserNode(
                    package="hal_hw_interface",
                    executable="hal_io",
                    log_cmd=True,
                    output="both",
                    emulate_tty=True,
                    parameters=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("x2_test_bench_demo"),
                                "config",
                                "hal_io.yaml",
                            ]
                        ),
                    ],
                    arguments=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("x2_test_bench_demo"),
                                "config",
                                "hal_io_config.yaml",
                            ]
                        ),
                    ],
                ),
                HalFiles(
                    hal_file_dir=PathJoinSubstitution(
                        [
                            FindPackageShare("x2_test_bench_demo"),
                            "halfiles",
                        ]
                    ),
                    hal_files=[
                        "x2_test_bench_demo.py",
                    ],
                    parameters=[
                        dict(
                            lcec_conf_file=PathJoinSubstitution(
                                [
                                    FindPackageShare("x2_test_bench_demo"),
                                    "halfiles",
                                    "ethercat.conf.xml",
                                ]
                            ),
                            use_sim=True,
                        ),
                        PathJoinSubstitution(
                            [
                                FindPackageShare("x2_test_bench_demo"),
                                "config",
                                "hal_config.yaml",
                            ]
                        ),
                    ],
                ),
            ],
        ),
    ]

    return LaunchDescription(arguments)
