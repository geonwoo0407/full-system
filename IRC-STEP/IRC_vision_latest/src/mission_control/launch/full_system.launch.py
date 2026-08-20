#!/usr/bin/env python3
"""Launch the STEP vision pipeline with a selectable C++ motion backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the camera-to-motion ROS graph."""
    enable_camera = LaunchConfiguration("enable_camera")
    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    display = LaunchConfiguration("display")
    publish_annotated_image = LaunchConfiguration(
        "publish_annotated_image"
    )
    metrics_mode = LaunchConfiguration("metrics_mode")
    max_fps = LaunchConfiguration("max_fps")
    initial_mission_phase = LaunchConfiguration("initial_mission_phase")
    backend_type = LaunchConfiguration("backend_type")
    enable_robot_hardware = LaunchConfiguration("enable_robot_hardware")
    motion_json_path = LaunchConfiguration("motion_json_path")
    explicit_torque_approval = LaunchConfiguration(
        "explicit_torque_approval"
    )
    robot_device_path = LaunchConfiguration("robot_device_path")
    robot_baud_rate = LaunchConfiguration("robot_baud_rate")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
        }.items(),
    )

    detector = Node(
        package="step",
        executable="yolo26_detector",
        name="yolo26_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": model_path,
                "device": device,
                "display": ParameterValue(display, value_type=bool),
                "publish_annotated_image": ParameterValue(
                    publish_annotated_image,
                    value_type=bool,
                ),
                "metrics_mode": metrics_mode,
                "max_fps": ParameterValue(max_fps, value_type=float),
            }
        ],
        remappings=[
            (
                "/camera/color/image_raw",
                "/camera/camera/color/image_raw",
            ),
        ],
    )

    analyzers = Node(
        package="step",
        executable="unified_vision_node",
        output="screen",
        emulate_tty=True,
        remappings=[
            (
                "/camera/color/image_raw",
                "/camera/camera/color/image_raw",
            ),
            (
                "/camera/aligned_depth_to_color/image_raw",
                "/camera/camera/aligned_depth_to_color/image_raw",
            ),
        ],
    )

    motion_decision = Node(
        package="mission_control",
        executable="motion_decision_node",
        name="motion_decision_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"initial_mission_phase": initial_mission_phase}],
    )

    motion_command_bridge = Node(
        package="mission_control",
        executable="motion_command_bridge_node",
        name="motion_command_bridge_node",
        output="screen",
        emulate_tty=True,
    )

    sdk_motion_executor = Node(
        package="irc_step_motion_executor",
        executable="sdk_motion_executor",
        name="sdk_motion_executor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "backend_type": backend_type,
                "enable_robot_hardware": ParameterValue(
                    enable_robot_hardware,
                    value_type=bool,
                ),
                "poll_period_ms": 20,
                "running_polls": 2,
                "settling_polls": 1,
                "explicit_torque_approval": ParameterValue(
                    explicit_torque_approval,
                    value_type=bool,
                ),
                "motion_json_path": motion_json_path,
                "robot_device_path": robot_device_path,
                "robot_baud_rate": ParameterValue(
                    robot_baud_rate,
                    value_type=int,
                ),
                "robot_motor_ids": list(range(23)),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Launch the RealSense camera and aligned depth stream.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("step"), "models", "best.engine"]
                ),
                description="YOLO .onnx or TensorRT .engine model path.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="tensorrt",
                description="Inference device used by yolo26_detector.",
            ),
            DeclareLaunchArgument(
                "display",
                default_value="true",
                description="Show the detector visualization window.",
            ),
            DeclareLaunchArgument(
                "publish_annotated_image",
                default_value="true",
                description="Publish /vision/detections/image.",
            ),
            DeclareLaunchArgument(
                "metrics_mode",
                default_value="auto",
                description="Metrics overlay selected by yolo26_detector.",
            ),
            DeclareLaunchArgument(
                "max_fps",
                default_value="30.0",
                description="Maximum detector processing rate.",
            ),
            DeclareLaunchArgument(
                "initial_mission_phase",
                default_value="AUTO",
                description=(
                    "Initial planner phase before /mission/phase is received."
                ),
            ),
            DeclareLaunchArgument(
                "backend_type",
                default_value="simulated",
                choices=["simulated", "robot_motion_player"],
                description=(
                    "Select the simulated or robot_motion_player backend."
                ),
            ),
            DeclareLaunchArgument(
                "enable_robot_hardware",
                default_value="false",
                description="Allow access to physical robot hardware.",
            ),
            DeclareLaunchArgument(
                "motion_json_path",
                default_value="",
                description=(
                    "RobotMotionPlayer motion JSON path; empty is allowed "
                    "for the simulated backend."
                ),
            ),
            DeclareLaunchArgument(
                "explicit_torque_approval",
                default_value="false",
                description=(
                    "Explicitly approve torque enable during hardware "
                    "initialization."
                ),
            ),
            DeclareLaunchArgument(
                "robot_device_path",
                default_value="/dev/ttyUSB0",
                description="Dynamixel serial device for the fixed SDK profile.",
            ),
            DeclareLaunchArgument(
                "robot_baud_rate",
                default_value="4000000",
                description="Dynamixel baud rate for the fixed SDK profile.",
            ),
            camera,
            detector,
            analyzers,
            motion_decision,
            motion_command_bridge,
            sdk_motion_executor,
        ]
    )
