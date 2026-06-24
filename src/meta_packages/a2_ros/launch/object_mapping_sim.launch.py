"""Launch the complete object-mapping pipeline in simulation."""
"""For movement, use another terminal 
a2 source
a2 stand
a2 unlock
a2 walk
a2 keyboard
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "scene",
            default_value="scene_test_meshes.xml",
            description="MuJoCo scene used for object mapping",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz with the simulation",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="yolov5l6",
            description="YOLOv5 ONNX model name",
        ),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a2_ros"), "launch", "sim.launch.py"]
            )
        ),
        launch_arguments={
            "scene": LaunchConfiguration("scene"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    rectifier = Node(
        package="image_proc",
        executable="rectify_node",
        name="camera_rectifier",
        output="screen",
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("image_rect", "/camera/image_rect"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    detection_and_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("object_detection"),
                    "launch",
                    "object_detection.launch.py",
                ]
            )
        ),
        launch_arguments={"model": LaunchConfiguration("model")}.items(),
    )

    return LaunchDescription(
        declared_arguments + [simulation, rectifier, detection_and_mapping]
    )
