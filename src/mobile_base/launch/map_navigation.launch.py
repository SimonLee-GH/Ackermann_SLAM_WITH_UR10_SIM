import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output

def generate_launch_description():
    run_headless = LaunchConfiguration("run_headless")
    params_file = LaunchConfiguration('params_file')

    # Starting bringup first, which might include robot hardware interfaces or other necessary setups
    bringup = ExecuteProcess(
        name="launch_bringup",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("mobile_base"),
                    "launch",
                    "display.launch.py",
                ]
            ),
            "use_rviz:=false",
            ["run_headless:=", run_headless],
            "use_localization:=false",
        ],
        shell=False,
        output="screen",
    )

    # Map server to load the existing map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': PathJoinSubstitution([FindPackageShare("mobile_base"), "map", "labmap.yaml"])}]
    )

    # Launch navigation with map_server
    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=True",
            ["params_file:=", params_file]
        ],
        shell=False,
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(NotSubstitution(run_headless)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # Ensuring that bringup is fully executed before starting the map server and navigation stack
    after_bringup = RegisterEventHandler(
        OnProcessIO(
            target_action=bringup,
            on_stdout=on_matching_output(
                "Bringup complete",  # You should specify the exact message indicating bringup completion
                [
                    LogInfo(msg="Bringup completed, starting map server and navigation..."),
                    map_server,
                    navigation,
                    rviz_node,
                ],
            ),
        )
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=[FindPackageShare("mobile_base"), "/config/nav2_params.yaml"],
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        ),
        DeclareLaunchArgument(
            name="rvizconfig",
            default_value=[
                FindPackageShare("mobile_base"),
                "/rviz/navigation_config.rviz",
            ],
            description="Absolute path to rviz config file",
        ),
        DeclareLaunchArgument(
            name="run_headless",
            default_value="False",
            description="Start in headless mode and don't start RViz (overrides use_rviz)",
        ),
        bringup,
        after_bringup,
    ])
