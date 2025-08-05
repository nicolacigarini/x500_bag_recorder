from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


ARGUMENTS = [DeclareLaunchArgument("agent_namespace", default_value='x500_1', description="Robot #1 namespace")]

def generate_launch_description():
    namespace = LaunchConfiguration("agent_namespace")
    bag_recorder_params_file = PathJoinSubstitution([FindPackageShare("x500_bag_recorder"), "config", "params.yaml"])

    bag_recorder_node = Node(
        package="x500_bag_recorder",
        executable="bag_recorder",
        name="x500_bag_recorder_node",
        parameters=[bag_recorder_params_file],
        namespace=namespace)
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(bag_recorder_node)
    return ld
    