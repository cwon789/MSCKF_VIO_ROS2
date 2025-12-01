from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  robot = LaunchConfiguration("robot")
  return LaunchDescription(
      [
          DeclareLaunchArgument("robot", default_value="fla1"),
          ExecuteProcess(
              cmd=[
                  "ros2",
                  "service",
                  "call",
                  ["/", robot, "/reset"],
                  "std_srvs/srv/Trigger",
                  "{}",
              ],
              output="screen",
          ),
      ]
  )
