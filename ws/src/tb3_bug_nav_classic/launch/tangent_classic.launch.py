from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg = get_package_share_directory('tb3_bug_nav_classic')
    world_default = os.path.join(pkg,'worlds','bug_world.world')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    def setup_spawn_and_node(context):
        sx, sy, gx, gy = 6.27553, -8.98436, -8.65364, 8.61971
        try:
            world_path = world.perform(context)
            with open(world_path, 'r') as f:
                txt = f.read()
            def find_pose(light_name):
                import re
                pattern = rf"<light[^>]*name=['\"]{light_name}['\"][^>]*>[\s\S]*?<pose>([^<]+)</pose>"
                m = re.search(pattern, txt)
                if m:
                    vals = [float(v) for v in m.group(1).strip().split()]
                    return vals[0], vals[1]
                return None
            # START = user_spot_light_0, GOAL = user_spot_light_1
            s = find_pose('user_spot_light_0')
            g = find_pose('user_spot_light_1')
            if s: sx, sy = s
            if g: gx, gy = g
            print(f"[tangent_launch] START from user_spot_light_0: ({sx:.3f}, {sy:.3f})")
            print(f"[tangent_launch] GOAL  from user_spot_light_1: ({gx:.3f}, {gy:.3f})")
        except Exception as e:
            print(f"[tangent_launch] Failed to read world lights: {e}")

        spawn = ExecuteProcess(
            cmd=[
                'ros2','run','gazebo_ros','spawn_entity.py',
                '-entity','waffle',
                '-file', os.path.join(tb3_gz,'models','turtlebot3_waffle','model.sdf'),
                '-x', str(sx), '-y', str(sy), '-z','0.08'
            ],
            output='screen'
        )

        node = Node(
            package='tb3_bug_nav_classic', executable='tangent_bug_node', output='screen',
            parameters=[{
                'goal_x': gx,
                'goal_y': gy,
            }]
        )
        echo_state = ExecuteProcess(
            cmd=['ros2','topic','echo','/bug_state'],
            output='screen'
        )
        return [spawn, node, echo_state]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('goal_x', default_value='7.5'),
        DeclareLaunchArgument('goal_y', default_value='-8.5'),
        DeclareLaunchArgument('gui', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros,'launch','gazebo.launch.py')),
            launch_arguments={'world': world, 'gui': gui}.items()
        ),

        TimerAction(period=2.0, actions=[OpaqueFunction(function=setup_spawn_and_node)]),
    ])
