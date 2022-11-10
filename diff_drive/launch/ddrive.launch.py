import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_share = FindPackageShare(package='diff_drive').find('diff_drive')
    gz_model_path = os.path.join(pkg_share, 'ddrive.urdf')
    os.environ["GAZEBO_MODEL_PATH"] = gz_model_path
    view_only = LaunchConfiguration('view_only')
    view_only_arg = DeclareLaunchArgument(name='view_only', default_value='None',
                                          choices=['False', 'True', 'None'],
                                          description='choose which jointstate publisher')
    config = os.path.join(
      get_package_share_directory('diff_drive'),
      'config',
      'robot.yaml')

    runlaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diff_drive'),
                'launch/ddrive_rviz.launch.py'
            ])
        ])
    )

    launch_gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                         [FindPackageShare('ros_ign_gazebo'),
                                          '/launch/ign_gazebo.launch.py']),
                                         launch_arguments={'gz_args':
                                         [FindPackageShare('diff_drive'),
                                          '/worlds/ddrive.world']}.items())

    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot',
                   '-x', '-1.0',
                   '-y', '4.0',
                   '-z', '0.6',
                   '-topic', '/robot_description'],
        output='screen')

    bridge1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
          '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
          '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
          '/world/ddrive_world/model/robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        remappings=[('/world/ddrive_world/model/robot/joint_state', '/joint_states')],
        output='screen',
        condition=IfCondition(PythonExpression([view_only, "==False"]))
    )

    bridge2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen',
        condition=IfCondition(PythonExpression([view_only, "==None", " or ", view_only, "==True"]))
    )

    flip = Node(
            package='diff_drive',
            namespace='flip',
            executable='flip',
            parameters=[config]
        )

    return LaunchDescription([
        view_only_arg,
        launch_gz,
        runlaunch,
        bridge1,
        bridge2,
        spawn_robot_cmd,
        flip
    ])
