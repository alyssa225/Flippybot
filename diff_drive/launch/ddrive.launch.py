import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    pkg_share = FindPackageShare(package='diff_drive').find('diff_drive')
    gz_model_path = os.path.join(pkg_share,'ddrive.urdf')
    os.environ["GAZEBO_MODEL_PATH"] = gz_model_path
    config = os.path.join(
      get_package_share_directory('diff_drive'),
      'config',
      'robot.yaml'
      )
    
    runlaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('diff_drive'),
                'launch/ddrive_rviz.launch.py'
            ])
        ])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command([TextSubstitution(text="xacro "),
                    PathJoinSubstitution(
                    [FindPackageShare("diff_drive"), "urdf/ddrive.urdf.xacro"])])}, config, ]
    )

    launch_gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        [FindPackageShare('ros_ign_gazebo'),
                                        '/launch/ign_gazebo.launch.py']),
                                        launch_arguments={'gz_args':[
                                            FindPackageShare('diff_drive'),'/worlds/ddrive.world'],}.items())
    
    spawn_robot_cmd=Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name','robot',
                    '-x', '1.0',
                    '-y', '-1.0',
                    '-z', '0.6',
                    '-topic', '/robot_description'],
                    output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
          '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
          '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
          '/world/ddrive_world/model/robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'

        ],
        # remappings=[('/joint_states', '/world/ddrive_world/model/robot/joint_state'),],
        output='screen'
    )

    flip = Node(
            package='diff_drive',
            namespace='flip',
            executable='flip',
            parameters=[config]
        )
    
    return LaunchDescription([
        launch_gz,
        runlaunch,
        robot_state_publisher_node,
        bridge,
        spawn_robot_cmd,
        flip
    ])

