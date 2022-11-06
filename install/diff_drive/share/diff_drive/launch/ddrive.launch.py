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


def generate_launch_description():

    pkg_share = FindPackageShare(package='diff_drive').find('diff_drive')
    ddrive_path = 'diff_drive'
    default_model_path = ddrive_path + '/urdf/ddrive.urdf.xacro'
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    gz_model_path = os.path.join(pkg_share,'ddrive.urdf')
    os.environ["GAZEBO_MODEL_PATH"] = gz_model_path
    world_file_name = 'ddrive.world'
    world_path = os.path.join('diff_drive', 'worlds', world_file_name)
    config = os.path.join(
      get_package_share_directory('turtle_brick'),
      'config',
      'turtle.yaml'
      )
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description} ]
    )

    launch_gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                                        [FindPackageShare('ros_ign_gazebo'),
                                        '/launch/ign_gazebo.launch.py']),
                                        launch_arguments={'gz_args':[
                                            FindPackageShare('diff_drive'),'/worlds/ddrive.world']}.items())
    
    spawn_robot_cmd=Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name','robot',
                    '-x', '3.0',
                    '-y', '3.0',
                    '-z', '0.6',
                    '-topic', '/robot_description'],
                    output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/gz_ros/test/serviceclient/world_control@ros_gz_interfaces/srv/ControlWorld',
        ],
        output='screen'
    )

    flip = Node(
            package='diff_drive',
            namespace='flip',
            executable='flip',
            parameters=[config]
        ),
    
    return LaunchDescription([
        launch_gz,
        model_arg,
        bridge,
        robot_state_publisher_node,
        spawn_robot_cmd
    ])

