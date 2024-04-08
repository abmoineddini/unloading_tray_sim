import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


from launch_ros.actions import Node
import xacro



def generate_launch_description():

    # # Specify the name of the package and path to xacro file within the package
    pkg_name = 'unloading_simulation'
    file_subpath = 'urdf/camera_stand.urdf.xacro'


    # # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # # Configure the node
    node_robot_state_publisher = Node(
        name='robot_state_publisher1',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    blue_tray_gazebo_world = PathJoinSubstitution([FindPackageShare('unloading_simulation'), 'world', 'blue_tray.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': blue_tray_gazebo_world,
            'server_required': 'true',
            'gui_required': 'true',
        }.items(),
    )

 

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'camera_stand'],
                    output='screen')




    # Run the node
    return LaunchDescription([
        # gazebo,
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
    ])