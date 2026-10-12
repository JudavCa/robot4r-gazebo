import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot4r_sim')
    desc = get_package_share_directory('Modelo_3D_STEP_Corregido_description')

    xacro_path = os.path.join(pkg, 'urdf', 'robot4r.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'pista.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'robot4r.rviz')

    # Gazebo resuelve package://Modelo_3D_STEP_Corregido_description/meshes/...
    # buscando en esta carpeta (el directorio que contiene al paquete instalado).
    recursos = os.path.dirname(desc)

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r', world_path],
            additional_env={'GZ_SIM_RESOURCE_PATH': recursos},
            output='screen'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen'),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-topic', 'robot_description', '-name', 'robot4r',
                       '-x', '0.5', '-y', '0', '-z', '0.06'],
            output='screen'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])