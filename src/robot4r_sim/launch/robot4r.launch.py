import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def lanzar(context, *args, **kwargs):
    modo = LaunchConfiguration('modo').perform(context)
    pista = LaunchConfiguration('pista').perform(context)
    if modo not in ('teleop', 'auto'):
        raise RuntimeError("modo debe ser 'teleop' o 'auto', no '%s'" % modo)

    pkg = get_package_share_directory('robot4r_sim')
    desc = get_package_share_directory('Modelo_3D_STEP_Corregido_description')
    xacro_path = os.path.join(pkg, 'urdf', 'robot4r.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'robot4r.rviz')

    if pista == 'propia':
        world_path = os.path.join(pkg, 'worlds', 'pista_propia.sdf')
        x0, y0, yaw0 = 0.6, 0.0, math.pi     # detrás de la línea (origen del STL); verificar en Gazebo
    elif pista == 'prueba':
        world_path = os.path.join(pkg, 'worlds', 'pista.sdf')   # pista anterior, ya probada
        x0, y0, yaw0 = 0.5, 0.0, 0.0
    elif pista == 'competencia':
        # TODO: ajustar el nombre del paquete y del world cuando llegue la pista del profesor.
        comp = get_package_share_directory('pista_competencia')
        world_path = os.path.join(comp, 'worlds', 'pista_competencia.sdf')
        x0, y0, yaw0 = -0.60, 0.0, 0.0      # spawn indicado en el enunciado
    else:
        raise RuntimeError("pista debe ser 'propia', 'prueba' o 'competencia', no '%s'" % pista)

    # Gazebo resuelve package://Modelo_3D_STEP_Corregido_description/meshes/...
    # buscando en el directorio que contiene al paquete instalado.
    recursos = os.pathsep.join([os.path.dirname(desc), os.path.dirname(pkg)])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    acciones = [
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
                       '-x', str(x0), '-y', str(y0), '-z', '0.06',
                       '-Y', str(yaw0)],
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
        # El AEB va siempre en la cadena: (teleop | auto) -> AEB -> /cmd_vel
        Node(
            package='robot4r_control', executable='aeb_node',
            parameters=[{'use_sim_time': True, 'modo': modo}],
            output='screen'),
    ]
    if modo == 'auto':
        acciones.append(Node(
            package='robot4r_control', executable='wall_follow',
            parameters=[{'use_sim_time': True}],
            output='screen'))
    return acciones


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='teleop',
                              description="'teleop' o 'auto'"),
        DeclareLaunchArgument('pista', default_value='propia',
                              description="'propia' o 'competencia'"),
        OpaqueFunction(function=lanzar),
    ])