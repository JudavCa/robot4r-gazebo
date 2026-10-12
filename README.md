# robot4r: robot de cuatro ruedas en Gazebo Sim (ROS 2 Jazzy)

Reporte de Robótica, Universidad EIA. Robot móvil con LiDAR y cámara simulado en
Gazebo Sim, conectado a ROS 2 con `ros_gz_bridge` y manejado con teleoperación por teclado.

## Paquetes

| Paquete | Contenido |
|---|---|
| `Modelo_3D_STEP_Corregido_description` | URDF exportado del CAD y mallas STL. De aquí se usan las mallas. |
| `robot4r_sim` | Xacro parametrizado, mundo, bridge, configuración de RViz2 y launch. |

## Dependencias

Probado con ROS 2 Jazzy y Gazebo Sim 8.11.0 en Ubuntu (WSL).

Paquetes apt de Jazzy: `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`,
`ros-jazzy-robot-state-publisher`, `ros-jazzy-rviz2`, `ros-jazzy-xacro`,
`ros-jazzy-teleop-twist-keyboard`.

## Compilar y ejecutar

```bash
cd <raíz del workspace>
colcon build
source install/setup.bash
ros2 launch robot4r_sim robot4r.launch.py
```

El launch levanta Gazebo con la pista, el spawn del robot, el bridge,
`robot_state_publisher` y RViz2. Para manejar el robot, en otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Baja las velocidades con `z` y frena con `k` antes de cerrar con `Ctrl+C`.

## Modelo

- Parámetros en `robot4r_sim/urdf/robot4r.urdf.xacro`: radio de rueda `a = 0.024 m`,
  semi-distancia entre ejes `l = 0.0475 m` y semi-vía `b = 0.0685 m`.
- `base_link` es el marco ROS (+X adelante, +Y izquierda). `cad_link` es el marco
  original del CAD, girado 180° respecto a `base_link`.
- El plugin DiffDrive maneja las cuatro ruedas (dos por lado) con
  `wheel_separation = 2b`.

## Topics puenteados

`/cmd_vel` (ROS a Gazebo) y `/odom`, `/tf`, `/joint_states`, `/clock`, `/scan`,
`/image_raw`, `/camera_info` (Gazebo a ROS).

`/joint_states` publica las ruedas en orden alfabético por nombre. Los nodos deben
buscarlas por nombre y no por posición.

## Notas

- Si la simulación corre más lenta que el tiempo real, los nodos con temporizadores
  deben usar `use_sim_time:=true`. El launch ya lo activa para sus nodos.
- `colcon build` avisa por las mayúsculas en el nombre del paquete de descripción.
  Es solo una advertencia.

## Estado

- Hecho: modelo, tracción, sensores, bridge, RViz2, launch único y teleoperación.
- Pendiente: pista propia, nodo de cinemática de las llantas y justificación de
  parámetros de los sensores.
- La pista actual es el pasillo recto de ejemplo de la guía del curso.