# robot4r: robot de cuatro ruedas en Gazebo Sim (ROS 2 Jazzy)

Reporte de Robótica, Universidad EIA. Robot móvil con LiDAR y cámara simulado en
Gazebo Sim, conectado a ROS 2 con `ros_gz_bridge` y manejado con teleoperación por teclado.

## Paquetes

| Paquete | Contenido |
|---|---|
| `Modelo_3D_STEP_Corregido_description` | URDF exportado del CAD y mallas STL. De aquí se usan las mallas. |
| `robot4r_sim` | Xacro parametrizado, mundo (pista), generador de la pista, bridge, configuración de RViz2 y launch. |
| `robot4r_control` | Nodo `odometria_ruedas`: integra la pose a partir de las velocidades de las ruedas y la compara con `/odom`. |

## Dependencias

Probado con ROS 2 Jazzy y Gazebo Sim 8.11.0 en Ubuntu (WSL).

Paquetes apt de Jazzy: `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`,
`ros-jazzy-robot-state-publisher`, `ros-jazzy-rviz2`, `ros-jazzy-xacro`,
`ros-jazzy-teleop-twist-keyboard`.

Opcionales:

- `ros-jazzy-tf2-tools`: para generar el árbol de frames con `view_frames`.
- `python3-matplotlib`: solo para la vista previa que genera `generar_pista.py`.

## Compilar y ejecutar

```bash
cd <raíz del workspace>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot4r_sim robot4r.launch.py
```

Con `--symlink-install`, los cambios en el xacro, el bridge, el mundo o el launch
no requieren recompilar: basta relanzar.

Antes de lanzar, conviene comprobar que no haya procesos de una sesión anterior
(un `gz sim` huérfano duplica tópicos y consume CPU):

```bash
ps aux | grep -E "gz|rviz|parameter_bridge|robot_state_publisher" | grep -v grep
```

El launch levanta Gazebo con la pista, el spawn del robot, el bridge,
`robot_state_publisher` y RViz2. Para manejar el robot, en otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Baja las velocidades con `z` y frena con `k` antes de cerrar con `Ctrl+C`.
Una velocidad lineal de 0.10 a 0.15 m/s y angular de unos 0.5 rad/s es cómoda
para el estrechamiento.

## Modelo

- Parámetros en `robot4r_sim/urdf/robot4r.urdf.xacro`: radio de rueda `a = 0.024 m`,
  semi-distancia entre ejes `l = 0.0475 m` y semi-vía `b = 0.0685 m`.
- `base_link` es el marco ROS (+X adelante, +Y izquierda). `cad_link` es el marco
  original del CAD, girado 180° respecto a `base_link`.
- El plugin DiffDrive maneja las cuatro ruedas (dos por lado) con
  `wheel_separation = 2b`. Al ser un robot de cuatro ruedas con dirección por
  diferencia de velocidades (skid-steer), las ruedas derrapan al girar y la
  odometría de DiffDrive no coincide exactamente con la cinemática ideal.

## Sensores

| Sensor | Frame | Tópico | Parámetros actuales |
|---|---|---|---|
| LiDAR (`gpu_lidar`) | `lidar_link` | `/scan` | 360 muestras, FOV 360°, rango 0.1 a 10 m, 10 Hz |
| Cámara | `camera_link` | `/image_raw`, `/camera_info` | 320x240, FOV 1.047 rad, 15 Hz |

Con estos valores, `camera_info` da fx = fy = 277.19 px, que concuerda con
`160 / tan(FOV/2)`.

## Pista

El mundo está en `robot4r_sim/worlds/pista.sdf` y **se genera con
`robot4r_sim/generar_pista.py`**. No se edita a mano: para cambiar la pista se
modifican los parámetros del script y se vuelve a generar.

Diseño (vista superior, unos 9 x 7 m dentro del piso de 20 x 20 m):

| Elemento | Valor |
|---|---|
| Longitud de la línea central | unos 19 m |
| Ancho de pasillo normal | 0.70 m (unas 4 veces el ancho del robot, ≈0.16 m) |
| Estrechamiento | 0.40 m durante 0.6 m, con transiciones de 1.0 m |
| Curvas | cinco de 90° y un slalom suave de ±60°, radio 0.8 a 1.0 m |
| Muros | 0.30 m de alto, 0.05 m de espesor, con colisión |
| Obstáculos | 2 cajas y 2 cilindros de 0.25 m de alto, con paso libre de unos 0.6 m |
| Spawn | (x = 0.5, y = 0, yaw = 0), marcado con un disco verde |
| Meta | marca amarilla al final del último tramo |

La altura de los muros y de los obstáculos supera la del LiDAR (≈0.11 m sobre el
suelo), así que todos son detectados.

Regenerar la pista:

```bash
cd src/robot4r_sim/worlds
python3 ../generar_pista.py     # escribe pista.sdf (y pista_preview.png si hay matplotlib)
```

El script verifica que la pista quepa en el piso y que ningún tramo se acerque a
otro. Si se cambia el spawn, hay que actualizar a la vez `SPAWN` en el script y
`-x -y` en `launch/robot4r.launch.py`.

## Topics puenteados

`/cmd_vel` (ROS a Gazebo) y `/odom`, `/tf`, `/joint_states`, `/clock`, `/scan`,
`/image_raw`, `/camera_info` (Gazebo a ROS).

`/joint_states` publica las ruedas en orden alfabético por nombre. Los nodos deben
buscarlas por nombre y no por posición.

## Cinemática de las llantas

El nodo `odometria_ruedas` (en `robot4r_control`) lee las velocidades de las cuatro
ruedas de `/joint_states`, calcula la velocidad del cuerpo, integra la pose y la
compara con la odometría del plugin DiffDrive.

Con `a` el radio de la rueda y `b` la semi-vía, y las ruedas en el orden
`ω = [ω_FL, ω_RL, ω_FR, ω_RR]` (izquierdas primero):

```
ξ = [u, r]ᵀ = W · ω

W = [  a/4      a/4      a/4     a/4  ]
    [ -a/4b    -a/4b     a/4b    a/4b ]
```

Es equivalente a promediar cada lado, con `ω_I = (ω_FL + ω_RL)/2` y
`ω_D = (ω_FR + ω_RR)/2`:

```
v = a (ω_D + ω_I) / 2          ω = a (ω_D − ω_I) / (2b)
ẋ = v cos θ      ẏ = v sin θ      θ̇ = ω
```

Solo interviene la semi-vía `b`: la semi-distancia entre ejes `l` no aparece en un
robot de este tipo. Con `a = 0.024 m` y `b = 0.0685 m`, `a/4 = 0.006` y
`a/4b = 0.0876`. La pose se integra por el método del punto medio, usando las
estampas de tiempo de `/joint_states`, y arranca desde la primera `/odom`.

Ejecutar (con la simulación corriendo):

```bash
source install/setup.bash
ros2 run robot4r_control odometria_ruedas --ros-args -p use_sim_time:=true
```

Publica `/odom_ruedas` (la odometría propia) y `/error_odometria` (`x`, `y` = error
de posición en metros y `z` = error de orientación en radianes, propia menos plugin).
Cada 2 s imprime un resumen en la terminal.

Resultados medidos con teleoperación (lecturas del resumen, una cada 2 s):

| Prueba | Error durante el movimiento | Error en reposo |
|---|---|---|
| Recta de 1.257 m | entre 0.0005 y 0.015 m | 0.001 m |
| Giro en el sitio de 45.8° | hasta 0.34° | 0.06° |

Observaciones:

- El error durante el movimiento oscila y cambia de signo. Se explica por el
  desfase de sincronización: el nodo compara contra una `/odom` que llega a 30 Hz
  de simulación, mientras `/joint_states` llega mucho más rápido. En reposo, ese
  desfase desaparece y queda el error real entre los dos métodos de integración.
- Las dos odometrías parten de las mismas ruedas, el mismo radio y la misma vía, así
  que coinciden entre sí por construcción. **No detectan el derrape** de las cuatro
  ruedas al girar: para medirlo habría que comparar contra la pose real de Gazebo.

## Verificación rápida

Con la simulación corriendo, en otra terminal con el workspace cargado:

```bash
ros2 topic list
ros2 topic echo /scan --field header.frame_id --once     # lidar_link
ros2 topic echo /camera_info --once
ros2 run tf2_tools view_frames                            # odom -> base_link -> ...
gz model --list                                           # debe incluir robot4r
gz topic -e -t /stats -n 1                                # real_time_factor
```

## Problemas conocidos

- **El robot puede no verse en la ventana principal de Gazebo.** Con la pista
  actual, el robot existe en el servidor (`gz model --list` lo lista y RViz lo
  dibuja) pero a veces no aparece en el Entity Tree del GUI. Abrir una segunda
  ventana con `gz sim -g` con la simulación corriendo lo muestra. La causa exacta
  no está confirmada; una posible mitigación es retrasar el nodo `create` en el
  launch con un `TimerAction`.
- **Factor de tiempo real bajo (RTF ≈ 0.08 a 0.13).** Medido con el robot quieto, la
  simulación corre a una fracción de la velocidad real en WSL. Por eso, en tiempo de
  pared, `/odom` llega a unos 6 Hz (configurado a 30 Hz de simulación) y la cámara a
  unos 7 Hz (configurada a 15 Hz): las tasas son correctas en tiempo de simulación.
  No está confirmada la causa (render por software, carga de CPU del GUI y de RViz, o
  el paso de 1 ms). Los nodos propios usan las estampas de los mensajes y
  `use_sim_time`, así que sus resultados no se ven afectados, solo tardan más en
  obtenerse.
- **Pocas lecturas cortas del LiDAR (≈0.11 m)** a unos −160° en `/scan`: alguna parte
  de la malla del propio robot queda cerca del sensor. No afecta la navegación.
- La cámara usa `camera_link` como frame y no un frame óptico.

## Notas

- Si la simulación corre más lenta que el tiempo real, los nodos con temporizadores
  deben usar `use_sim_time:=true`. El launch ya lo activa para sus nodos.
- `colcon build` avisa por las mayúsculas en el nombre del paquete de descripción.
  Es solo una advertencia.
- Archivos `*:Zone.Identifier` (los crea Windows al copiar archivos a WSL) están en
  `.gitignore` y no deben subirse al repositorio.

## Estado

- Hecho:
  - modelo, tracción, sensores, bridge, RViz2 y launch único;
  - pista propia (curvas, estrechamiento, obstáculos, muros con colisión y spawn definido);
  - teleoperación con recorrido completo de la pista, sin colisiones;
  - nodo de cinemática de las llantas, ejecutado y con errores medidos en recta y en giro.
- Pendiente:
  - justificación de los parámetros de los sensores, de la pista y del modelo en el Notion;
  - (opcional) comparar las odometrías contra la pose real de Gazebo para cuantificar
    el derrape, y guardar los errores en un CSV para reportar el error máximo y el RMSE.
