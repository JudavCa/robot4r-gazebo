#!/usr/bin/env python3
"""
aeb_node.py: frenado automático de emergencia (AEB) con LiDAR para robot4r.

Base: aeb_node.py del curso de Robótica Móvil (M. J. Ortiz Camacho, EIA, 2026-2).
Cambios respecto a la base:
  1. r_i se mide desde el parachoques: se resta la distancia del LiDAR al borde
     del cuerpo en la dirección de cada rayo.
  2. En modo 'ittc' solo cuentan los rayos cuyo punto de impacto cae dentro del
     corredor de avance (|y_i| < half_width). Con gate:=false se recupera la base.
  3. Parámetros del robot: a_max medida con prueba_limites.py y medidas del STL.

    /cmd_vel_teleop (o el topic al que se remapee) --> [aeb_node] --> /cmd_vel

    ros2 run robot4r_control aeb_node --ros-args -p use_sim_time:=true

Física (ver diapositivas de clase): r_dot_i = -v cos(th_i), iTTC_i = r_i / max(-r_dot_i, 0).
Umbral mínimo: TTC_min = t_lat + v / (2 a).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class AEBNode(Node):
    def __init__(self):
        super().__init__('aeb_node')

        self.declare_parameter('mode', 'ittc')          # 'ittc' | 'corridor'
        self.declare_parameter('gate', True)            # filtrar rayos fuera del corredor
        self.declare_parameter('ttc_threshold', 0.25)   # [s] umbral base
        self.declare_parameter('a_max', 2.0)            # [m/s^2] medida en prueba_limites.py
        self.declare_parameter('t_latency', 0.30)       # [s] ajustado con la prueba del muro (holgura 44 mm a 0.5 m/s)
        self.declare_parameter('half_width', 0.11)      # [m] semiancho 0.0765 + margen
        self.declare_parameter('front_offset', 0.086)   # [m] LiDAR -> frente del chasis
        self.declare_parameter('rear_offset', 0.113)    # [m] LiDAR -> parte trasera
        self.declare_parameter('side_offset', 0.0765)   # [m] LiDAR -> lado del cuerpo
        self.declare_parameter('v_min', 0.01)           # [m/s]
        self.declare_parameter('rate', 20.0)            # [Hz]
        self.declare_parameter('mask_deg', [-163.0, -157.0])  # eco propio (antena), medido en el spawn

        self.r = None
        self.th = None
        self.d_body = None
        self.v_meas = 0.0
        self.cmd = Twist()

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.declare_parameter('modo', 'teleop')      # 'teleop' | 'auto'
        topic_cmd = '/cmd_vel_auto' if self.get_parameter('modo').value == 'auto' else '/cmd_vel_teleop'
        self.create_subscription(Twist, topic_cmd, self.cmd_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_active = self.create_publisher(Bool, '/aeb/active', 10)
        self.pub_ttc = self.create_publisher(Float32, '/aeb/ttc', 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.loop)
        self.get_logger().info(
            'AEB listo | modo %s, gate %s, umbral base %.2f s, a_max %.1f m/s2'
            % (self.get_parameter('mode').value, self.get_parameter('gate').value,
               self.get_parameter('ttc_threshold').value,
               self.get_parameter('a_max').value))

    # ------------------------------------------------------------------ callbacks
    def scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        th = msg.angle_min + np.arange(r.size) * msg.angle_increment
        lo, hi = self.get_parameter('mask_deg').value
        propio = (np.degrees(th) >= lo) & (np.degrees(th) <= hi)   # eco del propio robot
        valid = (np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
                 & ~propio)
        r, th = r[valid], th[valid]

        c, s = np.cos(th), np.sin(th)
        fo = self.get_parameter('front_offset').value
        ro = self.get_parameter('rear_offset').value
        so = self.get_parameter('side_offset').value
        d_long = np.where(c >= 0.0, fo, ro) / np.maximum(np.abs(c), 1e-9)
        d_lat = so / np.maximum(np.abs(s), 1e-9)

        self.r, self.th = r, th
        self.d_body = np.minimum(d_long, d_lat)   # LiDAR -> borde del cuerpo en cada rayo

    def odom_cb(self, msg: Odometry):
        self.v_meas = msg.twist.twist.linear.x

    def cmd_cb(self, msg: Twist):
        self.cmd = msg

    # ------------------------------------------------------------------ TTC
    def compute_ttc(self, v: float) -> float:
        """Tiempo mínimo a colisión si el robot avanza con velocidad lineal v."""
        if self.r is None or self.r.size == 0 or abs(v) < self.get_parameter('v_min').value:
            return math.inf
        hw = self.get_parameter('half_width').value

        if self.get_parameter('mode').value == 'ittc':
            closing = v * np.cos(self.th)                 # = -r_dot
            mask = closing > 0.0                          # solo rayos que se acercan
            if self.get_parameter('gate').value:
                mask &= np.abs(self.r * np.sin(self.th)) < hw
            if not mask.any():
                return math.inf
            d = np.clip(self.r[mask] - self.d_body[mask], 0.0, None)   # desde el parachoques
            return float(np.min(d / closing[mask]))

        # Modo 'corridor' (base de clase): puntos dentro de la huella del robot
        x = self.r * np.cos(self.th)
        y = self.r * np.sin(self.th)
        if v > 0:
            mask = (x > 0.0) & (np.abs(y) < hw)
            d = x[mask] - self.get_parameter('front_offset').value
        else:
            mask = (x < 0.0) & (np.abs(y) < hw)
            d = -x[mask] - self.get_parameter('rear_offset').value
        if not mask.any():
            return math.inf
        return float(np.min(np.clip(d, 0.0, None)) / abs(v))

    def threshold(self, v: float) -> float:
        """Umbral dinámico: nunca menor que t_lat + |v| / (2a)."""
        a = self.get_parameter('a_max').value
        t_lat = self.get_parameter('t_latency').value
        return max(self.get_parameter('ttc_threshold').value, t_lat + abs(v) / (2.0 * a))

    # ------------------------------------------------------------------ lazo
    def loop(self):
        v_cmd = self.cmd.linear.x
        ttc_meas = self.compute_ttc(self.v_meas)   # ¿ya voy hacia algo?
        ttc_cmd = self.compute_ttc(v_cmd)          # ¿el comando me lleva hacia algo?

        emergency = ttc_meas < self.threshold(self.v_meas)
        unsafe_cmd = ttc_cmd < self.threshold(v_cmd)

        # Se permite el comando lineal solo si es seguro y, en emergencia,
        # solo si va en sentido contrario al movimiento actual (alejarse).
        allow_linear = (not unsafe_cmd) and not (emergency and v_cmd * self.v_meas > 0.0)

        out = Twist()
        out.linear.x = v_cmd if allow_linear else 0.0
        out.angular.z = self.cmd.angular.z         # el giro no cambia el rango
        self.pub_cmd.publish(out)

        active = emergency or (unsafe_cmd and abs(v_cmd) > 0.0)
        self.pub_active.publish(Bool(data=bool(active)))
        self.pub_ttc.publish(Float32(data=float(min(ttc_meas, ttc_cmd, 10.0))))

        if active:
            self.get_logger().warn('AEB ACTIVO  TTC=%.2f s' % min(ttc_meas, ttc_cmd),
                                   throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()