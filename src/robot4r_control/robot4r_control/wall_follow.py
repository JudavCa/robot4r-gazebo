#!/usr/bin/env python3
"""
wall_follow.py: seguimiento de pared con LiDAR (dos haces + PD) para robot4r.

Base: método de dos haces de las diapositivas del curso de Robótica Móvil
(M. J. Ortiz Camacho, EIA, 2026-2) y formulación del enunciado del reporte:
    alpha = atan((a cos(th) - b) / (a sin(th)))
    D_t   = b cos(alpha)
    D_t+1 = D_t + L sin(alpha)
    e     = D_t+1 - d_des          (pared izquierda: e > 0 = lejos de la pared)
    w     = Kp e + Kd de/dt
    v     = max(v_min, v_max (1 - |w| / w_max))
Haz b: perpendicular a la pared (+90 deg a la izquierda). Haz a: adelantado th grados.

    ros2 run robot4r_control wall_follow --ros-args -p use_sim_time:=true
Publica en /cmd_vel_auto; el AEB lo lleva a /cmd_vel.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow')

        self.declare_parameter('lado', 'izquierda')   # 'izquierda' | 'derecha'
        self.declare_parameter('d_des', 0.20)         # [m] distancia deseada desde el LiDAR
        self.declare_parameter('theta_deg', 45.0)     # [deg] separación entre los haces a y b
        self.declare_parameter('L', 0.5)              # [m] distancia de anticipación
        self.declare_parameter('kp', 4.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('v_max', 0.5)          # [m/s]
        self.declare_parameter('v_min', 0.15)         # [m/s]
        self.declare_parameter('w_max', 2.0)          # [rad/s]
        self.declare_parameter('promedio', 2)         # haces vecinos a cada lado

        self.s = 1.0 if self.get_parameter('lado').value == 'izquierda' else -1.0
        self.e_prev = None
        self.t_prev = None

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.pub_e = self.create_publisher(Float32, '/wall/error', 10)
        self.pub_d = self.create_publisher(Float32, '/wall/distancia', 10)
        self.get_logger().info(
            'wall_follow | lado %s, d_des %.2f m, theta %.0f deg, L %.2f m, Kp %.1f, Kd %.2f'
            % (self.get_parameter('lado').value, self.get_parameter('d_des').value,
               self.get_parameter('theta_deg').value, self.get_parameter('L').value,
               self.get_parameter('kp').value, self.get_parameter('kd').value))

    def rango_en(self, msg, ang):
        """Rango promedio alrededor del ángulo 'ang' [rad]; range_max si no hay eco válido."""
        n = len(msg.ranges)
        i = int(round((ang - msg.angle_min) / msg.angle_increment))
        k = int(self.get_parameter('promedio').value)
        v = [msg.ranges[(i + j) % n] for j in range(-k, k + 1)]
        v = [r for r in v if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return float(np.mean(v)) if v else msg.range_max

    def scan_cb(self, msg: LaserScan):
        p = self.get_parameter
        th = math.radians(p('theta_deg').value)
        b = self.rango_en(msg, self.s * math.pi / 2.0)           # perpendicular a la pared
        a = self.rango_en(msg, self.s * (math.pi / 2.0 - th))    # adelantado th

        alpha = math.atan2(a * math.cos(th) - b, a * math.sin(th))
        d_t = b * math.cos(alpha)
        d_t1 = d_t + p('L').value * math.sin(alpha)
        e = d_t1 - p('d_des').value                              # e > 0: lejos de la pared

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        de = 0.0
        if self.e_prev is not None and 0.0 < t - self.t_prev < 0.5:
            de = (e - self.e_prev) / (t - self.t_prev)
        self.e_prev, self.t_prev = e, t

        w_max = p('w_max').value
        w = self.s * (p('kp').value * e + p('kd').value * de)
        w = float(np.clip(w, -w_max, w_max))
        v = max(p('v_min').value, p('v_max').value * (1.0 - abs(w) / w_max))

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = w
        self.pub.publish(cmd)
        self.pub_e.publish(Float32(data=float(e)))
        self.pub_d.publish(Float32(data=float(d_t1)))


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollow()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()