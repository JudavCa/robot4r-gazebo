#!/usr/bin/env python3
"""
odometria_ruedas.py: odometría a partir de las ruedas y comparación con /odom.

Lee las velocidades de las ruedas de /joint_states, calcula la velocidad del
cuerpo xi = [u, r] con la matriz W (2x4), integra la pose y la compara con la
odometría del plugin DiffDrive.

    ros2 run robot4r_control odometria_ruedas --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Mismo orden del documento de cinemática de cuatro ruedas:
# 1 front_left, 2 rear_left, 3 front_right, 4 rear_right
RUEDAS = ['front_left_wheel_joint', 'rear_left_wheel_joint',
          'front_right_wheel_joint', 'rear_right_wheel_joint']


def envolver(ang):
    """Ángulo en [-pi, pi]."""
    return math.atan2(math.sin(ang), math.cos(ang))


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def matriz_W(a, b):
    """xi = W omega, con xi = [u, r] y omega = [w1, w2, w3, w4].

    Las ruedas izquierdas (1 y 2) están en y = +b en base_link (+X adelante,
    +Y izquierda) y las derechas (3 y 4) en y = -b.
    """
    k = a / 4.0
    c = a / (4.0 * b)
    return [[k, k, k, k],
            [-c, -c, c, c]]


class OdometriaRuedas(Node):
    def __init__(self):
        super().__init__('odometria_ruedas')

        self.a = self.declare_parameter('radio_rueda', 0.024).value   # a [m]
        self.b = self.declare_parameter('semi_via', 0.0685).value     # b [m]
        self.W = matriz_W(self.a, self.b)

        self.pose = None     # (x, y, theta) propia; se inicia con la primera /odom
        self.odom = None     # última (x, y, theta) de /odom
        self.t_prev = None   # estampa de tiempo del mensaje anterior de /joint_states

        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_ruedas', 10)
        self.pub_err = self.create_publisher(Vector3, '/error_odometria', 10)

        self.get_logger().info(
            f'odometria_ruedas | a={self.a} m, b={self.b} m\nW =\n{self.W}')

    def cb_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_de_quat(p.orientation))
        if self.pose is None:
            # Partimos de la pose del plugin, así la comparación mide el
            # error acumulado desde que arrancó este nodo.
            self.pose = self.odom

    def cb_joints(self, msg):
        if self.pose is None:
            return
        if len(msg.velocity) < len(msg.name):
            self.get_logger().warn('/joint_states no trae velocidades',
                                   throttle_duration_sec=5.0)
            return

        # Buscar cada rueda por su nombre: el orden del mensaje es alfabético.
        vel = dict(zip(msg.name, msg.velocity))
        try:
            w = [vel[nombre] for nombre in RUEDAS]
        except KeyError as falta:
            self.get_logger().warn(f'Falta la rueda {falta} en /joint_states',
                                   throttle_duration_sec=5.0)
            return

        # El tiempo sale de la estampa del mensaje (reloj de la simulación).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_prev is None:
            self.t_prev = t
            return
        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0 or dt > 1.0:
            return

        # xi = W omega
        u = sum(self.W[0][i] * w[i] for i in range(4))
        r = sum(self.W[1][i] * w[i] for i in range(4))

        # Integración de la pose (punto medio)
        x, y, th = self.pose
        th_m = th + 0.5 * r * dt
        x += u * math.cos(th_m) * dt
        y += u * math.sin(th_m) * dt
        th = envolver(th + r * dt)
        self.pose = (x, y, th)

        self.publicar(msg.header.stamp, u, r)

    def publicar(self, stamp, u, r):
        x, y, th = self.pose

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(th / 2.0)
        odom.pose.pose.orientation.w = math.cos(th / 2.0)
        odom.twist.twist.linear.x = u
        odom.twist.twist.angular.z = r
        self.pub_odom.publish(odom)

        if self.odom is None:
            return
        # Error = odometría de las ruedas (propia) - odometría del plugin
        ex = x - self.odom[0]
        ey = y - self.odom[1]
        eth = envolver(th - self.odom[2])
        self.pub_err.publish(Vector3(x=ex, y=ey, z=eth))
        self.get_logger().info(
            f'propia ({x:.3f}, {y:.3f}, {math.degrees(th):.1f} deg) | '
            f'plugin ({self.odom[0]:.3f}, {self.odom[1]:.3f}, '
            f'{math.degrees(self.odom[2]):.1f} deg) | '
            f'error pos {math.hypot(ex, ey):.4f} m, ang {math.degrees(eth):.2f} deg',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    nodo = OdometriaRuedas()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()