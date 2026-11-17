#!/usr/bin/env python3
"""Prueba de límites del DiffDrive. Usar con el launch recién abierto (robot en el spawn).

    python3 ~/prueba_limites.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

FASES = [  # nombre, v [m/s], w [rad/s], duración [s de simulación]
    ('aceleracion', 1.0, 0.0, 2.0),
    ('tope_lineal', 2.0, 0.0, 3.0),
    ('frenado', 0.0, 0.0, 2.0),
    ('tope_angular', 0.0, 6.0, 3.0),
    ('alto', 0.0, 0.0, 2.0),
]


class Prueba(Node):
    def __init__(self):
        super().__init__('prueba_limites')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.i = 0
        self.t0 = None
        self.datos = {f[0]: [] for f in FASES}
        self.fin = False

    def cb(self, msg):
        if self.fin:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        nombre, v, w, dur = FASES[self.i]
        if self.t0 is None:
            self.t0 = t
        self.datos[nombre].append((t - self.t0, msg.pose.pose.position.x,
                                   msg.twist.twist.linear.x,
                                   msg.twist.twist.angular.z))
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)
        if t - self.t0 >= dur:
            self.i += 1
            self.t0 = None
            if self.i >= len(FASES):
                self.pub.publish(Twist())
                self.fin = True


def informe(d):
    a = d['aceleracion']
    t95 = next((t for t, _x, vx, _w in a if vx >= 0.95), None)
    print('aceleracion: tiempo a 0.95 m/s = %s'
          % ('%.2f s' % t95 if t95 is not None else 'no llegó'))

    vmax = max(s[2] for s in d['tope_lineal'])
    print('tope lineal: vmax = %.3f m/s (límite 1.0)' % vmax)

    f = d['frenado']
    x0, v0 = f[0][1], f[0][2]
    parado = next(((t, x) for t, x, vx, _w in f if vx < 0.05), None)
    if parado:
        print('frenado desde %.2f m/s: %.2f s y %.3f m'
              % (v0, parado[0], parado[1] - x0))
    else:
        print('frenado desde %.2f m/s: no se detuvo en 2 s' % v0)

    wmax = max(abs(s[3]) for s in d['tope_angular'])
    print('tope angular: wmax = %.3f rad/s (límite 4.0)' % wmax)


def main():
    rclpy.init()
    nodo = Prueba()
    try:
        while rclpy.ok() and not nodo.fin:
            rclpy.spin_once(nodo, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub.publish(Twist())
        if nodo.fin:
            informe(nodo.datos)
        nodo.destroy_node()
        rclpy.try_shutdown()


main()