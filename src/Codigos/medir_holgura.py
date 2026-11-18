#!/usr/bin/env python3
"""Holgura mínima entre el chasis y los obstáculos, con el robot quieto.

Promedia N barridos por rayo (el LiDAR tiene ruido gaussiano de 0.01 m) y calcula
la distancia de cada punto a la caja del chasis vista desde el LiDAR.

    python3 medir_holgura.py [N]
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

FRENTE, ATRAS, LADO = 0.086, 0.113, 0.0765   # LiDAR -> borde del chasis [m]


class Medir(Node):
    def __init__(self, n):
        super().__init__('medir_holgura')
        self.n = n
        self.datos = []
        self.msg = None
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if len(self.datos) < self.n:
            r = np.array(msg.ranges, dtype=float)
            r[~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)] = np.nan
            self.datos.append(r)
            self.msg = msg


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rclpy.init()
    nodo = Medir(n)
    while rclpy.ok() and len(nodo.datos) < n:
        rclpy.spin_once(nodo, timeout_sec=1.0)
    msg = nodo.msg
    nodo.destroy_node()
    rclpy.try_shutdown()

    datos = np.array(nodo.datos)
    with np.errstate(all='ignore'):
        r = np.nanmean(datos, axis=0)
        sd = np.nanstd(datos, axis=0)
    th = msg.angle_min + np.arange(r.size) * msg.angle_increment
    x, y = r * np.cos(th), r * np.sin(th)
    dx = np.maximum(np.maximum(-ATRAS - x, x - FRENTE), 0.0)
    dy = np.maximum(np.abs(y) - LADO, 0.0)
    dist = np.hypot(dx, dy)
    dist[~np.isfinite(r)] = np.inf
    dist[(np.degrees(th) >= -163.0) & (np.degrees(th) <= -157.0)] = np.inf   # eco propio

    i = int(np.argmin(dist))
    f = int(np.argmin(np.abs(th)))
    print('barridos promediados: %d' % n)
    print('holgura mínima: %.1f mm, en el rayo a %.1f deg'
          % (dist[i] * 1000, math.degrees(th[i])))
    print('ruido de ese rayo (desv. est. entre barridos): %.1f mm' % (sd[i] * 1000))
    print('rayo frontal: %.1f mm desde el parachoques' % ((r[f] - FRENTE) * 1000))
    cerca = np.where(dist < 0.03)[0]
    print('rayos a menos de 30 mm del chasis [deg]:',
          [round(math.degrees(th[k]), 1) for k in cerca])


main()