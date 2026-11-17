#!/usr/bin/env python3
"""
prueba_traccion.py: compara /odom (DiffDrive) con la pose real de Gazebo.

Conduce el robot por su cuenta: una recta y un giro en el sitio. Para cada
maniobra detiene el robot cuando /odom indica la distancia o el ángulo pedidos,
espera a que se frene y lee la pose real del modelo en Gazebo. Al final imprime
la comparación.

Uso (simulación corriendo, workspace cargado, teleop cerrado):

    python3 prueba_traccion.py                       # 1 m de recta y 90 grados
    python3 prueba_traccion.py --distancia 2 --angulo 0
    python3 prueba_traccion.py --distancia 0 --angulo 150 --w 0.3

El tiempo de espera usa las estampas de /odom (tiempo de simulación), así que no
depende del factor de tiempo real.
"""
import argparse
import math
import re
import subprocess
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

MODELO = 'robot4r'
TOPICO_GZ = '/world/pista/dynamic_pose/info'


def envolver(a):
    """Ángulo en [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parsear_pose(texto, nombre):
    """Extrae (x, y, z, yaw) del modelo `nombre` del texto de `gz topic -e`.

    Los campos con valor cero no aparecen en la salida de protobuf, así que un
    campo ausente vale 0. Si falta todo el bloque `orientation`, es la identidad.
    """
    bloques = re.split(r'\npose \{', '\n' + texto)
    for b in bloques[1:]:
        m = re.search(r'name: "([^"]*)"', b)
        if not m or m.group(1) != nombre:
            continue

        def valor(seccion, campo):
            s = re.search(seccion + r' \{([^}]*)\}', b)
            if not s:
                return None
            f = re.search(r'\b' + campo + r': (\S+)', s.group(1))
            return float(f.group(1)) if f else 0.0

        x = valor('position', 'x') or 0.0
        y = valor('position', 'y') or 0.0
        z = valor('position', 'z') or 0.0
        qx = valor('orientation', 'x')
        if qx is None:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qy = valor('orientation', 'y')
            qz = valor('orientation', 'z')
            qw = valor('orientation', 'w')
        return x, y, z, yaw_de_quat(qx, qy, qz, qw)
    raise RuntimeError(f'No encontré el modelo "{nombre}" en la salida de gz')


def leer_gz():
    r = subprocess.run(['gz', 'topic', '-e', '-t', TOPICO_GZ, '-n', '1'],
                       capture_output=True, text=True, timeout=60)
    return parsear_pose(r.stdout, MODELO)


class Prueba(Node):
    def __init__(self):
        super().__init__('prueba_traccion')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.x = self.y = self.yaw = 0.0
        self.yaw_acum = 0.0
        self._yaw_prev = None
        self.t = None

    def cb_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = yaw_de_quat(q.x, q.y, q.z, q.w)
        if self._yaw_prev is None:
            self.yaw_acum = yaw
        else:
            self.yaw_acum += envolver(yaw - self._yaw_prev)
        self._yaw_prev = yaw
        self.x, self.y, self.yaw = p.position.x, p.position.y, yaw
        self.t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def paso(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)

    def esperar_odom(self):
        while self.t is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def frenar(self, espera_sim=2.0):
        """Frena y espera `espera_sim` s de tiempo de simulación."""
        for _ in range(5):
            self.paso(0.0, 0.0)
        t0 = self.t
        while self.t - t0 < espera_sim:
            self.paso(0.0, 0.0)

    def recta(self, distancia, v):
        x0, y0 = self.x, self.y
        while True:
            recorrido = math.hypot(self.x - x0, self.y - y0)
            if recorrido >= distancia:
                break
            restante = distancia - recorrido
            vel = v if restante > 0.10 else max(0.03, v * restante / 0.10)
            self.paso(vel, 0.0)
        self.frenar()

    def giro(self, grados, w):
        objetivo = math.radians(abs(grados))
        signo = 1.0 if grados > 0 else -1.0
        y0 = self.yaw_acum
        while True:
            hecho = abs(self.yaw_acum - y0)
            if hecho >= objetivo:
                break
            restante = objetivo - hecho
            vel = w if restante > 0.20 else max(0.10, w * restante / 0.20)
            self.paso(0.0, signo * vel)
        self.frenar()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distancia', type=float, default=1.0, help='recta [m], 0 la omite')
    ap.add_argument('--angulo', type=float, default=90.0, help='giro [grados], 0 lo omite')
    ap.add_argument('--v', type=float, default=0.10, help='velocidad lineal [m/s]')
    ap.add_argument('--w', type=float, default=0.40, help='velocidad angular [rad/s]')
    a = ap.parse_args()
    if abs(a.angulo) > 170:
        sys.exit('Con más de 170 grados el yaw real de Gazebo es ambiguo (da la vuelta). '
                 'Repite la prueba varias veces con ángulos menores.')

    rclpy.init()
    n = Prueba()
    try:
        n.esperar_odom()
        n.frenar(1.0)
        print('Leyendo pose inicial...')
        gz0 = leer_gz()
        od0 = (n.x, n.y, n.yaw_acum)
        print(f'  gazebo: x={gz0[0]:.4f} y={gz0[1]:.4f} yaw={math.degrees(gz0[3]):.2f} deg')
        print(f'  /odom : x={od0[0]:.4f} y={od0[1]:.4f} yaw={math.degrees(od0[2]):.2f} deg')

        gz1, od1 = gz0, od0
        if a.distancia > 0:
            print(f'\nRecta de {a.distancia} m a {a.v} m/s (según /odom)...')
            n.recta(a.distancia, a.v)
            gz1 = leer_gz()
            od1 = (n.x, n.y, n.yaw_acum)

        gz2, od2 = gz1, od1
        if abs(a.angulo) > 0:
            print(f'\nGiro de {a.angulo} grados a {a.w} rad/s (según /odom)...')
            n.giro(a.angulo, a.w)
            gz2 = leer_gz()
            od2 = (n.x, n.y, n.yaw_acum)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            n.paso(0.0, 0.0)
        n.destroy_node()
        rclpy.try_shutdown()

    print('\n================ RESULTADOS ================')
    if a.distancia > 0:
        d_od = math.hypot(od1[0] - od0[0], od1[1] - od0[1])
        d_gz = math.hypot(gz1[0] - gz0[0], gz1[1] - gz0[1])
        lat = (-(gz1[0] - gz0[0]) * math.sin(gz0[3]) + (gz1[1] - gz0[1]) * math.cos(gz0[3]))
        dyaw = math.degrees(envolver(gz1[3] - gz0[3]))
        print('RECTA')
        print(f'  distancia segun /odom   : {d_od:.4f} m')
        print(f'  distancia real (Gazebo) : {d_gz:.4f} m')
        print(f'  diferencia /odom - real : {d_od - d_gz:+.4f} m ({100 * (d_od - d_gz) / d_gz:+.2f} %)')
        print(f'  desviacion lateral real : {lat:+.4f} m')
        print(f'  cambio de yaw real      : {dyaw:+.2f} deg')
    if abs(a.angulo) > 0:
        y_od = math.degrees(od2[2] - od1[2])
        y_gz = math.degrees(envolver(gz2[3] - gz1[3]))
        dc_od = math.hypot(od2[0] - od1[0], od2[1] - od1[1])
        dc_gz = math.hypot(gz2[0] - gz1[0], gz2[1] - gz1[1])
        print('GIRO EN EL SITIO')
        print(f'  giro segun /odom        : {y_od:+.2f} deg')
        print(f'  giro real (Gazebo)      : {y_gz:+.2f} deg')
        print(f'  real / odom             : {y_gz / y_od:.3f}  (1.000 = sin derrape)')
        print(f'  desplazamiento del centro durante el giro: odom {dc_od:.4f} m, real {dc_gz:.4f} m')


if __name__ == '__main__':
    main()
