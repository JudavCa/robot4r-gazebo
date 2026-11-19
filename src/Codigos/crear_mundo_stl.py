#!/usr/bin/env python3
"""Genera el mundo de Gazebo de una pista a partir de un STL de muros.

    python3 crear_mundo_stl.py ruta/al/archivo.stl [nombre]

Copia el STL a robot4r_sim/meshes/<nombre>.stl y escribe robot4r_sim/worlds/<nombre>.sdf
(detecta mm o m y pone un piso que cubre la pista). Después hay que compilar:
    colcon build --packages-select robot4r_sim
"""
import math
import os
import shutil
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(AQUI, '..', 'robot4r_sim'))

PLANTILLA = """<?xml version="1.0"?>
<!-- Generado por crear_mundo_stl.py a partir de {stl}. El origen del STL es el de la pista. -->
<sdf version="1.9">
  <world name="pista">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sol">
      <pose>0 0 10 0 0 0</pose>
      <direction>0.3 0.3 -1</direction>
    </light>

    <model name="piso">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>{lado} {lado}</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>{lado} {lado}</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="muros">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry>
            <mesh>
              <uri>package://robot4r_sim/meshes/{base}.stl</uri>
              <scale>{e} {e} {e}</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="vis">
          <geometry>
            <mesh>
              <uri>package://robot4r_sim/meshes/{base}.stl</uri>
              <scale>{e} {e} {e}</scale>
            </mesh>
          </geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


def leer_stl(ruta):
    data = open(ruta, 'rb').read()
    n = int.from_bytes(data[80:84], 'little')
    if len(data) == 84 + 50 * n:
        reg = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
        return np.frombuffer(data[84:], dtype=reg)['v'].astype(float).reshape(-1, 3)
    v = [[float(x) for x in l.split()[1:4]]
         for l in data.decode(errors='ignore').splitlines()
         if l.strip().startswith('vertex')]
    return np.array(v)


def main():
    ruta = sys.argv[1]
    base = sys.argv[2] if len(sys.argv) > 2 else 'pista_competencia'
    p = leer_stl(ruta)
    ext = max(np.ptp(p[:, 0]), np.ptp(p[:, 1]))
    escala = 0.001 if ext > 200 else 1.0
    q = p * escala
    mn, mx = q.min(0), q.max(0)
    lado = int(math.ceil(max(mx[0] - mn[0], mx[1] - mn[1]) + 20.0))
    cx, cy = (mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0

    os.makedirs(os.path.join(PKG, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    destino = os.path.join(PKG, 'meshes', base + '.stl')
    if os.path.abspath(ruta) != os.path.abspath(destino):
        shutil.copyfile(ruta, destino)
    with open(os.path.join(PKG, 'worlds', base + '.sdf'), 'w') as f:
        f.write(PLANTILLA.format(stl=os.path.basename(ruta), cx=cx, cy=cy,
                                 lado=lado, base=base, e=escala))

    print('unidades detectadas: %s (escala %s)'
          % ('milímetros' if escala != 1.0 else 'metros', escala))
    print('triángulos: %d' % (len(p) // 3))
    print('x de %.2f a %.2f m, y de %.2f a %.2f m, z de %.2f a %.2f m'
          % (mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))
    print('piso: %d x %d m centrado en (%.2f, %.2f)' % (lado, lado, cx, cy))
    print('escrito: worlds/%s.sdf y meshes/%s.stl' % (base, base))
    if mx[2] < 0.25:
        print('AVISO: la altura máxima (%.2f m) es menor que 0.25 m' % mx[2])
    if abs(mn[2]) > 0.01:
        print('AVISO: el STL no empieza en z = 0 (%.3f m); la malla flota o se hunde' % mn[2])
    if len(p) // 3 > 50000:
        print('AVISO: más de 50 000 triángulos; la colisión puede volver lenta la simulación')
    if not (mn[0] <= 0.0 <= mx[0] and mn[1] <= 0.0 <= mx[1]):
        print('AVISO: el origen (0, 0) cae fuera de la pista; revisa dónde está la línea de salida')


main()