#!/usr/bin/env python3
"""
generar_pista.py - Genera worlds/pista.sdf para robot4r.

Uso:  python3 generar_pista.py  ->  escribe pista.sdf (y pista_preview.png si hay matplotlib)

La pista se define por su línea central como una secuencia de secciones
(rectas y arcos) con un ancho de pasillo que puede variar linealmente.
Los muros son las curvas paralelas a la línea central a +-ancho/2.
"""
import math

# ===================== PARÁMETROS DE DISEÑO =====================
ANCHO_ROBOT = 0.16     # m, vía 0.137 + ancho de rueda 0.018
W = 0.70               # ancho de pasillo normal  (~4.4 x robot)
N = 0.40               # ancho del estrechamiento (~2.5 x robot)
R = 0.80               # radio de curva (línea central)
ALTO_MURO = 0.30       # m, mayor que la altura del LiDAR (~0.11 m)
ESP_MURO = 0.05        # m, espesor
ALTO_OBST = 0.25       # m
PASO_ARCO_DEG = 5.0    # discretización de las curvas
SPAWN = (0.5, 0.0, 0.0)  # x, y, yaw (coincide con robot4r.launch.py)


def S(largo, w0=W, w1=None):
    """Recta de longitud 'largo'; el ancho va de w0 a w1."""
    return dict(tipo='S', L=largo, k=0.0, w0=w0, w1=w0 if w1 is None else w1, deg=0.0)


def A(radio, deg, w0=W, w1=None):
    """Arco de radio 'radio'; deg > 0 gira a la izquierda, deg < 0 a la derecha."""
    signo = 1.0 if deg > 0 else -1.0
    return dict(tipo='A', L=radio * math.radians(abs(deg)), k=signo / radio,
                w0=w0, w1=w0 if w1 is None else w1, deg=abs(deg))


# ===================== TRAZADO (línea central) =====================
SECCIONES = [
    S(2.0),                 # 0  salida recta (spawn en x = 0.5)
    A(R, +90),              # 1  curva izquierda
    S(1.5),                 # 2
    A(R, -90),              # 3  curva derecha
    S(1.8),                 # 4  zona de obstáculos 1
    S(1.0, W, N),           # 5  entrada al estrechamiento
    S(0.6, N),              # 6  estrechamiento
    S(1.0, N, W),           # 7  salida del estrechamiento
    A(R, +90),              # 8
    S(1.0),                 # 9
    A(R, +90),              # 10 retorno
    S(1.6),                 # 11 zona de obstáculos 2
    A(1.0, -60),            # 12 slalom suave (derecha)
    A(1.0, +60),            # 13 slalom suave (izquierda)
    S(1.2),                 # 14 meta
]

# (índice de sección, fracción 0..1, desplazamiento lateral (+ izquierda), tipo, tamaño)
OBSTACULOS = [
    (4, 0.30, +0.15, 'caja', 0.15),
    (4, 0.70, -0.15, 'cilindro', 0.08),   # tamaño = radio
    (11, 0.50, +0.15, 'caja', 0.20),
    (14, 0.40, -0.12, 'cilindro', 0.07),
]


# ===================== GEOMETRÍA =====================
def avanzar(pose, sec, d):
    """Pose (x, y, h) tras avanzar una distancia d dentro de la sección."""
    x, y, h = pose
    k = sec['k']
    if abs(k) < 1e-12:
        return (x + d * math.cos(h), y + d * math.sin(h), h)
    phi = d * k
    return (x + (math.sin(h + phi) - math.sin(h)) / k,
            y - (math.cos(h + phi) - math.cos(h)) / k,
            h + phi)


def muestrear():
    """Lista de (x, y, h, ancho) a lo largo de la línea central + poses por sección."""
    pose = (0.0, 0.0, 0.0)
    pts = []
    inicios = []
    for idx, sec in enumerate(SECCIONES):
        inicios.append(pose)
        n = 1 if sec['tipo'] == 'S' else max(1, math.ceil(sec['deg'] / PASO_ARCO_DEG))
        for i in range(0 if idx == 0 else 1, n + 1):
            f = i / n
            p = avanzar(pose, sec, sec['L'] * f)
            pts.append((p[0], p[1], p[2], sec['w0'] + (sec['w1'] - sec['w0']) * f))
        pose = avanzar(pose, sec, sec['L'])
    return pts, inicios


def muros(pts):
    izq = [(x - math.sin(h) * w / 2, y + math.cos(h) * w / 2) for x, y, h, w in pts]
    der = [(x + math.sin(h) * w / 2, y - math.cos(h) * w / 2) for x, y, h, w in pts]
    segs = list(zip(izq[:-1], izq[1:])) + list(zip(der[:-1], der[1:]))
    segs.append((izq[0], der[0]))      # tapa de inicio
    segs.append((izq[-1], der[-1]))    # tapa de meta
    return segs, izq, der


def caja_muro(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    largo = math.hypot(dx, dy) + ESP_MURO   # solape para tapar juntas en curvas
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, math.atan2(dy, dx), largo)


def verificar(pts):
    """Comprobaciones: dentro del piso y sin cruces entre tramos lejanos."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    assert max(map(abs, xs + ys)) < 9.0, 'La pista se sale del piso de 20x20 m'
    acum = [0.0]
    for a, b in zip(pts[:-1], pts[1:]):
        acum.append(acum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    minimo = 1e9
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            if acum[j] - acum[i] > 2.5:
                minimo = min(minimo, math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]))
    print(f'Longitud de la línea central: {acum[-1]:.2f} m')
    print(f'Mínima distancia entre tramos no contiguos: {minimo:.2f} m (debe ser > {W + 2 * ESP_MURO:.2f})')
    assert minimo > W + 2 * ESP_MURO, 'Dos tramos de la pista se acercan demasiado'
    return acum[-1]


# ===================== SDF =====================
CABECERA = """<?xml version="1.0"?>
<!-- pista.sdf generado por generar_pista.py. No editar a mano: cambia los parámetros del script.
     Spawn del robot: x={sx} y={sy} yaw={syaw} (debe coincidir con -x -y del launch). -->
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
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def generar_sdf(segs, obst_poses):
    o = [CABECERA.format(sx=SPAWN[0], sy=SPAWN[1], syaw=SPAWN[2])]

    o.append('    <!-- Muros: un solo modelo estático con una caja de colisión por tramo -->')
    o.append('    <model name="muros">\n      <static>true</static>\n      <link name="link">')
    for i, (a, b) in enumerate(segs):
        cx, cy, yaw, largo = caja_muro(a, b)
        o.append(f'''        <collision name="c{i}">
          <pose>{cx:.4f} {cy:.4f} {ALTO_MURO / 2:.3f} 0 0 {yaw:.5f}</pose>
          <geometry><box><size>{largo:.4f} {ESP_MURO} {ALTO_MURO}</size></box></geometry>
        </collision>
        <visual name="v{i}">
          <pose>{cx:.4f} {cy:.4f} {ALTO_MURO / 2:.3f} 0 0 {yaw:.5f}</pose>
          <geometry><box><size>{largo:.4f} {ESP_MURO} {ALTO_MURO}</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
        </visual>''')
    o.append('      </link>\n    </model>\n')

    for i, ((x, y, h), (tipo, tam)) in enumerate(obst_poses):
        if tipo == 'caja':
            geom = f'<box><size>{tam} {tam} {ALTO_OBST}</size></box>'
        else:
            geom = f'<cylinder><radius>{tam}</radius><length>{ALTO_OBST}</length></cylinder>'
        o.append(f'''    <model name="obstaculo_{i + 1}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {ALTO_OBST / 2:.3f} 0 0 {h:.4f}</pose>
      <link name="link">
        <collision name="col"><geometry>{geom}</geometry></collision>
        <visual name="vis">
          <geometry>{geom}</geometry>
          <material><ambient>0.2 0.4 0.9 1</ambient><diffuse>0.2 0.4 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>
''')

    # Marcas visuales (sin colisión): spawn y meta
    pts_meta = segs[-1]
    mx, my = (pts_meta[0][0] + pts_meta[1][0]) / 2, (pts_meta[0][1] + pts_meta[1][1]) / 2
    myaw = math.atan2(pts_meta[1][1] - pts_meta[0][1], pts_meta[1][0] - pts_meta[0][0])
    o.append(f'''    <model name="marca_spawn">
      <static>true</static>
      <pose>{SPAWN[0]} {SPAWN[1]} 0.002 0 0 {SPAWN[2]}</pose>
      <link name="link"><visual name="vis">
        <geometry><cylinder><radius>0.12</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.1 0.8 0.1 1</ambient><diffuse>0.1 0.8 0.1 1</diffuse></material>
      </visual></link>
    </model>

    <model name="marca_meta">
      <static>true</static>
      <pose>{mx - math.sin(myaw) * 0 + 0.0:.4f} {my:.4f} 0.002 0 0 {myaw:.5f}</pose>
      <link name="link"><visual name="vis">
        <geometry><box><size>0.06 {W} 0.002</size></box></geometry>
        <material><ambient>1 1 0.1 1</ambient><diffuse>1 1 0.1 1</diffuse></material>
      </visual></link>
    </model>
  </world>
</sdf>
''')
    return '\n'.join(o)


def main():
    pts, inicios = muestrear()
    verificar(pts)
    segs, izq, der = muros(pts)

    obst = []
    for idx, frac, lat, tipo, tam in OBSTACULOS:
        sec = SECCIONES[idx]
        x, y, h = avanzar(inicios[idx], sec, sec['L'] * frac)
        ox, oy = x - math.sin(h) * lat, y + math.cos(h) * lat
        obst.append(((ox, oy, h), (tipo, tam)))

    with open('pista.sdf', 'w') as f:
        f.write(generar_sdf(segs, obst))
    print(f'pista.sdf escrito: {len(segs)} tramos de muro, {len(obst)} obstáculos')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        for a, b in segs:
            ax.plot([a[0], b[0]], [a[1], b[1]], 'r-', lw=1.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts], 'k--', lw=0.8, label='línea central')
        for (x, y, _), (tipo, tam) in obst:
            ax.add_patch(plt.Circle((x, y), tam if tipo == 'cilindro' else tam * 0.7, color='b'))
        ax.plot(*SPAWN[:2], 'go', ms=10, label='spawn')
        ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend()
        ax.set_title('Pista (vista superior, metros)')
        fig.savefig('pista_preview.png', dpi=110)
        print('pista_preview.png escrito')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
