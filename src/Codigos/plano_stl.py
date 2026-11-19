#!/usr/bin/env python3
"""Vista superior de un STL (en metros) y datos de piso y muros.

    python3 plano_stl.py ruta/al/archivo.stl
Escribe plano_stl.png en el directorio actual.
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection

ruta = sys.argv[1]
data = open(ruta, 'rb').read()
n = int.from_bytes(data[80:84], 'little')
reg = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
tri = np.frombuffer(data[84:], dtype=reg)['v'].astype(float)
ext = max(np.ptp(tri[:, :, 0]), np.ptp(tri[:, :, 1]))
if ext > 200:
    tri = tri * 0.001          # milímetros -> metros
    print('(se interpretó el STL como milímetros y se pasó a metros)')

z = tri[:, :, 2]
a = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
base = np.all(np.abs(z) < 1e-6, axis=1)
alto = np.all(np.abs(z - z.max()) < 1e-6, axis=1)
print('triángulos horizontales en z = 0: %d, área %.2f m2' % (base.sum(), a[base].sum()))
print('triángulos horizontales en z = %.2f m: %d, área %.2f m2'
      % (z.max(), alto.sum(), a[alto].sum()))
print('(un piso de 14 x 6 m tendría unos 80 m2 en z = 0; solo los muros, mucho menos)')

fig, ax = plt.subplots(figsize=(13, 6))
ax.add_collection(PolyCollection(tri[:, :, :2], facecolors='none',
                                 edgecolors='k', linewidths=0.3))
ax.plot(0, 0, 'ro')
ax.annotate('origen del CAD', (0, 0), color='r')
ax.set_aspect('equal')
ax.autoscale_view()
ax.set_xticks(np.arange(np.floor(tri[:, :, 0].min()), np.ceil(tri[:, :, 0].max()) + 1, 1))
ax.set_yticks(np.arange(np.floor(tri[:, :, 1].min()), np.ceil(tri[:, :, 1].max()) + 1, 1))
ax.grid(alpha=0.3)
ax.set_title('Vista superior del STL (metros)')
fig.savefig('plano_stl.png', dpi=130)
print('plano_stl.png escrito')
horiz = np.all(np.abs(z - z[:, :1]) < 1e-6, axis=1)
zh = np.round(z[horiz, 0], 3)
for nivel in np.unique(zh):
    print('z = %.3f m: %d triángulos, área %.2f m2'
          % (nivel, (zh == nivel).sum(), a[horiz][zh == nivel].sum()))