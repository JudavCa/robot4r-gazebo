#!/usr/bin/env python3
"""Revisa un STL: unidades probables, tamaño, altura y número de triángulos.

    python3 inspeccionar_stl.py ruta/al/archivo.stl
"""
import sys

import numpy as np

ruta = sys.argv[1]
data = open(ruta, 'rb').read()
n = int.from_bytes(data[80:84], 'little')
if len(data) == 84 + 50 * n:
    reg = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
    tri = np.frombuffer(data[84:], dtype=reg)['v']
    formato = 'binario'
else:
    lineas = data.decode(errors='ignore').splitlines()
    v = np.array([[float(x) for x in l.split()[1:4]]
                  for l in lineas if l.strip().startswith('vertex')])
    tri = v.reshape(-1, 3, 3)
    formato = 'ASCII'

p = tri.reshape(-1, 3)
mn, mx = p.min(0), p.max(0)
print('formato: %s, triángulos: %d, tamaño del archivo: %.1f MB'
      % (formato, len(tri), len(data) / 1e6))
print('mínimo  x, y, z: %s' % np.round(mn, 3))
print('máximo  x, y, z: %s' % np.round(mx, 3))
print('tamaño  x, y, z: %s' % np.round(mx - mn, 3))
ext = max(mx[0] - mn[0], mx[1] - mn[1])
if ext > 200:
    print('unidades probables: milímetros (extensión %.0f): habría que escalar por 0.001' % ext)
elif ext > 0.5:
    print('unidades probables: metros (extensión %.2f)' % ext)
else:
    print('extensión %.4f: ¿metros muy pequeños o escala rara?' % ext)