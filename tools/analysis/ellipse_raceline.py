#!/usr/bin/env python3
"""Fit an ellipse to a map's keyframe path and write a raceline (map frame).

Usage:
  sm-cli export-keyframes map.simplemap --output keyframes.tum
  ellipse_raceline.py keyframes.tum raceline_ellipse.csv raceline_ellipse.png

CSV columns: x,y,z,vx,vy,ax,ay,heading (map frame; heading in deg from the map
x axis, CCW +). Speed limits are the V_MAX / A_LAT / A_LON constants below.
"""
import sys, math, numpy as np
kf_file, out_csv, out_png = sys.argv[1:4]
V_MAX, A_LAT, A_LON, DS = 5.0, 2.0, 1.0, 0.2          # m/s, m/s^2, m/s^2, m
k = np.loadtxt(kf_file); P = k[:, 1:3]; Z = k[:, 3]
# geometric least-squares ellipse fit: distance from each keyframe to the ellipse
from scipy.optimize import least_squares
x, y = P[:, 0], P[:, 1]
tt = np.linspace(0, 2*math.pi, 720, endpoint=False)
def pts(q):
    cx, cy, ra, rb, th = q
    return (cx + ra*np.cos(tt)*math.cos(th) - rb*np.sin(tt)*math.sin(th),
            cy + ra*np.cos(tt)*math.sin(th) + rb*np.sin(tt)*math.cos(th))
def res(q):
    ex, ey = pts(q)
    return np.min(np.hypot(x[:, None] - ex[None], y[:, None] - ey[None]), axis=1)
m = P.mean(0); U, sv, Vt = np.linalg.svd(P - m, full_matrices=False)
q0 = [m[0], m[1], 1.414*sv[0]/math.sqrt(len(P)), 1.414*sv[1]/math.sqrt(len(P)), math.atan2(Vt[0, 1], Vt[0, 0])]
q = least_squares(res, q0, loss='soft_l1', f_scale=2.0).x
cx, cy, ra, rb, theta = q
if rb > ra: ra, rb, theta = rb, ra, theta + math.pi/2
theta = (theta + math.pi/2) % math.pi - math.pi/2
# driving direction from the keyframe path (signed area)
area = 0.5*np.sum(x[:-1]*y[1:] - x[1:]*y[:-1]); ccw = area > 0
# dense parametric samples, then resample by arc length
t = np.linspace(0, 2*math.pi, 20000, endpoint=False)
if not ccw: t = -t
ex = cx + ra*np.cos(t)*math.cos(theta) - rb*np.sin(t)*math.sin(theta)
ey = cy + ra*np.cos(t)*math.sin(theta) + rb*np.sin(t)*math.cos(theta)
seg = np.hypot(np.diff(np.append(ex, ex[0])), np.diff(np.append(ey, ey[0]))); s = np.concatenate([[0], np.cumsum(seg)]); L = s[-1]
n = int(round(L / DS)); sq = np.arange(n) * (L / n)
X = np.interp(sq, s, np.append(ex, ex[0])); Y = np.interp(sq, s, np.append(ey, ey[0]))
# start the raceline at the ellipse point closest to the first keyframe
i0 = np.argmin(np.hypot(X - P[0, 0], Y - P[0, 1])); X, Y = np.roll(X, -i0), np.roll(Y, -i0)
ds = L / n
dX = (np.roll(X, -1) - np.roll(X, 1)) / (2*ds); dY = (np.roll(Y, -1) - np.roll(Y, 1)) / (2*ds)
ddX = (np.roll(X, -1) - 2*X + np.roll(X, 1)) / ds**2; ddY = (np.roll(Y, -1) - 2*Y + np.roll(Y, 1)) / ds**2
heading = np.arctan2(dY, dX); kappa = (dX*ddY - dY*ddX) / (dX**2 + dY**2)**1.5
# speed: lateral limit, then forward/backward passes with A_LON on the closed loop
v = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(np.abs(kappa), 1e-6)))
for _ in range(3):
    for i in range(2*n):
        j = i % n; v[j] = min(v[j], math.sqrt(v[j-1]**2 + 2*A_LON*ds))
    for i in range(2*n, 0, -1):
        j = i % n; v[j] = min(v[j], math.sqrt(v[(j+1) % n]**2 + 2*A_LON*ds))
a_t = v * (np.roll(v, -1) - np.roll(v, 1)) / (2*ds)        # tangential accel = v dv/ds
a_n = v*v*kappa                                            # normal accel (left +)
c, sn = np.cos(heading), np.sin(heading)
VX, VY = v*c, v*sn
AX, AY = a_t*c - a_n*sn, a_t*sn + a_n*c
# z: from the nearest keyframe (base_link at ground level)
idx = np.argmin(np.hypot(X[:, None] - P[None, :, 0], Y[:, None] - P[None, :, 1]), axis=1); Zr = Z[idx]
H = np.degrees(heading)
with open(out_csv, 'w') as f:
    f.write('# raceline (ellipse fitted to clean map keyframe path), frame: map\n')
    f.write(f'# center ({cx:.3f}, {cy:.3f}) m, semi-axes {ra:.3f} / {rb:.3f} m, major-axis angle {math.degrees(theta):.2f} deg, '
            f'{"CCW" if ccw else "CW"}, length {L:.2f} m, ds {ds:.3f} m\n')
    f.write(f'# speed limits: v_max {V_MAX} m/s, lateral {A_LAT} m/s^2, longitudinal {A_LON} m/s^2\n')
    f.write('# vx,vy,ax,ay: map-frame components [m/s, m/s^2]; heading: deg from map x axis, CCW +, (-180,180]\n')
    f.write('x,y,z,vx,vy,ax,ay,heading\n')
    for r in zip(X, Y, Zr, VX, VY, AX, AY, H):
        f.write(','.join(f'{q:.4f}' for q in r) + '\n')
d = np.min(np.hypot(P[:, None, 0] - X[None], P[:, None, 1] - Y[None]), axis=1)
print(f'center ({cx:.2f},{cy:.2f}) semi-axes {ra:.2f}/{rb:.2f} m angle {math.degrees(theta):.1f} deg, {"CCW" if ccw else "CW"}, length {L:.1f} m, {n} points')
print(f'keyframe distance to ellipse: p50 {np.median(d):.2f} p90 {np.percentile(d,90):.2f} max {d.max():.2f} m')
print(f'speed {v.min():.2f}..{v.max():.2f} m/s, |kappa| max {np.abs(kappa).max():.4f} 1/m (R min {1/np.abs(kappa).max():.1f} m), |a| max {np.hypot(AX,AY).max():.2f}, lap time {np.sum(ds/v):.1f} s')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
ax[0].plot(P[:, 0], P[:, 1], '.', ms=3, c='gray', label='clean map keyframes')
sc = ax[0].scatter(X, Y, c=v, s=4, cmap='viridis'); plt.colorbar(sc, ax=ax[0], label='speed [m/s]')
for i in range(0, n, n // 12): ax[0].arrow(X[i], Y[i], 3*c[i], 3*sn[i], head_width=1.2, color='r')
ax[0].plot(cx, cy, 'k+', ms=12); ax[0].set_aspect('equal'); ax[0].grid(); ax[0].legend(); ax[0].set_title('raceline (map frame)')
sq2 = np.arange(n) * ds
ax[1].plot(sq2, v, label='speed'); ax[1].plot(sq2, np.hypot(AX, AY), label='|a|'); ax[1].plot(sq2, H/60, label='heading/60 [deg]')
ax[1].set_xlabel('s [m]'); ax[1].grid(); ax[1].legend()
plt.tight_layout(); plt.savefig(out_png, dpi=80)
