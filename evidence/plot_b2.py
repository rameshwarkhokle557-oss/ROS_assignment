"""Plot the B2 square trajectories (ground truth) from b2_open.csv and b2_closed.csv."""
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

here = os.path.dirname(os.path.abspath(__file__))


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


fig, ax = plt.subplots(figsize=(6, 6))
for mode, color in (('open', 'tab:red'), ('closed', 'tab:green')):
    d = load(os.path.join(here, f'b2_{mode}.csv'))
    x0, y0, yaw0 = d['truth_x'][0], d['truth_y'][0], d['truth_yaw'][0]
    c, s = math.cos(yaw0), math.sin(yaw0)
    # express every point in the robot's start frame so both runs are comparable
    xs = [c * (x - x0) + s * (y - y0) for x, y in zip(d['truth_x'], d['truth_y'])]
    ys = [-s * (x - x0) + c * (y - y0) for x, y in zip(d['truth_x'], d['truth_y'])]
    ax.plot(xs, ys, color=color, label=f'{mode} loop')
    ax.plot(xs[-1], ys[-1], 'x', color=color, markersize=10)

ax.plot(0, 0, 'ko', label='start')
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('x (m), start frame')
ax.set_ylabel('y (m), start frame')
ax.set_title('B2: 1 m square, ground-truth trajectory (x = end point)')
ax.legend()
fig.savefig(os.path.join(here, 'b2_trajectory.png'), dpi=150)
print('saved', os.path.join(here, 'b2_trajectory.png'))
