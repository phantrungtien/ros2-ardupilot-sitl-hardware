#!/usr/bin/env python3
"""N3 voxel map -- the OCCUPANCY MAP the Dijkstra planner (N4) searches on.

Depth turns into a 3D grid of "there is something here" voxels. The planner needs
a graph; this is where the graph's nodes and their traversability come from.

This file is a LIBRARY, not a node. gate_planner.py owns the only node and folds
depth into the grid itself -- see the note above quat_to_matrix for why the mapper
stopped being a separate process.

  VoxelGrid (pure, no ROS) -- the data structure and every operation on it:
    integrate(points_map)  mark voxels that hold a depth point as OCCUPIED
    inflated(r)            grow obstacles by the drone radius so the drone can be
                           treated as a POINT in the graph
    clearance(inflated)    metres from each free voxel to the nearest obstacle
                           (the planner's "stay away from walls" cost)
    carve_corridor(...)    force a cylinder of voxels FREE -- see below
    This is what the self-check exercises.

  backproject_depth / quat_to_matrix -- what the planner's build step calls to turn
    a 32FC1 depth image into map-frame points.

    Why the depth IMAGE, not the ready-made /d435i/depth/points cloud: gz emits that
    cloud with x-FORWARD data while labelling its frame_id optical (z-forward), so
    TF-ing it by that frame_id rotates the map 90 deg. Back-projecting the depth
    image ourselves uses the SAME optical convention N1 verified against ground
    truth -- one convention across N1/N2/N3, no gz quirk to track.

Why carve_corridor exists (plan par. 3.2, the load-bearing trick):
  A gate ring is a THIN obstacle. Inflate it by the 0.6 m robot radius and the
  1.5 m hole shrinks to ~0.3 m -- the planner can no longer thread it and either
  fails or routes around the gate. So the hole is not left to the planner to find:
  for the gate it is targeting, N4 carves a virtual tube (radius ~0.4 m, along the
  ring normal, through the centre) and forces those voxels FREE. The rim stays
  occupied+inflated. carve_corridor is that operation; WHO calls it and for WHICH
  gate is the planner/FSM's call, not the mapper's -- so it lives on the grid but
  is never invoked in the live update loop here.
# ponytail: OCCUPIED-only accumulation, no log-odds ray-casting / free-space
# clearing. The gates course is STATIC -- obstacles never move, so there is
# nothing to clear. Everything not seen-as-a-point is traversable (UNKNOWN, which
# the planner penalises lightly). Add ray-cast clearing only if a moving obstacle
# ever appears; the grid API does not change.

Run:  python3 voxel_map.py --selfcheck      (nothing else to run -- it is a library)
"""
import math

import numpy as np
from scipy import ndimage

# ----- grid defaults (plan par. 3.1 / 9) -----
RES = 0.20                 # m per voxel
ROBOT_RADIUS = 0.40        # m
INFLATE_MARGIN = 0.20      # m
INFLATE_R = ROBOT_RADIUS + INFLATE_MARGIN     # 0.60 m
# Box around the course (iris spawn at origin; gates step east to x~20, y in +-2,
# z to ~5). Generous so nothing the drone flies through falls outside.
GRID_ORIGIN = (-2.0, -6.0, 0.0)               # world coord of the (0,0,0) corner
GRID_DIMS = (26.0, 12.0, 8.0)                 # m -> 130 x 60 x 40 = 312k voxels

BUILD_HZ = 5.0
PIXEL_STRIDE = 4          # back-project every Nth depth pixel (plan: subsample 4x)
DEPTH_TOPIC = '/d435i/depth/image_rect_raw'
MAP_FRAME = 'map'

# Depth intrinsics from realsense_d435i/model.sdf -- KEEP IN SYNC with the SDF and
# with gate_detect (N1), which back-projects with the same numbers. hfov 87 deg.
_W, _H = 848, 480
_CX, _CY = _W / 2.0, _H / 2.0
_FX = _CX / math.tan(1.518436 / 2.0)          # 446.80
_DEPTH_MIN, _DEPTH_MAX = 0.20, 10.0           # SDF clip


def backproject_depth(depth, stride=PIXEL_STRIDE):
    """32FC1 depth image (metres) -> (N,3) points in the DEPTH OPTICAL frame.

    Same convention as N1: X right, Y down, Z forward. Drops pixels outside the
    sensor's valid clip so the ground-plane-at-infinity and holes do not seed
    phantom voxels.
    """
    us = np.arange(0, _W, stride)
    vs = np.arange(0, _H, stride)
    uu, vv = np.meshgrid(us, vs)
    z = depth[vv, uu]
    m = np.isfinite(z) & (z >= _DEPTH_MIN) & (z <= _DEPTH_MAX)
    z, uu, vv = z[m], uu[m], vv[m]
    x = (uu - _CX) * z / _FX
    y = (vv - _CY) * z / _FX
    return np.stack([x, y, z], axis=-1).astype(np.float32)


def _unit(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def _ball(r):
    """Boolean sphere structuring element of radius r voxels."""
    L = np.arange(-r, r + 1)
    x, y, z = np.meshgrid(L, L, L, indexing='ij')
    return (x * x + y * y + z * z) <= r * r


class VoxelGrid:
    def __init__(self, origin=GRID_ORIGIN, dims_m=GRID_DIMS, res=RES):
        self.origin = np.asarray(origin, float)
        self.res = float(res)
        self.shape = tuple(int(math.ceil(d / res)) for d in dims_m)
        self.occ = np.zeros(self.shape, bool)      # True = a depth point landed here

    # --- index <-> world ---
    def world_to_idx(self, pts):
        pts = np.atleast_2d(np.asarray(pts, float))
        idx = np.floor((pts - self.origin) / self.res).astype(np.int64)
        inb = np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)
        return idx, inb

    def idx_to_world(self, idx):
        """Centre of the voxel(s)."""
        return self.origin + (np.asarray(idx, float) + 0.5) * self.res

    # --- build ---
    def integrate(self, pts_map):
        idx, inb = self.world_to_idx(pts_map)
        idx = idx[inb]
        if len(idx):
            self.occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True

    def inflated(self, radius_m=INFLATE_R):
        r = int(round(radius_m / self.res))
        return self.occ.copy() if r < 1 else ndimage.binary_dilation(self.occ, _ball(r))

    def clearance(self, inflated):
        """Metres from each FREE voxel to the nearest inflated obstacle."""
        return ndimage.distance_transform_edt(~inflated) * self.res

    def carve_corridor(self, occ, centre, normal, radius, length):
        """Force voxels inside the cylinder (axis=normal, through centre) FREE.

        Operates in place on `occ` (pass a COPY of an inflated grid at plan time).
        Only touches the cylinder's bounding sub-box, so it is cheap even on a big
        grid. Returns occ.
        """
        centre = np.asarray(centre, float)
        n = _unit(normal)
        half = length / 2.0
        reach = half + radius                       # conservative AABB half-extent
        lo, _ = self.world_to_idx(centre - reach)
        hi, _ = self.world_to_idx(centre + reach)
        lo = np.clip(lo[0], 0, np.array(self.shape) - 1)
        hi = np.clip(hi[0], 0, np.array(self.shape) - 1)
        if np.any(hi < lo):
            return occ
        gi, gj, gk = np.meshgrid(
            np.arange(lo[0], hi[0] + 1), np.arange(lo[1], hi[1] + 1),
            np.arange(lo[2], hi[2] + 1), indexing='ij')
        centres = self.idx_to_world(np.stack([gi, gj, gk], axis=-1))
        rel = centres - centre
        axial = rel @ n
        perp = np.linalg.norm(rel - axial[..., None] * n, axis=-1)
        inside = (np.abs(axial) <= half) & (perp <= radius)
        occ[gi[inside], gj[inside], gk[inside]] = False
        return occ


# =========================================================================
# Grid-building helpers used by the planner node (gate_planner.py). There is no
# mapper NODE here any more: the grid crossed to the planner as an RViz marker
# list, at 5 Hz over a depth-1 topic, and arrived about once every 25 s -- so the
# planner planned against a blank map and never learned better. Nothing outside
# the planner ever read the grid, so it never needed to be a message. gate_planner
# now folds depth in itself and this file is the pure data structure plus the two
# functions that feed it. `ros2 run simtofly_planner gate_planner` is the one node.
# =========================================================================
def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# =========================================================================
# self-check -- pure grid math, no ROS:  python3 voxel_map.py --selfcheck
# =========================================================================
def _selfcheck():
    g = VoxelGrid(origin=(0, 0, 0), dims_m=(4, 4, 4), res=0.2)   # 20^3

    # index round-trip
    idx, inb = g.world_to_idx([[2.0, 2.0, 2.0]])
    assert inb[0] and tuple(idx[0]) == (10, 10, 10), idx
    assert np.allclose(g.idx_to_world(idx[0]), [2.1, 2.1, 2.1]), g.idx_to_world(idx[0])

    # integrate marks the right voxel; out-of-bounds points are dropped silently
    g.integrate([[2.0, 2.0, 2.0], [99, 99, 99]])
    assert g.occ[10, 10, 10] and g.occ.sum() == 1

    # inflation grows the obstacle; clearance is ~0 at it and rises away
    inf = g.inflated(0.4)                     # r = 2 voxels
    assert inf.sum() > 1 and inf[10, 10, 10]
    clr = g.clearance(inf)
    assert clr[10, 10, 10] == 0.0
    assert clr[10, 10, 19] > 1.0              # far corner is well clear

    # --- the load-bearing test: a ring's hole closes under inflation, and
    #     carve_corridor reopens exactly a tube through it (plan par. 3.2) ---
    gr = VoxelGrid(origin=(0, 0, 0), dims_m=(4, 4, 4), res=0.2)
    centre = np.array([2.0, 2.0, 2.0])
    normal = np.array([1.0, 0.0, 0.0])        # hole axis along +X
    # lay an occupied ring (tube centreline r=0.83) in the y-z plane at x=2
    ang = np.linspace(0, 2 * math.pi, 200, endpoint=False)
    ring = np.stack([np.full_like(ang, 2.0),
                     2.0 + 0.83 * np.cos(ang),
                     2.0 + 0.83 * np.sin(ang)], axis=-1)
    gr.integrate(ring)
    c_idx = tuple(gr.world_to_idx([centre])[0][0])
    assert not gr.occ[c_idx], 'ring centre should start free (it is the hole)'

    inf = gr.inflated(0.6)                     # 0.6 m robot radius
    assert inf[c_idx], 'after inflation the 1.5 m hole must close over the centre'

    carved = gr.carve_corridor(inf.copy(), centre, normal, radius=0.4, length=1.0)
    # a line along the hole axis through the centre is now FREE...
    for dx in (-0.4, -0.2, 0.0, 0.2, 0.4):
        p = centre + np.array([dx, 0, 0])
        assert not carved[tuple(gr.world_to_idx([p])[0][0])], \
            'corridor not free at dx={}'.format(dx)
    # ...but the rim, well outside the 0.4 m tube, is still blocked
    rim = centre + np.array([0.0, 0.83, 0.0])
    assert carved[tuple(gr.world_to_idx([rim])[0][0])], 'carve ate the rim'

    # --- back-projection: a depth pixel maps to the right optical-frame point ---
    depth = np.full((_H, _W), np.inf, np.float32)
    depth[int(_CY), int(_CX)] = 5.0                    # centre pixel at 5 m
    depth[int(_CY), int(_CX) + 80] = 4.0               # 80 px to the right at 4 m
    pts = backproject_depth(depth, stride=1)
    ctr = pts[np.argmin(np.abs(pts[:, 0]) + np.abs(pts[:, 1]))]
    assert np.allclose(ctr, [0, 0, 5.0], atol=1e-4), ctr
    right = pts[np.argmax(pts[:, 0])]
    assert abs(right[2] - 4.0) < 1e-4 and right[0] > 0 and abs(right[1]) < 1e-4, right
    assert abs(right[0] - 80 * 4.0 / _FX) < 1e-3, right    # x = (du)*z/fx

    print('voxel_map N3 self-check OK '
          '(grid {}, ring hole closes @0.6m infl, corridor reopens)'.format(gr.shape))


def main():
    """This module is a library now -- the only thing to run is the self-check."""
    _selfcheck()


if __name__ == '__main__':
    main()
