#!/usr/bin/env python3
"""N1 gate_detector -- the VISION node of the fly-through stack (image only).

One frame in, one 3D answer out: find the orange ring in COLOUR, sample DEPTH on
its TUBE (never the hole), and recover the ring's 3D CENTRE, NORMAL and RADIUS by
geometry -- a plane through the rim points, then a circle in that plane. Publishes
a geometry_msgs/PoseArray on /gate/detections; the tracker / Dijkstra planner
(the AI half, built later) consume it.

This is PERCEPTION ONLY. It does not command the drone, does not read MAVROS, does
not touch TF. Output stays in the DEPTH optical frame (d435i_depth_optical_frame,
the frame the points were back-projected in). Transforming to `map` is the
tracker's job -- the plan keeps exactly one place that does frame maths, and it is
not here.

  PoseArray /gate/detections:
    header.stamp    = the colour frame's stamp (so a later TF lookup is exact)
    header.frame_id = 'd435i_depth_optical_frame'
    poses[i]        = one detected ring:
        position    = ring centre (metres, depth optical frame)
        orientation = quaternion whose LOCAL +Z is the ring normal, pointing along
                      the fly-through direction (away from the camera). +X/+Y roll
                      is arbitrary -- only the +Z axis carries meaning.

Two sensors, one geometry problem:
  colour  848x480 rgb8, hfov 69 deg -> the ring is a bright orange hoop, so a
          classical HSV + fitEllipse pins it (incl. the head-on vs oblique cue) with
          no trained model, on CPU, at stream rate.
  depth   848x480 float32 METRES, hfov 87 deg -> ranges the tube. Different focal
          and a 15 mm baseline from colour, so colour pixels are MAPPED into depth
          pixels before reading (color_px_to_depth_px), never assumed aligned.
Both arrive as ROS topics from ros_gz_bridge (config/d435i_bridge.yaml) and are
time-synced here with message_filters. The old build read colour over gst/UDP;
the bridge makes that unnecessary and gives us a shared stamp for free.

Detector stays CLASSICAL on purpose. To swap in a deep-learning ring detector,
replace detect_rings() ONLY -- return the same (contour, ellipse) list and the
whole 3D localisation below is unchanged.
# ponytail: classical finds one orange ellipse as well as DL would, with no model
# or dataset to train; the 3D maths, not the 2D box, is where the value is.

Run:  ros2 run simtofly_vision gate_detect [--gui]
      python3 gate_detect.py --selfcheck   # no camera, no ROS
(Gazebo with gates_course.sdf + the d435i bridge must be up. gates_mission.launch.py
starts all of it.)
"""
import math
import os
import time

# MUST come before numpy: OpenBLAS reads this when its library loads, and setting
# it afterwards does nothing. This one line was 90% of this node's CPU.
#
# Every frame runs a handful of SVDs on small point sets (fit_plane, fit_circle).
# They are microseconds of arithmetic, but OpenBLAS still hands each one to a
# pool of one-thread-per-core -- 20 here -- and that pool SPIN-WAITS between
# calls. Measured on one 848x480 frame: 949% CPU and 22 ms; pinned to one thread,
# 27% CPU and 9 ms. Both numbers improve, which is the giveaway that the
# parallelism was never buying anything: the threads were burning cores waiting
# for work far too small to split.
#
# Not a guess at OpenCV, which was the obvious suspect and is innocent: with
# cv2.setNumThreads(1) the CPU did not move. setdefault so an operator can still
# override it from the environment.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image

# ----- camera geometry -- read off realsense_d435i/model.sdf, the single source
# of truth. These reproduce what the model's camera_info publishes (verified live:
# colour K = fx=fy=616.92, cx=424, cy=240), so we skip a camera_info subscription
# and its extra sync -- the constant IS the published intrinsic.
W, H = 848, 480                      # both streams, same size and principal point
HFOV_COLOR = 1.204277                # rad (69 deg)
HFOV_DEPTH = 1.518436                # rad (87 deg)
CX, CY = W / 2.0, H / 2.0
FX_COLOR = CX / math.tan(HFOV_COLOR / 2.0)   # 616.92
FX_DEPTH = CX / math.tan(HFOV_DEPTH / 2.0)   # 446.80

# Colour sits 15 mm to the LEFT of depth (camera_link IS the depth/left-IR origin).
# Purely lateral: shifts u, never v, never Z.
COLOR_TO_DEPTH_Y = 0.015             # m

# Depth clip from the SDF, plus the min-Z the REAL D435i cannot see inside. The sim
# returns depth at 0.17 m; hardware does not, so reject it rather than trust it.
DEPTH_MIN = 0.195                    # m
DEPTH_MAX = 10.0                     # m (SDF <far>)

# ----- orange ring HSV band (OpenCV H is 0..180) -----
# Ring material rgb(1, 0.45, 0), emissive -> bright orange. CALIBRATE if the mask
# misses the ring (widen the band / raise MIN_AREA).
# ponytail: sim-tuning knob, expect to nudge on the first live run.
ORANGE_LO = np.array((3, 80, 80), np.uint8)
ORANGE_HI = np.array((25, 255, 255), np.uint8)
MIN_AREA = 150                       # px^2, ignore specks
_KERNEL = np.ones((5, 5), np.uint8)
_CROSS3 = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))   # see ring_pixels

# Shape gate: the ring's outer contour is an ellipse, so contourArea ~ ellipse area.
# A stray orange blob is not -- and un-gated, one such blob once hijacked a mission.
MIN_DEFO = 0.25                      # minor/major; below this = sliver, not a ring
FIT_LO, FIT_HI = 0.75, 1.25          # contourArea / ellipse area must be ~1.0

# ----- ring, from gate_ring/model.sdf (the CURRENT big ring, not the old 0.43 one) --
RING_CL_R = 0.83                     # tube CENTRELINE radius -- this circle IS the plane
RING_TUBE_R = 0.08                   # tube radius
RING_OUTER_DIAM = 2.0 * (RING_CL_R + RING_TUBE_R)   # 1.82 m, the painted outer edge
# What fitEllipse actually tracks: detection keys on the HOLE (see detect_rings),
# so the ellipse spans the INNER diameter. Sizing range off the outer diameter here
# would over-estimate every range by 1.82/1.50 = 21%.
RING_IN_DIAM = 2.0 * (RING_CL_R - RING_TUBE_R)      # 1.50 m

# Depth sees the tube's NEAR surface; the ring plane is at the centreline, 0.866*r
# further back (median of sqrt(r^2-u^2) over a round tube, not the flat r).
TUBE_TO_PLANE = RING_TUBE_R * math.sqrt(3.0) / 2.0   # 0.0693 m
RADIUS_TOL = 0.20                    # |fitted r - RING_CL_R| must be under this

# ----- localisation thresholds -----
MIN_RIM_PTS = 30                     # valid back-projected rim points to even try a fit
MAX_PLANE_RMS = 0.06                 # m; plane fit worse than this -> reject
MIN_CONF = 0.35                      # publish only above this

DEPTH_FRAME = 'd435i_depth_optical_frame'
COLOR_TOPIC = '/d435i/color/image_raw'
DEPTH_TOPIC = '/d435i/depth/image_rect_raw'


# =========================================================================
# 2D detection (colour)
# =========================================================================
def is_ring(contour, el):
    """True if the contour is ellipse-shaped (a ring), not a blob."""
    (a1, a2) = el[1]
    major, minor = max(a1, a2), min(a1, a2)
    if major < 1e-6 or minor / major < MIN_DEFO:
        return False
    fit = cv2.contourArea(contour) / (math.pi / 4.0 * major * minor)
    return FIT_LO <= fit <= FIT_HI


def detect_rings(rgb):
    """All rings in the frame -> list of (hole_contour, ellipse, mask).

    Detection keys on each ring's HOLE, not its outer edge. On a course you see
    several gates at once and their painted annuli OVERLAP in the image; the outer
    contours then merge into one connected blob. Measured on a real frame with four
    gates in view, RETR_EXTERNAL returned exactly ONE contour for all four -- and
    that merged blob still passed the ellipse test (b/a 0.73, fit 0.95), so N1
    handed localise() a giant bogus ellipse whose radius check then failed. Result:
    four clearly visible gates, zero detections, and nothing in the log to say why.

    The holes do not merge. Same frame, RETR_CCOMP found the four holes cleanly
    (fit 0.96-1.00 each), because a hole only closes up if a nearer ring covers it
    completely -- at which point that gate genuinely is not visible. This is the
    plan's edge case #5, and keying on holes is what handles it.

    Swap THIS for a DL detector (return the same list) and 3D localisation is
    unchanged. `rgb` is an RGB uint8 image (ROS rgb8), so convert RGB->HSV.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, ORANGE_LO, ORANGE_HI)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    # RETR_CCOMP gives two levels: [0] outer boundaries, [1] the holes inside them.
    # hierarchy[i][3] >= 0 means "contour i has a parent", i.e. i IS a hole.
    cnts, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return []
    out = []
    for c, h in zip(cnts, hier[0]):
        if h[3] < 0 or len(c) < 5 or cv2.contourArea(c) < MIN_AREA:
            continue
        el = cv2.fitEllipse(c)
        if is_ring(c, el):
            out.append((c, el, mask))
    return out


# How far out of the hole to reach for annulus pixels, as a fraction of the hole's
# minor axis. The tube is 0.08 m thick on a 1.5 m hole, so the painted band is
# ~11% of the hole diameter; 0.20 covers it with margin at any viewing angle while
# staying well short of a neighbouring ring.
ANNULUS_REACH = 0.20


def ring_pixels(hole_contour, mask):
    """The ring's ANNULUS pixels: orange, immediately around the HOLE (ys, xs).

    Grow the filled hole outward and intersect with the orange mask. Only the ring
    that owns this hole lies in that band, so overlapping gates do not contaminate
    each other -- and the hole itself is excluded, which matters because the depth
    inside it belongs to whatever is BEHIND the gate. That is the one place we must
    never sample (plan par. 2.1).
    """
    (_, axes, _) = cv2.fitEllipse(hole_contour)
    reach = max(2, int(round(ANNULUS_REACH * min(axes))))

    # Grow inside a crop, with an ITERATED 3x3 cross rather than one big
    # structuring element. A (2*reach+1)^2 MORPH_ELLIPSE is not separable, so it
    # costs O(reach^2) per pixel of the WHOLE frame, per ring, per frame -- and
    # reach scales with the ring's apparent size, so it peaks exactly as the
    # drone closes on the gate and the control loop needs the CPU. Measured on a
    # 640x480 frame: 59 ms per ring at reach 80, across 20 OpenCV threads. That
    # starved the SITL host badly enough to produce PreArm gyro/GPS failures and
    # real tracking-guard trips. This version measures 0.9 ms.
    #
    # Iterating a cross gives a DIAMOND, not a disc, so the band reaches slightly
    # less far diagonally. That cannot lose annulus pixels here: reach is 20% of
    # the hole diameter while the painted tube is only ~11% of it, so the orange
    # mask -- not the band's outer edge -- is what bounds the result. Verified on
    # synthetic rings at three scales: the returned pixel set is IDENTICAL to the
    # disc's, to the pixel. Being a subset of the disc, it also cannot reach
    # further into a neighbouring ring than the old code did.
    x, y, w, h = cv2.boundingRect(hole_contour)
    pad = reach + 1
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(mask.shape[1], x + w + pad), min(mask.shape[0], y + h + pad)
    filled = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.drawContours(filled, [hole_contour], -1, 255, -1, offset=(-x0, -y0))
    band = cv2.subtract(cv2.dilate(filled, _CROSS3, iterations=reach), filled)
    ys, xs = np.nonzero(cv2.bitwise_and(band, mask[y0:y1, x0:x1]))
    return ys + y0, xs + x0                    # back to full-frame coordinates


def range_from_mono(el):
    """Pinhole range off the known INNER diameter: dist = fx * D / major_px.

    `el` is the HOLE's ellipse (see detect_rings), so the diameter in play is the
    hole's, not the painted outer edge. The ellipse major axis is the
    least-foreshortened diameter, so it tracks the true size even on an oblique
    approach. Only used to seed the tiny baseline term in the colour->depth
    mapping; the geometry answer comes from depth.
    """
    major = max(el[1])
    return FX_COLOR * RING_IN_DIAM / major if major > 1e-6 else 0.0


def color_px_to_depth_px(xs, ys, z):
    """Colour pixels -> depth pixels. Same size and principal point, so only the
    focal (69 vs 87 deg) and the 15 mm lateral baseline differ.

    The baseline term needs a range `z` (the mono estimate); it is worth ~5 px at
    1.3 m and ~1 px at 5 m, so a small error in z enters at second order. The focal
    term, which does the real work, needs no range.
    """
    k = FX_DEPTH / FX_COLOR
    u = k * (xs - CX) - (FX_DEPTH * COLOR_TO_DEPTH_Y / z) + CX
    v = k * (ys - CY) + CY
    return (np.clip(u, 0, W - 1).astype(np.int32),
            np.clip(v, 0, H - 1).astype(np.int32))


# =========================================================================
# 3D localisation (depth) -- the core of N1
# =========================================================================
def backproject(u, v, z):
    """Depth pixels (u,v) + range z -> 3D points in the depth OPTICAL frame.

    Optical convention: X right, Y down, Z forward. fx=fy=FX_DEPTH, principal point
    (CX,CY). Returns an (N,3) array.
    """
    x = (u - CX) * z / FX_DEPTH
    y = (v - CY) * z / FX_DEPTH
    return np.column_stack((x, y, z))


def fit_plane(pts):
    """Least-squares plane through pts (N,3), with ONE round of outlier rejection.

    Returns (centroid, normal, u_hat, v_hat, rms, inlier_mask). u_hat/v_hat span the
    plane (for the circle fit). rms is the inlier RMS distance to the plane.
    # ponytail: SVD + one 2-sigma trim, not full RANSAC. The sim's depth edges are
    # mild; if real hardware noise breaks this, RANSAC drops in behind fit_plane's
    # signature with nothing downstream to change.
    """
    c0 = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - c0, full_matrices=False)
    n = vh[2]
    d = (pts - c0) @ n
    keep = np.abs(d) < 2.0 * (d.std() + 1e-9)
    if keep.sum() >= 3 and keep.sum() < len(pts):
        pts = pts[keep]
        c0 = pts.mean(axis=0)
        _, _, vh = np.linalg.svd(pts - c0, full_matrices=False)
        n = vh[2]
        d = (pts - c0) @ n
    else:
        keep = np.ones(len(pts), bool)
    rms = float(np.sqrt(np.mean(d[np.abs(d) < 2.0 * (d.std() + 1e-9)] ** 2))) \
        if len(d) else 1e9
    return c0, n / (np.linalg.norm(n) + 1e-12), vh[0], vh[1], rms, keep


def fit_circle(pts, c0, u_hat, v_hat):
    """Kasa algebraic circle fit in the plane -> (centre_3d, radius, rms).

    Project pts onto (u_hat, v_hat), fit centre+radius by least squares, lift the
    centre back to 3D. rms is the radial residual.
    """
    p = pts - c0
    u = p @ u_hat
    v = p @ v_hat
    A = np.column_stack((2 * u, 2 * v, np.ones_like(u)))
    b = u * u + v * v
    (uc, vc, cc), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(max(cc + uc * uc + vc * vc, 0.0))
    rms = float(np.sqrt(np.mean((np.hypot(u - uc, v - vc) - r) ** 2)))
    centre = c0 + uc * u_hat + vc * v_hat
    return centre, r, rms


def quat_from_zaxis(n):
    """Quaternion (x,y,z,w) whose local +Z maps to unit vector n. Roll is free."""
    z = n / (np.linalg.norm(n) + 1e-12)
    # any axis not parallel to z gives a valid frame
    a = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(a, z)
    x /= np.linalg.norm(x) + 1e-12
    y = np.cross(z, x)
    m = np.column_stack((x, y, z))          # rotation matrix, columns = new axes
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    else:                                   # numerically safer branch for t<=0
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
        qw = (m[k, j] - m[j, k]) / s
        qx, qy, qz = q
    return qx, qy, qz, qw


def localise(contour, el, mask, depth):
    """One ring's 2D detection + depth image -> (centre, normal, radius, conf) or None.

    centre/normal are in the depth optical frame; conf in [0,1]. None means depth
    could not answer for this ring (too few valid points, bad plane, wrong radius).
    """
    ys, xs = ring_pixels(contour, mask)
    if len(xs) < MIN_RIM_PTS:
        return None
    mono = range_from_mono(el)
    u, v = color_px_to_depth_px(xs.astype(np.float64), ys.astype(np.float64), mono)
    z = depth[v, u]
    ok = np.isfinite(z) & (z >= DEPTH_MIN) & (z <= DEPTH_MAX)
    if ok.sum() < MIN_RIM_PTS:
        return None
    pts = backproject(u[ok].astype(np.float64), v[ok].astype(np.float64), z[ok])

    c0, n, uh, vh, prms, keep = fit_plane(pts)
    if prms > MAX_PLANE_RMS:
        return None
    centre, r, crms = fit_circle(pts[keep], c0, uh, vh)
    if abs(r - RING_CL_R) > RADIUS_TOL:
        return None

    # Orient the normal along the fly-through direction FIRST -- away from the camera
    # at the origin (n . centre > 0) -- because the next step steps along it.
    if float(n @ centre) < 0:
        n = -n
    # Depth saw the tube's NEAR surface, on the camera side of the centreline; the
    # plane we fly through is the centreline, TUBE_TO_PLANE farther from the camera,
    # i.e. along +n. (Do this after the flip, or it pushes the wrong way.)
    centre = centre + n * TUBE_TO_PLANE

    inlier_ratio = keep.sum() / len(pts)
    conf = inlier_ratio * math.exp(-crms / 0.05) * math.exp(-abs(r - RING_CL_R) / 0.2)
    if conf < MIN_CONF:
        return None
    return centre, n, float(r), float(conf)


def detect(rgb, depth):
    """Full frame -> list of detections [(centre, normal, radius, conf, ellipse)].

    ellipse is kept only so the GUI can redraw it; the published contract is the
    3D fields.
    """
    out = []
    for contour, el, mask in detect_rings(rgb):
        got = localise(contour, el, mask, depth)
        if got is not None:
            centre, n, r, conf = got
            out.append((centre, n, r, conf, el))
    return out


# =========================================================================
# ROS node
# =========================================================================
def make_pose(centre, normal):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(c) for c in centre)
    qx, qy, qz, qw = quat_from_zaxis(np.asarray(normal, float))
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def draw_overlay(bgr, dets):
    cv2.drawMarker(bgr, (int(CX), int(CY)), (255, 255, 0), cv2.MARKER_CROSS, 20, 1)
    for centre, n, r, conf, el in dets:
        cv2.ellipse(bgr, el, (0, 255, 0), 2)
        cx, cy = int(el[0][0]), int(el[0][1])
        cv2.circle(bgr, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(bgr, 'C=({:+.2f},{:+.2f},{:+.2f}) r={:.2f} n=({:+.2f},{:+.2f},'
                    '{:+.2f}) c={:.2f}'.format(*centre, r, *n, conf),
                    (cx - 120, cy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(bgr, '{} gate(s)'.format(len(dets)), (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)


class GateDetector:
    def __init__(self, node, gui):
        self.node = node
        self.gui = gui
        self.pub = node.create_publisher(PoseArray, '/gate/detections', 10)
        # Reliability is the bridge's default (verified against parameter_bridge in
        # gates_mission.launch.py) -- it does NOT use qos_profile_sensor_data, so
        # best-effort here would match nothing.
        sub_c = Subscriber(node, Image, COLOR_TOPIC)
        sub_d = Subscriber(node, Image, DEPTH_TOPIC)
        # slop 33 ms ~ one frame at 30 Hz; queue 5 rides out the rate mismatch
        # (colour ~30, depth ~15).
        self.sync = ApproximateTimeSynchronizer([sub_c, sub_d], 5, 0.033)
        self.sync.registerCallback(self.on_pair)
        self.n = 0
        self.n_seen = 0            # frames that produced at least one gate
        self.n_last_log = 0
        self.t_last_log = None

    def on_pair(self, color_msg, depth_msg):
        if depth_msg.encoding != '32FC1':
            self.node.get_logger().warn(
                'depth encoding {} != 32FC1'.format(depth_msg.encoding), once=True)
            return
        rgb = np.frombuffer(color_msg.data, np.uint8).reshape(H, W, 3)
        depth = np.frombuffer(depth_msg.data, np.float32).reshape(H, W)

        dets = detect(rgb, depth)

        msg = PoseArray()
        msg.header.stamp = color_msg.header.stamp
        msg.header.frame_id = DEPTH_FRAME
        msg.poses = [make_pose(c, n) for c, n, r, conf, el in dets]
        self.pub.publish(msg)

        self.n += 1
        if dets:
            self.n_seen += 1
        # A detection is news, so log it at ~1 Hz. "Nothing seen" is the NORMAL
        # state -- the drone spends most of a yaw sweep pointed at empty field --
        # so it goes out every 5 s instead of 15 frames, and carries the frame
        # count and rate rather than repeating itself. That way a stalled camera
        # (counter frozen) reads differently from an empty view (counter climbing),
        # and a dead node with a window still open reads differently from both.
        now = time.monotonic()
        if self.t_last_log is None:
            self.t_last_log = now
        if dets and self.n % 15 == 0:
            c, n, r, conf, _ = dets[0]
            self.node.get_logger().info(
                '{} gate(s); nearest C=({:+.2f},{:+.2f},{:+.2f}) r={:.2f} '
                'n=({:+.2f},{:+.2f},{:+.2f}) conf={:.2f}'.format(
                    len(dets), *c, r, *n, conf))
            self.t_last_log, self.n_last_log = now, self.n
        elif not dets and now - self.t_last_log >= 5.0:
            dt = now - self.t_last_log
            self.node.get_logger().info(
                'no gate | frames={} ({:.1f} Hz), {} of them had one'.format(
                    self.n, (self.n - self.n_last_log) / dt, self.n_seen))
            self.t_last_log, self.n_last_log = now, self.n
        if self.gui:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            draw_overlay(bgr, dets)
            cv2.imshow('gate_detect N1 (q=quit)', bgr)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                rclpy.shutdown()


# =========================================================================
# self-check -- no camera, no ROS:  python3 gate_detect.py --selfcheck
# =========================================================================
def _synthetic(dist_m, tilt_deg=0.0):
    """A ring at dist_m, optionally tilted about the image X axis by tilt_deg, drawn
    into an rgb8 frame + a float32 depth image. Forward-projects the model's own
    geometry through the model's own intrinsics, and models the tube as the ROUND
    cylinder it is (surface at range - sqrt(r^2-u^2)), so a flat-disc bug in
    TUBE_TO_PLANE cannot hide behind a flat-disc synthetic.
    """
    rgb = np.zeros((H, W, 3), np.uint8)
    orange = (255, 115, 0)              # RGB
    cv2.circle(rgb, (int(CX), int(CY)),
               int(FX_COLOR * (RING_CL_R + RING_TUBE_R) / dist_m), orange, -1)
    cv2.circle(rgb, (int(CX), int(CY)),
               int(FX_COLOR * (RING_CL_R - RING_TUBE_R) / dist_m), (0, 0, 0), -1)

    yy, xx = np.mgrid[0:H, 0:W]
    # metres from ring centre in the depth image plane; tilt stretches the Y radius
    ky = math.cos(math.radians(tilt_deg))
    rho = np.hypot((xx - CX), (yy - CY) / max(ky, 1e-3)) * dist_m / FX_DEPTH
    u = rho - RING_CL_R
    on_tube = np.abs(u) < RING_TUBE_R
    depth = np.full((H, W), 50.0, np.float32)
    # tilt tips the near surface: +Z shift proportional to Y offset, tan(tilt)
    zoff = (yy - CY) * dist_m / FX_DEPTH * math.tan(math.radians(tilt_deg))
    depth[on_tube] = (dist_m + zoff[on_tube]
                      - np.sqrt(RING_TUBE_R ** 2 - u[on_tube] ** 2))
    return rgb, depth.astype(np.float32)


def _selfcheck():
    assert abs(FX_COLOR - 616.92) < 0.5, FX_COLOR
    assert abs(FX_DEPTH - 446.80) < 0.5, FX_DEPTH
    assert abs(TUBE_TO_PLANE - 0.0693) < 1e-3, TUBE_TO_PLANE

    # --- geometry unit tests on clean 3D point sets (the maths, no images) ---
    rng = np.random.default_rng(0)
    ang = rng.uniform(0, 2 * math.pi, 400)
    ring = np.column_stack((0.83 * np.cos(ang), 0.83 * np.sin(ang),
                            np.zeros_like(ang)))          # flat ring, radius 0.83, z=0
    ring += np.array([0.2, -0.1, 3.0])                    # move it in space
    c0, n, uh, vh, rms, keep = fit_plane(ring)
    centre, r, crms = fit_circle(ring, c0, uh, vh)
    assert abs(r - 0.83) < 1e-3, r
    assert np.allclose(centre, [0.2, -0.1, 3.0], atol=1e-3), centre
    assert abs(abs(n[2]) - 1.0) < 1e-3, n                 # plane is z=const -> n ~ +/-z

    # quaternion round-trip: +Z of the quat must come back as n
    for nn in ([0, 0, 1], [0, 0.5, 0.87], [0.3, -0.2, 0.93]):
        nn = np.array(nn, float); nn /= np.linalg.norm(nn)
        qx, qy, qz, qw = quat_from_zaxis(nn)
        # rotate local +Z by the quat
        zc = np.array([2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx),
                       1 - 2 * (qx * qx + qy * qy)])
        assert np.allclose(zc, nn, atol=1e-6), (zc, nn)

    # --- full pipeline: head-on ring, depth must localise centre/normal/radius ---
    for truth in (2.0, 3.5, 5.0):
        rgb, depth = _synthetic(truth)
        dets = detect(rgb, depth)
        assert len(dets) == 1, 'expected one ring at {} m, got {}'.format(truth, len(dets))
        c, n, r, conf, _ = dets[0]
        assert abs(c[0]) < 0.05 and abs(c[1]) < 0.05, ('off-centre', c)
        assert abs(c[2] - truth) < 0.05, ('range', c[2], truth)
        assert abs(r - RING_CL_R) < 0.08, ('radius', r)
        assert n[2] > 0.9, ('normal should face the camera axis', n)   # +Z, toward flight

    # --- TWO RINGS WHOSE PAINTED BANDS TOUCH must stay two rings ---
    # This is the regression that hid for a whole debugging session: on a course you
    # see several gates at once, their annuli merge into ONE connected component,
    # and an outer-contour detector then fits a single giant ellipse over the pair.
    # It even passes the shape gate, so nothing logs an error -- the range check
    # downstream just quietly rejects it and N1 reports "no gate" with four gates
    # filling the frame. Keying on the HOLES is what fixes it, and this asserts it.
    rgb, depth = _synthetic(5.0)
    r_out_px = int(FX_COLOR * (RING_CL_R + RING_TUBE_R) / 5.0)
    r_in_px = int(FX_COLOR * (RING_CL_R - RING_TUBE_R) / 5.0)
    shift = int(1.9 * r_out_px)                   # far enough apart to keep 2 holes,
    cx2 = int(CX) + shift                         # close enough that the bands touch
    cv2.circle(rgb, (cx2, int(CY)), r_out_px, (255, 115, 0), -1)
    cv2.circle(rgb, (cx2, int(CY)), r_in_px, (0, 0, 0), -1)
    mask = cv2.inRange(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV), ORANGE_LO, ORANGE_HI)
    merged, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assert len(merged) == 1, \
        'test is not exercising the bug: the two bands must MERGE into one blob'
    found = detect_rings(rgb)
    assert len(found) == 2, \
        'two touching rings collapsed into {} candidate(s)'.format(len(found))
    xs = sorted(el[0][0] for _, el, _ in found)
    assert abs(xs[0] - CX) < 3 and abs(xs[1] - cx2) < 3, ('hole centres', xs)

    # --- ring_pixels: the cheap ROI/diamond band == the old full-frame disc ---
    # The band shape changed for speed (see ring_pixels), so pin the EQUIVALENCE
    # rather than trusting the argument. Near range is the case that matters:
    # that is where reach -- and the old cost -- blew up.
    for dist in (5.0, 2.0, 1.2):
        rgb, _ = _synthetic(dist)
        for hole, _el, m in detect_rings(rgb):
            old = np.zeros(m.shape, np.uint8)
            cv2.drawContours(old, [hole], -1, 255, -1)
            (_, axes, _) = cv2.fitEllipse(hole)
            reach = max(2, int(round(ANNULUS_REACH * min(axes))))
            disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                             (2 * reach + 1, 2 * reach + 1))
            want = np.nonzero(cv2.bitwise_and(
                cv2.subtract(cv2.dilate(old, disc), old), m))
            got = ring_pixels(hole, m)
            assert np.array_equal(want[0], got[0]) and \
                np.array_equal(want[1], got[1]), \
                'band khac cach cu o {} m (reach {})'.format(dist, reach)

    # --- oblique ring: the normal must tilt, not stay dead-on ---
    rgb, depth = _synthetic(3.0, tilt_deg=30.0)
    dets = detect(rgb, depth)
    assert len(dets) == 1, 'oblique ring not found'
    _, n, _, _, _ = dets[0]
    tilt = math.degrees(math.acos(min(abs(n[2]), 1.0)))
    assert 15.0 < tilt < 45.0, 'normal tilt {:.1f} deg, expected ~30'.format(tilt)

    # --- rejects: blank frame, and a big non-elliptical orange blob ---
    assert detect(np.zeros((H, W, 3), np.uint8), np.full((H, W), 3.0, np.float32)) == []
    blob = np.zeros((H, W, 3), np.uint8)
    cv2.fillPoly(blob, [np.array([(100, 100), (300, 120), (140, 300)])], (255, 115, 0))
    assert detect(blob, np.full((H, W), 3.0, np.float32)) == [], 'triangle blob passed'

    # --- no valid depth -> no detection (perception must not invent a pose) ---
    rgb, _ = _synthetic(3.0)
    assert detect(rgb, np.full((H, W), np.inf, np.float32)) == [], 'inf depth localised'

    print('gate_detect N1 self-check OK (fx_color={:.1f} fx_depth={:.1f})'
          .format(FX_COLOR, FX_DEPTH))


def main():
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    gui = ('--gui' in sys.argv) or os.environ.get('GATE_GUI') == '1'
    rclpy.init()
    node = rclpy.create_node('gate_detect')
    GateDetector(node, gui)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
