#!/usr/bin/env python3
"""Gate (racing ring) detector -- the VISION half of the fly-through mission.

Reads the drone's nose-mounted RealSense D435i (model://iris_with_d435i in
~/gz_ws/src/drone_vision) and, every frame, finds the orange ring, fits an
ellipse to it, and ranges it with the DEPTH stream. Publishes the ring geometry
on /gate/obs; the CONTROL half (simtofly_mavros_sitl/mission_gates.py) flies
through it.

Two streams, two transports -- not an accident:
  colour  848x480 RGB, hfov 69 deg.  Arrives as H264/RTP on UDP 5600, because the
          colour sensor still carries ardupilot_gazebo's GstCameraPlugin. Read via
          a gst-launch subprocess: the OpenCV build on this machine was compiled
          WITHOUT GStreamer (cv2.getBuildInformation -> "GStreamer: NO"), so
          cv2.VideoCapture(CAP_GSTREAMER) is unavailable.
  depth   848x480 float32 metres, hfov 87 deg. A plain Gazebo topic, bridged into
          ROS by ros_gz_bridge (see gates_mission.launch.py) and subscribed here.

Both sensors declare 30 Hz, but the sim does not deliver it: measured, depth lands
at ~15 Hz (four 848x480 renders per frame is more than this machine does in real
time, and the GUI ImageDisplay panels make it worse). So /gate/obs is a ~15 Hz
range at best, and mission_gates is built for that -- its COMMIT phase stops using
vision entirely and flies a frozen world point off /mavros/local_position, which is
exactly the moment a slow, stale camera would otherwise hurt most.

Detector: deliberately CLASSICAL (HSV threshold -> largest contour ->
cv2.fitEllipse). The ring is a solid bright-orange hoop, so this pins the
ellipse -- including its "deformation" (minor/major axis ratio, the head-on
vs oblique cue) -- with no trained model, on CPU, at stream rate. To swap in a
deep-learning detector, replace detect_ring() ONLY; the /gate/obs contract and
the depth ranging both keep working off the contour it returns.
# ponytail: classical does exactly what DL would here (find one orange ellipse);
# DL needs a gate-trained model + dataset that don't exist yet.

/gate/obs = Float32MultiArray([visible, cx_off, cy_off, fill, defo, dist]):
  visible : 1.0 if a ring was found this frame else 0.0
  cx_off  : ring-centre horizontal offset, -1 (left)  .. +1 (right)
  cy_off  : ring-centre vertical   offset, -1 (top)   .. +1 (bottom)
  fill    : ellipse major axis / image width  (apparent size ~ closeness)
  defo    : minor/major axis ratio  (1.0 = head-on circle -> 0 = edge-on)
  dist    : metres to the ring PLANE (depth stream; mono fallback -- see range())

Run:  ros2 run simtofly_vision gate_detect [--gui]
(Gazebo with gates_course.sdf must be up, and the depth bridge running, so both
streams exist. gates_mission.launch.py starts all of it.)
"""
import math
import os
import shlex
import subprocess

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# ----- camera geometry -- every number below is read off realsense_d435i/model.sdf.
# Nothing here is a round catalogue figure; keep them in step with that file.
W, H = 848, 480                     # both streams, same size and principal point
HFOV_COLOR = 1.204277               # rad (69 deg)
HFOV_DEPTH = 1.518436               # rad (87 deg)
CX, CY = W / 2.0, H / 2.0

# fx = (W/2) / tan(hfov/2). Comes out 616.9 (colour) / 446.9 (depth), which is what
# the model's own camera_info publishes -- the SDF is the single source of truth.
FX_COLOR = CX / math.tan(HFOV_COLOR / 2.0)
FX_DEPTH = CX / math.tan(HFOV_DEPTH / 2.0)

# Colour sits 15 mm to the LEFT of depth (Intel's own extrinsic; camera_link IS the
# depth/left-IR origin). Purely lateral, so it shifts u but never v, and never Z.
COLOR_TO_DEPTH_Y = 0.015            # m

# Depth clip from the SDF, plus the min-Z the REAL D435i cannot see inside. The sim
# happily returns depth at 0.17 m; hardware does not, so reject it here instead of
# flying on a number the real camera would never give us.
DEPTH_MIN = 0.195                   # m
DEPTH_MAX = 10.0                    # m (SDF <far>)
DEPTH_TIMEOUT = 0.5                 # s; older depth frame -> fall back to mono

UDP_PORT = 5600
DEPTH_TOPIC = '/d435i/depth/image_rect_raw'
# GstCameraPlugin derives its enable topic as <sensor topic> + "/enable_streaming"
# (GstCameraPlugin.cc:221), and the colour sensor names its own topic in the SDF --
# so unlike the old gimbal camera this does NOT embed the world name. Confirmed
# against a running sim, which prints:
#   GstCameraPlugin: enable topic [/d435i/color/image_raw/enable_streaming]
# Override with GATE_ENABLE_TOPIC if the model's <topic> ever changes.
ENABLE_TOPIC = os.environ.get(
    'GATE_ENABLE_TOPIC', '/d435i/color/image_raw/enable_streaming')

# gst pipeline = the user's known-good receiver, but ending in a raw-BGR fdsink
# (read from stdout) instead of autovideosink. It is pinned to the sensor's OWN
# 848x480: rescaling to 640x480 would squash X by 0.755, turning a head-on circle
# into an ellipse and corrupting both `defo` and the major axis we range by.
GST = ('gst-launch-1.0 -q udpsrc port={p} '
       "caps=application/x-rtp,media=video,clock-rate=90000,"
       "encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert "
       '! videoscale ! video/x-raw,format=BGR,width={w},height={h} '
       '! fdsink sync=false').format(p=UDP_PORT, w=W, h=H)

# ----- orange ring HSV band (OpenCV H is 0..180) -----
# Ring material is rgb(1, 0.45, 0), emissive -> renders bright orange.
# CALIBRATE if the mask misses the ring (widen the band / raise MIN_AREA).
# ponytail: sim-tuning knob, expect to nudge these on the first run.
ORANGE_LO = np.array((3, 80, 80), np.uint8)
ORANGE_HI = np.array((25, 255, 255), np.uint8)
MIN_AREA = 150                      # px^2, ignore specks
_KERNEL = np.ones((5, 5), np.uint8)

# Shape gate: the ring's OUTER contour is an ellipse, so its area matches the
# area of the fitted ellipse. A stray orange blob (terrain, horizon haze, sun)
# does not -- and un-gated, one such blob hijacked the whole mission: the
# controller "saw a ring", stopped its search sweep and flew off at the blob.
MIN_DEFO = 0.25                     # minor/major; below this = sliver, not a ring
FIT_LO, FIT_HI = 0.75, 1.25         # contourArea / ellipse area must be ~1.0

# ----- ring, from gate_ring/model.sdf -----
RING_DIAM_M = 0.982                 # OUTER diameter = 2*(0.431 tube centreline + 0.06 tube)
RING_TUBE_CL = 0.431                # tube centreline radius -- this circle IS the ring plane
RING_TUBE_R = 0.06                  # tube radius

# Depth sees the tube's NEAR SURFACE; the ring PLANE is where the tube's centreline is.
# The gap between them is NOT the tube radius, because the tube is a round cylinder, not
# a flat face: at lateral offset u from the tube's axis the visible surface lies
# sqrt(r^2 - u^2) in front of the plane, and image pixels are spread uniformly over
# u in [-r, r]. The MEDIAN of sqrt(r^2 - u^2) is r*sqrt(3)/2 -- 0.866 r, not r.
#
# Using r flat (0.060) overshoots by 8 mm, and that showed up in a ground-truth rig as a
# dead-constant +0.010 m at every range from 1.2 m to 5 m. Constant, not scaled: the
# signature of a geometry mistake rather than noise.
TUBE_TO_PLANE = RING_TUBE_R * math.sqrt(3.0) / 2.0      # 0.0520 m
MIN_DEPTH_PX = 20                   # need this many valid ring pixels to trust the median


def enable_streaming():
    """Ask the Gazebo camera sensor to start streaming (best-effort, once)."""
    cmd = ['gz', 'topic', '-t', ENABLE_TOPIC, '-m', 'gz.msgs.Boolean',
           '-p', 'data: 1']
    try:
        subprocess.run(cmd, timeout=5,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('[gate_detect] enable_streaming -> {}'.format(ENABLE_TOPIC))
    except Exception as e:
        print('[gate_detect] enable_streaming failed ({}); is Gazebo up? If the '
              'Gazebo log shows a different "enable topic", set GATE_ENABLE_TOPIC.'
              .format(e))


def read_exact(stream, n):
    """Read exactly n bytes from a pipe (fdsink may deliver in chunks)."""
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def is_ring(contour, el):
    """True if the contour is actually ellipse-shaped (i.e. a ring, not a blob)."""
    (a1, a2) = el[1]
    major, minor = max(a1, a2), min(a1, a2)
    if major < 1e-6 or minor / major < MIN_DEFO:
        return False
    fit = cv2.contourArea(contour) / (math.pi / 4.0 * major * minor)
    return FIT_LO <= fit <= FIT_HI


def detect_ring(frame):
    """Largest orange ellipse -> (contour, ellipse, orange_mask) or None.

    Swap THIS function for a deep-learning detector (e.g. YOLOv8-seg mask ->
    fitEllipse); return the same triple and everything downstream is unchanged.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ORANGE_LO, ORANGE_HI)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    rings = []
    for c in cnts:
        if len(c) < 5 or cv2.contourArea(c) < MIN_AREA:
            continue
        el = cv2.fitEllipse(c)
        if is_ring(c, el):
            rings.append((cv2.contourArea(c), c, el))
    if not rings:
        return None
    _, contour, el = max(rings, key=lambda r: r[0])
    return contour, el, mask


def ring_pixels(contour, orange_mask):
    """The ring's ANNULUS pixels: orange AND inside the outer contour.

    Filling the contour alone would include the HOLE, and the hole is exactly what
    we must not sample -- depth there reads whatever is behind the gate.
    """
    disc = np.zeros(orange_mask.shape, np.uint8)
    cv2.drawContours(disc, [contour], -1, 255, -1)
    return np.nonzero(cv2.bitwise_and(disc, orange_mask))     # (ys, xs)


def range_from_mono(el):
    """Pinhole range off the known ring size: dist = fx * D / major_axis_px.

    The ellipse major axis is the least-foreshortened diameter, so it stays ~ the
    true ring diameter even on an oblique approach. fitEllipse tracks the OUTER
    edge, so RING_DIAM_M is the outer diameter and this lands on the ring PLANE.
    """
    major = max(el[1])
    return FX_COLOR * RING_DIAM_M / major if major > 1e-6 else 0.0


def color_px_to_depth_px(xs, ys, z):
    """Map colour pixels -> depth pixels. Both streams are 848x480 with the same
    principal point, so only two things differ: the focal (69 vs 87 deg) and the
    15 mm lateral baseline.

    The baseline term needs a range to divide by; `z` is the mono estimate. That is
    fine and not circular: the whole term is worth only ~5 px at 1.3 m and ~1 px at
    5 m, so an error in `z` enters at second order. The focal term -- which does the
    real work -- needs no range at all.
    """
    k = FX_DEPTH / FX_COLOR
    u = k * (xs - CX) - (FX_DEPTH * COLOR_TO_DEPTH_Y / z) + CX
    v = k * (ys - CY) + CY
    return (np.clip(u, 0, W - 1).astype(np.int32),
            np.clip(v, 0, H - 1).astype(np.int32))


def range_from_depth(contour, orange_mask, depth, mono):
    """Median depth over the ring's annulus, or None if depth cannot answer.

    Sampling the ring's TUBE (not its centre) is the whole point: the centre is the
    hole, and depth there reads whatever is behind the gate. + TUBE_TO_PLANE then
    moves the answer from "the metal I can see" to "the plane I must fly through".
    """
    if depth is None:
        return None
    ys, xs = ring_pixels(contour, orange_mask)
    if len(xs) < MIN_DEPTH_PX:
        return None
    u, v = color_px_to_depth_px(xs.astype(np.float64), ys.astype(np.float64), mono)
    d = depth[v, u]
    d = d[np.isfinite(d) & (d >= DEPTH_MIN) & (d <= DEPTH_MAX)]
    if len(d) < MIN_DEPTH_PX:
        return None
    return float(np.median(d)) + TUBE_TO_PLANE


def observe(frame, depth=None):
    """Frame (+ latest depth image) -> (obs_6_tuple, ellipse, info).

    `info` reports which range actually went out on /gate/obs -- 'depth' or 'MONO'
    -- plus both candidate values. Do NOT infer the source from "a depth frame was
    available": range_from_depth() can still bail (too few valid ring pixels, ring
    past the 10 m clip) and silently fall back, which is exactly how a broken depth
    path hides in plain sight.
    """
    found = detect_ring(frame)
    if found is None:
        return [0.0] * 6, None, ('none', 0.0, 0.0)
    contour, el, orange_mask = found
    (cx, cy), (a1, a2), _ = el
    major, minor = max(a1, a2), min(a1, a2)

    mono = range_from_mono(el)
    depth_m = range_from_depth(contour, orange_mask, depth, mono)
    src = 'depth' if depth_m is not None else 'MONO'
    dist = mono if depth_m is None else depth_m
    return ([1.0,
             (cx - CX) / CX,
             (cy - CY) / CY,
             major / W,
             (minor / major) if major > 1e-6 else 0.0,
             dist],
            el,
            (src, depth_m if depth_m is not None else float('nan'), mono))


def draw_overlay(frame, el, obs):
    """Annotate the live frame: ring ellipse, its centre, image crosshair,
    and the obs values -- so `--gui` shows what the detector sees."""
    cx0, cy0 = int(CX), int(CY)
    cv2.drawMarker(frame, (cx0, cy0), (255, 255, 0), cv2.MARKER_CROSS, 20, 1)
    vis, offx, offy, fill, defo, dist = obs
    if el is not None and vis > 0.5:
        centred = abs(offx) < 0.05 and abs(offy) < 0.05
        col = (0, 255, 0) if centred else (0, 165, 255)
        cv2.ellipse(frame, el, col, 2)
        cx, cy = int(el[0][0]), int(el[0][1])
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(frame, (cx0, cy0), (cx, cy), (0, 0, 255), 1)
    txt = '{}  off=({:+.2f},{:+.2f}) fill={:.2f} defo={:.2f} dist={:.2f}m'.format(
        'RING' if vis > 0.5 else 'searching', offx, offy, fill, defo, dist)
    cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)


class Depth:
    """Latest depth frame, as a float32 metres array. Stale frames read as None."""

    def __init__(self, node):
        self.img = None
        self.t = 0.0
        self.node = node
        # KEEP_LAST(1). The capture loop drains one callback per colour frame, so a
        # deeper queue could hand us a frame several deliveries old (KEEP_LAST(n)
        # gives the reader the OLDEST of the n it holds) -- and at 1.6 m/s, 0.3 s of
        # backlog is half a metre of range error against a 50 mm margin. Measured, no
        # backlog actually forms today: the sim renders depth at ~15 Hz, not the SDF's
        # nominal 30, so it publishes slower than we consume. Depth 1 costs nothing
        # and makes that a property of the code rather than of the frame rate.
        #
        # Reliability is the plain default, verified against the parameter_bridge in
        # gates_mission.launch.py: it does NOT publish on qos_profile_sensor_data, as
        # many sensor topics do, so best-effort here would match nothing.
        node.create_subscription(Image, DEPTH_TOPIC, self._cb, 1)

    def _cb(self, msg):
        if msg.encoding != '32FC1':
            self.node.get_logger().warn(
                'depth encoding {} != 32FC1'.format(msg.encoding), once=True)
            return
        self.img = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width)
        self.t = self.node.get_clock().now().nanoseconds * 1e-9

    def latest(self):
        if self.img is None:
            return None
        age = self.node.get_clock().now().nanoseconds * 1e-9 - self.t
        return self.img if age < DEPTH_TIMEOUT else None


def run(node, gui):
    """Main-thread capture loop: read colour frame -> detect -> range -> publish.

    The colour read blocks, so the loop lives in the main thread (cv2.imshow must).
    spin_once(timeout_sec=0) drains the depth subscription without blocking it.
    """
    pub = node.create_publisher(Float32MultiArray, '/gate/obs', 10)
    depth = Depth(node)
    enable_streaming()
    proc = subprocess.Popen(shlex.split(GST), stdout=subprocess.PIPE)
    nbytes = W * H * 3
    n = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)      # drain depth callbacks
            raw = read_exact(proc.stdout, nbytes)
            if raw is None:
                node.get_logger().warn('camera stream ended')
                break
            frame = np.frombuffer(raw, np.uint8).reshape(H, W, 3)
            d = depth.latest()
            obs, el, (src, d_m, mono_m) = observe(frame, d)
            pub.publish(Float32MultiArray(data=obs))
            n += 1
            if n % 30 == 0:              # ~1 Hz heartbeat
                # Print BOTH ranges and which one was published. If `src` reads MONO
                # while the sim is up, the depth path is broken -- and the two numbers
                # side by side say how badly.
                node.get_logger().info(
                    'ring {} off=({:+.2f},{:+.2f}) fill={:.2f} defo={:.2f} '
                    'dist={:.2f}m via {} (depth={:.2f} mono={:.2f}) depth_img={}'
                    .format('SEEN' if obs[0] else 'none ', obs[1], obs[2], obs[3],
                            obs[4], obs[5], src, d_m, mono_m,
                            'fresh' if d is not None else 'STALE'))
            if gui:
                draw_overlay(frame, el, obs)
                cv2.imshow('gate_detect (q=quit)', frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
    finally:
        try:
            proc.terminate()
        except Exception:
            pass
        if gui:
            cv2.destroyAllWindows()


def _synthetic(dist_m):
    """A head-on orange ring at dist_m, drawn into a colour frame + a depth image.

    Forward-projects the model's own geometry through the model's own intrinsics, so
    the test exercises the real numbers rather than a made-up scale.

    The depth image models the tube as the ROUND CYLINDER it is -- surface at
    dist - sqrt(r^2 - u^2), u being the offset from the tube centreline. An earlier
    version painted it as a flat disc at dist - r, which quietly agreed with the
    wrong + RING_TUBE_R correction and let a 10 mm bias through. A synthetic that
    shares your bug cannot catch it.
    """
    frame = np.zeros((H, W, 3), np.uint8)
    cv2.circle(frame, (int(CX), int(CY)),
               int(FX_COLOR * (RING_DIAM_M / 2.0) / dist_m), (0, 115, 255), -1)
    cv2.circle(frame, (int(CX), int(CY)),
               int(FX_COLOR * (RING_TUBE_CL - RING_TUBE_R) / dist_m), (0, 0, 0), -1)

    yy, xx = np.mgrid[0:H, 0:W]
    rho = np.hypot(xx - CX, yy - CY) * dist_m / FX_DEPTH      # metres from ring centre
    u = rho - RING_TUBE_CL                                    # offset from tube centreline
    on_tube = np.abs(u) < RING_TUBE_R
    depth = np.full((H, W), 50.0, np.float32)                 # far background
    depth[on_tube] = dist_m - np.sqrt(
        RING_TUBE_R ** 2 - u[on_tube] ** 2)
    return frame, depth.astype(np.float32)


def _selfcheck():
    """No camera, no ROS. Run: python3 gate_detect.py --selfcheck"""
    # Intrinsics must reproduce the model's published camera_info.
    assert abs(FX_COLOR - 616.92) < 0.5, FX_COLOR
    assert abs(FX_DEPTH - 446.80) < 0.5, FX_DEPTH

    # 1) Depth is ACTUALLY the source -- assert the source flag, not just the number.
    # A silent fall-back to mono would still produce a plausible range here, which is
    # how a dead depth path stays invisible.
    frame, depth = _synthetic(5.0)
    obs, _, (src, d_m, mono_m) = observe(frame, depth)
    assert obs[0] == 1.0, 'ring not detected'
    assert src == 'depth', 'range came from {}, not depth'.format(src)
    assert obs[5] == d_m, 'published range is not the depth one'
    assert abs(obs[1]) < 0.02 and abs(obs[2]) < 0.02, 'should be centred'
    assert obs[4] > 0.95, 'head-on ring should be circular: defo={}'.format(obs[4])

    # 15 mm, tight enough to fail if TUBE_TO_PLANE reverts to the flat-face RING_TUBE_R
    # (that is worth 8 mm of bias, and a ground-truth rig measured it at 10 mm).
    assert abs(obs[5] - 5.0) < 0.015, 'depth range off: {}'.format(obs[5])
    assert abs(TUBE_TO_PLANE - 0.0520) < 1e-4, TUBE_TO_PLANE

    # 2) Depth must be read on the TUBE, never at the centre. The centre is the HOLE,
    # so the background there is at 50 m: if we ever sampled it, the range would bolt.
    for truth in (3.0, 1.5):
        frame, depth = _synthetic(truth)
        obs, _, (src, _, _) = observe(frame, depth)
        assert src == 'depth'
        assert abs(obs[5] - truth) < 0.015, \
            'range off at {} m: {} (hole leaking into the median?)'.format(truth, obs[5])

    # 3) Mono fallback: no depth at all, and depth that is all-invalid. Both must fall
    # back AND SAY SO, rather than reporting a depth range that was never measured.
    frame, _ = _synthetic(3.0)
    obs_none, _, (src_none, d_none, mono_none) = observe(frame, None)
    assert src_none == 'MONO', src_none
    assert math.isnan(d_none) and obs_none[5] == mono_none
    assert abs(obs_none[5] - 3.0) < 0.15, 'mono fallback off: {}'.format(obs_none[5])
    obs_inf, _, (src_inf, _, _) = observe(frame, np.full((H, W), np.inf, np.float32))
    assert src_inf == 'MONO', 'all-inf depth must not pass as a depth measurement'
    assert abs(obs_inf[5] - obs_none[5]) < 1e-6

    # 4) Off-centre ring reads to the right; blank frame and a big non-elliptical
    # orange blob both read as "no ring" (that blob once hijacked a whole mission).
    frame = np.zeros((H, W, 3), np.uint8)
    cv2.ellipse(frame, (int(W * 0.75), H // 2), (60, 40), 0, 0, 360, (0, 115, 255), -1)
    obs, _, _ = observe(frame)
    assert obs[0] == 1.0 and obs[1] > 0.3, obs[1]
    assert observe(np.zeros((H, W, 3), np.uint8))[0][0] == 0.0
    blob = np.zeros((H, W, 3), np.uint8)
    cv2.fillPoly(blob, [np.array([(100, 100), (300, 120), (140, 300)])], (0, 115, 255))
    assert observe(blob)[0][0] == 0.0, 'triangle blob passed the ring shape gate'

    print('gate_detect self-check OK (fx_color={:.1f} fx_depth={:.1f})'
          .format(FX_COLOR, FX_DEPTH))


def main():
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    gui = ('--gui' in sys.argv) or os.environ.get('GATE_GUI') == '1'
    rclpy.init()
    node = rclpy.create_node('gate_detect')
    try:
        run(node, gui)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
