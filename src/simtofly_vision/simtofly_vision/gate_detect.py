#!/usr/bin/env python3
"""Gate (racing ring) detector -- the VISION half of the fly-through mission.

Pulls the drone's gimbal camera H264 stream (Gazebo publishes it as RTP on
UDP 127.0.0.1:5600) and, every frame, finds the orange ring and fits an
ellipse to it. Publishes the ring geometry on /gate/obs; the CONTROL half
(simtofly_mavros_sitl/mission_gates.py) flies through it with two PIDs.

Why gst-launch as a subprocess instead of cv2.VideoCapture: the OpenCV build
on this machine was compiled WITHOUT GStreamer (cv2.getBuildInformation ->
"GStreamer: NO"), so CAP_GSTREAMER is unavailable. Instead we run the system
gst-launch-1.0 (the same pipeline the sim streams with) and read raw BGR
frames off its stdout. gz/gst-launch are installed on this machine.

Detector: deliberately CLASSICAL (HSV threshold -> largest contour ->
cv2.fitEllipse). The ring is a solid bright-orange hoop, so this pins the
ellipse -- including its "deformation" (minor/major axis ratio, the head-on
vs oblique cue the controller wants) -- with no trained model, on CPU, at
stream rate. To swap in a deep-learning detector, replace detect_ring() ONLY;
the published /gate/obs contract stays the same.  # ponytail: classical does
# exactly what DL would here (find one orange ellipse); DL needs a gate-trained
# model + dataset that don't exist yet -- upgrade path is one function.

/gate/obs = Float32MultiArray([visible, cx_off, cy_off, fill, defo]):
  visible : 1.0 if a ring was found this frame else 0.0
  cx_off  : ring-centre horizontal offset, -1 (left)  .. +1 (right)
  cy_off  : ring-centre vertical   offset, -1 (top)   .. +1 (bottom)
  fill    : ellipse major axis / image width  (apparent size ~ closeness)
  defo    : minor/major axis ratio  (1.0 = head-on circle -> 0 = edge-on)

Run:  ros2 run simtofly_vision gate_detect
(Gazebo with the gates world must already be up so the camera stream exists.)
"""
import math
import os
import shlex
import subprocess

import cv2
import numpy as np
import rclpy
from std_msgs.msg import Float32MultiArray

# ----- camera stream (matches models/gimbal_small_3d/model.sdf) -----
W, H = 640, 480                     # gimbal camera resolution
UDP_PORT = 5600
# The enable-streaming service topic embeds the WORLD name; the gates world is
# "gates_course" (NOT iris_runway). Override with GZ_WORLD if you rename it.
WORLD = os.environ.get('GZ_WORLD', 'gates_course')
ENABLE_TOPIC = ('/world/{w}/model/iris_with_gimbal/model/gimbal/link/'
                'pitch_link/sensor/camera/image/enable_streaming').format(w=WORLD)

# gst pipeline = the user's known-good receiver, but ending in a raw-BGR fdsink
# (read from stdout) instead of autovideosink. videoscale pins the frame size
# so the byte count per frame is exactly W*H*3 regardless of the sensor caps.
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

# ----- monocular depth: 2D detection (ellipse) + known ring size -----
# Pinhole: dist = focal_px * D_ring / major_axis_px. The ellipse major axis is
# the least-foreshortened diameter, so it stays ~ the true ring diameter even
# on an oblique approach -> robust range estimate that the controller flies by.
HFOV = 2.0                          # rad, camera horizontal FOV (model.sdf)
FOCAL_PX = (W / 2.0) / math.tan(HFOV / 2.0)   # ~205 px
RING_DIAM_M = 0.98                  # orange ring OUTER diameter (m); fitEllipse
# tracks the outer edge. CALIBRATE against a known standoff if dist reads off.
# ponytail: metric-scale knob -- the one number that sets range accuracy.


def enable_streaming():
    """Ask the Gazebo camera sensor to start streaming (best-effort, once)."""
    cmd = ['gz', 'topic', '-t', ENABLE_TOPIC, '-m', 'gz.msgs.Boolean',
           '-p', 'data: 1']
    try:
        subprocess.run(cmd, timeout=5,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('[gate_detect] enable_streaming -> {}'.format(ENABLE_TOPIC))
    except Exception as e:
        print('[gate_detect] enable_streaming failed ({}); is Gazebo up and '
              'the world name "{}" right?'.format(e, WORLD))


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
    """Largest orange ellipse -> ((cx, cy), (axisA, axisB), angle) or None.

    Swap THIS function for a deep-learning detector (e.g. YOLOv8-seg mask ->
    fitEllipse); keep the return shape and /gate/obs is unchanged.
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
            rings.append((cv2.contourArea(c), el))
    if not rings:
        return None
    return max(rings, key=lambda r: r[0])[1]


def obs_from_ellipse(el):
    """Fitted ellipse -> the 6-tuple published on /gate/obs.
    [visible, cx_off, cy_off, fill, defo, dist_m]."""
    if el is None:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    (cx, cy), (a1, a2), _ = el
    major, minor = max(a1, a2), min(a1, a2)
    dist = FOCAL_PX * RING_DIAM_M / major if major > 1e-6 else 0.0
    return [1.0,
            (cx - W / 2.0) / (W / 2.0),
            (cy - H / 2.0) / (H / 2.0),
            major / W,
            (minor / major) if major > 1e-6 else 0.0,
            dist]


def observe(frame):
    """Frame -> the 5-tuple published on /gate/obs."""
    return obs_from_ellipse(detect_ring(frame))


def draw_overlay(frame, el, obs):
    """Annotate the live frame: ring ellipse, its centre, image crosshair,
    and the obs values -- so `--gui` shows what the detector sees."""
    cx0, cy0 = W // 2, H // 2
    cv2.drawMarker(frame, (cx0, cy0), (255, 255, 0), cv2.MARKER_CROSS, 20, 1)
    vis, offx, offy, fill, defo, dist = obs
    if el is not None and vis > 0.5:
        centred = abs(offx) < 0.15 and abs(offy) < 0.15
        col = (0, 255, 0) if centred else (0, 165, 255)
        cv2.ellipse(frame, el, col, 2)
        cx, cy = int(el[0][0]), int(el[0][1])
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(frame, (cx0, cy0), (cx, cy), (0, 0, 255), 1)
    txt = '{}  off=({:+.2f},{:+.2f}) fill={:.2f} defo={:.2f} dist={:.2f}m'.format(
        'RING' if vis > 0.5 else 'searching', offx, offy, fill, defo, dist)
    cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)


def run(node, gui):
    """Main-thread capture loop: read frame -> detect -> publish (-> draw).

    gate_detect only PUBLISHES (no subscriptions/timers), so it needs no
    executor spin -- running the blocking capture in the main thread lets
    cv2.imshow (which must be on the main thread) work when gui=True.
    """
    pub = node.create_publisher(Float32MultiArray, '/gate/obs', 10)
    enable_streaming()
    proc = subprocess.Popen(shlex.split(GST), stdout=subprocess.PIPE)
    nbytes = W * H * 3
    n = 0
    try:
        while rclpy.ok():
            raw = read_exact(proc.stdout, nbytes)
            if raw is None:
                node.get_logger().warn('camera stream ended')
                break
            frame = np.frombuffer(raw, np.uint8).reshape(H, W, 3)
            el = detect_ring(frame)
            obs = obs_from_ellipse(el)
            pub.publish(Float32MultiArray(data=obs))
            n += 1
            if n % 30 == 0:              # ~1 Hz heartbeat
                node.get_logger().info(
                    'ring {} off=({:+.2f},{:+.2f}) fill={:.2f} defo={:.2f} '
                    'dist={:.2f}m'.format('SEEN' if obs[0] else 'none ',
                                          obs[1], obs[2], obs[3], obs[4], obs[5]))
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


def _selfcheck():
    """Draw a synthetic orange ellipse, assert observe() locates it.
    No camera/ROS. Run: python3 gate_detect.py --selfcheck"""
    frame = np.zeros((H, W, 3), np.uint8)
    # BGR of rgb(1,0.45,0) ~ (0,115,255); a ring off to the right, squashed.
    cv2.ellipse(frame, (int(W * 0.75), H // 2), (60, 40), 0, 0, 360,
                (0, 115, 255), -1)
    vis, cx, cy, fill, defo, dist = observe(frame)
    assert vis == 1.0, 'ring not detected'
    assert cx > 0.3, cx                       # correctly to the right
    assert abs(cy) < 0.1, cy                  # vertically centred
    assert 0.5 < defo < 0.85, defo            # ~40/60 axis ratio
    assert 1.0 < dist < 2.5, dist             # major~120px -> ~1.7 m
    assert observe(np.zeros((H, W, 3), np.uint8))[0] == 0.0  # blank -> none

    # Shape gate: a big non-elliptical orange blob must NOT read as a ring.
    blob = np.zeros((H, W, 3), np.uint8)
    cv2.fillPoly(blob, [np.array([(100, 100), (300, 120), (140, 300)])],
                 (0, 115, 255))                # triangle: too big for its ellipse
    assert observe(blob)[0] == 0.0, 'triangle blob passed the ring shape gate'
    print('gate_detect self-check OK (cx={:+.2f} fill={:.2f} defo={:.2f} '
          'dist={:.2f}m)'.format(cx, fill, defo, dist))


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
