#!/usr/bin/env python3
"""N7 plan_overlay -- draw the plan onto the camera image (plan v2 par. 9).

Projects, into every colour frame:

  * the Dijkstra path from N4          thin and faint -- the voxel staircase
  * the trajectory actually COMMANDED  thick -- what N5 really told the vehicle,
                                       taken from the setpoint stream itself
  * every gate on N2's map             circle + normal arrow; the ones N6 has
                                       judged done are drawn in the passed colour
  * a HUD                              gates done, speed, accel, tracking error

par. 9.1 asks for the Mueller curve sampled every 0.05 s off a `/traj/active`
topic carrying the quintic coefficients. That topic does not exist, and adding it
means a custom message package plus a second copy of the sampling maths that
could disagree with the one N5 flies. The 20 Hz setpoint stream is the same curve
already serialised -- N5 samples the quintic and publishes the result -- so this
subscribes to that instead. What you lose is the part of the trajectory still in
the FUTURE; what you gain is that the drawn line is by construction the line the
vehicle was actually given, not a re-derivation that might drift from it.

Also not drawn, for the same reason: tau/Tf of the running segment and the
trajectory cost. Both live inside N5 and neither is published anywhere.

View with:  ros2 run rqt_image_view rqt_image_view /plan_overlay/image

Run:  ros2 run simtofly_gates plan_overlay
      python3 plan_overlay.py --selfcheck      (projection maths, no ROS)
"""
import collections
import math

import cv2
import numpy as np

try:
    from simtofly_gates.voxel_map import MAP_FRAME, quat_to_matrix
except ImportError:
    from voxel_map import MAP_FRAME, quat_to_matrix

COLOR_TOPIC = '/d435i/color/image_raw'
INFO_TOPIC = '/d435i/color/camera_info'
OUT_TOPIC = '/plan_overlay/image'
TF_TIMEOUT = 0.02       # s. Same reasoning as gate_planner: a tf2 timeout is a
                        # BLOCKING wait on a single-threaded executor, so it can
                        # only ever starve the listener that would satisfy it.
MAX_TF_SLIP = 0.15      # s; newest-transform fallback beyond this is refused --
                        # par. 9.1 "TF chua co o stamp t -> ... qua thi bo frame
                        # (khong ve sai)". A wrong overlay is worse than none: it
                        # looks like a planner bug and costs an afternoon.
                        #
                        # DELIBERATELY looser than gate_planner's 0.05: the two
                        # pay different prices for the same staleness. There it
                        # writes obstacles into a persistent grid, where a wrong
                        # pose leaves a wall that outlives the frame. Here it
                        # shifts a drawn line for one frame, by slip x speed =
                        # 0.15 x 0.8 = 12 cm at 5 m range, about 15 px -- less
                        # than the line is thick. At 0.05 this dropped 60% of
                        # frames (measured: 524 of 1307 drawn) and the overlay
                        # ran at 12 Hz, under the 15-30 Hz par. 1.3 asks for.
Z_MIN = 0.05            # m; nearer than this along the optical axis is behind
                        # the camera as far as the projection is concerned
TRAIL_S = 15.0          # s of commanded positions kept for the drawn trajectory
SETPOINT_HZ = 20.0      # what N5 streams at; only sizes the trail buffer
R_GATE = 0.75           # m; ring hole radius, for the drawn circle

# BGR, because that is what cv2 draws in and what bgr8 ships.
C_PATH = (120, 120, 120)        # Dijkstra path: grey, deliberately quiet
C_TRAJ_NEAR = (60, 220, 255)    # commanded trajectory, near -> yellow
C_TRAJ_FAR = (200, 90, 40)      # ...and far -> blue. Gradient by depth (par. 9.1)
C_GATE = (60, 90, 240)          # gate still to fly: red
C_DONE = (90, 220, 90)          # gate N6 has passed: green
C_HUD = (245, 245, 245)


def project(pts_map, T_map_opt, K, shape):
    """World points -> (pixel, z) pairs, dropping whatever is behind the camera.

    `T_map_opt` is (R, t) for map <- optical, i.e. what tf2 hands back when asked
    for MAP_FRAME <- the image's frame. The projection needs the INVERSE, so it
    is applied here rather than making every caller remember which way round it
    goes -- getting that backwards produces a plausible-looking overlay that is
    simply wrong, which is exactly the failure MAX_TF_SLIP above exists to avoid.
    """
    R, t = T_map_opt
    pts = np.atleast_2d(np.asarray(pts_map, float))
    # np.atleast_2d([]) is shape (1, 0), NOT empty -- len() says 1 and the matmul
    # below then fails on a broadcast, so test the size.
    if pts.size == 0:
        return np.zeros((0, 2), int), np.zeros(0)
    opt = (pts - t) @ R                      # R.T @ (p - t), vectorised
    z = opt[:, 2]
    keep = z > Z_MIN
    opt, z = opt[keep], z[keep]
    if not len(opt):
        return np.zeros((0, 2), int), np.zeros(0)
    fx, fy, cx, cy = K
    u = fx * opt[:, 0] / z + cx
    v = fy * opt[:, 1] / z + cy
    # Keep off-frame points instead of dropping them (par. 9.1): one just outside
    # still anchors the segment leading to it, so the line runs to the edge
    # rather than vanishing. cv2.line clips for us; the bound only stops a
    # near-zero z from overflowing the int cast.
    u = np.clip(u, -1e4, 1e4)
    v = np.clip(v, -1e4, 1e4)
    return np.stack([u, v], 1).astype(int), z


def thickness_for(z):
    """Perspective-ish line width: thicker up close (par. 9.1 'do day ~ 1/z')."""
    return int(np.clip(6.0 / max(float(z), 0.3), 1, 8))


def lerp_color(a, b, s):
    s = float(np.clip(s, 0.0, 1.0))
    return tuple(int(round(a[i] + (b[i] - a[i]) * s)) for i in range(3))


def draw_polyline(img, uv, z, near, far, z_far=8.0, base=None):
    """Polyline whose colour and width follow depth, segment by segment."""
    for i in range(len(uv) - 1):
        zi = 0.5 * (z[i] + z[i + 1])
        col = base if base is not None else lerp_color(near, far, zi / z_far)
        cv2.line(img, tuple(uv[i]), tuple(uv[i + 1]), col,
                 1 if base is not None else thickness_for(zi), cv2.LINE_AA)


def circle_points(centre, normal, radius=R_GATE, n=24):
    """A ring of `n` points in the plane through `centre` normal to `normal`."""
    normal = np.asarray(normal, float)
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    # any vector not parallel to the normal works as the seed for the basis
    seed = np.array([0.0, 0.0, 1.0])
    if abs(seed @ normal) > 0.9:
        seed = np.array([1.0, 0.0, 0.0])
    a = np.cross(normal, seed)
    a /= np.linalg.norm(a) + 1e-12
    b = np.cross(normal, a)
    th = np.linspace(0, 2 * math.pi, n + 1)
    return centre + radius * (np.cos(th)[:, None] * a + np.sin(th)[:, None] * b)


def _build_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from geometry_msgs.msg import PoseArray, PoseStamped
    from mavros_msgs.msg import PositionTarget
    from nav_msgs.msg import Path
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                         LookupException, TransformListener)

    def mavros_qos():
        return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          durability=QoSDurabilityPolicy.VOLATILE,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10)

    def latched():
        return QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=QoSReliabilityPolicy.RELIABLE,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=1)

    class PlanOverlay(Node):
        def __init__(self):
            super().__init__('plan_overlay')
            self.tf_buf = Buffer()
            TransformListener(self.tf_buf, self)
            self.K = None
            self.path = None
            self.gates = []            # [(centre, normal)]
            self.passed = []           # centres N6 calls done
            self.trail = collections.deque(maxlen=int(TRAIL_S * SETPOINT_HZ))
            self.cmd_p = None
            self.vel = 0.0
            self.acc = 0.0
            self.pose = None
            self.n_frames = 0
            self.n_drawn = 0

            self.pub = self.create_publisher(Image, OUT_TOPIC, 1)
            self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 1)
            self.create_subscription(Image, COLOR_TOPIC, self.on_image, 1)
            self.create_subscription(Path, '/planner/path', self.on_path,
                                     latched())
            self.create_subscription(PoseArray, '/gate/map', self.on_gates, 10)
            self.create_subscription(PoseArray, '/mission/passed',
                                     self.on_passed, latched())
            self.create_subscription(PositionTarget,
                                     '/mavros/setpoint_raw/local',
                                     self.on_setpoint, 10)
            self.create_subscription(PoseStamped,
                                     '/mavros/local_position/pose',
                                     self.on_pose, mavros_qos())
            self.create_timer(5.0, self.heartbeat)

        # ---------- inputs ----------
        def on_info(self, msg):
            # K = [fx 0 cx; 0 fy cy; 0 0 1]. Taken from camera_info rather than
            # hardcoded: N1 hardcodes its intrinsics and says so, but an overlay
            # that silently uses the wrong focal length draws a plan that misses
            # the ring it is aimed at, and looks like a planner fault.
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

        def on_path(self, msg):
            self.path = np.array([[q.pose.position.x, q.pose.position.y,
                                   q.pose.position.z] for q in msg.poses])

        def on_gates(self, msg):
            out = []
            for p in msg.poses:
                o = p.orientation
                R = quat_to_matrix(o.x, o.y, o.z, o.w)
                out.append((np.array([p.position.x, p.position.y,
                                      p.position.z]), R[:, 2]))
            self.gates = out

        def on_passed(self, msg):
            self.passed = [np.array([p.position.x, p.position.y, p.position.z])
                           for p in msg.poses]

        def on_setpoint(self, msg):
            p = np.array([msg.position.x, msg.position.y, msg.position.z])
            self.cmd_p = p
            self.trail.append(p)
            v, a = msg.velocity, msg.acceleration_or_force
            self.vel = float(np.linalg.norm([v.x, v.y, v.z]))
            self.acc = float(np.linalg.norm([a.x, a.y, a.z]))

        def on_pose(self, msg):
            p = msg.pose.position
            self.pose = np.array([p.x, p.y, p.z])

        def heartbeat(self):
            """Say what is missing. An overlay node that publishes nothing and
            logs nothing is indistinguishable from a dead one."""
            missing = []
            if self.K is None:
                missing.append('camera_info')
            if not self.n_frames:
                missing.append('anh mau')
            if self.path is None:
                missing.append('/planner/path')
            if not self.trail:
                missing.append('setpoint N5')
            if missing:
                self.get_logger().info('dang doi: {}'.format(', '.join(missing)))
            else:
                self.get_logger().info(
                    've {}/{} frame ({} vong, {} xong)'.format(
                        self.n_drawn, self.n_frames, len(self.gates),
                        len(self.passed)))

        # ---------- TF ----------
        def lookup(self, header):
            """map <- the image's own frame, at its own stamp when possible.

            Same two-step as gate_planner.lookup: exact stamp first, then the
            newest transform provided it is no more than MAX_TF_SLIP stale.
            """
            import rclpy.duration
            import rclpy.time
            try:
                return self.tf_buf.lookup_transform(
                    MAP_FRAME, header.frame_id, header.stamp,
                    timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT))
            except ExtrapolationException:
                pass
            except (LookupException, ConnectivityException) as e:
                self.get_logger().warn(
                    'no TF {}<-{}: {}'.format(MAP_FRAME, header.frame_id, e),
                    throttle_duration_sec=5.0)
                return None
            try:
                tf = self.tf_buf.lookup_transform(MAP_FRAME, header.frame_id,
                                                  rclpy.time.Time())
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                return None
            want = header.stamp.sec + header.stamp.nanosec * 1e-9
            got = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if abs(want - got) > MAX_TF_SLIP:
                self.get_logger().warn(
                    'TF tre {:.0f} ms -- bo frame'.format((want - got) * 1e3),
                    throttle_duration_sec=5.0)
                return None
            return tf

        # ---------- the frame ----------
        def on_image(self, msg):
            self.n_frames += 1
            if self.K is None or msg.encoding not in ('rgb8', 'bgr8'):
                if self.K is None:
                    self.get_logger().warn('chua co camera_info',
                                           throttle_duration_sec=5.0)
                else:
                    self.get_logger().warn(
                        'encoding {} khong ho tro'.format(msg.encoding),
                        once=True)
                return
            tf = self.lookup(msg.header)
            if tf is None:
                return
            q, tr = tf.transform.rotation, tf.transform.translation
            T = (quat_to_matrix(q.x, q.y, q.z, q.w),
                 np.array([tr.x, tr.y, tr.z]))

            img = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)
            img = img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()
            self.draw(img, T)
            self.n_drawn += 1

            out = Image()
            out.header = msg.header
            out.height, out.width = img.shape[:2]
            out.encoding = 'bgr8'
            out.step = 3 * out.width
            out.data = img.tobytes()
            self.pub.publish(out)

        def draw(self, img, T):
            K, shape = self.K, img.shape
            if self.path is not None and len(self.path) > 1:
                uv, z = project(self.path, T, K, shape)
                draw_polyline(img, uv, z, C_TRAJ_NEAR, C_TRAJ_FAR, base=C_PATH)
            if len(self.trail) > 1:
                uv, z = project(np.array(self.trail), T, K, shape)
                draw_polyline(img, uv, z, C_TRAJ_NEAR, C_TRAJ_FAR)
            for centre, normal in self.gates:
                done = any(np.linalg.norm(centre - d) < 1.0 for d in self.passed)
                col = C_DONE if done else C_GATE
                uv, z = project(circle_points(centre, normal), T, K, shape)
                if len(uv) > 1:
                    draw_polyline(img, uv, z, col, col, base=col)
                # normal arrow, so the direction the gate must be flown is visible
                tip, zt = project(np.array([centre, centre + 0.8 * normal]),
                                  T, K, shape)
                if len(tip) == 2:
                    cv2.arrowedLine(img, tuple(tip[0]), tuple(tip[1]), col, 2,
                                    cv2.LINE_AA, tipLength=0.3)
            self.hud(img)

        def hud(self, img):
            err = ('--' if (self.pose is None or self.cmd_p is None)
                   else '{:.2f} m'.format(
                       float(np.linalg.norm(self.pose - self.cmd_p))))
            lines = [
                'vong: {}/{} xong'.format(len(self.passed), len(self.gates)),
                '|v| {:.2f} m/s   |a| {:.2f} m/s2'.format(self.vel, self.acc),
                # NOT the same quantity as N5's tracking guard, and must not be
                # labelled as if it were. This is the RAW gap to the current
                # setpoint, which at cruise is mostly the GUIDED shaping lag
                # (~0.5 m at 0.8 m/s). N5 subtracts that lag before deciding
                # anything -- see SHAPE_LAG in trajectory_follower. Showing the
                # raw number under the name "tracking error" would make a
                # perfectly healthy flight look like it was 0.7 m off the plan.
                'lech so voi lenh: {} (gom ca tre GUIDED)'.format(err),
                'path: {} wpt'.format(0 if self.path is None else len(self.path)),
            ]
            for i, s in enumerate(lines):
                org = (8, 20 + 18 * i)
                # black underlay first: white text over a bright sky is unreadable
                cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            C_HUD, 1, cv2.LINE_AA)

    return rclpy, PlanOverlay


def main():
    rclpy, PlanOverlay = _build_node()
    rclpy.init()
    node = PlanOverlay()
    node.get_logger().info('N7 plan_overlay -> {}'.format(OUT_TOPIC))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


def _selfcheck():
    """Projection maths, no ROS:  python3 plan_overlay.py --selfcheck"""
    K = (616.9, 616.9, 424.0, 240.0)
    shape = (480, 848, 3)
    # camera at the origin looking along +X of map, optical axes: z fwd (=+X),
    # x right (=-Y), y down (=-Z). That is the rpy(-90, 0, -90) the launch ships.
    R = np.array([[0.0, 0.0, 1.0],
                  [-1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0]])
    T = (R, np.zeros(3))

    # a point straight ahead lands on the principal point
    uv, z = project([[5.0, 0.0, 0.0]], T, K, shape)
    assert len(uv) == 1 and abs(z[0] - 5.0) < 1e-9, (uv, z)
    assert np.allclose(uv[0], [424, 240]), uv

    # ...one to the LEFT in map (+Y) must land LEFT of centre in the image
    uv, _ = project([[5.0, 1.0, 0.0]], T, K, shape)
    assert uv[0][0] < 424, uv
    # ...and one ABOVE (+Z) must land ABOVE centre (v smaller: image y is down)
    uv, _ = project([[5.0, 0.0, 1.0]], T, K, shape)
    assert uv[0][1] < 240, uv

    # behind the camera is dropped, never wrapped round to a bogus pixel
    assert len(project([[-5.0, 0.0, 0.0]], T, K, shape)[0]) == 0
    assert len(project([], T, K, shape)[0]) == 0
    # a mixed batch keeps only what is in front
    uv, z = project([[5, 0, 0], [-5, 0, 0], [3, 0, 0]], T, K, shape)
    assert len(uv) == 2 and np.allclose(z, [5.0, 3.0]), (uv, z)

    # the transform really is inverted: move the CAMERA +1 in X and the same
    # world point must project nearer, not further
    _, z_moved = project([[5.0, 0.0, 0.0]], (R, np.array([1.0, 0.0, 0.0])),
                         K, shape)
    assert abs(z_moved[0] - 4.0) < 1e-9, z_moved

    # a gate ring lies in its own plane: every point is R_GATE from the centre,
    # and none of them has any component along the normal
    C, n = np.array([5.0, 0.0, 3.5]), np.array([1.0, 0.0, 0.0])
    ring = circle_points(C, n)
    assert np.allclose(np.linalg.norm(ring - C, axis=1), R_GATE)
    assert np.allclose((ring - C) @ n, 0.0, atol=1e-9)
    # ...including when the normal is the axis the basis seed uses
    ring_z = circle_points(C, np.array([0.0, 0.0, 1.0]))
    assert np.allclose(np.linalg.norm(ring_z - C, axis=1), R_GATE)

    # near lines are drawn thicker than far ones, and nothing degenerates to 0
    assert thickness_for(0.5) > thickness_for(8.0) >= 1
    assert lerp_color((0, 0, 0), (10, 20, 30), 0.5) == (5, 10, 15)

    # drawing over a real buffer must not throw or leave it untouched
    img = np.zeros(shape, np.uint8)
    uv, z = project(np.array([[3.0, -0.5, 0.0], [5.0, 0.5, 0.0]]), T, K, shape)
    draw_polyline(img, uv, z, C_TRAJ_NEAR, C_TRAJ_FAR)
    assert img.any(), 'polyline khong ve duoc gi'

    print('plan_overlay N7 self-check OK (chieu diem, vong, ve polyline)')


if __name__ == '__main__':
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
    else:
        main()
