"""
person_follow_reid.py
---------------------------------------------------------------
Bám 1 người (front + back) bằng 2 NHÁNH YOLO song song:
  - Nhánh detect : yolo11m.pt + BoT-SORT + OSNet ReID   -> box + ID, khóa/bám người
  - Nhánh pose   : yolo11m-pose.pt (17 điểm COCO)        -> skeleton + tâm thân + FRONT/BACK
Ghép 2 nhánh: lấy người trong nhánh pose trùng box mục tiêu nhất (IoU).

Lấy tâm người (ưu tiên giảm dần, mượt nhất cho điều khiển):
  1) trung bình 2 vai (5,6) + 2 hông (11,12)   [torso - ổn định nhất]
  2) trung điểm 2 vai                           [khi thiếu hông]
  3) tâm bbox                                    [khi không có pose]

(Bản gọn hơn: chỉ cần 1 model yolo11m-pose.pt + .track() là có cả box+ID+keypoint,
 nhẹ gấp đôi. Bản này tách 2 nhánh đúng theo yêu cầu.)
"""

import threading
import time
from collections import deque

import cv2
import numpy as np
import torch
from ultralytics import YOLO
try:
    from torchreid.reid.utils import FeatureExtractor
except ImportError:
    from torchreid.utils import FeatureExtractor

# ===== ROS2: publish lệnh sang node điều khiển (follow_control.py) =====
# 2 topic: /follow/command (String)  +  /follow/target (Vector3: ang_x, ang_y, z)
# QoS mặc định (RELIABLE, depth 10) -> KHỚP với subscriber bên follow_control.
ROS_OK = False
_node = _pub_cmd = _pub_tgt = None
try:
    import rclpy
    from std_msgs.msg import String
    from geometry_msgs.msg import Vector3
    ROS_OK = True
except Exception as _e:
    print(f"[ROS] rclpy không sẵn ({_e}) -> chỉ in, KHÔNG publish. "
          f"Hãy chạy trong terminal đã 'source /opt/ros/<distro>/setup.bash'.")


def init_ros():
    """Khởi tạo node + 2 publisher. Gọi 1 lần trong main()."""
    global _node, _pub_cmd, _pub_tgt
    if not ROS_OK:
        return
    rclpy.init()
    _node = rclpy.create_node("vision_follow")
    _pub_cmd = _node.create_publisher(String, "/follow/command", 10)
    _pub_tgt = _node.create_publisher(Vector3, "/follow/target", 10)
    print("[ROS] vision_follow node OK -> publishing /follow/command, /follow/target")


def shutdown_ros():
    if ROS_OK and _node is not None:
        _node.destroy_node()
        rclpy.shutdown()


# ===== CẤU HÌNH =====
DET_WEIGHTS  = "yolo11m.pt"        # nhánh detect + ID
POSE_WEIGHTS = "yolo11m-pose.pt"   # nhánh pose 17 điểm
DEVICE    = "cuda"
CONF      = 0.4
CAM_INDEX = 2                      # 2 là camera rời (Rapoo); 0 là webcam laptop ( check lại bằng `v4l2-ctl --list-devices` )
HFOV_DEG  = 70.0                   # góc nhìn NGANG của camera (tra spec/calib). Webcam ~60-70

# ===== ƯỚC LƯỢNG KHOẢNG CÁCH (mono + chiều cao người ~1.70m) =====
ADULT_HEIGHT_M = 1.70   # giả định người trưởng thành
# Các đoạn THẲNG ĐỨNG trên cơ thể + tỷ lệ chiều dài thật so với chiều cao H
# (nhân trắc học). Dùng đoạn dọc vì khi người xoay trái/phải (FRONT/BACK/side)
# chiều dài chiếu theo trục dọc ảnh gần như không đổi -> sai số thấp nhất.
# Ưu tiên đoạn DÀI nhất còn nhìn thấy (đoạn càng dài, sai số pixel càng nhỏ).
#   (tên, keypoint trên, keypoint dưới, tỷ_lệ/H)
DIST_SEGMENTS = [
    ("full",  [0],      [15, 16], 0.87),   # mũi -> cổ chân  (~1.48m)
    ("legs+", [5, 6],   [15, 16], 0.78),   # vai  -> cổ chân (~1.33m)
    ("torso", [5, 6],   [11, 12], 0.29),   # vai  -> hông    (~0.49m, khi chân bị che)
    ("thigh", [11, 12], [13, 14], 0.245),  # hông -> gối     (~0.42m)
    ("head",  [1, 2, 3, 4], [5, 6], 0.12), # mắt/tai -> vai  (~0.20m, TẦM GẦN: chỉ thấy đầu+vai)
]
MIN_SEG_PX = 20          # đoạn < 20px -> bỏ (sai số lượng tử hóa quá lớn)
DIST_RANGE = (0.3, 15.0) # mét, ngoài khoảng này coi là outlier

# ===== BÙ NGHIÊNG CHO CAMERA GẮN CỨNG (strapdown, ArduPilot) =====
# Camera gắn cứng -> khi drone nghiêng, trục quang không còn ngang:
#   - roll : ảnh xoay -> đo đoạn theo ĐỘ DÀI EUCLID (bất biến xoay) thay vì Δy.
#   - pitch: đoạn dọc bị co ~cos(pitch) -> chia lại để khôi phục.
# Attitude lấy từ MAVLink ATTITUDE (xem class AttitudeSource).
MAV_CONNECT = None       # vd "udp:127.0.0.1:14550" (SITL/MAVProxy) hoặc "/dev/serial0"
MAV_BAUD    = 57600
PITCH_CLAMP_COS = 0.34   # ~70deg: chặn chia cho số quá nhỏ khi pitch lớn

# ===== LỆNH ĐIỀU KHIỂN CẤP CAO (vision chỉ xuất command, control tự suy setpoint) =====
STANDOFF_MIN_M = 1.5     # Z < ngưỡng này -> quá gần -> BACK_OFF
# FOLLOW : thấy thân, bám bình thường (control giữ standoff theo z + yaw theo ang_x)
# BACK_OFF: quá gần / chỉ thấy đầu -> control lùi ra, tạm giữ yaw
# SEARCH : từng khóa mục tiêu, đang mất -> control hold + quét, KHÔNG tiến
# IDLE   : chưa khóa mục tiêu
CMD_COLORS = {"FOLLOW": (0, 255, 0), "BACK_OFF": (0, 165, 255),
              "SEARCH": (0, 0, 255), "IDLE": (180, 180, 180)}

# COCO-17: 0 nose,1 l_eye,2 r_eye,5 l_sho,6 r_sho,11 l_hip,12 r_hip ...
SKELETON = [(5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
            (0, 1), (0, 2), (1, 3), (2, 4), (0, 5), (0, 6)]
FACE_IDX, SHO_IDX, HIP_IDX = [0, 1, 2], [5, 6], [11, 12]


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def person_center(kp_xy, kp_conf, bbox, th=0.5):
    """Tâm người: torso -> vai -> tâm bbox."""
    core = [i for i in SHO_IDX + HIP_IDX if kp_conf[i] > th]
    if len(core) >= 3:
        c = kp_xy[core].mean(axis=0)
        return float(c[0]), float(c[1]), "torso"
    sho = [i for i in SHO_IDX if kp_conf[i] > th]
    if len(sho) == 2:
        c = kp_xy[sho].mean(axis=0)
        return float(c[0]), float(c[1]), "shoulders"
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2, (y1 + y2) / 2, "bbox"


def facing(kp_conf, th=0.5):
    return "FRONT" if float(np.mean(kp_conf[FACE_IDX])) > th else "BACK"


def estimate_distance(kp_xy, kp_conf, bbox, fy, frame_h, pitch_rad=0.0, th=0.5):
    """Khoảng cách camera->người (m), pinhole: Z = fy * L_thật / h_pixel.

    Tự chọn đoạn cơ thể tốt nhất theo thứ tự DIST_SEGMENTS:
      full -> legs+ -> torso -> thigh -> bbox (fallback).
    CAMERA GẮN CỨNG (strapdown):
      - Đo đoạn bằng ĐỘ DÀI EUCLID giữa 2 đầu (||p_bot - p_top||), KHÔNG chỉ |dy|
        -> bất biến với ROLL của drone (ảnh xoay không đổi chiều dài đoạn).
      - Bù PITCH: camera chúc xuống θ làm đoạn dọc co ~cos(θ); chia lại để khôi phục
        chiều dài tương đương khi camera ngang.  pitch_rad lấy từ read_attitude().
    Trả về (Z mét, tên đoạn) hoặc (None, None).
    """
    cosp = max(np.cos(pitch_rad), PITCH_CLAMP_COS)  # clamp tránh chia ~0 khi pitch quá lớn
    if kp_xy is not None and kp_conf is not None:
        for name, top_i, bot_i, ratio in DIST_SEGMENTS:
            top = [i for i in top_i if kp_conf[i] > th]
            bot = [i for i in bot_i if kp_conf[i] > th]
            if not top or not bot:
                continue
            p_top = kp_xy[top].mean(axis=0)
            p_bot = kp_xy[bot].mean(axis=0)
            seg = float(np.hypot(p_bot[0] - p_top[0], p_bot[1] - p_top[1]))  # roll-invariant
            if seg < MIN_SEG_PX:         # quá ngắn -> bỏ
                continue
            h_eff = seg / cosp           # khôi phục về "camera ngang" (bù pitch)
            return fy * ADULT_HEIGHT_M * ratio / h_eff, name
    # Fallback: chiều cao bbox ~ chiều cao người, CHỈ khi:
    #  - người không bị cắt bởi mép trên/dưới khung hình, VÀ
    #  - bbox có dáng "toàn thân đứng" (cao/rộng >= 1.8). Bbox đầu+vai khi đứng
    #    sát camera gần như vuông -> nếu coi là 1.70m sẽ thổi Z lên gấp nhiều lần.
    # LƯU Ý strapdown: bbox AABB phình to khi ảnh roll -> kém tin; chỉ dùng khi bí.
    x1, y1, x2, y2 = bbox
    bh, bw = (y2 - y1), max(x2 - x1, 1e-6)
    if y1 > 5 and y2 < frame_h - 5 and bh > 2 * MIN_SEG_PX and bh / bw >= 1.8:
        return fy * ADULT_HEIGHT_M / (bh / cosp), "bbox"
    return None, None


class AttitudeSource:
    """Đọc roll/pitch (RAD) của drone từ MAVLink ATTITUDE (ArduPilot).
    Quy ước ArduPilot/MAVLink: pitch>0 = MŨI NGẨNG LÊN, roll>0 = NGHIÊNG PHẢI.
    Chạy 1 thread nền đọc liên tục; nếu không kết nối được -> trả (0,0)
    (coi camera ngang -> hành vi y như chưa bù)."""

    def __init__(self, connect=None, baud=57600):
        self.roll = 0.0
        self.pitch = 0.0
        self._lock = threading.Lock()
        self.ok = False
        if not connect:
            return
        try:
            from pymavlink import mavutil
            self._m = mavutil.mavlink_connection(connect, baud=baud)
            self._m.wait_heartbeat(timeout=5)
            # Xin luồng ATTITUDE ~30Hz (ArduPilot dùng EXTRA1 cho attitude)
            self._m.mav.request_data_stream_send(
                self._m.target_system, self._m.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30, 1)
            threading.Thread(target=self._loop, daemon=True).start()
            self.ok = True
            print(f"[ATT] MAVLink OK: {connect}")
        except Exception as e:
            print(f"[ATT] không kết nối MAVLink ({e}) -> pitch=roll=0")

    def _loop(self):
        while True:
            msg = self._m.recv_match(type="ATTITUDE", blocking=True)
            if msg is not None:
                with self._lock:
                    self.roll, self.pitch = float(msg.roll), float(msg.pitch)

    def get(self):
        with self._lock:
            return self.roll, self.pitch


_ATT = None   # singleton, khởi tạo lười ở lần gọi đầu


def read_attitude():
    """(roll, pitch) RAD để bù camera GẮN CỨNG (strapdown) trên ArduPilot.
    - Khoảng cách: chia cos(pitch) -> KHÔNG phụ thuộc dấu pitch.
    - ang_y = ang_ảnh - pitch  (đúng với quy ước pitch NGẨNG-DƯƠNG của ArduPilot).
    Mặc định (0,0) khi MAV_CONNECT=None -> hành vi như camera ngang.
    KIỂM DẤU 1 lần: nghiêng/chúi drone bằng tay, xem ang_x/ang_y/Z có GIỮ NGUYÊN
    trên 1 vật cố định không; nếu lệch ngược -> đổi dấu roll hoặc pitch tại đây."""
    global _ATT
    if _ATT is None:
        _ATT = AttitudeSource(MAV_CONNECT, MAV_BAUD)
    return _ATT.get()


def decide_command(has_target, z, z_src, lost_target):
    """Quy trạng thái nhìn thấy mục tiêu -> 1 LỆNH cấp cao (không tính setpoint)."""
    if has_target:
        if z_src == "head":                       # chỉ thấy đầu+vai = quá gần để khung thân
            return "BACK_OFF"
        if z is not None and z < STANDOFF_MIN_M:  # quá gần theo khoảng cách
            return "BACK_OFF"
        return "FOLLOW"
    if lost_target:                               # từng khóa, đang mất -> quét tìm
        return "SEARCH"
    return "IDLE"


def emit_command(cmd, ang_x=None, ang_y=None, z=None):
    """Gửi LỆNH sang node điều khiển. GỌI MỖI FRAME.
    - Publish ra /follow/command + /follow/target mỗi frame (control cần stream liên tục).
    - In ra console khi lệnh ĐỔI cho đỡ rối."""
    # publish (mỗi frame)
    if ROS_OK and _pub_cmd is not None:
        _pub_cmd.publish(String(data=cmd))
        v = Vector3()
        v.x = float(ang_x) if ang_x is not None else 0.0
        v.y = float(ang_y) if ang_y is not None else 0.0
        v.z = float(z) if z is not None else -1.0   # z<=0 -> control coi như "không có z"
        _pub_tgt.publish(v)
    # in khi đổi lệnh
    if getattr(emit_command, "_last", None) != cmd:
        ax = f"{ang_x:+.1f}" if ang_x is not None else "--"
        ay = f"{ang_y:+.1f}" if ang_y is not None else "--"
        zz = f"{z:.2f}" if z is not None else "--"
        print(f"[CMD] {cmd:8s} ang_x={ax} ang_y={ay} z={zz}")
        emit_command._last = cmd


def draw_pose(frame, kp_xy, kp_conf, color, th=0.3):
    for a, b in SKELETON:
        if kp_conf[a] > th and kp_conf[b] > th:
            cv2.line(frame, tuple(kp_xy[a].astype(int)), tuple(kp_xy[b].astype(int)), color, 2)
    for i, p in enumerate(kp_xy):
        if kp_conf[i] > th:
            cv2.circle(frame, tuple(p.astype(int)), 3, color, -1)

def draw_crosshair(frame, size=14, color=(255, 255, 0)):
    """Dấu + tại tâm khung hình để so độ lệch mục tiêu."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (cx - size, cy), (cx + size, cy), color, 2)
    cv2.line(frame, (cx, cy - size), (cx, cy + size), color, 2)
    cv2.circle(frame, (cx, cy), 2, color, -1)

class TargetTracker:
    """Quản lý 1 mục tiêu duy nhất qua ID + ReID gallery."""

    def __init__(self, gallery_size=15, reacquire_sim=0.55,
                 diversity_sim=0.90, max_lost=90):
        self.target_id = None
        self.gallery = deque(maxlen=gallery_size)
        self.lost_frames = 0
        self.reacquire_sim = reacquire_sim
        self.diversity_sim = diversity_sim
        self.max_lost = max_lost

    def _best_sim(self, feat):
        if not self.gallery:
            return 0.0
        return max(float(np.dot(feat, g)) for g in self.gallery)

    def _add_to_gallery(self, feat):
        if self._best_sim(feat) < self.diversity_sim:
            self.gallery.append(feat)

    def update(self, ids, areas, feats):
        if len(ids) == 0:
            if self.target_id is not None:
                self.lost_frames += 1
                if self.lost_frames > self.max_lost:
                    self._reset()
            return None
        if self.target_id is None:
            idx = int(np.argmax(areas))
            self.target_id = ids[idx]
            self.gallery.clear()
            self._add_to_gallery(feats[idx])
            self.lost_frames = 0
            return idx
        if self.target_id in ids:
            idx = ids.index(self.target_id)
            self._add_to_gallery(feats[idx])
            self.lost_frames = 0
            return idx
        self.lost_frames += 1
        best_idx, best_sim = -1, 0.0
        for i, f in enumerate(feats):
            s = self._best_sim(f)
            if s > best_sim:
                best_sim, best_idx = s, i
        if best_sim >= self.reacquire_sim:
            self.target_id = ids[best_idx]
            self._add_to_gallery(feats[best_idx])
            self.lost_frames = 0
            return best_idx
        if self.lost_frames > self.max_lost:
            self._reset()
        return None

    def _reset(self):
        self.target_id = None
        self.gallery.clear()
        self.lost_frames = 0


def extract_features(extractor, frame, boxes):
    crops, valid = [], []
    h, w = frame.shape[:2]
    for i, (x1, y1, x2, y2) in enumerate(boxes):
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(w, int(x2)), min(h, int(y2))
        if x2 - x1 < 8 or y2 - y1 < 16:
            continue
        crops.append(cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2RGB))
        valid.append(i)
    feats = [None] * len(boxes)
    if crops:
        with torch.no_grad():
            emb = extractor(crops).cpu().numpy()
        emb /= (np.linalg.norm(emb, axis=1, keepdims=True) + 1e-9)
        for j, i in enumerate(valid):
            feats[i] = emb[j]
    return feats


def main():
    det_model = YOLO(DET_WEIGHTS)
    pose_model = YOLO(POSE_WEIGHTS)
    extractor = FeatureExtractor(model_name="osnet_x0_25", model_path="", device=DEVICE)
    tracker = TargetTracker()
    dist_hist = deque(maxlen=7)   # lọc median khoảng cách (chống nhiễu keypoint)
    prev_target_id = None
    init_ros()                    # ROS2: bật publisher /follow/command, /follow/target

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Không mở được camera index {CAM_INDEX}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(5):
        cap.read()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t0 = time.perf_counter()
        H, W = frame.shape[:2]

        # --- Nhánh 1: detect + BoT-SORT (ID) ---
        det = det_model.track(frame, persist=True, classes=[0],
                              conf=CONF, tracker="botsort.yaml", verbose=False)[0]
        ids, boxes, areas = [], [], []
        if det.boxes is not None and det.boxes.id is not None:
            xyxy = det.boxes.xyxy.cpu().numpy()
            tids = det.boxes.id.int().cpu().tolist()
            for b, tid in zip(xyxy, tids):
                ids.append(tid)
                boxes.append(b)
                areas.append((b[2] - b[0]) * (b[3] - b[1]))

        # --- Nhánh 2: pose 17 điểm (chạy 1 lần cho cả khung) ---
        pres = pose_model(frame, classes=[0], conf=CONF, verbose=False)[0]
        pose_boxes = pres.boxes.xyxy.cpu().numpy() if pres.boxes is not None else np.empty((0, 4))
        pose_kxy = pres.keypoints.xy.cpu().numpy() if pres.keypoints is not None else None
        pose_kcf = (pres.keypoints.conf.cpu().numpy()
                    if pres.keypoints is not None and pres.keypoints.conf is not None else None)

        # --- ReID + khóa/bám/tìm lại ---
        feats = extract_features(extractor, frame, boxes) if boxes else []
        keep = [i for i, f in enumerate(feats) if f is not None]
        ids_k = [ids[i] for i in keep]
        areas_k = [areas[i] for i in keep]
        feats_k = [feats[i] for i in keep]
        boxes_k = [boxes[i] for i in keep]
        tgt_idx = tracker.update(ids_k, areas_k, feats_k)

        # --- Vẽ mọi người (box + id) ---
        for i, (b, tid) in enumerate(zip(boxes_k, ids_k)):
            x1, y1, x2, y2 = map(int, b)
            is_tgt = (i == tgt_idx)
            color = (0, 255, 0) if is_tgt else (160, 160, 160)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2 if is_tgt else 1)
            if not is_tgt:
                cv2.putText(frame, f"id{tid}", (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # --- Lệnh mặc định mỗi frame (sẽ được ghi đè bên dưới nếu có thông tin) ---
        cmd, cmd_ax, cmd_ay, cmd_z = "IDLE", None, None, None

        # --- Ghép pose vào mục tiêu (IoU) + tâm + độ lệch + khoảng cách ---
        if tgt_idx is not None:
            tb = boxes_k[tgt_idx]
            cx, cy, src, ori = (tb[0]+tb[2])/2, (tb[1]+tb[3])/2, "bbox", "?"
            kp_xy_t, kp_conf_t = None, None
            if pose_kxy is not None and len(pose_boxes) > 0:
                ious = [iou(tb, pb) for pb in pose_boxes]
                j = int(np.argmax(ious))
                if ious[j] > 0.3:                      # tìm được pose của đúng người
                    kp_xy_t, kp_conf_t = pose_kxy[j], pose_kcf[j]
                    draw_pose(frame, kp_xy_t, kp_conf_t, (0, 255, 0))
                    cx, cy, src = person_center(kp_xy_t, kp_conf_t, tb)
                    ori = facing(kp_conf_t)
            cv2.line(frame, (W // 2, H // 2), (int(cx), int(cy)), (0, 0, 255), 1)
            cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            # px -> ĐỘ (pinhole): fx = (W/2)/tan(HFOV/2); pixel vuông -> fy = fx
            fx = (W / 2) / np.tan(np.radians(HFOV_DEG) / 2)
            # Camera GẮN CỨNG: lấy tư thế drone để khử roll + bù pitch
            roll, pitch = read_attitude()
            dx, dy = cx - W / 2, cy - H / 2
            # de-roll: xoay offset về hệ "camera ngang" theo roll của drone
            cr, sr = np.cos(roll), np.sin(roll)
            dx_l = cr * dx + sr * dy
            dy_l = -sr * dx + cr * dy
            ang_x = np.degrees(np.arctan2(dx_l, fx))                 # + : người lệch PHẢI
            ang_y = np.degrees(np.arctan2(dy_l, fx)) - np.degrees(pitch)  # + : DƯỚI đường chân trời thật
            # (đã trừ pitch -> ang_y là độ cao THẬT của người, không lẫn việc drone chúi)

            # --- Khoảng cách (mono, theo chiều cao người 1.70m, bù pitch) ---
            if tracker.target_id != prev_target_id:          # đổi mục tiêu -> xóa lịch sử
                dist_hist.clear()
                prev_target_id = tracker.target_id
            z_raw, z_src = estimate_distance(kp_xy_t, kp_conf_t, tb, fx, H, pitch_rad=pitch)
            if z_raw is not None and DIST_RANGE[0] < z_raw < DIST_RANGE[1]:
                dist_hist.append(z_raw)
            z = float(np.median(dist_hist)) if dist_hist else None
            z_txt = f"Z={z:.2f}m ({z_src})" if z is not None else "Z=?"

            # --- Lệnh cấp cao cho mục tiêu hiện tại ---
            cmd = decide_command(True, z, z_src, False)
            cmd_ax, cmd_ay, cmd_z = ang_x, ang_y, z

            cv2.putText(frame, f"TARGET id{ids_k[tgt_idx]} [{ori}] {z_txt}",
                        (int(tb[0]), int(tb[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(frame, f"center={src} ang_x={ang_x:+.1f}deg ang_y={ang_y:+.1f}deg",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        elif tracker.target_id is not None:
            cmd = decide_command(False, None, None, True)    # đang mất -> SEARCH
            cv2.putText(frame, f"SEARCHING... lost={tracker.lost_frames}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # --- Xuất LỆNH sang code điều khiển (mỗi frame) ---
        emit_command(cmd, cmd_ax, cmd_ay, cmd_z)

        fps = 1.0 / (time.perf_counter() - t0 + 1e-9)
        cv2.putText(frame, f"FPS {fps:.1f}  views {len(tracker.gallery)}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"CMD: {cmd}", (10, H - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, CMD_COLORS.get(cmd, (255, 255, 255)), 2)
        draw_crosshair(frame)
        cv2.imshow("person-follow 2yolo (q=quit, r=reset)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("r"):
            tracker._reset()
            dist_hist.clear()

    cap.release()
    cv2.destroyAllWindows()
    shutdown_ros()


if __name__ == "__main__":
    main()