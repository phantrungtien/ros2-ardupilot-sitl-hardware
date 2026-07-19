# KẾ HOẠCH THIẾT KẾ HỆ THỐNG DRONE TỰ ĐỘNG BAY QUA VÒNG — **v2**
## ROS2 + ArduPilot SITL + Gazebo + RealSense D435i + Dijkstra Planner + **Mueller Trajectory Generator** + Camera Overlay

> **Phạm vi:** Kế hoạch, pipeline, mapping, thuật toán, sơ đồ giải thuật cho **mô phỏng** trên nền mô hình Gazebo đã build sẵn (`gazebo_ardupilot_depth_camera`).

---

# CHANGELOG v1 → v2

| # | Thay đổi | Lý do |
|---|---|---|
| C1 | **Thêm tầng sinh quỹ đạo Mueller** (bậc 5, tối thiểu jerk) làm lõi node N5 | v1 chỉ có carrot-chasing + spline hình học → không kiểm soát được vận tốc/gia tốc giữa các waypoint |
| C2 | **Đổi giao diện setpoint**: `/mavros/setpoint_position/local` (PoseStamped) → `/mavros/setpoint_raw/local` (PositionTarget, pos+vel+acc+yaw) | ArduCopter hiện đại **có** hỗ trợ pos+vel+accel (đã xác minh, xem §5.1). Không đổi thì quỹ đạo Mueller mất hết giá trị |
| C3 | **Đoạn TRAVERSE** không còn ép đường thẳng + v cố định → 2 đoạn quỹ đạo Mueller với ràng buộc vận tốc = n, gia tốc = 0 tại tâm vòng | Chui vòng mượt, đúng tư thế, không giật vận tốc ở hai đầu |
| C4 | **Bỏ spline Catmull-Rom** ở §4.4(3) và bỏ gán vận tốc thủ công ở §4.4(5) | Bị Mueller thay thế hoàn toàn (Mueller cho cả hình học lẫn profile thời gian) |
| C5 | **Giới hạn feasibility lấy theo tham số ArduCopter**, không lấy theo giới hạn vật lý motor | ArduCopter clamp trước (ANGLE_MAX 30° ⇒ a_ngang ≤ 5.66 m/s²) — đây mới là ràng buộc thắt |
| C6 | **Sửa điều kiện PASSED**: nội suy điểm cắt mặt phẳng thay vì kiểm tra tại vị trí hiện tại | v1 có lỗi logic — xem §7 mục R3 |
| C7 | **Cảnh báo hiệu năng Dijkstra**: 600k voxel × 26 láng giềng ≈ 15.6M cạnh, **không** đạt <200 ms nếu viết Python | v1 nêu <200 ms mà không nói điều kiện — xem §7 mục R1 |
| C8 | Thêm nguồn ước lượng gia tốc `a₀`, thống nhất tên topic, thêm metric jerk/tracking | Rà soát nhất quán |
| **C9** | **Viết lại §5.3**: kiểm feasibility theo `WPNAV_SPEED`/`WPNAV_ACCEL`/`PSC_JERK_XY` thay vì thrust/body-rate; **thêm giới hạn jerk** (thiếu hẳn); sửa `a_horiz_max` 4.0 → **1.75 m/s²** | Đọc source ArduCopter: GUIDED đã tự jerk-shape sẵn (`shape_pos_vel_accel_xy`). `WPNAV_ACCEL` mặc định chỉ **2.5 m/s²**, nên giá trị 4.0 của bản trước **đã bị clamp ngay từ đề xuất** — đúng cái bẫy C5 cảnh báo |
| **C10** | **Thêm §13 — Chuyển sang phần cứng thật**: ngân sách sai số 50 mm, nhiễu depth ~Z², trôi frame `map`, ngân sách độ trễ, tính toán trên companion | v1 và v2 đều chỉ gia cố tầng control. Cái sẽ giết mission thật nằm ở perception/state-estimation/độ trễ — chưa có dòng nào |

Danh sách đầy đủ các điểm cần hiệu chỉnh phát hiện khi rà soát: **§7**.

---

# PHẦN 0 — GIẢ ĐỊNH VỀ MÔ HÌNH ĐÃ BUILD & VIỆC CẦN BỔ SUNG

| Thành phần | Giả định | Trạng thái |
|---|---|---|
| Simulator | Gazebo + plugin `ardupilot_gazebo` | Kiểm phiên bản → chọn `ros_gz_bridge` hay `gazebo_ros` |
| Drone | Iris quadcopter, ArduPilot SITL (ArduCopter) qua JSON backend | `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON` |
| Camera | D435i hướng trước | ✅ `/d435i/color/image_raw`, `/d435i/depth/image_rect_raw`, `/d435i/color/camera_info`, `/d435i/depth/points`, `/d435i/imu` |
| Depth encoding | ✅ **`32FC1` mét** (SDF `R_FLOAT32`) | Vẫn assert lại ở frame đầu N1 |
| ROS2 | ✅ Humble + MAVROS (`mavros_apm.launch.py`), SITL `udp://:14550` | — |
| **Phiên bản ArduCopter** | **≥ 4.1** (bắt buộc, xem §5.1) | ⚠️ **PHẢI KIỂM TRA** — quyết định C2 |
| Vòng (gate) | Chưa có | Bổ sung, xem §0.1 |

## 0.1. Bổ sung model vòng vào world

- Model SDF `gate_ring`: **torus** (hoặc 8–12 cylinder ghép), bán kính trong `r_in = 0.75 m` (lọt lòng 1.5 m), độ dày ống `0.08 m`.
- **Màu nổi bật đồng nhất** (cam/đỏ) → phân đoạn HSV, không cần train YOLO giai đoạn đầu.
- Có `<collision>` để depth camera thấy vành và để va chạm là thật.
- Kịch bản: 3–5 vòng, cách nhau 4–8 m, cao 1.5–3 m, có vòng **nghiêng yaw ±30°** và **lệch độ cao**.
- Đặt tên `gate_1, gate_2, ...` — **chỉ dùng làm ground-truth đánh giá**, thuật toán không được đọc pose thật từ Gazebo.

## 0.2. Việc chuẩn bị MỚI của v2 — đọc tham số ArduCopter

Trước khi thiết kế giới hạn quỹ đạo, phải đọc các tham số sau từ SITL (`param show`), vì chúng là **ràng buộc thắt thực sự**:

```
ANGLE_MAX          (mặc định 3000 = 30°)  → giới hạn gia tốc ngang
WPNAV_SPEED        (cm/s)                 → giới hạn vận tốc ngang
WPNAV_ACCEL        (cm/s²)                → giới hạn gia tốc ngang của bộ điều khiển
WPNAV_SPEED_UP/DN, WPNAV_ACCEL_Z          → giới hạn theo trục z
PSC_*                                     → gain bộ điều khiển vị trí
GUID_OPTIONS                              → tuỳ chọn chế độ GUIDED
```

---

# PHẦN 1 — KIẾN TRÚC TỔNG THỂ

## 1.1. Nguyên tắc phân tầng (bất di bất dịch)

```
ArduPilot (SITL) = ổn định bay, EKF, failsafe, mixer      → KHÔNG đụng vào
ROS2 (companion) = perception, bản đồ, planner, sinh
                   quỹ đạo, máy trạng thái                 → toàn bộ phần "AI"
```

ROS2 chỉ gửi **setpoint** ở chế độ **GUIDED**; không bao giờ can thiệp vòng điều khiển trong của ArduPilot, không override failsafe, không bao giờ viết PWM/PID attitude từ companion.

## 1.2. Sơ đồ khối hệ thống (v2)

```
┌────────────────────────────── GAZEBO ──────────────────────────────┐
│  World (sàn + các gate_ring)      Iris + D435i (RGB/Depth/Info)    │
└──────┬──────────────────────────────────────────────┬──────────────┘
       │ JSON lockstep                                │ ros_gz_bridge
       ▼                                              ▼
┌─────────────────┐  MAVLink   ┌──────────┐   /d435i/color/image_raw
│ ArduPilot SITL  │◄──────────►│  MAVROS2 │   /d435i/depth/image_rect_raw
│  (ArduCopter)   │  udp:14550 │          │   /d435i/color/camera_info
└─────────────────┘            └────┬─────┘
                                    │ /mavros/local_position/pose      (ENU)
                                    │ /mavros/local_position/velocity_local
                                    │ /mavros/setpoint_raw/local       ◄── ĐỔI (C2)
                                    │ /mavros/state, arming, set_mode, takeoff
     ┌──────────────────────────────┴───────────────────────────────┐
     │                        ROS2 STACK                            │
     │                                                              │
     │  [N1] gate_detector       RGB+Depth → pose 3D các vòng       │
     │  [N2] gate_tracker/map    lọc, gán ID, quản lý danh sách     │
     │  [N3] voxel_mapper        Depth → occupancy voxel 3D + inflate│
     │  [N4] dijkstra_planner    đồ thị voxel → path ngắn nhất      │
     │  [N5] trajectory_manager  ★ MUELLER: path → quỹ đạo bậc 5    │
     │                             → sample p,v,a @20Hz → setpoint  │
     │  [N6] mission_fsm         máy trạng thái toàn cục            │
     │  [N7] plan_overlay        chiếu path + quỹ đạo lên ảnh (GUI) │
     └──────────────────────────────────────────────────────────────┘
```

## 1.3. Bảng topic thiết kế (v2)

| Topic | Kiểu msg | Nguồn → Đích | Tần số |
|---|---|---|---|
| `/d435i/color/image_raw` | sensor_msgs/Image (rgb8, 848×480) | Gazebo → N1, N7 | 30 Hz |
| `/d435i/depth/image_rect_raw` | sensor_msgs/Image (**32FC1 mét**) | Gazebo → N1, N3 | 30 Hz |
| `/d435i/color/camera_info` | sensor_msgs/CameraInfo (fx=fy=616.9, cx=424, cy=240) | Gazebo → N1, N7 | 30 Hz |
| `/d435i/depth/points` | sensor_msgs/PointCloud2 | Gazebo → N3 | 30 Hz |
| `/mavros/local_position/pose` | geometry_msgs/PoseStamped (ENU) | MAVROS → tất cả | 30–50 Hz |
| `/mavros/local_position/velocity_local` | geometry_msgs/TwistStamped (ENU) | MAVROS → **N5** | 30–50 Hz |
| `/gates/detections` | `GateDetectionArray` | N1 → N2 | ~15 Hz |
| `/gates/map` | `GateArray` (ID, passed/pending) | N2 → N4, N5, N6, N7 | 5 Hz |
| `/mapping/voxel_grid` | tự định nghĩa / MarkerArray | N3 → N4 | 5 Hz |
| `/planner/path` | nav_msgs/Path (frame `map`) | N4 → N5, N7 | theo sự kiện |
| **`/traj/active`** ★ | **`TrajectorySegmentArray`** (α,β,γ mỗi trục, t_start, Tf, yaw) | **N5 → N7, log** | theo sự kiện |
| **`/mavros/setpoint_raw/local`** ★ | **mavros_msgs/PositionTarget** (ENU, pos+vel+acc+yaw) | **N5 → MAVROS** | **20 Hz liên tục** |
| `/plan_overlay/image` | sensor_msgs/Image | N7 → rqt/RViz/Foxglove | 15–30 Hz |

> **Ghi chú C8:** v1 vẽ `/camera/...` trong sơ đồ khối nhưng dùng `/d435i/...` trong bảng. v2 thống nhất dùng `/d435i/...` ở mọi nơi.

## 1.4. Cây frame TF

```
map (== odom của EKF ArduPilot, ENU)
 └── base_link            (từ /mavros/local_position/pose)
      └── camera_link      (extrinsic tĩnh: x=+0.116, y=+0.0175)
           └── d435i_color_optical_frame   (optical: z-trước, x-phải, y-xuống)
```

| Hệ | Quy ước | Dùng ở đâu |
|---|---|---|
| Optical | X-phải, Y-xuống, Z-trước | back-projection pixel→3D |
| ROS/ENU | X-Đông, Y-Bắc, Z-Lên | map, planner, **quỹ đạo Mueller**, setpoint |
| NED | X-Bắc, Y-Đông, Z-Xuống | nội bộ ArduPilot — MAVROS tự chuyển, ta chỉ làm việc ENU |

Mọi detect **phải chuyển về `map` ngay** kèm timestamp của frame ảnh (tf2 lookup đúng thời điểm ảnh, **không** dùng pose mới nhất).

> ⚠️ **Gotcha mới của v2:** quỹ đạo Mueller cần vector trọng lực. Trong **ENU**, `gravity = (0, 0, −9.81)`. Sai dấu ở đây làm toàn bộ kiểm tra thrust sai mà quỹ đạo vẫn "trông đúng" — bug rất khó thấy.

---

# PHẦN 2 — PERCEPTION: TỪ D435i RA POSE 3D CỦA VÒNG (N1, N2)

*(Không đổi so với v1 — pipeline này độc lập với thay đổi ở tầng điều khiển.)*

## 2.1. Insight quan trọng nhất

> **Tâm vòng là lỗ rỗng** — depth tại tâm là background. **Không lấy depth ở tâm bounding box.** Phải lấy depth trên **vành (rim)**, rồi suy ra tâm 3D + pháp tuyến bằng hình học.

## 2.2. Pipeline node N1 — gate_detector

```
RGB ──► (1) Đồng bộ RGB–Depth theo timestamp (message_filters, lệch < 33 ms)
        (2) Phân đoạn màu HSV → mask nhị phân
        (3) Morphology open/close → khử nhiễu
        (4) findContours → lọc theo: diện tích tối thiểu; fitEllipse được (≥5 điểm);
            tỉ lệ trục b/a > 0.25; độ "rỗng ruột" (tỉ lệ pixel mask trong ellipse thấp)
        (5) Với mỗi ellipse ứng viên:
              a. N ≈ 36 pixel mẫu TRÊN VÀNH (dọc chu vi, dày ±3 px)
              b. Đọc depth, loại 0/NaN/ngoài [0.3, 8] m
              c. Cần ≥ 12 điểm hợp lệ, ngược lại bỏ
              d. Back-project: X=(u−cx)Z/fx, Y=(v−cy)Z/fy
        (6) RANSAC fit MẶT PHẲNG → normal n, loại outlier
        (7) Chiếu inlier lên mặt phẳng → fit ĐƯỜNG TRÒN 2D → tâm C_cam, r_est
        (8) Kiểm chéo: |r_est − 0.75| < 0.15 m
        (9) TF: C_cam, n_cam → C_map, n_map (đúng timestamp ảnh)
        (10) Chuẩn hóa hướng: chọn n sao cho n·(C − p_drone) > 0
        (11) Publish GateDetection {C_map, n_map, r_est, độ tin cậy}
```

Độ tin cậy = f(số điểm vành hợp lệ, RMS fit tròn, tỉ lệ inlier RANSAC). Dưới ngưỡng → không publish.

**Nâng cấp tùy chọn:** thay (2)–(4) bằng YOLOv8; các bước (5)→(11) giữ nguyên — lý do tách detection 2D và định vị 3D thành hai tầng.

## 2.3. Node N2 — gate_tracker

1. **Data association:** detection mới ↔ gate đã biết nếu `‖C_new − C_i‖ < 1.0 m`; không khớp → gate mới (`TENTATIVE`).
2. **Lọc trung bình có trọng số / EKF đơn giản** theo độ tin cậy. `TENTATIVE → CONFIRMED` sau ≥ 5 detection nhất quán.
3. **Thứ tự vòng:** theo kịch bản, hoặc greedy "CONFIRMED chưa qua, gần nhất theo hướng bay".
4. **Trạng thái:** `TENTATIVE / CONFIRMED / TARGET / PASSED`.
5. Gate CONFIRMED **không bị xóa khi ra khỏi FOV** — bản đồ vòng bền vững.

---

# PHẦN 3 — MAPPING: VOXEL OCCUPANCY 3D (N3)

## 3.1. Thiết kế lưới

| Tham số | Giá trị | Lý do |
|---|---|---|
| Kích thước voxel | 0.20 m | lọt lòng 1.5 m ≈ 7 voxel |
| Phạm vi lưới | 30×20×8 m quanh gốc `map` | ~600k voxel |
| Cập nhật | ray-casting log-odds: dọc tia = FREE, điểm cuối = OCCUPIED | tự clearing |
| Tần số | 5 Hz (subsample depth 4×) | môi trường tĩnh |
| Inflation | `r_robot(0.4) + margin(0.2) = 0.6 m` | drone là điểm trong đồ thị |
| Chưa quan sát | `UNKNOWN` — đi được nhưng **phạt chi phí** | tránh bế tắc lúc mới cất cánh |

## 3.2. Xử lý đặc biệt cho vòng (mấu chốt, giữ nguyên từ v1)

- Vành vòng mảnh → sau inflation 0.6 m, lỗ còn `1.5 − 2×0.6 = 0.3 m` → Dijkstra gần như không thể tự chui.
- **Giải pháp:** không bắt planner tự tìm lỗ. Với vòng TARGET, **khoét cưỡng bức "ống đi qua" (virtual corridor)**: hình trụ bán kính `r_in − 0.35 = 0.4 m`, trục = pháp tuyến n, dài 1.5 m xuyên tâm; voxel trong ống ép về FREE chi phí thấp. Vành vẫn OCCUPIED + inflate.
- Vòng **không phải** TARGET: giữ nguyên inflation đầy đủ (cấm bay xuyên sai thứ tự).

> **Ghi chú v2:** virtual corridor vẫn cần, nhưng vai trò nhẹ đi — vì đoạn xuyên vòng giờ do Mueller sinh trực tiếp (§5.4) chứ không do Dijkstra. Corridor chủ yếu để Dijkstra **tìm được đường tới P_pre** mà không bị vành vòng chặn.

---

# PHẦN 4 — GLOBAL PLANNER: DIJKSTRA (N4)

## 4.1. Cấu trúc bài toán — chuỗi anchor

```
Với mỗi vòng i (tâm C_i, pháp tuyến n_i đã chuẩn hóa cùng chiều bay):
   P_pre_i  = C_i − d_pre  · n_i     (d_pre  = 1.5 m)
   P_gate_i = C_i
   P_post_i = C_i + d_post · n_i     (d_post = 1.5 m)

Nhiệm vụ = Dijkstra(vị trí hiện tại → P_pre_1)
         + ★ QUỸ ĐẠO MUELLER  P_pre_1 → P_gate_1 → P_post_1   (§5.4)
         + Dijkstra(P_post_1 → P_pre_2)
         + ... lặp ...
         + Dijkstra(P_post_cuối → điểm hạ cánh)
```

Đoạn xuyên vòng **không giao cho Dijkstra**. v1 ép đường thẳng; **v2 dùng quỹ đạo Mueller có ràng buộc vận tốc/gia tốc tại tâm** — mượt hơn và vẫn đảm bảo chui vuông góc qua tâm.

## 4.2. Định nghĩa đồ thị

- **Đỉnh:** voxel FREE hoặc UNKNOWN (sau inflation, đã khoét corridor).
- **Cạnh:** 26-láng giềng.

```
w(u,v) = dist(u,v) × [ 1 + λ_c · cost_clearance(v) + λ_u · 1{v là UNKNOWN} ]

  dist(u,v)          = 1, √2, √3 (× voxel_size)
  cost_clearance(v)  = max(0, 1 − d_obs(v)/d_safe)    (distance transform 3D, d_safe = 1.0 m)
  λ_c = 2.0   → ngắn NHƯNG né xa vật cản
  λ_u = 0.5   → hơi ngại vùng chưa quan sát
```

- Ràng buộc độ cao: cấm `z < 0.5 m` và `z > z_max`.

## 4.3. Giả mã Dijkstra

```
DIJKSTRA(G, s, t):
    dist[v] ← ∞ ∀v;  dist[s] ← 0;  prev[v] ← null
    PQ ← min-heap {(0, s)}
    visited ← ∅
    while PQ không rỗng:
        (d, u) ← pop(PQ)
        if u ∈ visited: continue          # lazy deletion
        visited ← visited ∪ {u}
        if u == t: break                  # dừng sớm
        for v in 26_neighbors(u):
            if occupied(v) or ngoài lưới or vi phạm z: continue
            nd ← d + w(u, v)
            if nd < dist[v]:
                dist[v] ← nd;  prev[v] ← u
                push(PQ, (nd, v))
    if dist[t] == ∞: return FAILURE
    path ← truy ngược prev từ t về s, đảo chiều
    return path
```

Độ phức tạp `O(E log V)`.

> ⚠️ **HIỆU CHỈNH C7 (quan trọng):** v1 nói "<100–200 ms" mà không nêu điều kiện. Thực tế 600k đỉnh × 26 láng giềng ≈ **15.6 triệu lần relax cạnh**. Con số <200 ms chỉ đạt được nếu:
> 1. **Viết N3/N4 bằng C++** (Python thuần sẽ mất vài giây — trượt yêu cầu), **và/hoặc**
> 2. **Thu hẹp vùng tìm kiếm**: chỉ dựng đồ thị trong hộp bao của đoạn `[s, t]` nở thêm 3 m, thay vì toàn lưới 30×20×8. Với đoạn 5 m điển hình, hộp ~11×6×6 m ≈ 50k voxel — nhanh gấp ~12 lần.
>
> **Khuyến nghị:** làm cả hai. Nếu buộc phải dùng Python, hạ voxel xuống 0.25 m **và** bắt buộc thu hẹp hộp, rồi đo lại — đừng giả định.

**Ghi chú trung thực về "tối ưu":** Dijkstra cho đường ngắn nhất **theo hàm chi phí đã định nghĩa**. A* = Dijkstra + heuristic Euclid, cùng kết quả nhưng nhanh hơn — nêu trong báo cáo như hướng mở rộng.

## 4.4. Hậu xử lý path (v2 — đã rút gọn)

```
Path voxel ──► (1) Shortcut / line-of-sight pruning:
                    hai con trỏ, nếu đoạn thẳng p_i→p_j không cắt voxel OCCUPIED
                    (ray-cast 3D) thì bỏ các điểm giữa
           ──► (2) Nội suy lại waypoint cách đều 0.5 m
           ──► (3) ❌ BỎ spline Catmull-Rom  → Mueller thay thế (C4)
           ──► (4) ❌ BỎ gán vận tốc thủ công → Mueller thay thế (C4)
           ──► (5) Gán yaw mong muốn cho từng waypoint:
                    • đoạn tự do:      yaw = hướng vận tốc (camera nhìn tới)
                    • P_pre → P_post:  yaw = hướng của n_i (khóa yaw khi chui)
           ──► Xuất /planner/path (chỉ HÌNH HỌC, không có thời gian)
```

**Ranh giới trách nhiệm rõ ràng:** N4 lo **"đi đường nào"** (hình học, tránh vật cản). N5 lo **"đi như thế nào theo thời gian"** (vận tốc, gia tốc, jerk). Không chồng lấn.

## 4.5. Khi nào replan

| Trigger | Hành động |
|---|---|
| Gate TARGET lệch > 0.3 m so với lúc plan | replan đoạn hiện tại |
| Voxel OCCUPIED mới cắt path | replan ngay từ trạng thái hiện tại |
| Dijkstra FAILURE | RECOVERY (§8) |
| Gate mới chen giữa thứ tự | N2 sắp lại → replan |
| Định kỳ | **KHÔNG** replan theo chu kỳ — chỉ theo sự kiện, tránh dao động |

---

## 4.6. Hiệu năng đo thật (khép R1)

Profile trên **lưới thật 312k voxel** (130×60×40 @ 0.2 m), máy rảnh, tách từng chặng:

| Chặng | clearance | build_graph | dijkstra | **tổng** |
|---|---|---|---|---|
| tới vòng gần nhất, 3.5 m | 5 ms | 58 ms | 26 ms | **47 ms** |
| chặng dài nhất D435i nuôi nổi, 8.5 m | 4 ms | 54 ms | 33 ms | **75 ms** |
| *(retry toàn lưới — chỉ khi thất bại)* | | | | **330 ms** |

**R1 lo nhầm chỗ.** Nút thắt không phải bản thân Dijkstra mà là **dựng ma trận thưa** (~2/3 thời gian). Nhưng cả hai đều dư ngưỡng: `pick_target` luôn nhắm **vòng gần nhất chưa qua**, mà D435i chỉ thấy ~10 m, nên sub-box không bao giờ dài hơn chừng đó. **Chặng thật 47–75 ms, ngưỡng 200 ms.**

**Con số 292 ms trước đây là do tranh CPU** (Gazebo + SITL + MAVROS + vision + planner cùng một laptop), không phải giới hạn thuật toán. Đã thử tối ưu builder bằng `flatnonzero` + số học stride: chỉ nhanh **1.1×** — bỏ, không đáng đánh đổi độ phức tạp.

> ⚠️ **Bug thật tìm được khi đào chỗ này — nặng hơn vấn đề ban đầu.**
>
> `tick()` chạy 2 Hz. Khi plan thất bại, `self.path` vẫn `None` ⇒ tick sau `needs_replan` lại trả `'no path'` ⇒ plan lại. Mỗi lần như vậy trả đủ **330 ms cho retry toàn lưới**:
>
> | | tick quét cả lưới | tải executor |
> |---|---|---|
> | trước | 20/20 | **73.6%** |
> | sau (rate-limit 5 s) | 2/20 | **20.1%** |
>
> Tức một bản đồ chưa giải được sẽ **bóp nghẹt executor vĩnh viễn** và bỏ đói `build_grid` dùng chung — trong khi log bị throttle 2 s nên nhìn vào chỉ thấy vài dòng warn thưa. Đúng loại thất bại im lặng như [[sitl-runtime-gotchas]].
>
> Sửa: `dijkstra_path(..., whole_grid_retry=False)` cho caller quyết định, `WHOLE_GRID_RETRY_S = 5.0`. Vẫn quét toàn lưới đều đặn nên **không mất** khả năng tìm đường vòng rộng hơn `PAD`. Log FAILURE giờ ghi rõ `ca luoi` hay `chi sub-box`.

**Còn lại cho phần cứng:** đo lại trên companion computer (§13.5) — con số trên là của desktop.

# PHẦN 5 ★ — TẦNG SINH QUỸ ĐẠO MUELLER (N5) — **PHẦN MỚI CỦA v2**

## 5.0. Vì sao cần

v1 dùng **carrot-chasing**: nhả waypoint kế tiếp, đổi khi vào bán kính 0.3 m, để ArduPilot tự nội suy. Hệ quả:
- Không kiểm soát vận tốc/gia tốc **giữa** các waypoint → giật ở khúc cua, overshoot.
- Spline Catmull-Rom chỉ làm mượt **hình học**, không nói gì về **thời gian** → không có profile vận tốc nhất quán.
- Vào/ra vòng bằng đường thẳng cứng với v cố định → đổi vận tốc đột ngột ở P_pre và P_post.

Thuật toán Mueller *(M. W. Mueller, M. Hehn, R. D'Andrea, "A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation", IEEE T-RO, 2015)* giải đúng lỗ hổng này.

## 5.1. ⚠️ ĐIỀU KIỆN TIÊN QUYẾT — đã xác minh

**ArduCopter hiện đại CÓ hỗ trợ setpoint pos + vel + accel** qua `SET_POSITION_TARGET_LOCAL_NED`. Tài liệu chính thức ArduPilot (`ardupilot.org/dev/docs/copter-commands-in-guided-mode.html`) liệt kê rõ:

```
type_mask:
  Use Position     : 0b110111111000 / 3576
  Use Velocity     : 0b110111000111 / 3527
  Use Acceleration : 0b110000111111 / 3135
  Use Pos+Vel      : 0b110111000000 / 3520
  Use Pos+Vel+Accel: 0b110000000000 / 3072   ★ ← CÁI TA CẦN
  Use Yaw          : 0b100111111111 / 2559
```

Ràng buộc quan trọng từ tài liệu:
- **Khi cấp Pos, Vel và/hoặc Accel thì phải cấp đủ cả 3 trục.** Không được chỉ đặt một trục.
- Ít nhất một trong Pos/Vel/Accel phải có (chỉ gửi Yaw đơn độc không được hỗ trợ).
- **Nếu gửi lệnh vận tốc/gia tốc thì phải gửi lại ít nhất mỗi giây** — sau 3 giây không nhận được, xe sẽ dừng. Stream 20 Hz của ta thỏa mái.
- Mã nguồn `ArduCopter/mode_guided.cpp` có sẵn bộ điều khiển `pos_vel_accel` / `vel_accel` / `accel` → xác nhận thêm.

**Mask thực dùng của ta = Pos+Vel+Accel + Yaw.** Mask 3072 đang **bỏ qua** yaw (bit11) và yaw_rate (bit12). Muốn dùng cả yaw, xóa bit11:

```
Pos+Vel+Accel+Yaw = 0b100000000000 = 2048
```

> **Cảnh báo về tài liệu cũ:** DroneKit và nhiều blog cũ (≈2015–2020) khẳng định "acceleration is ignored by ArduPilot". Điều đó **đúng với ArduCopter đời cũ** nhưng **không còn đúng** với bản hiện tại.
>
> ✅ **ĐÃ ĐO LIVE 2026-07-18 — không còn là giả định.** SITL ở đây là **ArduCopter V4.6.3**, và nó **CÓ nhận accel**. Phép đo A/B cô lập (hai mask chỉ khác nhau ở 3 bit accel, cùng giữ một vị trí ở `v = 0`, feed-forward `a = 2.0 m/s²` trên +X):
>
> | | không accel (mask 2496) | có accel (mask 2048) |
> |---|---|---|
> | độ lệch đỉnh | 0.021 / 0.038 / 0.036 m | **1.017 / 1.008 / 1.017 m** |
> | tốc độ đỉnh | ≤ 0.05 m/s | **≈ 0.83 m/s** |
>
> Chênh trung vị **+0.980 m**, lặp lại 3/3. **⇒ Chốt mask 2048 cho toàn dự án. Fallback 3520 KHÔNG cần dùng.**
>
> Chạy lại bất cứ lúc nào: `ros2 run simtofly_mavros_sitl hold_and_watch --probe-accel`. Đáng chạy lại khi **đổi phiên bản ArduCopter hoặc chuyển sang phần cứng thật** — đây là giả định nền của cả tầng N5.

Với MAVROS, dùng `mavros_msgs/PositionTarget` trên `/mavros/setpoint_raw/local`, `coordinate_frame = FRAME_LOCAL_NED`, điền `position`, `velocity`, `acceleration_or_force`, `yaw`, và `type_mask` bằng tổ hợp hằng `IGNORE_*`. **MAVROS tự chuyển ENU→NED**, phía ROS ta luôn làm việc bằng ENU.

## 5.2. Nền tảng toán học (tóm tắt để triển khai)

Mỗi trục x, y, z **độc lập**, là một *triple integrator* (vị trí → vận tốc → gia tốc), điều khiển là **jerk**. Cực tiểu hóa `∫₀^T j(t)² dt`. Theo nguyên lý cực tiểu Pontryagin, costate của vị trí là hằng ⇒ costate của gia tốc là bậc 2 ⇒ **jerk tối ưu là đa thức bậc 2**:

```
j(t) = (α/2)t² + βt + γ
a(t) = a₀ + γt + (β/2)t² + (α/6)t³
v(t) = v₀ + a₀t + (γ/2)t² + (β/6)t³ + (α/24)t⁴
p(t) = p₀ + v₀t + (a₀/2)t² + (γ/6)t³ + (β/24)t⁴ + (α/120)t⁵
```

Bóc phần "trôi tự do" (khi jerk = 0):

```
Δa = a_f − a₀
Δv = v_f − v₀ − a₀T
Δp = p_f − p₀ − v₀T − ½a₀T²
```

**Trường hợp đầy đủ (pos+vel+acc ràng buộc)** — dùng cho hầu hết đoạn của ta:

```
α = ( 60T²Δa − 360TΔv + 720Δp ) / T⁵
β = (−24T³Δa + 168T²Δv − 360TΔp ) / T⁵
γ = (  3T⁴Δa −  24T³Δv +  60T²Δp ) / T⁵
```

**Trường hợp chỉ ràng buộc vị trí** (dùng khi không quan tâm vận tốc cuối, ví dụ điểm hạ cánh):

```
α = 20Δp/T⁵,   β = −20Δp/T⁴,   γ = 10Δp/T³
```

Thành phần đích **để tự do** thì Pontryagin cho điều kiện transversality thay vì điều kiện biên cố định. Đặc biệt: **vị trí cuối tự do ⟺ λ₁ = 0 ⟺ α = 0**. Đây là lý do mọi nhánh không ràng buộc vị trí đều có α = 0.

**Chi phí** (dùng để xếp hạng ứng viên):

```
cost = γ² + βγT + β²T²/3 + αγT²/3 + αβT³/4 + α²T⁴/20
```

## 5.3. Kiểm tra khả thi (feasibility) — và giới hạn phải đặt bằng bao nhiêu

> ⚠️ **HIỆU CHỈNH C9 — mục này ĐÃ VIẾT LẠI. Bản trước kiểm SAI đại lượng và đặt sai giá trị.**

## 5.3.0. Truy vết đường đi thật của setpoint (đã đọc source, không phải suy đoán)

Đọc trong `~/ardupilot` (`ArduCopter-stable`):

```
mode_guided.cpp:911    pos_control->input_pos_vel_accel_xy(pos, vel, accel, false)
        ↓
AC_PosControl.cpp      shape_pos_vel_accel_xy(pos, vel, accel, ...,
                            _vel_max_xy_cms,      ← WPNAV_SPEED
                            _accel_max_xy_cmss,   ← WPNAV_ACCEL
                            _jerk_max_xy_cmsss,   ← PSC_JERK_XY
                            _dt, limit_output)
        ↓
AC_PosControl.h:21     POSCONTROL_JERK_XY = 5.0 m/s³   (mặc định PSC_JERK_XY)
AC_WPNav.h:16          WPNAV_ACCELERATION = 250 cm/s²  (mặc định WPNAV_ACCEL)
```

**Kết luận then chốt: ArduCopter ĐÃ tự chạy một bộ sinh quỹ đạo giới hạn jerk (S-curve shaping) rồi.**

Gửi (p, v, a) của Mueller vào **không phải** là "drone bay theo đa thức bậc 5". Đó là đưa đa thức bậc 5 làm **đầu vào cho một bộ shaping khác, và bộ đó sẽ nắn lại**. Quỹ đạo drone thực bay **không phải** quỹ đạo Mueller.

Hai hệ quả bắt buộc phải nuốt:

1. **`f_min` / `f_max` / `ω_max` gần như vô nghĩa trong kiến trúc này.** Đó là ràng buộc của bài báo Mueller, nơi platform điều khiển **thrust + body rate trực tiếp** ở ≥50 Hz. Ta không điều khiển như vậy — ta nói chuyện với một bộ điều khiển vị trí phân tầng. Drone chạm giới hạn của shaper từ rất lâu trước khi chạm giới hạn thrust.
2. **Ràng buộc thắt thật là bộ ba `WPNAV_SPEED` / `WPNAV_ACCEL` / `PSC_JERK_XY`** — và bộ ba này phải là thứ Mueller kiểm tra.

## 5.3.1. Giới hạn phải đặt bằng bao nhiêu

Vẫn giữ nguyên tinh thần C5 (đặt thấp hơn ArduCopter ~30% để nó không bao giờ phải clamp), nhưng áp lên **đúng ba đại lượng**:

| Tham số Mueller | Giá trị khởi điểm | Căn cứ (mặc định ArduCopter) |
|---|---|---|
| `v_horiz_max` | **≈ 70% `WPNAV_SPEED`** | đọc thật ở §0.2. Mission bay ~0.8 m/s nên hầu như không thắt |
| `a_horiz_max` | **1.75 m/s²** | 70% của `WPNAV_ACCEL` = **2.5 m/s²** |
| ★ `j_horiz_max` | **3.5 m/s³** | 70% của `PSC_JERK_XY` = **5.0 m/s³** ← **v2 bản trước thiếu hẳn dòng này** |
| `a_vert_max` | **0.7 m/s²** | 70% của `WPNAV_ACCEL_Z` = **1.0 m/s²** (đọc thật). ⚠️ **Chặt hơn phương ngang 2.5 lần** — quỹ đạo lên/xuống giữa các vòng lệch độ cao sẽ chạm giới hạn này TRƯỚC |
| `j_vert_max` | **3.5 m/s³** | 70% của `PSC_JERK_Z` = 5.0 (đọc thật) |
| `minTimeSection` | **0.02 s** | độ phân giải chia đôi đệ quy |

> ⚠️ **Hai cái sai cụ thể của bảng cũ:**
>
> - Bảng cũ đặt `a_horiz_max = 4.0 m/s²`, suy từ `g·tan(ANGLE_MAX=30°) = 5.66`. Nhưng **`WPNAV_ACCEL` mặc định chỉ 2.5 m/s²** — thấp hơn nhiều, và nó mới là cái chặn trước. Đặt 4.0 nghĩa là **đã bị clamp ngay từ giá trị đề xuất**: đúng y cái thất bại âm thầm mà C5 vừa cảnh báo, chỉ khác là bản thân C5 dính vào.
> - **Thiếu hoàn toàn giới hạn jerk** — trong khi jerk chính là đại lượng Mueller tối thiểu hoá, *và* là đại lượng ArduCopter clamp. Bỏ trống ô này thì cả tầng Mueller mất ý nghĩa.
>
> **`WPNAV_ACCEL` là tham số, không phải định luật vật lý.** Nếu 1.75 m/s² bay chậm quá, cách đúng là **nâng `WPNAV_ACCEL` lên rồi nâng giới hạn Mueller theo**, chứ không phải để Mueller đòi vượt. Chỉnh tham số thì có kiểm soát; bị clamp thì không.

## 5.3.2. Hai phép kiểm tra Mueller — cái nào còn dùng

**(a) Input feasibility — THAY THẾ.** Không kiểm thrust/body-rate nữa. Dùng `GetMinMaxAcc` trên từng đoạn để kiểm `|a(t)| ≤ a_max` theo trục, và vì `j(t) = (α/2)t² + βt + γ` là đa thức bậc 2 nên **jerk đỉnh lấy được ở dạng đóng**: cực trị tại `t = −β/α` (nếu nằm trong [0,Tf]) cộng hai đầu mút. Không cần chia đôi đệ quy cho jerk. Đây là phép kiểm rẻ hơn hẳn bản gốc.

**(b) Position feasibility — GIỮ NGUYÊN, vẫn rất giá trị.** Với một nửa không gian (điểm biên + pháp tuyến): giải nghiệm đa thức vận tốc bậc 4 chiếu lên pháp tuyến để tìm cực trị, rồi chỉ kiểm vị trí tại các cực trị + hai đầu. Không cần quét dày theo thời gian. Với biên lọt vòng chỉ 50 mm thì đây là phép kiểm đáng giá nhất của cả tầng Mueller.

> **Giữ lại bảng thrust/body-rate cũ để tham chiếu:** nếu sau này bỏ ArduCopter GUIDED để điều khiển thrust + body rate trực tiếp (hoặc port sang bộ điều khiển khác không có shaping), thì bảng `f_min=5.0 / f_max=13.0 / ω_max=3.0` mới là đúng. Trong kiến trúc hiện tại thì không.

Mọi giá trị trên là **điểm khởi đầu, phải tinh chỉnh bằng đo đạc** ở Giai đoạn 5 (so gia tốc/jerk lệnh với `/mavros/local_position/pose` và `velocity_local`).

## 5.4. Ứng dụng 1 (giá trị nhất) — đoạn TRAVERSE chui vòng

Đây là chỗ Mueller tỏa sáng, vì cho phép chỉ định **cả gia tốc tại đích**.

```
Đoạn A: P_pre → P_gate
   Trạng thái đầu : p = P_pre,  v = 0 (hoặc v hiện tại), a = a hiện tại
   Trạng thái đích: p = C_i
                    v = v_traverse · n_i     ← vận tốc ĐÚNG HƯỚNG PHÁP TUYẾN
                    a = 0                    ← không lắc ngang khi qua tâm
   → dùng công thức pos+vel+acc đầy đủ (§5.2)

Đoạn B: P_gate → P_post
   Trạng thái đầu : p = C_i, v = v_traverse · n_i, a = 0   (liên tục với đoạn A)
   Trạng thái đích: p = P_post_i, v = v_traverse · n_i, a = 0
   → tiếp tục thẳng, mượt, sẵn sàng cho Dijkstra đoạn kế
```

**Kết quả:** drone vào vòng mượt, qua tâm với vận tốc đúng dọc trục và gia tốc ngang bằng 0, ra vòng mượt — thay vì bám cứng đường thẳng rồi giật vận tốc ở hai đầu. Ràng buộc `a = 0` tại tâm chính là thứ v1 hoàn toàn không diễn đạt được.

> ⚠️ **Bẫy ở đoạn B — phải ghi cứng `Tf`.** Đoạn B có hai đầu **cùng** `v = v_traverse·n` và `a = 0`, đi thẳng dọc `n`. Mueller chỉ ra **đúng** đường thẳng vận tốc hằng khi
>
> ```
> Tf_B  =  ‖P_post − C‖ / v_traverse      (ghi cứng, KHÔNG đưa vào vòng lặp ứng viên §5.5)
> ```
>
> **Đặt lệch thì hỏng ở TỐC ĐỘ, không phải ở hình học** — đo thật trong `mueller.py --selfcheck`, sửa lại điều bản trước của mục này viết sai:
>
> | `Tf_B` | lệch ngang | dải tốc độ qua lỗ |
> |---|---|---|
> | đúng `d/v` | 0 | **0.80 → 0.80** (hằng) |
> | dài 1.5× | 0 | **0.30** → 0.80 (tụt) |
> | ngắn 0.6× | 0 | 0.80 → **1.80** (vọt) |
>
> Vì biên của đoạn B **thẳng hàng theo `n`** (đoạn A bàn giao đúng `p=C, v=v_traverse·n, a=0`), mọi trục vuông góc **đồng nhất bằng 0**: không có chuyện phình ngang, cũng không overshoot quá `P_post`. Cái mất là ràng buộc `a = 0` tại tâm — lý do tồn tại của cả §5.4 — và tốc độ vọt lên 1.8 m/s trong lỗ thì phá luôn giả định mà `CheckPositionFeasibility` đã kiểm.
>
> ⚠️ **Hệ quả cho việc kiểm tra:** `CheckPositionFeasibility` **không bắt được** lỗi này (hình học vẫn hoàn hảo). Muốn bắt thì phải kiểm **tốc độ hằng** trên đoạn B. Đoạn A tự do chọn `Tf`, đoạn B thì không.

**Kiểm tra bắt buộc trước khi phát:**
- `CheckInputFeasibility(a_horiz_max, j_horiz_max, a_vert_max, j_vert_max, 0.02)` → phải trả `InputFeasible`. **Kiểm vel/accel/jerk, KHÔNG kiểm thrust/body-rate** (§5.3.2).
- `CheckPositionFeasibility` với **4–6 mặt phẳng tiếp tuyến** của hình trụ an toàn (trục = n_i, bán kính = `r_in − 0.25 = 0.5 m`) → đảm bảo không chạm vành.
- Thất bại → tăng `Tf` (đi chậm hơn) và thử lại; xem §5.6.

## 5.5. Ứng dụng 2 — chọn vận tốc chui bằng candidate generation

Mueller "rapid" vì sinh và sàng lọc **hàng nghìn quỹ đạo/giây**. Thay vì để `v_traverse = 0.5 m/s` cố định như v1, ta **sinh ứng viên và chọn**:

```
CHỌN_QUỸ_ĐẠO_CHUI(P_pre, C, n, P_post, trạng_thái_hiện_tại):
    best ← null
    for v_tr in {0.4, 0.6, 0.8, 1.0, 1.2}:            # m/s
        for Tf in {0.8, 1.2, 1.6, 2.0, 2.5, 3.0}:      # s
            traj ← Mueller(trạng_thái_hiện_tại → {C, v_tr·n, 0}, Tf)
            if traj.CheckInputFeasibility(...) ≠ InputFeasible: continue
            if bất kỳ mặt phẳng trụ nào CheckPositionFeasibility ≠ Feasible: continue
            score ← traj.cost + λ_T · Tf              # λ_T ≈ 1.0: phạt đi chậm
            if score < best.score: best ← traj
    return best        # null → nới ràng buộc hoặc báo FAILURE
```

30 ứng viên là quá nhẹ với thuật toán này (mỗi ứng viên chỉ vài chục phép tính + vài nghiệm đa thức). Đây là cách **thay con số 0.5 m/s tùy chọn của v1 bằng một lựa chọn có cơ sở**.

> ⚠️ **Nhưng hoãn mục này lại — làm sau cùng, nếu còn cần.** Ba lý do:
>
> 1. Bộ shaping của ArduCopter (§5.3.0) sẽ nắn lại quỹ đạo phía dưới, nên độ tinh của việc chọn `v_traverse` bị **mài mòn** trước khi tới drone.
> 2. Nó thêm một knob phải chỉnh (`λ_T`) để chọn trong dải hẹp 0.4–1.2 m/s — knob mới không có căn cứ đo đạc.
> 3. Trên companion computer thật, vòng lặp 30 ứng viên bằng Python là chi phí thật (§13.5).
>
> **Chốt `v_traverse = 0.8 m/s` cố định, bay, đo, rồi chỉnh tay.** Mở lại §5.5 chỉ khi số đo cho thấy một giá trị cố định thật sự không đủ.

## 5.6. Ứng dụng 3 — nối các waypoint tự do (đoạn Dijkstra)

> ⚠️ **KHÔNG** thay cả đoạn FLY_TO_PRE dài bằng **một** quỹ đạo Mueller đơn. Mueller nối 2 điểm bằng một đường trơn duy nhất — giữa môi trường nhiều vật cản, đường đó dễ infeasible và sẽ phải chia nhỏ liên tục. **Dijkstra + shortcut vẫn giữ vai trò tránh vật cản.**

Kiến trúc **hai tầng** đúng:

```
Tầng global (N4, Dijkstra) : cho chuỗi waypoint W₀ … W_m đã đảm bảo thông thoáng
Tầng local  (N5, Mueller)  : nối từng cặp W_k → W_{k+1} bằng quỹ đạo bậc 5
```

Quy tắc sinh cho mỗi đoạn:

```
SINH_ĐOẠN(W_k, W_{k+1}, trạng_thái_đầu):
    d  ← ‖W_{k+1} − W_k‖
    Tf ← max(d / v_cruise, Tf_min=0.5)
    v_đích ← v_cruise · hướng(W_{k+2} − W_k)      # nhìn trước 1 waypoint → mượt qua góc
                                                   # waypoint cuối chuỗi: v_đích = 0
    a_đích ← 0
    lặp tối đa 5 lần:
        traj ← Mueller(trạng_thái_đầu → {W_{k+1}, v_đích, a_đích}, Tf)
        nếu InputFeasible và PositionFeasible(hành lang quanh đoạn): return traj
        Tf ← Tf × 1.4                              # đi chậm hơn → luôn dễ khả thi hơn
    return FAILURE → chèn thêm waypoint giữa, hoặc báo N6
```

**Hành lang cho `CheckPositionFeasibility`:** 4 mặt phẳng bao quanh đoạn `W_k → W_{k+1}`, cách trục đoạn `0.5 m`, đảm bảo quỹ đạo trơn không "phình" ra khỏi hành lang mà Dijkstra đã chứng minh là an toàn.

**Vì sao tăng Tf luôn giúp:** với cùng Δp, các hệ số α, β, γ tỉ lệ nghịch với lũy thừa của T (α ~ 1/T⁵). Kéo dài Tf làm giảm mạnh gia tốc và jerk đòi hỏi.

## 5.7. Vòng lặp thực thi N5 (20 Hz) — thay thế carrot-chasing

```
KHỞI TẠO: traj_active ← null, t_start ← 0

TIMER 20 Hz (KHÔNG BAO GIỜ BỊ CHẶN BỞI PLANNER — thread riêng):
    τ ← now() − t_start

    nếu traj_active = null:
        publish setpoint GIỮ NGUYÊN vị trí hiện tại (v=0, a=0)   # Rule: không ngừng stream
        return

    nếu τ > traj_active.Tf:
        nếu còn đoạn kế tiếp trong hàng đợi:
            traj_active ← đoạn kế; t_start ← now(); τ ← 0
        ngược lại:
            publish giữ điểm cuối (v=0, a=0); return

    p ← traj_active.GetPosition(τ)
    v ← traj_active.GetVelocity(τ)
    a ← traj_active.GetAcceleration(τ)
    yaw ← nội suy yaw mong muốn (§4.4 mục 5)

    publish PositionTarget{ frame=LOCAL_NED, type_mask=2048,
                            position=p, velocity=v,
                            acceleration_or_force=a, yaw=yaw }
```

**Ưu điểm bất ngờ về an toàn:** quỹ đạo là **hàm của thời gian**, nên nếu planner treo hoặc chậm, N5 vẫn tiếp tục sample quỹ đạo hợp lệ cuối cùng cho tới hết `Tf` rồi giữ vị trí — suy giảm êm, không bao giờ đứt stream. Carrot-chasing của v1 không có tính chất này.

## 5.8. ⚠️ Lấy trạng thái đầu (p₀, v₀, a₀) ở đâu — chi tiết dễ sai

| Đại lượng | Nguồn | Ghi chú |
|---|---|---|
| `p₀` | `/mavros/local_position/pose` | ENU, sẵn có |
| `v₀` | `/mavros/local_position/velocity_local` | ENU, sẵn có |
| `a₀` | **Vấn đề** — không có topic gia tốc ENU sạch | xem dưới |

Ba lựa chọn cho `a₀`, xếp theo thứ tự khuyến nghị:

1. **★ Dùng gia tốc LỆNH của quỹ đạo đang chạy** `traj_active.GetAcceleration(τ)`. Đây là cách **đúng nhất khi replan giữa chừng**: đảm bảo quỹ đạo mới nối liền C² với quỹ đạo cũ, không giật. Dùng cho mọi lần replan.
2. **Đặt `a₀ = 0`** khi khởi tạo từ trạng thái hover đứng yên. Đơn giản, đúng.
3. Vi phân số `v₀` hoặc dùng `/mavros/imu/data` — **không khuyến nghị**: vi phân khuếch đại nhiễu, còn IMU đo cả trọng lực và ở body frame nên phải trừ g và xoay về ENU, dễ sai dấu.

> **Nguyên tắc:** dùng trạng thái **lệnh** (commanded), không dùng trạng thái **đo** (measured), làm điều kiện đầu khi replan. Trạng thái đo chỉ dùng để (a) khởi tạo lần đầu và (b) giám sát tracking error.

## 5.9. Chính sách tái sinh quỹ đạo

| Tình huống | Xử lý |
|---|---|
| Chuyển sang đoạn kế trong chuỗi | Tái sinh trước hạn 0.3 s, dùng trạng thái lệnh tại `τ = Tf` → liên tục hoàn hảo |
| Replan do gate lệch / path bị chặn | Tái sinh từ trạng thái **lệnh** tại thời điểm hiện tại (§5.8 mục 1) |
| Tracking error > 0.5 m | Không tái sinh vội — hover, chờ ổn định, rồi tái sinh từ trạng thái **đo** |
| Đang ở TRAVERSE | **KHÔNG tái sinh** (khóa như v1) — quỹ đạo đã kiểm tra feasibility, vòng đã ngoài FOV |

---

# PHẦN 6 — GIAO TIẾP ArduPilot & MÁY TRẠNG THÁI (N6)

## 6.1. Trình tự khởi động (ArduCopter GUIDED qua MAVROS2)

```
1. Chờ /mavros/state.connected == true, EKF sẵn sàng
2. ★ BẮT ĐẦU STREAM /mavros/setpoint_raw/local ở 20 Hz TRƯỚC (giữ vị trí hiện tại)
3. set_mode("GUIDED")            (service /mavros/set_mode)
4. arming(true)                  (service /mavros/cmd/arming)
5. takeoff(alt = 2.0 m)          (service /mavros/cmd/takeoff)
6. Khi |z − 2.0| < 0.15 m và ổn định 1 s → N5 bắt đầu nhận quỹ đạo từ planner
   Stream KHÔNG BAO GIỜ NGẮT, kể cả khi "đứng yên" (publish lại vị trí giữ)
```

> **Hiệu chỉnh so với v1:** v1 ghi "ArduCopter khác PX4 — không cần stream trước khi đổi mode". Đúng về mặt kỹ thuật, nhưng **stream trước vẫn là thói quen an toàn hơn** và bắt buộc nếu sau này port sang PX4. v2 đưa stream lên bước 2. Ngoài ra tài liệu ArduPilot nêu rõ: lệnh vel/accel phải được gửi lại ít nhất mỗi giây, nếu 3 giây không có lệnh xe sẽ dừng — stream 20 Hz thỏa mái nhưng **không được để planner chặn thread stream**.

## 6.2. Máy trạng thái nhiệm vụ (N6)

```
                ┌─────────┐
                │  INIT   │ chờ MAVROS + camera + TF sẵn sàng
                └────┬────┘
                     ▼
                ┌─────────┐  stream→GUIDED→ARM→TAKEOFF, đạt 2 m
                │ TAKEOFF │────────────────┐ lỗi arm/mode → ABORT
                └────┬────┘                │
                     ▼                     │
              ┌────────────┐  chưa có gate CONFIRMED:
        ┌────►│   SEARCH   │  yaw-scan tại chỗ ±60°, hoặc bay chậm khảo sát
        │     └────┬───────┘
        │          │ có gate CONFIRMED chưa PASSED
        │          ▼
        │     ┌────────────┐ chọn TARGET, dựng anchor P_pre/P_gate/P_post
        │     │    PLAN    │ Dijkstra (N4) → chuỗi waypoint
        │     │            │ ★ + Mueller (N5) sinh quỹ đạo từng đoạn (§5.6)
        │     └────┬───────┘ FAILURE (Dijkstra hoặc Mueller) ──► RECOVERY
        │          ▼
        │     ┌────────────┐ N5 sample quỹ đạo @20Hz; detect vẫn chạy
        │     │ FLY_TO_PRE │ gate lệch >0.3 m → PLAN;  path bị chặn → PLAN
        │     └────┬───────┘ tracking error >0.5 m → hover + tái sinh
        │          ▼
        │     ┌────────────┐ hover ngắn tại P_pre, khóa yaw theo n,
        │     │   ALIGN    │ xác nhận lại tâm vòng lần cuối bằng detect
        │     └────┬───────┘ ★ + chạy §5.5 chọn quỹ đạo chui tối ưu
        │          │         mất detect & sai lệch lớn → SEARCH_LOCAL
        │          ▼         không tìm được quỹ đạo khả thi → RECOVERY
        │     ┌────────────┐ ★ bám 2 đoạn quỹ đạo Mueller P_pre→C→P_post
        │     │  TRAVERSE  │ KHÔNG tái sinh giữa chừng
        │     └────┬───────┘ KHÔNG tin detect (vòng quá gần, ngoài FOV)
        │          ▼
        │     ┌────────────┐ ★ điều kiện PASSED sửa lại — xem §7 R3
        │     │ CHECK_PASS │
        │     └────┬───────┘ đánh dấu gate PASSED
        │          │ còn gate chưa qua ──────────┐
        │          ▼                             │
        │     hết gate                           │
        │     ┌────────────┐                     │
        │     │    LAND    │                     │
        │     └────────────┘                     │
        └────────────────────────────────────────┘
   (mọi trạng thái)──lỗi nghiêm trọng/timeout──► ABORT: set_mode("LAND"/"RTL")
```

## 6.3. Ràng buộc an toàn xuyên suốt

- **Geofence mềm** phía ROS2: ra khỏi hộp bay → ABORT (RTL).
- **Watchdog:** mất `/mavros/local_position/pose` > 0.5 s hoặc mất ảnh > 1 s → giữ setpoint hiện tại; > 5 s → LAND.
- **Timeout từng trạng thái:** SEARCH 60 s, FLY_TO_PRE 45 s, ALIGN 10 s, TRAVERSE 15 s.
- **Không bao giờ chống lại failsafe của ArduPilot.** N6 subscribe `/mavros/state`; mode bị đổi ngoài ý muốn → dừng phát mục tiêu mới, log sự kiện.
- ★ **Mới:** N5 chạy timer riêng ở thread/callback-group riêng. Planner (N4) **không được** chặn nó. Nếu N4 chậm, N5 vẫn phát quỹ đạo cũ — đây là lý do §5.7 thiết kế như vậy.
- ★ **Mới:** giám sát tracking error `‖p_đo − p_lệnh‖` liên tục; > 0.5 m → hover; > 1.0 m → ABORT. Đây là **cách duy nhất phát hiện ArduCopter đang clamp âm thầm** (§5.3).

---

# PHẦN 7 ★ — DANH SÁCH ĐIỂM CẦN HIỆU CHỈNH PHÁT HIỆN KHI RÀ SOÁT v1

Ngoài phần tích hợp Mueller, đây là những chỗ trong v1 mình thấy nên sửa:

| # | Vấn đề trong v1 | Mức độ | Hiệu chỉnh |
|---|---|---|---|
| **R1** | ~~"Dijkstra < 200 ms" nêu như sự thật, không kèm điều kiện~~ | ~~Cao~~ **ĐÃ ĐÓNG** | Đã làm đúng cả hai việc R1 đề: `scipy.sparse.csgraph` (search chạy trong C) + thu hộp bao `AABB(s,t)+3 m`. **Đo thật trên lưới 312k: chặng thật 47–75 ms**. Nút thắt hoá ra là *dựng ma trận thưa*, không phải search. §4.6 |
| **R2** | Giới hạn gia tốc/vận tốc không được liên hệ với tham số ArduCopter | **Cao** | ArduCopter clamp trước theo `ANGLE_MAX`/`WPNAV_*`. Đặt giới hạn Mueller thấp hơn ~30%. §5.3 |
| **R3** | Điều kiện PASSED `n·(p−C) > 0.5 m` **và** lệch trục `< r_in` kiểm tại **vị trí hiện tại**. Sai: ở 0.5 m sau vòng thì drone đã có thể trôi ngang ra ngoài, hoặc đã chui đúng nhưng lúc kiểm lại lệch → phán đoán sai cả hai chiều | **Cao** | Phát hiện **đổi dấu** của `s(t) = n·(p(t)−C)` giữa hai chu kỳ, **nội suy tuyến tính điểm cắt** `p*`, rồi kiểm lệch trục **tại `p*`**: `‖(p*−C) − (n·(p*−C))n‖ < r_in − 0.1`. Đây mới đúng nghĩa "chui qua lỗ" |
| **R4** | Không nêu nguồn ước lượng gia tốc — thiếu hẳn khi chuyển sang Mueller | **Cao** | Dùng trạng thái **lệnh**, không dùng đo. §5.8 |
| **R5** | Vector trọng lực chưa nêu quy ước | **Trung bình** | ENU ⇒ `gravity = (0,0,−9.81)`. Sai dấu → toàn bộ kiểm tra thrust sai âm thầm. §1.4 |
| **R6** | Sơ đồ khối dùng `/camera/...`, bảng topic dùng `/d435i/...` | **Thấp** | Thống nhất `/d435i/...`. §1.3 |
| **R7** | Không có metric nào đo độ mượt — mà "mượt" chính là lý do dùng Mueller | **Trung bình** | Thêm metric: tổng ∫jerk², gia tốc đỉnh, tracking error RMS. §10 |
| **R8** | v1 nói "ArduCopter không cần stream trước khi đổi mode" | **Thấp** | Đúng nhưng nên stream trước cho an toàn và để dễ port. §6.1 |
| **R9** | Edge case #18 nói depth có thể là 16UC1 trong khi §0 đã xác nhận 32FC1 | **Thấp** | Giữ nguyên — đây là assert phòng thủ hợp lý, không phải mâu thuẫn |
| **R10** | ALIGN bắt buộc hover 1 s rồi mới chui | **Thấp** | Với Mueller, có thể **fly-through** (không dừng): quỹ đạo tới P_pre kết thúc với `v = v_traverse·n` thay vì 0, nối liền vào đoạn chui. Nhanh và mượt hơn. **Chỉ bật sau khi Giai đoạn 5 đạt 10/10** |
| **R11** | ~~Chưa có phương án dự phòng nếu ArduCopter không nhận accel~~ | ~~Cao~~ **ĐÃ ĐÓNG** | **Đo live 2026-07-18 trên ArduCopter V4.6.3: CÓ nhận accel** (chênh trung vị +0.980 m so với nhánh không accel, 3/3 vòng). Dùng mask **2048**, không cần fallback 3520. Cách đo: `hold_and_watch --probe-accel`, §12 GĐ2 |
| **R12** | Dòng tiêu đề §0 có ký tự thừa `.s` | **Thấp** | Đã sửa |
| **R13** | ★ **Chính v2 dính bẫy C5.** §5.3 đặt `a_horiz_max = 4.0 m/s²` suy từ `ANGLE_MAX`, nhưng `WPNAV_ACCEL` mặc định **2.5 m/s²** chặn trước. Và **thiếu hẳn giới hạn jerk** dù `PSC_JERK_XY = 5.0 m/s³` clamp đúng đại lượng Mueller tối thiểu hoá | **Cao** | Viết lại §5.3 theo source đã đọc. Bỏ kiểm thrust/body-rate (chỉ đúng khi điều khiển trực tiếp, không đúng qua GUIDED). §5.3 |
| **R14** | ★ Không có phần nào nói về phần cứng thật. Số đo <1 cm hiện tại đo trong sim với depth **hoàn hảo**; depth thật sai ~0.22 m ở 5 m. `MAX_PLANE_RMS = 0.06` sẽ **loại sạch detection xa, không log** | **Cao** | Thêm §13 đầy đủ: ngân sách sai số, chiến lược "không tin pose xa", danh sách sửa code, nghiệm thu H1–H6. §13 |

---

# PHẦN 8 — EDGE CASES & CÁCH XỬ LÝ

| # | Trường hợp | Phát hiện | Xử lý |
|---|---|---|---|
| 1 | Depth tại tâm vòng là background | thiết kế sẵn | chỉ sample depth trên vành (§2.1) |
| 2 | Vành nhiều pixel depth 0/NaN | < 12 điểm hợp lệ | bỏ frame; N2 giữ pose lọc từ frame trước |
| 3 | Vòng nhìn quá nghiêng | b/a < 0.25 | không định vị từ góc này; đổi góc nhìn |
| 4 | Nhiều vòng trong 1 khung hình | nhiều ellipse | detect độc lập; N2 gán ID theo khoảng cách 3D |
| 5 | Hai vòng thẳng hàng, vòng sau lộ qua lỗ vòng trước | ellipse lồng nhau | tâm trong nằm trong ellipse ngoài + depth vành lệch > 1 m → tách 2 gate |
| 6 | Pháp tuyến bị lật chiều | tích vô hướng | chuẩn hóa `n·(C − p_drone) > 0` tại CONFIRMED, cố định từ đó |
| 7 | Gate pose trôi do nhiễu | N2 theo dõi covariance | khóa cứng pose khi vào ALIGN; TRAVERSE không cập nhật |
| 8 | Dijkstra FAILURE | `dist[t]=∞` | (a) nới λ_c, cho qua UNKNOWN nhiều hơn; (b) lùi mục tiêu về điểm quan sát tốt; (c) 3 lần thất bại → ABORT |
| 9 | Path bị voxel mới chặn | kiểm mỗi map update | hover → replan từ trạng thái lệnh hiện tại |
| 10 | Mất detect khi FLY_TO_PRE | gate đã CONFIRMED | vẫn bay tới P_pre theo map; ALIGN xác nhận lại |
| 11 | ALIGN xác nhận lệch > 0.3 m | so detect mới vs map | cập nhật pose, dựng lại anchor, sinh lại quỹ đạo chui |
| 12 | Drone chui lệch, chạm vành | kiểm tại **điểm cắt nội suy** (R3) | ghi FAIL vòng đó; sim: tiếp tục — **đây là metric đánh giá chính** |
| 13 | Vượt mặt phẳng nhưng ngoài lỗ | điều kiện PASSED kèm lệch trục tại điểm cắt | không tính PASSED; quay lại PLAN cho chính vòng đó |
| 14 | Vòng ngoài FOV hoàn toàn | SEARCH timeout | mở rộng quỹ đạo khảo sát; hết giờ → LAND, báo thiếu dữ liệu |
| 15 | EKF trôi / TF sai timestamp | lookup tf lỗi hoặc jump vị trí | dùng tf đúng stamp ảnh; jump > 1 m/frame → hover chờ ổn định |
| 16 | Setpoint stream nghẽn (< 5 Hz) | tự đo tần số publish | N5 timer riêng, callback-group riêng; planner thread khác |
| 17 | Vòng sát nhau (< 3 m) | khoảng cách C_i, C_j | co `d_pre/d_post` còn 1.0 m; `P_post_i` quá gần vành j → kẹp theo khoảng cách an toàn |
| 18 | Depth 16UC1 (mm) thay vì 32FC1 | kiểm encoding lúc khởi động | tự quy đổi; assert đơn vị ở frame đầu |
| **19** ★ | **Mueller không tìm được quỹ đạo khả thi** | 5 lần tăng `Tf` vẫn infeasible | chèn waypoint giữa (chia đôi đoạn); vẫn thất bại → hạ `v_cruise` toàn cục 30%; vẫn thất bại → RECOVERY |
| **20** ★ | **ArduCopter clamp âm thầm** (đòi gia tốc quá `ANGLE_MAX`) | tracking error tăng dần dù không có lỗi nào | hạ `a_horiz_max`; log cảnh báo. Phát hiện được nhờ giám sát §6.3 |
| **21** ★ | **Phiên bản ArduCopter không nhận accel** | test GĐ 2: gửi accel, xe không phản ứng | fallback `type_mask = 3520` (Pos+Vel), bỏ trường accel |
| **22** ★ | **Quỹ đạo mới giật khi replan** | nhảy gia tốc tại điểm nối | dùng trạng thái **lệnh** làm điều kiện đầu (§5.8), không dùng đo |
| **23** ★ | **Planner treo, `traj_active` hết hạn** | `τ > Tf` mà không có đoạn kế | giữ điểm cuối với `v=0, a=0` — suy giảm êm, stream không đứt (§5.7) |

---

# PHẦN 9 — HIỂN THỊ PLAN TRÊN GUI CAMERA (N7)

## 9.1. Lớp chính — overlay lên ảnh camera

```
Với mỗi frame RGB (stamp t):
  1. Lấy T_map→optical(t) từ tf2 (đúng stamp, có fallback)
  2. Với từng điểm mẫu:
        p_opt = T_map→optical · w
        nếu p_opt.z <= 0.05 (sau lưng camera) → loại
        u = fx·x/z + cx ;  v = fy·y/z + cy
        nếu (u,v) ngoài ảnh → clip theo biên
  3. Vẽ:
        • ★ QUỸ ĐẠO Mueller: sample p(τ) mỗi 0.05 s → polyline mượt
          (khác v1: vẽ quỹ đạo thực sẽ bay, không phải path voxel răng cưa)
        • path voxel gốc của Dijkstra: nét mảnh, mờ (để so sánh)
        • màu gradient theo khoảng cách; độ dày ~ 1/z (phối cảnh)
        • 3 anchor P_pre/P_gate/P_post: marker riêng (tam giác/tròn/vuông)
        • vòng TARGET: ellipse ước lượng + tâm + mũi tên pháp tuyến
        • ★ HUD: state FSM, gate i/N, ‖v‖ hiện tại, ‖a‖ hiện tại,
                 τ/Tf của đoạn đang chạy, tracking error, cost quỹ đạo
  4. Publish /plan_overlay/image
```

Xem bằng `rqt_image_view`, RViz Image panel, hoặc Foxglove.

**Trường hợp riêng:** quỹ đạo đổi giữa frame → luôn vẽ cái mới nhất kèm số hiệu; TF chưa có ở stamp t → dùng stamp gần nhất < 50 ms, quá thì bỏ frame (không vẽ sai).

## 9.2. Lớp phụ — không gian 3D (debug/demo)

- RViz: `nav_msgs/Path`, MarkerArray cho voxel OCCUPIED, mũi tên pháp tuyến gate, TF tree.
- ★ Thêm: vẽ **vector vận tốc và gia tốc lệnh** tại vị trí drone — nhìn thấy ngay khi gia tốc vọt quá giới hạn.
- Gazebo GUI (tùy chọn): `gz.msgs.Marker` vẽ quỹ đạo trong cửa sổ 3D — dùng khi quay video demo.

---

# PHẦN 10 — SƠ ĐỒ GIẢI THUẬT TỔNG (END-TO-END, v2)

```
        ┌───────────────┐      ┌───────────────┐
        │ RGB 30Hz      │      │ Depth 30Hz    │
        └──────┬────────┘      └──────┬────────┘
               └────────┬─────────────┘
                        ▼ sync theo stamp
              ┌──────────────────┐
              │ N1 DETECT vòng   │──ellipse──rim depth──plane RANSAC──circle fit
              └────────┬─────────┘
                       ▼ GateDetection (map frame)
              ┌──────────────────┐        ┌───────────────────┐
              │ N2 TRACK/MAP gate│        │ N3 VOXEL MAP 5Hz  │◄── Depth
              │ ID, lọc, thứ tự  │        │ raycast + inflate │
              └────────┬─────────┘        └─────────┬─────────┘
                       │  gate TARGET + anchors      │ grid + distance field
                       └──────────────┬──────────────┘
                                      ▼ (sự kiện: target mới / lệch / chặn)
                          ┌──────────────────────┐
                          │ N4 DIJKSTRA          │  26-neighbor, hộp thu hẹp
                          │ + shortcut           │  w = dist×(1+λc·clearance)
                          └──────────┬───────────┘
                                     ▼ nav_msgs/Path  (CHỈ HÌNH HỌC)
                          ┌──────────────────────────────┐
                          │ ★ N5 MUELLER TRAJECTORY      │
                          │   • đoạn tự do: §5.6         │
                          │   • đoạn chui vòng: §5.4-5.5 │
                          │   • feasibility check §5.3   │
                          │   • Tf ×1.4 nếu infeasible   │
                          └──────────┬───────────────────┘
                                     ▼ TrajectorySegment(α,β,γ,Tf) mỗi trục
             ┌───────────────────────┼─────────────────────────┐
             ▼                       ▼                         ▼
   ┌──────────────────┐   ┌──────────────────────┐  ┌──────────────────┐
   │ N6 FSM nhiệm vụ  │──►│ ★ N5 SAMPLER 20Hz    │  │ N7 OVERLAY       │
   │ (sơ đồ §6.2)     │   │ p(τ), v(τ), a(τ), yaw│  │ chiếu quỹ đạo→ảnh│
   └──────────────────┘   │ thread riêng, không  │  │ + HUD v, a, err  │
                          │ bị planner chặn      │  └──────────────────┘
                          └────────┬─────────────┘
                                   ▼
            ★ /mavros/setpoint_raw/local  (PositionTarget, mask 2048)
                                   ▼
              ArduPilot GUIDED (SITL) → Gazebo → cảm biến → lặp lại
```

---

# PHẦN 11 — THAM SỐ HỆ THỐNG (BẢNG TRA NHANH v2)

| Nhóm | Tham số | Giá trị khởi điểm |
|---|---|---|
| Vòng | r_in / độ dày ống | 0.75 m / 0.08 m |
| Anchor | d_pre / d_post | 1.5 m / 1.5 m (co 1.0 m nếu vòng sát nhau) |
| Detect | min điểm vành / sai số bán kính / RMS plane tối đa | **30 / ±0.20 m / 0.06 m** (giá trị đã tinh chỉnh trong `gate_detect.py`, thay cho 12 / ±0.15 m đề xuất ban đầu). ⚠️ `MAX_PLANE_RMS` phải đổi theo khoảng cách khi ra phần cứng — §13.6 |
| Detect | vòng: bán kính tâm ống / đường kính trong / ngoài | 0.83 m / **1.50 m** (dùng cho range mono) / 1.82 m |
| Camera | `FX_COLOR` / `FX_DEPTH` @848×480 | **616.92 px** (hfov 69°) / **446.80 px** (hfov 87°) |
| Tracker | ngưỡng association / số detect để CONFIRMED | 1.0 m / 5 |
| Voxel | size / phạm vi / tần số / inflation | 0.2 m / 30×20×8 m / 5 Hz / 0.6 m |
| Dijkstra | λ_c / λ_u / d_safe / hộp thu hẹp / giới hạn thời gian | 2.0 / 0.5 / 1.0 m / **đoạn + 3 m** / 200 ms |
| Bay | v_cruise / v_traverse | 1.0 m/s / **0.8 m/s cố định** (§5.5 hoãn) |
| ★ Mueller | **a_horiz_max** (70% `WPNAV_ACCEL`=2.5) | **1.75 m/s²** — xem C9/R13, KHÔNG phải 4.0 |
| ★ Mueller | **j_horiz_max** (70% `PSC_JERK_XY`=5.0) | **3.5 m/s³** |
| ★ Mueller | v_horiz_max | **70% `WPNAV_SPEED`** — đọc thật ở §0.2 |
| ★ Mueller | a_vert_max / j_vert_max | **70% `WPNAV_ACCEL_Z` / `PSC_JERK_Z`** |
| ★ Mueller | minTimeSection / hệ số nới Tf / số lần thử | **0.02 s / ×1.4 / 5** |
| ★ Mueller | Tf_min / **Tf đoạn B (chui vòng)** | **0.5 s / ghi cứng `‖P_post−C‖ / v_traverse`** (§5.4) |
| ~~Mueller~~ | ~~f_min / f_max / ω_max~~ | ~~5.0 / 13.0 m/s² / 3.0 rad/s~~ — **KHÔNG dùng qua GUIDED**, chỉ đúng khi điều khiển thrust+body-rate trực tiếp (§5.3.2) |
| ★ Hành lang | bán kính cho CheckPositionFeasibility (đoạn tự do / chui vòng) | **0.5 m / r_in − 0.25 = 0.5 m** |
| ★ Setpoint | topic / type_mask / tần số | **`setpoint_raw/local` / 2048 (fallback 3520) / 20 Hz** |
| ★ Tracking | ngưỡng hover / ngưỡng ABORT | **0.5 m / 1.0 m** |
| PASSED | tiến dọc trục / lệch trục **tại điểm cắt nội suy** | đổi dấu s(t) / < r_in − 0.1 m |
| Watchdog | mất pose / mất ảnh / timeout state | 0.5 s / 1 s / theo §6.3 |

---

# PHẦN 12 — KẾ HOẠCH TRIỂN KHAI & TIÊU CHÍ NGHIỆM THU (v2)

> **Trạng thái cập nhật 2026-07-18.** Cột "Đã đo" là số thật trên SITL, không phải ước lượng. Đọc kèm §13: **mọi số dưới đây đo trong sim, nơi depth hoàn hảo** — chưa nói gì về phần cứng.

| GĐ | Trạng thái | Nội dung | Nghiệm thu | Đã đo |
|---|---|---|---|---|
| 1 | ✅ | Thêm model vòng; xác nhận topic, encoding depth, TF tĩnh; **đọc tham số ArduCopter (§0.2)** | topic 30 Hz; depth đúng mét; TF đủ chuỗi; bảng `ANGLE_MAX`, `WPNAV_*` | topic verify qua `d435i_bridge.yaml`; depth `32FC1` mét; focal 616.92 px @848×480. **Param đọc thật từ SITL** (`mav.parm`, ArduCopter V4.6.3): `WPNAV_ACCEL` **250 cm/s²**, `PSC_JERK_XY` **5.0**, `PSC_JERK_Z` **5.0**, `WPNAV_ACCEL_Z` **100 cm/s²**, `WPNAV_SPEED` **1000 cm/s**, `ANGLE_MAX` **3000**. Khớp đúng giá trị mặc định đọc từ source ở §5.3.0 |
| **2** ★ | ✅ **R11 ĐÃ CHỐT** | **MAVROS: GUIDED→ARM→TAKEOFF. Test riêng: `setpoint_raw` có/không accel** | stream 20 Hz không đứt; **chốt mask** | **ArduCopter V4.6.3 CÓ nhận accel.** A/B cô lập (2496 vs 2048), a=2.0 m/s² trên +X, 3 vòng: A lệch **0.021 / 0.038 / 0.036 m** (v ≤ 0.05) — B lệch **1.017 / 1.008 / 1.017 m** (v ≈ 0.83). Chênh trung vị **+0.980 m**. ⇒ **N5 dùng mask 2048**, không cần fallback |
| 3 | ✅ | N1+N2: đứng yên detect 1 vòng | sai số tâm < 0.15 m; normal < 10°; ổn định ≥ 30 s | **sai số tâm < 0.01 m** (`ID 0 → (4.996, 0.013, 3.503)` vs thật `(5, 0, 3.5)`); r 0.833 vs 0.83; **100% frame** ra detection |
| 4 | ✅ | N3+N4: dựng voxel, Dijkstra tới điểm cho trước có vật cản | path không cắt OCCUPIED; **plan < 200 ms** | path đúng: 16 wp, 6.68 m, kết tại `(6.5, 0, 3.5)` = `C + 1.5n` ✓. **Đã profile trên lưới thật 312k** (§4.6): chặng thật **47–75 ms**, dư ngưỡng rộng rãi. Con số 292 ms cũ là do **tranh CPU với cả stack SITL**, không phải giới hạn thuật toán. Đã sửa một bug nặng hơn: retry toàn lưới lặp mỗi tick |
| **5** ★ | ✅ | **N5 Mueller độc lập: bay quỹ đạo bậc 5 giữa 2 điểm cho trước, không perception** | tracking error RMS < 0.15 m; **gia tốc/jerk lệnh không bao giờ bị ArduCopter clamp**; C² khi nối đoạn | Đo live bằng `verify_n5` (803 mẫu, bay 6.50 m): **tracking RMS 0.042 m**, max 0.140 → đạt. **accel lệnh đỉnh ngang 0.959** vs `WPNAV_ACCEL` 2.5 → **không bị clamp**; dọc 0.000 vs 1.0. `type_mask` 2048. C² đã assert trong `trajectory_follower --selfcheck`. Kết thúc đúng `P_post` |
| 6 | ⬜ chưa làm | Ghép FSM: bay qua 1 vòng thẳng | 10/10 lần PASSED, không chạm vành; **khoảng cách nhỏ nhất tới vành > 0.25 m** | — |
| 7 | ⬜ chưa làm | 3–5 vòng, có vòng nghiêng + lệch cao | ≥ 9/10 hoàn thành chuỗi; log thời gian toàn tuyến | ⚠️ `<far>` D435i = 10 m ⇒ vòng ở 15/20 m có **0 pixel depth**; bản đồ xây dần khi bay tới |
| 8 | ⬜ chưa làm | N7 overlay + quay video demo | quỹ đạo hiển thị đúng vị trí ảnh ở mọi góc quay | — |
| 9 | ⬜ chưa làm | Stress-test edge cases §8 (kể cả 19–23 mới) | mỗi case kích hoạt đúng nhánh, không crash, không ABORT oan | — |
| **10** ★ | ⬜ chưa làm | **So sánh định lượng v1 vs v2** (carrot-chasing vs Mueller) trên cùng kịch bản | bảng số liệu cho báo cáo — xem metric dưới | — |
| **11** ★ | ⬜ chưa làm | **Chuyển sang phần cứng thật** — đo σ_Z thật, trôi `map`, độ trễ, thời gian plan trên companion | **H1–H6 của §13.7**; ngân sách sai số cộng dồn **< 50 mm** trước khi bay thật | — |

**Hai chỗ hở phải đóng trước khi viết N5:**

1. **GĐ2 — test mask 2048 vs 3520 (R11).** Rẻ: `hold_and_watch` đang hover giữ vị trí sẵn, thêm ~20 dòng bắn `setpoint_raw` mask 2048 với accel khác 0 rồi so `/mavros/local_position/velocity_local` là biết ArduCopter đời này có nhận accel không. Cả tầng N5 dựng trên câu trả lời này.
2. **GĐ4 — đỉnh 292 ms vượt ngưỡng.** R1 đã vạch sẵn đường (thu hộp bao, hạ voxel 0.2 → 0.25 m). Đừng để trôi sang phần cứng rồi mới sửa.

**Metric đánh giá cuối (v2 mở rộng):**

| Metric | Ý nghĩa | v1 có? |
|---|---|---|
| Tỉ lệ pass/vòng | độ tin cậy | ✅ |
| Khoảng cách nhỏ nhất tới vành khi chui | an toàn | ✅ |
| Tổng chiều dài path / đường chim bay | tính "tối ưu" hình học | ✅ |
| Thời gian plan trung bình, số lần replan | hiệu năng | ✅ |
| ★ **Tổng ∫jerk² toàn tuyến** | độ mượt — lý do dùng Mueller | ❌ mới |
| ★ **Gia tốc đỉnh** so với `g·tan(ANGLE_MAX)` | có bị clamp không | ❌ mới |
| ★ **Tracking error RMS và max** | chất lượng bám quỹ đạo | ❌ mới |
| ★ **Thời gian hoàn thành toàn tuyến** | v2 nên nhanh hơn v1 nhờ không phải dừng ở mỗi waypoint | ❌ mới |

---

# PHẦN 13 ★ — CHUYỂN SANG PHẦN CỨNG THẬT

> **Vì sao có phần này:** v1→v2 nâng cấp toàn bộ ở tầng **control**. Nhưng khi rời SITL, thứ giết mission nằm ở tầng **perception + state estimation + độ trễ** — ba thứ mà cả v1 lẫn v2 chưa có dòng nào. Mọi số đo đẹp hiện tại (sai số tâm <1 cm) đều đo trong sim, nơi depth là **hoàn hảo tuyệt đối**. Chúng **sẽ không tái lập** trên phần cứng.

## 13.1. Ngân sách sai số — con số chi phối mọi thứ

Lỗ vòng `r_in = 0.75 m`, drone Iris rộng ~0.45 m ⇒ **biên lọt vòng mỗi bên chỉ ~50 mm** (số này đã chốt từ giai đoạn dựng world). Mọi nguồn sai số dưới đây **cộng dồn** vào 50 mm đó:

| Nguồn | SITL | Phần cứng thật |
|---|---|---|
| Nhiễu depth → sai số tâm & pháp tuyến | ~0 | §13.2 — nguồn lớn nhất |
| Trôi frame `map` (EKF) giữa lúc plan và lúc chui | ~0 | §13.3 |
| Độ trễ pipeline × vận tốc bay | ~0 | §13.4 |
| Bám quỹ đạo (tracking error) | đo được ở GĐ5 | + gió, + rung |

**Ngân sách phải lập thành bảng và cộng lại trước khi bay thật.** Nếu tổng vượt 50 mm thì không phải chỉnh code — mà phải **làm vòng to hơn hoặc bay chậm hơn**.

## 13.2. Nhiễu depth thật — và vì sao nó buộc phải đổi chiến lược

Sai số depth stereo tăng theo **bình phương** khoảng cách:

```
σ_Z  =  Z² · σ_d / (f_depth · B)

f_depth = 446.8 px   (hfov 87°, W=848 → 424/tan(43.5°))
B       = 0.050 m    (baseline stereo D435i thật)
σ_d     ≈ 0.2 px     (sai lệch disparity thực tế, đã tính subpixel)
⇒ f·B   = 22.34 px·m
```

| Z | σ_Z (depth thật) | σ_z (range mono từ đường kính lỗ) |
|---|---|---|
| 5.0 m | **0.224 m** | ~0.054 m |
| 3.0 m | 0.081 m | ~0.033 m |
| 1.5 m | **0.020 m** | ~0.016 m |
| 1.0 m | 0.009 m | ~0.011 m |

*(cột mono: lỗ 1.5 m ở 5 m chiếm `616.92×1.5/5 ≈ 185 px`; sai số fit ellipse ~2 px ⇒ ~1.1% khoảng cách)*

**Ba kết luận rút ra:**

1. **Quyết định thiết kế sẵn có của N1 là đúng, và đúng vì lý do sẽ chỉ lộ ra trên phần cứng thật.** N1 lấy khoảng cách từ **đường kính lỗ** (hình học mono), không từ depth. Ở 5 m, mono sai ~5 cm còn depth sai ~22 cm — **tốt hơn 4 lần**. Giữ nguyên, đừng ai "tối ưu" nó sang dùng depth.
2. **Pháp tuyến `n` là chỗ sẽ vỡ.** `n` đến từ plane-fit trên các điểm vành back-project từ depth — tức là đúng cái đại lượng nhiễu nhất. Ống vòng dày 0.08 m, ở 5 m chỉ chiếm `0.08/5 × 446.8 ≈ 7 px`, cộng flying pixels ở mép (luôn **lệch về phía camera**, là bias hệ thống nên **lấy trung bình không khử được**). Mà cả §5.4 dựng trên `n`.
3. ⚠️ **Dự đoán cụ thể: trên phần cứng thật, N1 sẽ im lặng loại sạch mọi detection xa.** `MAX_PLANE_RMS = 0.06` trong `gate_detect.py` được hiệu chỉnh theo depth sim. Ở 5 m thật, σ_Z = 0.22 m ⇒ plane RMS vượt 0.06 ⇒ **reject**, không log nào giải thích. Ở 1.5 m, σ_Z = 0.02 m ⇒ qua. Đây là **cùng một loại lỗi im lặng** với bug `RETR_EXTERNAL` đã trả giá ở N1.

**⇒ Chiến lược bắt buộc đổi: KHÔNG tin pose vòng ở xa.**

```
Xa (> 3 m)   : chỉ dùng BEARING + range mono. Không tin n. Bay tới theo hướng.
Gần (< 2 m)  : depth đủ tốt (σ_Z ~2-3 cm) → lúc này mới dựng n, mới chốt P_pre/P_post
TRAVERSE     : khoá, như cũ
```

Điều này **hợp** với `D_PRE = D_POST = 1.5 m` đang dùng: đúng khoảng cách mà depth vừa đủ tin. Không phải đổi hằng số, phải đổi **thứ tự tin tưởng**.

## 13.3. Trôi frame `map`

N2 lưu pose vòng trong `map`. Bay trong nhà không GPS, `map` của EKF3 **trôi vài chục cm** là bình thường. Vòng phát hiện từ 5 m trước, lúc bay tới thì toạ độ đã cũ.

- §4.5 hiện replan khi gate lệch **> 0.3 m**. Ngưỡng đó **to gấp 6 lần biên an toàn 50 mm** — vô dụng cho phần cứng. Siết xuống **0.05 m** cho pha gần.
- Cách chắc hơn: ở pha ALIGN/TRAVERSE, **tính P_pre/P_post trong frame body/camera tại thời điểm đó**, không đọc lại toạ độ `map` đã lưu. Trôi frame không ảnh hưởng nếu cả vòng lẫn drone cùng đo trong một khung tại cùng thời điểm.
- Bật `EK3_SRC*` dùng VIO/optical-flow nếu bay trong nhà, và **log `/mavros/local_position/pose` so với đo ngoài** để biết trôi bao nhiêu trước khi tin.

## 13.4. Ngân sách độ trễ — hiện chưa có ở đâu trong plan

SITL ~0 ms. Thật: `USB → driver → ROS → N1 detect → N2 track → N4 plan → MAVLink` dễ **100–200 ms**. Ở 0.8 m/s là **8–16 cm** — lại ăn hết biên 50 mm.

- **Phải đo, không đoán**: đóng dấu `stamp` ảnh gốc và giữ nó xuyên suốt tới lúc phát setpoint, log hiệu.
- **Bù**: N2 đã có low-pass; thêm dự đoán tiến `Δt` bằng vận tốc drone (`/mavros/local_position/velocity_local`) khi biến đổi pose vòng về `map`.
- Đây là chỗ Mueller **thực sự có lợi**: quỹ đạo là hàm thời gian nên bù trễ chỉ là sample tại `τ + Δt`. Carrot không làm được.

## 13.5. Tính toán trên companion computer

Số hiện tại (112–292 ms cho Dijkstra) đo trên **desktop**. Trên Pi/Jetson chậm 3–10 lần ⇒ **có thể vượt 200 ms**.

- Đo lại trên chính companion trước khi tin. Nếu trượt: hạ voxel 0.2 → 0.25 m, thu hộp bao (R1 đã vạch sẵn đường).
- **Không** viết vòng lặp sinh 30 ứng viên Mueller của §5.5 bằng Python trên companion. Chốt `v_traverse` cố định (xem §5.5 đã ghi chú).
- Grid 312k voxel là bộ nhớ thật trên máy nhỏ — kiểm RAM.

## 13.6. Danh sách thay đổi cụ thể trong code

| File | Thay đổi | Lý do |
|---|---|---|
| `gate_detect.py` | `MAX_PLANE_RMS` 0.06 → **theo khoảng cách**, ví dụ `max(0.06, 1.5·σ_Z(z))` | §13.2 mục 3 — nếu không sẽ reject sạch, im lặng |
| `gate_detect.py` | Bật RANSAC thay 2-sigma trim trong `fit_plane` | Móc đã chừa sẵn ở comment `fit_plane`; flying pixels thật là outlier có bias, trim 2σ không đủ |
| `gate_detect.py` | Publish thêm **cờ tin cậy** `normal_valid` theo khoảng cách | Để N4/N6 biết khi nào được tin `n` |
| `gate_tracker.py` | Lọc bearing riêng, pose đầy đủ riêng | Xa thì chỉ bearing đáng tin |
| `gate_planner.py` | Ngưỡng replan 0.3 → **0.05 m** ở pha gần | §13.3 |
| toàn bộ | Log `stamp` gốc → đo trễ đầu-cuối | §13.4 |

## 13.7. Nghiệm thu bổ sung cho phần cứng (GĐ 11)

| # | Kiểm tra | Đạt khi |
|---|---|---|
| H1 | Đo σ_Z thật của D435i ở 1/2/3/5 m so bia phẳng đã biết | có bảng số thật, thay bảng ước lượng §13.2 |
| H2 | Đo trôi `map` sau 5 phút hover | < 0.10 m, hoặc chuyển nguồn EKF |
| H3 | Đo trễ đầu-cuối (ảnh → setpoint) | có con số, đưa vào ngân sách §13.1 |
| H4 | Đo lại thời gian plan **trên companion** | < 200 ms |
| H5 | Bảng ngân sách sai số cộng dồn | tổng **< 50 mm**, nếu không → vòng to hơn / bay chậm hơn |
| H6 | Bay qua 1 vòng thật, có lưới bảo vệ | 10/10 PASSED, khoảng cách nhỏ nhất tới vành > 0.10 m |

> **Nguyên tắc chuyển giao:** không bay thật cho tới khi H1–H5 có **số đo thật**, không phải ước lượng. Sim đã dạy đúng một bài học đắt (bug `RETR_EXTERNAL`, 0 detection không log): **hệ này thất bại im lặng, không thất bại ồn ào.** Phần cứng chỉ làm điều đó tệ hơn.

---

# PHỤ LỤC A — RỦI RO THIẾT KẾ ĐÃ LƯỜNG TRƯỚC

1. **Depth mép vật thể nhiễu mạnh** → sample vành dày ±3 px + percentile + RANSAC.
2. **Inflation nuốt mất lỗ vòng** → virtual corridor (§3.2). Bỏ qua điểm này thì Dijkstra luôn FAILURE hoặc bay vòng ra ngoài.
3. **Replan liên tục gây dao động** → replan theo sự kiện; khóa khi ALIGN/TRAVERSE.
4. **Sai frame ENU/NED/optical** → quy về `map` ngay tại nguồn; chỉ MAVROS chạm tới NED.
5. **Dijkstra chậm nếu lưới quá mịn** → voxel 0.2 m + dừng sớm + **hộp thu hẹp** (R1). Nếu vẫn chậm mới hạ 0.25 m, không đổi thuật toán.
6. ★ **Phụ thuộc phiên bản ArduCopter** → rủi ro lớn nhất của v2. Chốt ở GĐ 2, có sẵn fallback (§5.1, R11).
7. ★ **Đặt giới hạn feasibility theo motor thay vì theo ArduCopter** → drone bám sai quỹ đạo mà không báo lỗi. Phát hiện bằng giám sát tracking error (§6.3, R2).
8. ★ **Dùng gia tốc đo thay vì gia tốc lệnh khi replan** → giật ở điểm nối, có thể kích hoạt dao động (§5.8, R4).
9. ★ **Lạm dụng Mueller cho đoạn dài nhiều vật cản** → infeasible liên tục. Mueller là **local**, Dijkstra là **global**; không đảo vai trò (§5.6).
10. ★ **Quên rằng ArduCopter đã jerk-shape sẵn** → tưởng drone bay theo đa thức Mueller, thực ra bay theo quỹ đạo đã bị nắn lại; kiểm feasibility sai đại lượng (thrust/body-rate thay vì vel/accel/jerk). Chính v2 đã dính (§5.3, R13).
11. ★ **Tin số đo của sim khi sang phần cứng** → depth sim hoàn hảo, depth thật sai ~0.22 m ở 5 m; ngưỡng hiệu chỉnh theo sim sẽ **loại sạch detection mà không log**. Đây là rủi ro lớn nhất của cả dự án khi ra thực địa (§13).

---

# PHỤ LỤC B — RANH GIỚI TRÁCH NHIỆM (một trang tóm tắt)

| Tầng | Câu hỏi nó trả lời | Không được làm |
|---|---|---|
| N1/N2 Perception | "Vòng ở đâu, hướng nào?" | không đọc pose thật từ Gazebo |
| N3 Voxel map | "Chỗ nào đi được?" | không quyết định đường đi |
| N4 Dijkstra | "**Đi đường nào** để tránh vật cản?" | không quyết định vận tốc/thời gian |
| ★ N5 Mueller | "**Đi như thế nào theo thời gian?**" (p, v, a, jerk) | không tránh vật cản toàn cục — chỉ kiểm tra hành lang cục bộ |
| N6 FSM | "**Bây giờ đang làm giai đoạn nào?**" | không tính toán quỹ đạo |
| ArduPilot | "Giữ thăng bằng, bám setpoint, an toàn" | ROS2 không bao giờ can thiệp |

Vi phạm ranh giới này là nguồn gốc của gần như mọi bug khó trong hệ thống loại này.
