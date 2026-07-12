# simtofly_vision

Hai node vision, độc lập nhau:

| Node | Dùng cho | Publish |
|------|----------|---------|
| `person_follow` | `trackinghuman` — bám người (YOLO + OSNet ReID) | `/follow/command`, `/follow/target` |
| `gate_detect`   | `mission_gates` — bay xuyên vòng (D435i) | `/gate/obs` |

---

## `gate_detect` — dò vòng bằng RealSense D435i

Đọc camera D435i gắn mũi drone (`model://iris_with_d435i`), tìm vòng cam bằng HSV +
`fitEllipse`, rồi **đo khoảng cách bằng luồng depth**.

> **Cần world + model:** drone (có D435i ở mũi), vòng đua và world nằm ở repo riêng —
> [phantrungtien/gazebo_ardupilot_depth_camera](https://github.com/phantrungtien/gazebo_ardupilot_depth_camera).
> Clone vào **`~/gz_ws`** (launch hardcode đường này) rồi build `ardupilot_gazebo` bên trong
> nó. Thông số camera + những thứ sim KHÔNG mô phỏng được nằm trong README của repo đó.

**Hai luồng, hai đường truyền khác nhau** — đây là lý do phải chạy thêm `ros_gz_bridge`:

| Luồng | Đường truyền | Vì sao |
|---|---|---|
| color 848×480, hfov 69° | H264/RTP trên UDP 5600, đọc bằng `gst-launch` | Sensor color còn mang `GstCameraPlugin` của ardupilot_gazebo. OpenCV trên máy này build **không có GStreamer** nên `cv2.VideoCapture(CAP_GSTREAMER)` vô dụng. |
| depth 848×480, hfov 87° | Topic Gazebo → ROS qua `ros_gz_bridge` | Không có plugin stream nào, phải bridge. |

`/gate/obs` = `Float32MultiArray([visible, cx_off, cy_off, fill, defo, dist])`, `dist` là
**mét tới MẶT PHẲNG vòng**.

Hai chi tiết dễ làm sai, đều đã xử lý trong code:

- **Không lấy depth ở tâm vòng** — tâm là *cái lỗ*, depth ở đó đọc nền phía sau. Phải
  lấy median depth trên **vành ống** (`orange_mask ∩ contour đã fill`).
- **Camera thấy mặt GẦN của ống**, mà tâm ống mới nằm trên mặt phẳng vòng → cộng thêm
  bán kính ống (0.06 m). Bỏ khoản này là hụt 6 cm, trong khi biên lọt vòng chỉ có 5 cm.

Nếu depth stale hoặc vòng ở ngoài far-clip 10 m, node tự **fallback về ước lượng mono**
(kích thước vòng đã biết: đường kính ngoài 0.982 m).

```bash
# 6 cửa sổ, gồm cả DepthBridge, và gate_detect chạy kèm GUI (xem ellipse nó fit lên vòng)
ros2 launch simtofly_mavros_sitl gates_mission.launch.py

# hoặc qua launch chung — tự chọn gates_course.sdf + tự mở DepthBridge, nhưng KHÔNG có GUI
ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_gates

python3 simtofly_vision/gate_detect.py --selfcheck          # kiểm hình học, không cần sim
```

> Không thấy frame nào? `ENABLE_TOPIC` (mặc định `/d435i/color/image_raw/enable_streaming`)
> suy ra từ `GstCameraPlugin.cc:221`. Xem dòng `GstCameraPlugin: enable topic [...]` trong
> cửa sổ Gazebo để đối chiếu, rồi override bằng env `GATE_ENABLE_TOPIC`.

---

## `person_follow` — bám người

Publish 2 topic mà `simtofly_mavros_sitl/trackinghuman` subscribe:

| Topic | Type | Nội dung |
|-------|------|----------|
| `/follow/command` | `std_msgs/String` | `FOLLOW` \| `BACK_OFF` \| `SEARCH` \| `IDLE` |
| `/follow/target`  | `geometry_msgs/Vector3` | `x`=ang_x(deg), `y`=ang_y(deg), `z`=khoảng cách(m) |

### Dependency (KHÔNG phải rosdep — cài bằng pip)

Node cần các gói ML nặng, cài vào **cùng Python với ROS** (Humble = Python 3.10):

```bash
pip install torch ultralytics opencv-python numpy
pip install torchreid          # ReID (OSNet)
pip install pymavlink          # tùy chọn: bù roll/pitch camera gắn cứng
```

> Nếu dùng virtualenv, phải tạo với `--system-site-packages` để node vẫn
> `import rclpy` được; nếu không, node rơi vào nhánh fallback "chỉ in, KHÔNG publish".

Model `yolo11m.pt` + `yolo11m-pose.pt` ultralytics **tự tải** lần chạy đầu (không
commit vào git — xem `.gitignore`). Cần **camera thật** (`CAM_INDEX` trong file) và
GPU CUDA (`DEVICE="cuda"`).

### Chạy

```bash
# terminal 1: vision (publish /follow/*)
ros2 run simtofly_vision person_follow

# terminal 2: control (subscribe /follow/*, bay theo)
ros2 run simtofly_mavros_sitl trackinghuman
```

Hoặc dùng launch gộp: `ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=trackinghuman`
(tự mở thêm cửa sổ vision).
