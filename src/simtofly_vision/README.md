# simtofly_vision

Vision node bám người (YOLO detect + pose + OSNet ReID). Publish 2 topic mà
`simtofly_mavros_sitl/trackinghuman` subscribe:

| Topic | Type | Nội dung |
|-------|------|----------|
| `/follow/command` | `std_msgs/String` | `FOLLOW` \| `BACK_OFF` \| `SEARCH` \| `IDLE` |
| `/follow/target`  | `geometry_msgs/Vector3` | `x`=ang_x(deg), `y`=ang_y(deg), `z`=khoảng cách(m) |

## Dependency (KHÔNG phải rosdep — cài bằng pip)

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

## Chạy

```bash
# terminal 1: vision (publish /follow/*)
ros2 run simtofly_vision person_follow

# terminal 2: control (subscribe /follow/*, bay theo)
ros2 run simtofly_mavros_sitl trackinghuman
```

Hoặc dùng launch gộp: `ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=trackinghuman`
(tự mở thêm cửa sổ vision).
