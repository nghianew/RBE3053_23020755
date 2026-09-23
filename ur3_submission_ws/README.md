# UR3/UR3e Letter Drawer

Workspace nộp GitHub cho bài toán điều khiển robot UR3/UR3e vẽ chữ N và hình tròn bằng ROS 2, Gazebo và MoveIt 2.

## Tương thích Ubuntu

| Ubuntu | ROS 2 nên dùng | Lệnh source |
|---|---|---|
| 22.04 LTS | Humble | `source /opt/ros/humble/setup.bash` |
| 24.04 LTS | Jazzy | `source /opt/ros/jazzy/setup.bash` |

Nếu thầy sử dụng Ubuntu 22.04 LTS, không dùng ROS 2 Jazzy; hãy cài ROS 2 Humble và các package Universal Robots phiên bản hoặc branch tương thích với Humble. Mã nguồn `ur3_letter_drawer` không cần đổi logic C++, chỉ cần thay lệnh `source` và dùng đúng dependency bên ngoài.

## Thành phần trong workspace

Workspace này chỉ chứa package tự viết:

```text
ur3_submission_ws/
├── README.md
└── src/
    └── ur3_letter_drawer/
```

Các package Universal Robots là dependency bên ngoài, không được chép vào repository này. Khi chạy, cần đặt chúng trong một ROS 2 workspace khác hoặc clone/install chúng theo hướng dẫn của Universal Robots.

## Package tự viết

`ur3_letter_drawer` gồm:

- `draw_letter_n.cpp`: tạo và thực thi quỹ đạo chữ N.
- `draw_circle.cpp`: tạo và thực thi quỹ đạo hình tròn.
- `draw_letter.launch.py`: khởi động mô phỏng đầy đủ và node vẽ chữ N.
- `draw_n_only.launch.py`: chạy riêng node chữ N trên mô phỏng đã khởi động.
- `draw_circle_only.launch.py`: chạy riêng node hình tròn trên mô phỏng đã khởi động.
- `BAO_CAO.md`: sườn báo cáo và hướng dẫn hoàn thiện nội dung nộp.

## Dependency cần cài hoặc có trong workspace khác

- ROS 2 Humble trên Ubuntu 22.04 hoặc ROS 2 Jazzy trên Ubuntu 24.04
- `ur_simulation_gz`
- `ur_moveit_config`
- `moveit_ros_planning_interface`
- `geometry_msgs`
- `visualization_msgs`

## Build

```bash
cd ~/ur3_submission_ws
# Ubuntu 22.04:
source /opt/ros/humble/setup.bash
# Ubuntu 24.04:
# source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_letter_drawer --symlink-install
source install/setup.bash
```

## Chạy

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

# Khởi động Gazebo + MoveIt + RViz và vẽ chữ N
ros2 launch ur3_letter_drawer draw_letter.launch.py ur_type:=ur3e

# Chạy riêng hình tròn khi mô phỏng đã chạy
ros2 launch ur3_letter_drawer draw_circle_only.launch.py ur_type:=ur3e
```

## RViz Marker

- Chữ N: `/letter_path_n`
- Hình tròn: `/letter_path_circle`

Trong RViz, thêm display `Marker` và chọn topic tương ứng.
