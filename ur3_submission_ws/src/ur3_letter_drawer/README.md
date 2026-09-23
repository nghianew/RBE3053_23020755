# UR3/UR3e Letter Drawer

Gói này dùng MoveIt 2 và mô phỏng Gazebo của Universal Robots để vẽ chữ `N` và hình tròn trên mặt phẳng làm việc của TCP.

## 1. Cấu trúc workspace

```text
~/ur3_ws/
├── src/
│   ├── Universal_Robots_ROS2_Driver/
│   ├── Universal_Robots_ROS2_GZ_Simulation/
│   └── ur3_letter_drawer/
├── build/
├── install/
├── log/
└── .vscode/
```

- `src/` chứa mã nguồn ROS 2.
- `build/`, `install/`, `log/` là output của quá trình build và có thể được tạo lại bất cứ lúc nào.
- `ur3_letter_drawer` là package chính của dự án.

## 2. Tương thích Ubuntu và ROS 2

- Ubuntu 22.04 LTS: dùng ROS 2 Humble.
- Ubuntu 24.04 LTS: dùng ROS 2 Jazzy.
- Các package `ur_simulation_gz` và `ur_moveit_config` phải cùng phiên bản hoặc branch tương thích với distro ROS 2 đang dùng.
- Code của package này không cần sửa khi chuyển từ Humble sang Jazzy; chỉ thay lệnh `source` và dependency bên ngoài.

Nếu thầy dùng Ubuntu 22.04, thay:

```bash
source /opt/ros/jazzy/setup.bash
```

bằng:

```bash
source /opt/ros/humble/setup.bash
```

## 3. Build ban đầu

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash  # Ubuntu 22.04
# source /opt/ros/jazzy/setup.bash  # Ubuntu 24.04
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
colcon build --packages-select ur3_letter_drawer --symlink-install
source install/setup.bash
```

## 4. Chạy đầy đủ: Gazebo + MoveIt + RViz + vẽ chữ N

Dùng lệnh này khi bạn muốn khởi động cả sim và tiến trình vẽ mới:

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash  # Ubuntu 22.04
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source install/setup.bash
ros2 launch ur3_letter_drawer draw_letter.launch.py ur_type:=ur3e
```

- `ur_type:=ur3` dùng cho UR3.
- `draw_letter.launch.py` sẽ khởi động cả Gazebo, MoveIt và RViz theo trình tự rồi mới chạy node vẽ.

## 5. Chạy riêng: chỉ vẽ chữ N trên sim đang chạy

Dùng khi Gazebo + MoveIt đã chạy sẵn:

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash  # Ubuntu 22.04
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source install/setup.bash
ros2 launch ur3_letter_drawer draw_n_only.launch.py ur_type:=ur3e
```

## 6. Chạy riêng: chỉ vẽ hình tròn trên sim đang chạy

```bash
cd ~/ur3_ws
source /opt/ros/humble/setup.bash  # Ubuntu 22.04
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source install/setup.bash
ros2 launch ur3_letter_drawer draw_circle_only.launch.py ur_type:=ur3e
```

## 7. Mẹo quan trọng

- Không chạy nhiều launch cùng lúc trên cùng máy với cùng `ROS_DOMAIN_ID`.
- Nếu đang debug nhiều máy, mỗi máy nên dùng `ROS_DOMAIN_ID` khác nhau.
- Khuyến nghị đặt `ROS_LOCALHOST_ONLY=1` nếu muốn chặn ROS 2 giao tiếp chéo qua mạng LAN.
- Trước khi relaunch, tắt stale Gazebo/RViz/MoveIt session cũ để tránh duplicate process.

## 8. Thiết kế chuyển động

- Mặt phẳng vẽ là Cartesian `base_link` Y-Z.
- Kích thước mặc định: 0.16 m × 0.20 m.
- Bước nội suy Cartesian: 5 mm.
- Trước mỗi đoạn vẽ, TCP được nâng lên 4 cm (`pen_lift`) rồi mới hạ xuống để vẽ.
- Dùng `computeCartesianPath(..., avoid_collisions=true)` để tính quỹ đạo.
- `N` được vẽ theo thứ tự: trái-dưới → trái-trên → phải-dưới → phải-trên.
- Hình tròn được tạo quanh tâm hiện tại với bán kính tương đối so với khung chữ N.

## 9. RViz Marker

Các node publish đường vẽ lên các topic sau:

```text
/letter_path_n
/letter_path_circle
```

Trong RViz:

1. Add → `Marker`
2. Chọn topic đúng với node đang chạy:
   - N: `/letter_path_n`
   - Circle: `/letter_path_circle`
3. Đặt `Fixed Frame` thành `world` hoặc `base_link` tùy màn hình hiện hành.

Nếu RViz không hiện line, hãy kiểm tra:

- launch có chạy thành công hay không,
- `ROS_DOMAIN_ID` có khớp hay không,
- `Fixed Frame` có đúng hay không,
- topic đang subscribe có đúng với node đang publish hay không.

## 10. Lưu ý về khởi động

`ros2 run` đơn lẻ không đủ vì MoveIt client cần `robot_description` cục bộ. Vì vậy, `draw_n_only.launch.py` và `draw_circle_only.launch.py` được dùng để cung cấp tham số cần thiết cho node đang chạy.
