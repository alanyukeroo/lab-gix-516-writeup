# 🤖 TurtleBot Run Commands

## Case 1 — Simulation TurtleBot4 🖥️

> `use_sim_time:=True` | Model: `waffle` | No physical hardware needed

### Terminal 1 — Gazebo + Nav2 + RViz (sekaligus)
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  nav2:=true \
  slam:=false \
  localization:=true \
  rviz:=true \
  world:=maze \
  map:=/home/ubuntu/turtlebot4_ws/src/lab5/maps/gazebo_maze.yaml
```

> ⚠️ Setelah RViz terbuka, set **2D Pose Estimate** dulu untuk posisi awal robot di map!

### Terminal 2 — YOLO (kamera simulasi)
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
ros2 launch yolo_bringup yolov8.launch.py \
  model:=~/turtlebot4_ws/yolov8n.pt \
  input_image_topic:=/oakd/rgb/preview/image_raw \
  image_reliability:=2
```

### Terminal 3 — Person Obstacle Node
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
python3 ~/turtlebot4_ws/person_obstacle_node.py
```

### Terminal 4 — Dashboard
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
python3 ~/turtlebot4_ws/dashboard_robot.py
```

---

## Case 2 — Real World TurtleBot3 Burger 🍓

> `use_sim_time:=False` | Model: `burger` | Pi + USB Camera

### 🍓 Raspberry Pi

**Terminal 1 — Hardware:**
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Terminal 2 — USB Camera:**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=320 \
  -p image_height:=240 \
  -p framerate:=10.0 \
  -p pixel_format:=mjpeg2rgb \
  -r image_raw:=/image_raw
```

### 💻 Laptop

**Terminal 1 — YOLO:**
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
ros2 launch yolo_bringup yolov8.launch.py \
  model:=~/turtlebot4_ws/yolov8n.pt \
  input_image_topic:=/image_raw \
  image_reliability:=2
```

**Terminal 2 — Person Obstacle Node:**
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
python3 ~/turtlebot4_ws/person_obstacle_node.py
```

**Terminal 3 — Nav2:**
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
ros2 launch turtlebot4_navigation nav2.launch.py \
  use_sim_time:=False \
  map:=$HOME/maps/my_map.yaml
```

**Terminal 4 — Dashboard:**
```bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=38
python3 ~/turtlebot4_ws/dashboard_robot.py
```

---

## 📊 Perbandingan

| | Simulation TB4 | Real World TB3 |
|---|---|---|
| Launch command | `turtlebot4_ignition.launch.py` | `robot.launch.py` |
| `use_sim_time` | `True` (auto) | `False` |
| Robot model | `waffle` | `burger` |
| Camera topic | `/oakd/rgb/preview/image_raw` | `/image_raw` |
| Initial pose | Set via **2D Pose Estimate** di RViz | Otomatis dari odometry |
| USB Camera | ❌ tidak perlu | ✅ Pi Terminal 2 |
| Hardware Pi | ❌ tidak perlu | ✅ Pi Terminal 1 |
| Jumlah terminal | 4 (semua laptop) | 2 Pi + 4 laptop |

---

## ✅ Checklist Sebelum Run

### Simulation
```bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=38
```

### Real World
```bash
# Di laptop
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=38
ping 10.155.234.163        # pastikan Pi nyambung

# Di Pi (cek .bashrc sudah ada belum)
echo $ROS_DOMAIN_ID        # harus 38
echo $TURTLEBOT3_MODEL     # harus burger
```

---

## 🔄 Urutan Run yang Bener

### Simulation
```
Terminal 1: turtlebot4_ignition.launch.py (Gazebo + Nav2 + RViz)
      ↓
Set 2D Pose Estimate di RViz ← WAJIB sebelum lanjut!
      ↓
Terminal 2: YOLO
      ↓
Terminal 3: Person Obstacle Node
      ↓
Terminal 4: Dashboard
```

### Real World
```
Pi: robot.launch.py        ← pertama
Pi: usb_cam                ← kedua
      ↓
Laptop: YOLO               ← setelah /image_raw nyampe
Laptop: Person Obstacle    ← setelah YOLO jalan
Laptop: Nav2               ← setelah map ada
Laptop: Dashboard          ← terakhir
```

---

## 🗺️ Mapping (Real World Only)

> Lakukan sekali sebelum navigasi. Tidak diperlukan untuk simulasi.

**Pi — Terminal 1:**
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Pi — Terminal 2:**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=320 \
  -p image_height:=240 \
  -p framerate:=10.0 \
  -p pixel_format:=mjpeg2rgb \
  -r image_raw:=/image_raw
```

**Laptop — Terminal 1 — SLAM:**
```bash
source ~/turtlebot4_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
```

**Laptop — Terminal 2 — Teleop:**
```bash
source ~/turtlebot4_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Laptop — Terminal 3 — Save Map (setelah mapping selesai):**
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

**Laptop — Terminal 4 — Dashboard (opsional, untuk label waypoint):**
```bash
source ~/turtlebot4_ws/install/setup.bash
python3 ~/turtlebot4_ws/dashboard_robot.py
# Klik START MAPPING → gerak robot → klik 📍 SAVE CURRENT POSITION
```