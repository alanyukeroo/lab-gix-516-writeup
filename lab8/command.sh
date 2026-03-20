ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze


ros2 launch turtlebot4_navigation slam.launch.py


ros2 launch turtlebot4_navigation nav2.launch.py


need to check:

ubuntu@g14:~/turtlebot4_ws$ ros2 topic list | grep -E "/map|/scan|/cmd_vel|/aruco|/navigate"
/aruco_pose
/cmd_vel
/cmd_vel_nav
/cmd_vel_teleop
/diffdrive_controller/cmd_vel_unstamped
/map
/map_metadata
/scan
/slam_toolbox/scan_visualization


ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze slam:=true nav2:=true rviz:=true


ros2 topic pub --once /aruco_pose geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'camera_frame'
  },
  pose: {
    position: {x: 0.5, y: 0.0, z: 0.3},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"


ros2 topic pub --once /aruco_pose geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'oakd_rgb_camera_optical_frame'
  },
  pose: {
    position: {x: 0.5, y: 0.0, z: 0.3},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"



ros2 run aruco_markers aruco_markers --ros-args \
  -p marker_size:=0.2 \
  -p camera_frame:=oakd_rgb_camera_optical_frame \
  -p image_topic:=/oakd/rgb/preview/image_raw \
  -p camera_info_topic:=/oakd/rgb/preview/camera_info \
  -p dictionary:=DICT_4X4_50



  ###### ARUCO GENERATOR SCRIPT FOR GAZEBO 

  python3 -c "
import cv2
import cv2.aruco as aruco
import numpy as np

# Generate marker kecil
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker = aruco.generateImageMarker(dictionary, 0, 200)

# Buat canvas putih 500x500
canvas = np.ones((500, 500), dtype=np.uint8) * 255

# Taruh marker di tengah canvas
x_offset = (500 - 200) // 2
y_offset = (500 - 200) // 2
canvas[y_offset:y_offset+200, x_offset:x_offset+200] = marker

cv2.imwrite('marker_small.png', canvas)
print('Done!')
"

cp marker_small.png ~/turtlebot4_ws/src/turtlebot4_simulator/turtlebot4_ignition_bringup/models/aruco_cube/materials/textures/marker.png

cd ~/turtlebot4_ws
colcon build --packages-select turtlebot4_ignition_bringup
source install/setup.bash

#CHECK ALL STATUS

ros2 topic echo /stop_status

ros2 topic echo /hazard_detection

# IN RASP turtlerobot4 

sudo systemctl restart turtlebot4.service && journalctl -u turtlebot4.service -f


ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.2
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.2
