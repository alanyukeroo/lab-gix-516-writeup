import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from irobot_create_msgs.action import Undock, Dock
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
from enum import Enum
import math
import subprocess
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration

# FIXED: import dari package yang benar
from aruco_markers_msgs.msg import MarkerArray


# ============================================================
# STATE DEFINITIONS
# ============================================================
class RobotState(Enum):
    UNDOCK          = 0   # Lepas dari dock
    EXPLORE         = 1   # Eksplorasi frontier otonom untuk peta maze
    SAVE_ARUCO      = 2   # Simpan pose ArUco dalam frame /map
    RETURN_HOME     = 3   # Nav2: kembali ke posisi start (0,0)
    APPROACH_CUBE   = 4   # Nav2: menuju ArUco cube
    STOP_AT_CUBE    = 5   # Berhenti 10cm dari cube
    DONE            = 6   # Selesai
    SPIN_SEARCH     = 7   # Spin 360° mencari ArUco di semua posisi


# ============================================================
# BRAIN NODE
# ============================================================
class BrainNode(Node):

    def __init__(self):
        super().__init__('robot_brain')
        self.state = RobotState.UNDOCK
        self.action_in_progress = False

        self.blacklisted_frontiers = []
        self._last_sent_goal = None

        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = 'map'
        self.home_pose.pose.position.x = 0.0
        self.home_pose.pose.position.y = 0.0
        self.home_pose.pose.orientation.w = 1.0

        # ArUco
        self.aruco_map_pose    = None   # Pose cube dalam /map frame (PoseStamped)
        self.aruco_camera_pose = None   # Pose cube raw dari kamera (PoseStamped)

        self.target_distance   = 0.8
        self.safe_front_dist   = 0.8
        self.corner_dist       = 0.35
        self.prev_error        = 0.0
        self.integral          = 0.0
        self.front_distances   = []
        self.right_distances   = []
        self.left_distances    = []

        self.stuck_counter     = 0
        self.last_positions    = []
        self.recovery_active   = False
        self.recovery_steps    = 0

        self.map_data = None

        # Spin search state
        self.spin_yaw_accumulated = 0.0
        self.spin_last_yaw        = None
        self.spin_visited_poses   = []  # Posisi yang sudah di-spin

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 1. Decompress right camera image
        self._decompress_proc = subprocess.Popen([
            'ros2', 'run', 'image_transport', 'republish',
            'compressed', 'raw',
            '--ros-args',
            '-r', 'in/compressed:=/oakd/right/image_raw/compressed',
            '-r', 'out:=/oakd/right/image_raw',
        ])
        self.get_logger().info("image decompress node diluncurkan ✓")

        # 2. Relay camera_info dari right ke prefix yang dipakai aruco
        self._relay_proc = subprocess.Popen([
            'ros2', 'run', 'topic_tools', 'relay',
            '/oakd/right/camera_info',
            '/oakd/right/camera_info_relay',
        ])
        self.get_logger().info("camera_info relay diluncurkan ✓")

        # 3. Launch aruco_markers node
        self._aruco_proc = subprocess.Popen([
            'ros2', 'run', 'aruco_markers', 'aruco_markers',
            '--ros-args',
            '-p', 'marker_size:=0.2',
            '-p', 'camera_frame:=oakd_right_camera_optical_frame',
            '-p', 'image_topic:=/oakd/right/image_raw',
            '-p', 'camera_info_topic:=/oakd/right/camera_info',
            '-p', 'dictionary:=DICT_4X4_50',
        ])
        self.get_logger().info("aruco_markers node diluncurkan otomatis ✓")

        self.undock_client = ActionClient(self, Undock,         '/undock')
        self.dock_client   = ActionClient(self, Dock,           '/dock')
        self.nav_client    = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # FIXED: subscribe ke /aruco/markers dengan tipe MarkerArray
        self.create_subscription(MarkerArray,                  '/aruco/markers', self.aruco_callback,  10)
        self.create_subscription(LaserScan,                    '/scan',          self.scan_callback,   10)
        self.create_subscription(PoseWithCovarianceStamped,    '/amcl_pose',     self.pose_callback,   10)
        self.create_subscription(OccupancyGrid,                '/map',           self.map_callback,    10)

        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("=== BrainNode siap! State: UNDOCK ===")

    # ----------------------------------------------------------
    # CALLBACKS
    # ----------------------------------------------------------
    def aruco_callback(self, msg: MarkerArray):
        """
        /aruco/markers → aruco_markers_msgs/MarkerArray
        Struktur: msg.markers[i].id, .pose (PoseStamped), .pixel_x, .pixel_y
        """
        if self.state not in (RobotState.EXPLORE, RobotState.SPIN_SEARCH):
            return
        if len(msg.markers) == 0:
            return

        # Ambil marker pertama (bisa filter by ID jika butuh marker spesifik)
        marker = msg.markers[0]

        self.get_logger().info(
            f"ArUco ID={marker.id} terdeteksi! "
            f"z={marker.pose.pose.position.z:.2f}m "
            f"pixel=({marker.pixel_x:.0f},{marker.pixel_y:.0f})"
        )

        # marker.pose sudah bertipe PoseStamped — langsung simpan
        self.aruco_camera_pose = marker.pose

        # Cancel active Nav2 goal so state machine can proceed
        if hasattr(self, '_nav_goal_handle') and self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self.action_in_progress = False

        # Stop robot, lanjut SAVE_ARUCO
        self.cmd_vel_pub.publish(Twist())
        self.state = RobotState.SAVE_ARUCO

    def scan_callback(self, msg):
        pass

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_positions.append((x, y))
        if len(self.last_positions) > 50:
            self.last_positions.pop(0)

    def map_callback(self, msg):
        self.map_data = msg

    # ----------------------------------------------------------
    # STATE MACHINE
    # ----------------------------------------------------------
    def state_machine_loop(self):
        if self.action_in_progress:
            return

        if self.state == RobotState.UNDOCK:
            self.get_logger().info("State: UNDOCK")
            self.send_undock_goal()

        elif self.state == RobotState.EXPLORE:
            self.run_autonomous_exploration()

        elif self.state == RobotState.SAVE_ARUCO:
            self.save_aruco_in_map_frame()

        elif self.state == RobotState.RETURN_HOME:
            self.get_logger().info("State: RETURN_HOME", throttle_duration_sec=2.0)
            self.send_nav_goal(self.home_pose, RobotState.APPROACH_CUBE)

        elif self.state == RobotState.APPROACH_CUBE:
            if self.aruco_map_pose is None:
                self.get_logger().error("Pose ArUco tidak tersedia! Kembali EXPLORE.")
                self.state = RobotState.EXPLORE
                return
            self.get_logger().info("State: APPROACH_CUBE - Menuju ArUco cube...")
            target = self.compute_approach_pose(self.aruco_map_pose, offset=0.10)
            self.send_nav_goal(target, RobotState.STOP_AT_CUBE)

        elif self.state == RobotState.STOP_AT_CUBE:
            self.get_logger().info("State: STOP_AT_CUBE - Berhenti di depan cube!")
            self.cmd_vel_pub.publish(Twist())
            self.state = RobotState.DONE

        elif self.state == RobotState.SPIN_SEARCH:
            self.run_spin_search()

        elif self.state == RobotState.DONE:
            self.get_logger().info("=== SELESAI! ===")
            self.timer.cancel()

    # ----------------------------------------------------------
    # AUTONOMOUS EXPLORATION (Frontier)
    # ----------------------------------------------------------
    def run_autonomous_exploration(self):
        if self.map_data is None:
            self.get_logger().info("Menunggu data peta...", throttle_duration_sec=2.0)
            return

        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            self.last_positions.append((robot_x, robot_y))
            if len(self.last_positions) > 50:
                self.last_positions.pop(0)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("Menunggu TF...", throttle_duration_sec=2.0)
            return

        width  = self.map_data.info.width
        height = self.map_data.info.height
        res    = self.map_data.info.resolution
        orig_x = self.map_data.info.origin.position.x
        orig_y = self.map_data.info.origin.position.y
        data   = self.map_data.data

        # step lebih kecil = lebih banyak frontier ditemukan
        step = max(1, int(0.3 / res))

        # Kumpulkan semua frontier dulu, pilih yang terdekat
        best_frontier = None
        best_dist = float('inf')

        for y in range(1, height - 1, step):
            for x in range(1, width - 1, step):
                idx = y * width + x
                if data[idx] == 0:
                    neighbors = [
                        data[(y-1)*width + x], data[(y+1)*width + x],
                        data[y*width + (x-1)], data[y*width + (x+1)]
                    ]
                    if -1 in neighbors:
                        target_x = orig_x + (x * res)
                        target_y = orig_y + (y * res)
                        dist = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
                        if dist < 0.3:
                            continue
                        # blacklist radius diperbesar ke 0.5m
                        is_blacklisted = any(
                            math.sqrt((target_x - bx)**2 + (target_y - by)**2) < 0.5
                            for bx, by in self.blacklisted_frontiers
                        )
                        if not is_blacklisted and dist < best_dist:
                            best_dist = dist
                            best_frontier = (target_x, target_y)

        if best_frontier is not None:
            target_x, target_y = best_frontier
            frontier_pose = PoseStamped()
            frontier_pose.header.frame_id = 'map'
            frontier_pose.pose.position.x = target_x
            frontier_pose.pose.position.y = target_y
            frontier_pose.pose.orientation.w = 1.0
            self._last_sent_goal = best_frontier
            self.get_logger().info(f"Frontier terpilih: {best_dist:.2f}m")
            self.send_nav_goal(frontier_pose, RobotState.EXPLORE)
            return

        # Kalau frontier habis → spin search di semua posisi yang pernah dikunjungi
        if len(self.blacklisted_frontiers) > 0:
            self.get_logger().info("Semua frontier habis, reset blacklist & mulai SPIN_SEARCH!")
            self.blacklisted_frontiers.clear()
            self.spin_yaw_accumulated = 0.0
            self.spin_last_yaw = None
            self.state = RobotState.SPIN_SEARCH
        else:
            self.get_logger().info("Mulai SPIN_SEARCH!", throttle_duration_sec=2.0)
            self.spin_yaw_accumulated = 0.0
            self.spin_last_yaw = None
            self.state = RobotState.SPIN_SEARCH

    # ----------------------------------------------------------
    # SPIN SEARCH - spin 360° di posisi sekarang cari ArUco
    # ----------------------------------------------------------
    def run_spin_search(self):
        # Spin perlahan 360°, aruco_callback akan trigger kalau ketemu
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return

        import math as _math

        # Hitung current yaw dari quaternion
        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = _math.atan2(siny, cosy)

        if self.spin_last_yaw is None:
            self.spin_last_yaw = current_yaw
            self.spin_yaw_accumulated = 0.0
            self.get_logger().info("SPIN_SEARCH: mulai spin 360°...")

        # Hitung delta yaw
        delta = current_yaw - self.spin_last_yaw
        if delta > _math.pi:
            delta -= 2 * _math.pi
        elif delta < -_math.pi:
            delta += 2 * _math.pi

        self.spin_yaw_accumulated += abs(delta)
        self.spin_last_yaw = current_yaw

        if self.spin_yaw_accumulated < 2 * _math.pi:
            # Masih spinning
            twist = Twist()
            twist.angular.z = 0.3  # spin pelan
            self.cmd_vel_pub.publish(twist)
        else:
            # Selesai 360° tapi ArUco belum ketemu
            self.cmd_vel_pub.publish(Twist())  # stop
            self.get_logger().info("SPIN_SEARCH: 360° selesai, ArUco belum ketemu. Reset explore.")
            # Catat posisi ini sudah di-spin
            self.spin_visited_poses.append((
                transform.transform.translation.x,
                transform.transform.translation.y
            ))
            # Reset dan coba explore lagi dari awal
            self.blacklisted_frontiers.clear()
            self.spin_yaw_accumulated = 0.0
            self.spin_last_yaw = None
            self.state = RobotState.EXPLORE

    # ----------------------------------------------------------
    # SAVE ARUCO IN MAP FRAME
    # ----------------------------------------------------------
    def save_aruco_in_map_frame(self):
        if self.aruco_camera_pose is None:
            self.get_logger().error("Tidak ada data ArUco!")
            self.state = RobotState.EXPLORE
            return

        try:
            # Lookup transform: map <- camera_optical_frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.aruco_camera_pose.header.frame_id,  # oakd_right_camera_optical_frame
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )

            # FIXED: do_transform_pose menerima Pose (bukan PoseStamped)
            pose_in_map = tf2_geometry_msgs.do_transform_pose(
                self.aruco_camera_pose.pose,  # geometry_msgs/Pose
                transform
            )

            self.aruco_map_pose = PoseStamped()
            self.aruco_map_pose.header.frame_id = 'map'
            self.aruco_map_pose.header.stamp    = self.get_clock().now().to_msg()
            self.aruco_map_pose.pose            = pose_in_map

            self.get_logger().info(
                f"ArUco pose disimpan di /map: "
                f"x={pose_in_map.position.x:.2f}, y={pose_in_map.position.y:.2f}"
            )
            self.state = RobotState.RETURN_HOME

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF2 gagal: {e}. Retry...")

    # ----------------------------------------------------------
    # COMPUTE APPROACH POSE
    # ----------------------------------------------------------
    def compute_approach_pose(self, cube_pose: PoseStamped, offset: float) -> PoseStamped:
        cx = cube_pose.pose.position.x
        cy = cube_pose.pose.position.y
        hx = self.home_pose.pose.position.x
        hy = self.home_pose.pose.position.y

        dx   = cx - hx
        dy   = cy - hy
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.01:
            return cube_pose

        nx = dx / dist
        ny = dy / dist

        approach = PoseStamped()
        approach.header.frame_id = 'map'
        approach.header.stamp    = self.get_clock().now().to_msg()
        approach.pose.position.x = cx - nx * offset
        approach.pose.position.y = cy - ny * offset
        yaw = math.atan2(dy, dx)
        approach.pose.orientation.z = math.sin(yaw / 2.0)
        approach.pose.orientation.w = math.cos(yaw / 2.0)
        return approach

    # ----------------------------------------------------------
    # UNDOCK
    # ----------------------------------------------------------
    def send_undock_goal(self):
        self.action_in_progress = True
        self.undock_client.wait_for_server()
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Undock ditolak. Lanjut EXPLORE.")
            self.action_in_progress = False
            self.state = RobotState.EXPLORE
            return
        goal_handle.get_result_async().add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        self.get_logger().info("Undock selesai. Mulai EXPLORE.")
        self.action_in_progress = False
        self.state = RobotState.EXPLORE

    # ----------------------------------------------------------
    # NAVIGASI (Nav2)
    # ----------------------------------------------------------
    def send_nav_goal(self, target_pose: PoseStamped, next_state: RobotState):
        self.action_in_progress = True
        target_pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_client.wait_for_server()
        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        self._nav_next_state = next_state
        self._nav_goal_handle = None
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Nav2 sisa: {dist:.2f}m", throttle_duration_sec=2.0)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal ditolak!")
            if self._last_sent_goal:
                self.blacklisted_frontiers.append(self._last_sent_goal)
            self.action_in_progress = False
            return
        self._nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Nav2 SUKSES → {self._nav_next_state}")
            self.state = self._nav_next_state
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Nav2 goal di-cancel (ArUco detected).")
            # state sudah di-set oleh aruco_callback, jangan override
        elif status == GoalStatus.STATUS_ABORTED:
            # STATUS_ABORTED (6) sering terjadi saat robot sudah sangat dekat
            # tapi Nav2 tidak bisa reach exact pose karena obstacle/tolerance.
            # Treat as success untuk RETURN_HOME dan APPROACH_CUBE.
            if self._nav_next_state in (RobotState.APPROACH_CUBE, RobotState.STOP_AT_CUBE):
                self.get_logger().info(f"Nav2 ABORTED tapi close enough → lanjut {self._nav_next_state}")
                self.state = self._nav_next_state
            else:
                self.get_logger().warn(f"Nav2 ABORTED (EXPLORE), blacklist & retry")
                if self._last_sent_goal:
                    self.blacklisted_frontiers.append(self._last_sent_goal)
        else:
            self.get_logger().warn(f"Nav2 GAGAL (status={status}), retry")
        self.action_in_progress = False


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node dihentikan.")
    finally:
        node.cmd_vel_pub.publish(Twist())
        # Matikan semua subprocess
        for proc_name in ['_aruco_proc', '_decompress_proc', '_relay_proc']:
            proc = getattr(node, proc_name, None)
            if proc and proc.poll() is None:
                proc.terminate()
        node.get_logger().info("Semua subprocess dihentikan.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
