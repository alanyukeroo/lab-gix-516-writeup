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
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration


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


# ============================================================
# BRAIN NODE
# ============================================================
class BrainNode(Node):

    # ----------------------------------------------------------
    # INIT
    # ----------------------------------------------------------
    def __init__(self):
        super().__init__('robot_brain')
        self.state = RobotState.UNDOCK
        self.action_in_progress = False

        # - Pose awal robot (di atas dock) -
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = 'map'
        self.home_pose.pose.position.x = 0.0
        self.home_pose.pose.position.y = 0.0
        self.home_pose.pose.orientation.w = 1.0

        # - ArUco -
        self.aruco_map_pose = None          # Pose cube dalam /map frame
        self.aruco_camera_pose = None       # Pose cube raw dari kamera

        # - Parameter asli tetap dipertahankan -
        self.target_distance   = 0.8        
        self.safe_front_dist   = 0.8      
        self.corner_dist       = 0.35       
        self.prev_error        = 0.0
        self.integral          = 0.0
        self.front_distances   = []
        self.right_distances   = []
        self.left_distances    = []

        # - Recovery -
        self.stuck_counter     = 0
        self.last_positions    = []
        self.recovery_active   = False
        self.recovery_steps    = 0

        # - Map Data untuk Eksplorasi -
        self.map_data = None

        # - TF2 -
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # - Action Clients -
        self.undock_client   = ActionClient(self, Undock,         '/undock')
        self.dock_client     = ActionClient(self, Dock,           '/dock')
        self.nav_client      = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # - Publishers -
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # - Subscribers -
        self.create_subscription(PoseStamped,              '/aruco_pose',  self.aruco_callback,    10)
        self.create_subscription(LaserScan,                '/scan',        self.scan_callback,     10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',   self.pose_callback,     10)
        self.create_subscription(OccupancyGrid,            '/map',         self.map_callback,      10)

        # - Timer utama -
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("=== BrainNode siap! State: UNDOCK ===")

    # ----------------------------------------------------------
    # CALLBACKS
    # ----------------------------------------------------------
    def aruco_callback(self, msg):
        if self.state != RobotState.EXPLORE:
            return

        self.aruco_camera_pose = msg
        self.get_logger().info("ArUco cube terdeteksi! Menyimpan pose...")
        self.cmd_vel_pub.publish(Twist())
        self.state = RobotState.SAVE_ARUCO

    def scan_callback(self, msg):
        pass # Dipertahankan agar format parameter tidak berubah

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_positions.append((x, y))
        if len(self.last_positions) > 50:
            self.last_positions.pop(0)

    def map_callback(self, msg):
        """Menyimpan data peta terbaru untuk dianalisis oleh frontier."""
        self.map_data = msg

    # ----------------------------------------------------------
    # STATE MACHINE UTAMA
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
            self.get_logger().info("State: RETURN_HOME - Kembali ke dock...")
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
            self.get_logger().info("State: STOP_AT_CUBE - Robot berhenti di depan cube!")
            self.cmd_vel_pub.publish(Twist())
            self.state = RobotState.DONE

        elif self.state == RobotState.DONE:
            self.get_logger().info("=== SELESAI! Urutan lengkap berhasil. ===")
            self.timer.cancel()

    # ----------------------------------------------------------
    # AUTONOMOUS EXPLORATION (Frontier + A*)
    # ----------------------------------------------------------
    def run_autonomous_exploration(self):
        if self.map_data is None:
            self.get_logger().info("Menunggu data peta dari /map...", throttle_duration_sec=2.0)
            return

        # Ambil posisi robot dari TF (map -> base_link)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Catat posisi untuk mengecek apakah robot stuck
            self.last_positions.append((robot_x, robot_y))
            if len(self.last_positions) > 50:
                self.last_positions.pop(0)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("Menunggu sistem TF mempublikasikan posisi robot...", throttle_duration_sec=2.0)
            return

        width = self.map_data.info.width
        height = self.map_data.info.height
        res = self.map_data.info.resolution
        orig_x = self.map_data.info.origin.position.x
        orig_y = self.map_data.info.origin.position.y
        data = self.map_data.data

        step = max(1, int(0.5 / res)) 

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

                        # Abaikan target yang jaraknya di bawah 0.5 meter
                        if dist > 0.5:
                            frontier_pose = PoseStamped()
                            frontier_pose.header.frame_id = 'map'
                            frontier_pose.pose.position.x = target_x
                            frontier_pose.pose.position.y = target_y
                            frontier_pose.pose.orientation.w = 1.0
                            
                            self.get_logger().info(f"Target baru jarak: {dist:.2f}m, merencanakan rute A*...")
                            self.send_nav_goal(frontier_pose, RobotState.EXPLORE)
                            return

        self.get_logger().info("Tidak ada area baru yang ditemukan.", throttle_duration_sec=2.0)

    # ----------------------------------------------------------
    # SIMPAN POSE ARUCO DALAM FRAME /MAP (via TF2)
    # ----------------------------------------------------------
    def save_aruco_in_map_frame(self):
        if self.aruco_camera_pose is None:
            self.get_logger().error("Tidak ada data ArUco!")
            self.state = RobotState.EXPLORE
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.aruco_camera_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )

            pose_in_map = tf2_geometry_msgs.do_transform_pose(
                self.aruco_camera_pose.pose,
                transform
            )

            self.aruco_map_pose = PoseStamped()
            self.aruco_map_pose.header.frame_id = 'map'
            self.aruco_map_pose.header.stamp     = self.get_clock().now().to_msg()
            self.aruco_map_pose.pose             = pose_in_map

            self.get_logger().info(
                f"ArUco pose disimpan di map: "
                f"x={pose_in_map.position.x:.2f}, "
                f"y={pose_in_map.position.y:.2f}"
            )

            self.state = RobotState.RETURN_HOME

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF2 transform gagal: {e}. Coba lagi...")

    # ----------------------------------------------------------
    # HITUNG POSE PENDEKATAN (offset dari cube)
    # ----------------------------------------------------------
    def compute_approach_pose(self, cube_pose: PoseStamped, offset: float) -> PoseStamped:
        cx = cube_pose.pose.position.x
        cy = cube_pose.pose.position.y
        hx = self.home_pose.pose.position.x
        hy = self.home_pose.pose.position.y

        dx    = cx - hx
        dy    = cy - hy
        dist  = math.sqrt(dx**2 + dy**2)

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
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Undock ditolak. Lanjut EXPLORE.")
            self.action_in_progress = False
            self.state = RobotState.EXPLORE
            return
        self.get_logger().info("Undock diterima.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

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
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Nav2 sisa jarak: {dist:.2f}m", throttle_duration_sec=2.0)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal ditolak!")
            self.action_in_progress = False
            return
        self.get_logger().info("Nav2 goal diterima, menunggu hasil...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Nav2 SUKSES. Transisi ke: {self._nav_next_state}")
            self.state = self._nav_next_state
        else:
            self.get_logger().warn(f"Nav2 GAGAL (status={status}). Coba ulang...")

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
        node.get_logger().info("Node dihentikan oleh user.")
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()