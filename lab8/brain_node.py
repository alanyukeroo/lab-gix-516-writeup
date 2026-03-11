import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.action import Undock
from sensor_msgs.msg import LaserScan
from enum import Enum
import math

class RobotState(Enum):
    UNDOCK = 0
    EXPLORE = 1
    RETURN_HOME = 2
    FINAL_SPRINT = 3
    DONE = 4

class WallFollowState(Enum):
    SEARCH = 0
    FOLLOW = 1
    TURN = 2
    RECOVER = 3
    

class BrainNode(Node):
    def __init__(self):
        super().__init__('robot_brain')
        self.state = RobotState.UNDOCK
        self.action_in_progress = False

        # Variabel untuk Adaptive Wall Follower
        self.target_distance = 1
        self.safe_front_distance = 1
        self.prev_error = 0.0
        self.integral = 0.0
        self.front_distances = []
        self.right_distances = []
        
        # Action Client for undocking
        self.undock_client = ActionClient(self, Undock, '/undock')
        
        # Publisher for manual driving
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscription for the camera detection
        self.aruco_sub = self.create_subscription(
            PoseStamped, 
            '/aruco_pose', 
            self.aruco_callback, 
            10
        )

        # Subscription for the LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        self.target_cube_pose = None
        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def aruco_callback(self, msg):
        if self.state == RobotState.EXPLORE:
            self.target_cube_pose = msg
            self.get_logger().info("Cube spotted. Stopping exploration.")
            
            # Stop the robot immediately
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            
            self.state = RobotState.RETURN_HOME

    def state_machine_loop(self):
        if self.action_in_progress:
            return

        if self.state == RobotState.UNDOCK:
            self.get_logger().info("Sending undock command.")
            self.send_undock_goal()

        elif self.state == RobotState.EXPLORE:
            drive_msg = Twist()
            
            # 1. Prioritas utama, hindari tabrakan depan
            if self.front_distances and min(self.front_distances) < self.safe_front_distance:
                self.get_logger().info("Ada halangan di depan. Belok kiri tajam.")
                drive_msg.linear.x = 0.05
                drive_msg.angular.z = 0.5
                self.cmd_vel_pub.publish(drive_msg)
                return
                
            # Jika sensor kanan tidak melihat apa-apa, jalan lurus pelan
            if not self.right_distances:
                drive_msg.linear.x = 0.15
                drive_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(drive_msg)
                return
                
            actual_distance = min(self.right_distances)
            raw_error = self.target_distance - actual_distance
            error = max(min(raw_error, 0.8), -0.8)
            
        # 2. Logika PID Asimetris yang Benar
            if error > 0:
                # Terlalu dekat dengan dinding -> Belok kiri dengan agresif
                kp = 1.2
                kd = 0.2
            else:
                # Kehilangan dinding (tikungan luar) -> Belok kanan dengan sangat halus
                kp = 0.3  # Turunkan drastis dari 1.0 menjadi 0.3
                kd = 0.1
            
            self.integral += error
            self.integral = max(min(self.integral, 1.0), -1.0)
            
            derivative = error - self.prev_error
            steering = (kp * error) + (0.01 * self.integral) + (kd * derivative)
            self.prev_error = error
            
            # 3. Batasi putaran setir
            steering = max(min(steering, 1.5), -0.6)
            
            # 4. Kecepatan adaptif, melambat saat belok tajam
            forward_speed = 0.2 / (1.0 + 2.0 * abs(steering))
            forward_speed = max(min(forward_speed, 0.2), 0.15)
            
            drive_msg.linear.x = forward_speed
            drive_msg.angular.z = steering
            self.cmd_vel_pub.publish(drive_msg)

        elif self.state == RobotState.RETURN_HOME:
            self.get_logger().info("Preparing to drive back to the start dock.")
            self.state = RobotState.FINAL_SPRINT

        elif self.state == RobotState.FINAL_SPRINT:
            self.get_logger().info("Ready for the final sprint.")
            self.state = RobotState.DONE

        elif self.state == RobotState.DONE:
            self.get_logger().info("Sequence complete.")
            self.timer.cancel()

    def send_undock_goal(self):
        self.action_in_progress = True
        self.undock_client.wait_for_server()
        goal_msg = Undock.Goal()
        
        self.send_goal_future = self.undock_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Undock rejected. The robot is likely already free. Moving to explore.")
            self.action_in_progress = False
            self.state = RobotState.EXPLORE
            return

        self.get_logger().info("Undock accepted.")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        self.get_logger().info("Undock complete.")
        self.action_in_progress = False
        self.state = RobotState.EXPLORE

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        right_dists = []
        front_dists = []
        
        for i, range_val in enumerate(msg.ranges):
            # Abaikan data error
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
                
            angle = angle_min + i * angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            # Area Kanan
            if -2.09 < angle < -1.04:
                right_dists.append(range_val)
                
            # Area Depan
            elif -0.99 < angle < 0.30:
                front_dists.append(range_val)
                
        # Simpan ke variabel global agar bisa dibaca oleh state machine
        self.front_distances = front_dists
        self.right_distances = right_dists

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#### MASIH NABRAK CORNER, PERLU DI UPDATE! 

