import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import paramiko # Tambahkan ini di sini

import customtkinter as ctk
import threading
import time
import tkinter as tk

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class RobotActionClient(Node):
    def __init__(self, ui_app):
        super().__init__('gui_action_client')
        self.ui = ui_app
        
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher untuk servo gix
        self.servo_pos = 0.0 
        self.servo_pub = self.create_publisher(JointTrajectory, '/gix_controller/joint_trajectory', 10)
        
        self.goal_handle = None

    def set_servo_position(self, pos):
        """Mengirim perintah posisi ke servo gix"""
        msg = JointTrajectory()
        msg.joint_names = ['gix']
        
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        msg.points.append(point)
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Setting servo gix to: {pos}')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ui.after(0, self.ui.update_robot_marker, x, y)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def send_waypoint(self, x, y):
        if not self.action_client.server_is_ready():
            self.ui.after(0, self.ui.show_status, "Movement Stopped: Planner Offline", "#e74c3c")
            return
        
        # Servo turun saat mulai jalan
        self.set_servo_position(0.0)
        self.servo_pos = 0.0
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.ui.after(0, self.ui.update_distance, dist)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.ui.after(0, self.ui.show_status, "Target Rejected", "#e74c3c")
            return
        
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.ui.after(0, self.ui.show_status, "Delivery Reached", "#2ecc71")
            # Servo naik saat sampai
            self.set_servo_position(1.0)
            self.servo_pos = 1.0
        else:
            self.ui.after(0, self.ui.show_status, "Movement Stopped", "#e74c3c")

    def cancel_movement(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.publish_twist(0.0, 0.0)
        self.set_servo_position(0.0)
        self.servo_pos = 0.0


class RobotDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Robot Operator Hub")
        self.geometry("1000x650")

        rclpy.init(args=None)
        self.ros_node = RobotActionClient(self)
        
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

        self.start_time = 0
        self.timer_running = False

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Sidebar
        self.sidebar = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")  

        self.logo_label = ctk.CTkLabel(self.sidebar, text="ROBOT CONTROL", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.pack(pady=(20, 10))

        # Tombol Toggle Servo
        self.servo_btn = ctk.CTkButton(self.sidebar, text="LIFT SERVO", 
                                fg_color="#9b59b6", hover_color="#8e44ad",
                                command=self.toggle_servo)
        self.servo_btn.pack(pady=10, padx=20, fill="x")

        # Auto Dispatch Section
        self.create_waypoint_button("Home / Kitchen", 0.0, 0.0)
        self.create_waypoint_button("Dispatch to Table 1", 1.0, 1.0)
        self.create_waypoint_button("Dispatch to Table 2", 2.5, -1.0)

        self.cancel_btn = ctk.CTkButton(self.sidebar, text="CANCEL AUTO", 
                                        fg_color="#e74c3c", hover_color="#c0392b",
                                        command=self.cancel_movement)
        self.cancel_btn.pack(pady=10, padx=20, fill="x")

        self.hw_btn = ctk.CTkButton(self.sidebar, text="START HARDWARE", 
                                     fg_color="#27ae60", hover_color="#219150",
                                     command=self.start_robot_hardware) # Menghubungkan ke def tadi
        self.hw_btn.pack(pady=10, padx=20, fill="x")

        # Teleop Section (Sumbu kontrol manual)
        self.teleop_label = ctk.CTkLabel(self.sidebar, text="MANUAL TELEOP", font=ctk.CTkFont(size=16, weight="bold"))
        self.teleop_label.pack(pady=(20, 5))
        # ... (Slider speed dan turn tetap sama)
        self.linear_slider = ctk.CTkSlider(self.sidebar, from_=0.1, to=1.0, number_of_steps=9)
        self.linear_slider.set(0.5)
        self.linear_slider.pack(padx=20, pady=5)
        self.angular_slider = ctk.CTkSlider(self.sidebar, from_=0.1, to=2.0, number_of_steps=19)
        self.angular_slider.set(1.0)
        self.angular_slider.pack(padx=20, pady=5)

        # D-pad
        self.dpad = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        self.dpad.pack(pady=10)
        self.btn_w = ctk.CTkButton(self.dpad, text="W", width=40, command=self.move_forward)
        self.btn_w.grid(row=0, column=1, pady=2, padx=2)
        self.btn_a = ctk.CTkButton(self.dpad, text="A", width=40, command=self.turn_left)
        self.btn_a.grid(row=1, column=0, pady=2, padx=2)
        self.btn_s = ctk.CTkButton(self.dpad, text="S", width=40, command=self.move_backward)
        self.btn_s.grid(row=1, column=1, pady=2, padx=2)
        self.btn_d = ctk.CTkButton(self.dpad, text="D", width=40, command=self.turn_right)
        self.btn_d.grid(row=1, column=2, pady=2, padx=2)
        self.btn_stop = ctk.CTkButton(self.dpad, text="STOP", width=120, fg_color="#e67e22", hover_color="#d35400", command=self.stop_robot)
        self.btn_stop.grid(row=2, column=0, columnspan=3, pady=5)

        self.bind('<KeyPress-w>', lambda e: self.move_forward())
        self.bind('<KeyPress-s>', lambda e: self.move_backward())
        self.bind('<KeyPress-a>', lambda e: self.turn_left())
        self.bind('<KeyPress-d>', lambda e: self.turn_right())
        self.bind('<KeyPress-space>', lambda e: self.stop_robot())

        # Main Control UI
        self.main_frame = ctk.CTkFrame(self, corner_radius=15)
        self.main_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        self.status_label = ctk.CTkLabel(self.main_frame, text="System Ready", text_color="#2ecc71", font=("Arial", 24, "bold"))
        self.status_label.pack(pady=10)

        self.info_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        self.info_frame.pack(pady=10, fill="x", padx=20)
        self.dist_label = ctk.CTkLabel(self.info_frame, text="Dist: --", font=("Arial", 16))
        self.dist_label.pack(side="left", fill="x", expand=True)
        self.time_label = ctk.CTkLabel(self.info_frame, text="Time: --", font=("Arial", 16))
        self.time_label.pack(side="right", fill="x", expand=True)

        self.map_frame = ctk.CTkFrame(self.main_frame)
        self.map_frame.pack(pady=20, expand=True, fill="both")
        self.canvas = tk.Canvas(self.map_frame, bg="#2b2b2b", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=5, pady=5)
        self.canvas.bind("<Configure>", self.draw_map_grid)
        
        self.robot_dot = None
        self.goal_dot = None
        self.robot_pos = (0.0, 0.0)
        self.pixels_per_meter = 40

    def toggle_servo(self):
        # Logika toggle dipindah ke kelas Dashboard
        if self.ros_node.servo_pos == 1.0:
            target_pos = 2.0
            label_text = "LOWER SERVO"
        else:
            target_pos = 1.0
            label_text = "LIFT SERVO"
            
        self.ros_node.servo_pos = target_pos
        self.ros_node.set_servo_position(target_pos)
        self.servo_btn.configure(text=label_text)
        self.show_status(f"Servo: Set to {target_pos}", "#3498db")

    def start_robot_hardware(self):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect('10.155.234.163', username='ubuntu', password='robot1234', timeout=5)

            # Cek apakah hardware sudah jalan
            stdin, stdout, stderr = ssh.exec_command("pgrep -f hardware.launch.py")
            if stdout.read().decode().strip():
                self.show_status("Hardware already running!", "#f1c40f")
                ssh.close()
                return

            # Perbaikan: Tambahkan export LDS_MODEL (sesuaikan modelnya: LDS-01 atau LDS-02)
            command = (
                "bash -c 'source /opt/ros/humble/setup.bash && "
                "source ~/turtlebot3_ws/install/setup.bash && "
                "export LDS_MODEL=LDS-01 && "
                "ros2 launch turtlebot3_gix_bringup hardware.launch.py'"
            )
            
            self.show_status("Hardware starting...", "#2ecc71")
            
            # Memanggil self.stream_logs yang sudah ada di kelas ini
            thread = threading.Thread(target=self.stream_logs, args=(ssh, command), daemon=True)
            thread.start()
            
        except Exception as e:
            self.show_status("Check connection!", "#e74c3c")
            print(f"Error: {e}")

    def stream_logs(self, ssh, command):
        """Menampilkan log dari Raspberry Pi ke terminal laptop G14"""
        try:
            stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
            for line in iter(stdout.readline, ""):
                # Cetak ke terminal tempat kamu menjalankan dashboard.py
                print(f"[PI LOG]: {line}", end="")
            ssh.close()
        except Exception as e:
            print(f"SSH Log Stream Error: {e}")   



    def move_forward(self):
        self.ros_node.publish_twist(self.linear_slider.get(), 0.0)
        self.show_status("Manual: Moving Forward", "#3498db")

    def move_backward(self):
        self.ros_node.publish_twist(-self.linear_slider.get(), 0.0)
        self.show_status("Manual: Moving Backward", "#3498db")

    def turn_left(self):
        self.ros_node.publish_twist(0.0, self.angular_slider.get())
        self.show_status("Manual: Turning Left", "#3498db")

    def turn_right(self):
        self.ros_node.publish_twist(0.0, -self.angular_slider.get())
        self.show_status("Manual: Turning Right", "#3498db")

    def stop_robot(self):
        self.ros_node.publish_twist(0.0, 0.0)
        self.show_status("Manual: Stopped", "#e67e22")

    def draw_map_grid(self, event=None):
        self.canvas.delete("all")
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        cx, cy = w/2, h/2
        for i in range(-10, 11):
            offset = i * self.pixels_per_meter
            self.canvas.create_line(cx + offset, 0, cx + offset, h, fill="#404040")
            self.canvas.create_line(0, cy + offset, w, cy + offset, fill="#404040")
        self.canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="white")
        self.update_robot_marker(self.robot_pos[0], self.robot_pos[1])

    def update_robot_marker(self, x, y):
        self.robot_pos = (x, y)
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        cx, cy = w/2, h/2
        screen_x = cx + (x * self.pixels_per_meter)
        screen_y = cy - (y * self.pixels_per_meter)
        if self.robot_dot: self.canvas.delete(self.robot_dot)
        self.robot_dot = self.canvas.create_oval(screen_x-8, screen_y-8, screen_x+8, screen_y+8, fill="#3498db", outline="white")

    def create_waypoint_button(self, name, x, y):
        btn = ctk.CTkButton(self.sidebar, text=name, command=lambda: self.send_robot(x, y, name))
        btn.pack(pady=5, padx=20, fill="x")

    def send_robot(self, x, y, location_name):
        self.ros_node.cancel_movement()
        self.show_status(f"Auto: Moving to {location_name}", "#f1c40f")
        self.start_time, self.timer_running = time.time(), True
        self.update_timer()
        self.ros_node.send_waypoint(x, y)

    def update_distance(self, dist):
        self.dist_label.configure(text=f"Dist: {dist:.2f} m")

    def show_status(self, text, color):
        self.status_label.configure(text=text, text_color=color)
        if any(word in text for word in ["Reached", "Stopped", "Cancelled"]): self.timer_running = False

    def update_timer(self):
        if self.timer_running:
            elapsed = time.time() - self.start_time
            self.time_label.configure(text=f"Time: {elapsed:.1f} s")
            self.after(100, self.update_timer)

    def cancel_movement(self):
        self.ros_node.cancel_movement()
        self.timer_running = False
        self.show_status("Auto: Cancelled", "#e74c3c")

    def on_closing(self):
        # Memperbaiki error shutdown dengan pengecekan rclpy.ok()
        if rclpy.ok():
            self.ros_node.destroy_node()
            rclpy.shutdown()
        self.destroy()

if __name__ == "__main__":
    app = RobotDashboard()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()