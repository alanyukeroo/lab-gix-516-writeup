import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import paramiko

import customtkinter as ctk
import threading
import time
import tkinter as tk
import json
import os

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

WAYPOINTS_FILE = "waypoints.json"
SSH_HOST = '10.155.234.163'
SSH_USER = 'ubuntu'
SSH_PASS = 'robot1234'


# ─────────────────────────────────────────────
#  ROS NODE
# ─────────────────────────────────────────────
class RobotActionClient(Node):
    def __init__(self, ui_app):
        super().__init__('gui_action_client')
        self.ui = ui_app

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.servo_pos = 0.0
        self.servo_pub = self.create_publisher(
            JointTrajectory, '/gix_controller/joint_trajectory', 10)

        self.goal_handle = None
        self.current_x = 0.0
        self.current_y = 0.0

        # Subscribe person detection dari person_obstacle_node
        self.person_sub = self.create_subscription(
            Bool, '/person_detected', self.person_detected_callback, 10)
        self.person_detected = False

    def set_servo_position(self, pos):
        msg = JointTrajectory()
        msg.joint_names = ['gix']
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Setting servo gix to: {pos}')

    def person_detected_callback(self, msg: Bool):
        self.person_detected = msg.data
        self.ui.after(0, self.ui.update_person_status, msg.data)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.ui.after(0, self.ui.update_robot_marker, self.current_x, self.current_y)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def send_waypoint(self, x, y):
        if not self.action_client.server_is_ready():
            self.ui.after(0, self.ui.show_status, "Movement Stopped: Planner Offline", "#e74c3c")
            return
        self.set_servo_position(0.0)
        self.servo_pos = 0.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.ui.after(0, self.ui.update_distance, dist)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.ui.after(0, self.ui.show_status, "Target Rejected", "#e74c3c")
            return
        self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.ui.after(0, self.ui.show_status, "Delivery Reached ✓", "#2ecc71")
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


# ─────────────────────────────────────────────
#  DIALOG: Label Waypoint Baru
# ─────────────────────────────────────────────
class WaypointDialog(ctk.CTkToplevel):
    def __init__(self, parent, x, y):
        super().__init__(parent)
        self.title("Tambah Waypoint")
        self.geometry("320x180")
        self.resizable(False, False)

        self.result_name = None
        self.x = x
        self.y = y

        ctk.CTkLabel(self, text=f"Koordinat: ({x:.3f}, {y:.3f})",
                     font=ctk.CTkFont(size=13)).pack(pady=(20, 5))
        ctk.CTkLabel(self, text="Nama Lokasi:").pack()
        self.entry = ctk.CTkEntry(self, placeholder_text="e.g. Kitchen / Table 1")
        self.entry.pack(padx=30, fill="x", pady=8)

        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.pack(fill="x", padx=30)
        ctk.CTkButton(btn_frame, text="Simpan", command=self._save).pack(side="left", expand=True, padx=4)
        ctk.CTkButton(btn_frame, text="Batal", fg_color="#555",
                      command=self.destroy).pack(side="right", expand=True, padx=4)

        self.entry.bind("<Return>", lambda e: self._save())

        # FIX: tunggu window visible dulu baru grab_set & focus
        self.after(150, self._setup_grab)

    def _setup_grab(self):
        try:
            self.grab_set()
            self.entry.focus()
        except Exception:
            pass

    def _save(self):
        name = self.entry.get().strip()
        if name:
            self.result_name = name
            self.destroy()


# ─────────────────────────────────────────────
#  MAIN DASHBOARD
# ─────────────────────────────────────────────
class RobotDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Robot Operator Hub")
        self.geometry("1100x700")

        rclpy.init(args=None)
        self.ros_node = RobotActionClient(self)
        self.ros_thread = threading.Thread(
            target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

        self.start_time = 0
        self.timer_running = False
        self.mapping_active = False
        self._slam_ssh = None

        # Waypoints: list of {"name": str, "x": float, "y": float}
        self.waypoints = []
        self._load_waypoints()

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self._build_sidebar()
        self._build_main()

        self.robot_pos = (0.0, 0.0)
        self.pixels_per_meter = 40
        self.robot_dot = None
        self.waypoint_dots = {}

    # ── SIDEBAR ──────────────────────────────
    def _build_sidebar(self):
        self.sidebar = ctk.CTkFrame(self, width=260, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_propagate(False)

        ctk.CTkLabel(self.sidebar, text="ROBOT CONTROL",
                     font=ctk.CTkFont(size=18, weight="bold")).pack(pady=(18, 4))

        # ── SLAM ────────────────────────────
        ctk.CTkLabel(self.sidebar, text="─── MAPPING ───",
                     font=ctk.CTkFont(size=11), text_color="#888").pack(pady=(8, 2))

        self.slam_btn = ctk.CTkButton(
            self.sidebar, text="▶  START MAPPING",
            fg_color="#1a6b3c", hover_color="#145730",
            command=self.toggle_slam)
        self.slam_btn.pack(pady=4, padx=16, fill="x")

        self.save_map_btn = ctk.CTkButton(
            self.sidebar, text="💾  SAVE MAP",
            fg_color="#2471a3", hover_color="#1a5276",
            command=self.save_map)
        self.save_map_btn.pack(pady=4, padx=16, fill="x")

        # ── WAYPOINTS ───────────────────────
        ctk.CTkLabel(self.sidebar, text="─── WAYPOINTS ───",
                     font=ctk.CTkFont(size=11), text_color="#888").pack(pady=(10, 2))

        # Tombol simpan posisi robot sekarang
        self.save_pos_btn = ctk.CTkButton(
            self.sidebar, text="📍  SAVE CURRENT POSITION",
            fg_color="#d35400", hover_color="#b94600",
            command=self.save_current_position)
        self.save_pos_btn.pack(pady=4, padx=16, fill="x")

        self.waypoint_frame = ctk.CTkScrollableFrame(self.sidebar, height=150)
        self.waypoint_frame.pack(padx=10, pady=4, fill="x")
        self._refresh_waypoint_buttons()

        ctk.CTkButton(self.sidebar, text="📁  LOAD WAYPOINTS",
                      fg_color="#555", hover_color="#444",
                      command=self._load_waypoints_and_refresh).pack(pady=4, padx=16, fill="x")

        ctk.CTkButton(self.sidebar, text="💾  SAVE WAYPOINTS",
                      fg_color="#555", hover_color="#444",
                      command=self._save_waypoints).pack(pady=2, padx=16, fill="x")

        # ── AUTO ────────────────────────────
        ctk.CTkLabel(self.sidebar, text="─── AUTO ───",
                     font=ctk.CTkFont(size=11), text_color="#888").pack(pady=(10, 2))

        ctk.CTkButton(self.sidebar, text="🏠  HOME / KITCHEN",
                      command=lambda: self._send_by_name("Kitchen")).pack(pady=3, padx=16, fill="x")

        self.cancel_btn = ctk.CTkButton(
            self.sidebar, text="✖  CANCEL AUTO",
            fg_color="#e74c3c", hover_color="#c0392b",
            command=self.cancel_movement)
        self.cancel_btn.pack(pady=4, padx=16, fill="x")

        # ── SERVO ───────────────────────────
        self.servo_btn = ctk.CTkButton(
            self.sidebar, text="⬆  LIFT SERVO",
            fg_color="#9b59b6", hover_color="#8e44ad",
            command=self.toggle_servo)
        self.servo_btn.pack(pady=4, padx=16, fill="x")

        # ── HARDWARE ────────────────────────
        self.hw_btn = ctk.CTkButton(
            self.sidebar, text="⚙  START HARDWARE",
            fg_color="#27ae60", hover_color="#219150",
            command=self.start_robot_hardware)
        self.hw_btn.pack(pady=4, padx=16, fill="x")

        # ── TELEOP ──────────────────────────
        ctk.CTkLabel(self.sidebar, text="─── TELEOP ───",
                     font=ctk.CTkFont(size=11), text_color="#888").pack(pady=(10, 2))

        self.linear_slider = ctk.CTkSlider(self.sidebar, from_=0.1, to=1.0, number_of_steps=9)
        self.linear_slider.set(0.5)
        self.linear_slider.pack(padx=16, pady=3)

        self.angular_slider = ctk.CTkSlider(self.sidebar, from_=0.1, to=2.0, number_of_steps=19)
        self.angular_slider.set(1.0)
        self.angular_slider.pack(padx=16, pady=3)

        dpad = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        dpad.pack(pady=6)
        ctk.CTkButton(dpad, text="W", width=42, command=self.move_forward).grid(row=0, column=1, pady=2, padx=2)
        ctk.CTkButton(dpad, text="A", width=42, command=self.turn_left).grid(row=1, column=0, pady=2, padx=2)
        ctk.CTkButton(dpad, text="S", width=42, command=self.move_backward).grid(row=1, column=1, pady=2, padx=2)
        ctk.CTkButton(dpad, text="D", width=42, command=self.turn_right).grid(row=1, column=2, pady=2, padx=2)
        ctk.CTkButton(dpad, text="STOP", width=130,
                      fg_color="#e67e22", hover_color="#d35400",
                      command=self.stop_robot).grid(row=2, column=0, columnspan=3, pady=5)

        self.bind('<KeyPress-w>', lambda e: self.move_forward())
        self.bind('<KeyPress-s>', lambda e: self.move_backward())
        self.bind('<KeyPress-a>', lambda e: self.turn_left())
        self.bind('<KeyPress-d>', lambda e: self.turn_right())
        self.bind('<KeyPress-space>', lambda e: self.stop_robot())

    # ── MAIN PANEL ───────────────────────────
    def _build_main(self):
        self.main_frame = ctk.CTkFrame(self, corner_radius=15)
        self.main_frame.grid(row=0, column=1, padx=16, pady=16, sticky="nsew")

        self.status_label = ctk.CTkLabel(
            self.main_frame, text="System Ready",
            text_color="#2ecc71", font=("Arial", 22, "bold"))
        self.status_label.pack(pady=(12, 4))

        self.hint_label = ctk.CTkLabel(
            self.main_frame,
            text="💡 Gerakin robot ke lokasi → klik '📍 SAVE CURRENT POSITION'",
            font=ctk.CTkFont(size=11), text_color="#888")
        self.hint_label.pack()

        # Warning orang terdeteksi
        self.person_warning = ctk.CTkLabel(
            self.main_frame,
            text="",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color="#e74c3c")
        self.person_warning.pack(pady=2)

        info = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        info.pack(pady=6, fill="x", padx=20)

        self.dist_label = ctk.CTkLabel(info, text="Dist: --", font=("Arial", 15))
        self.dist_label.pack(side="left", fill="x", expand=True)

        self.pos_label = ctk.CTkLabel(info, text="Pos: (0.00, 0.00)", font=("Arial", 15))
        self.pos_label.pack(side="left", fill="x", expand=True)

        self.time_label = ctk.CTkLabel(info, text="Time: --", font=("Arial", 15))
        self.time_label.pack(side="right", fill="x", expand=True)

        map_outer = ctk.CTkFrame(self.main_frame)
        map_outer.pack(pady=10, expand=True, fill="both")

        self.canvas = tk.Canvas(map_outer, bg="#1e1e2e", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=5, pady=5)
        self.canvas.bind("<Configure>", self.draw_map_grid)

        # Klik kanan = tambah waypoint dari posisi klik (saat mapping aktif)
        self.canvas.bind("<Button-3>", self._on_canvas_right_click)

    # ─────────────────────────────────────────
    #  WAYPOINT MANAGEMENT
    # ─────────────────────────────────────────
    def _load_waypoints(self):
        if os.path.exists(WAYPOINTS_FILE):
            try:
                with open(WAYPOINTS_FILE, "r") as f:
                    self.waypoints = json.load(f)
                print(f"[INFO] Loaded {len(self.waypoints)} waypoints dari {WAYPOINTS_FILE}")
            except Exception as e:
                print(f"[WARN] Gagal load waypoints: {e}")
                self.waypoints = []
        else:
            # Default kosong — user harus mapping dulu
            self.waypoints = []

    def _load_waypoints_and_refresh(self):
        self._load_waypoints()
        self._refresh_waypoint_buttons()
        self.draw_map_grid()
        self.show_status("Waypoints dimuat ulang ✓", "#2ecc71")

    def _save_waypoints(self):
        try:
            with open(WAYPOINTS_FILE, "w") as f:
                json.dump(self.waypoints, f, indent=2)
            self.show_status(f"Waypoints disimpan ke {WAYPOINTS_FILE} ✓", "#2ecc71")
            print(f"[INFO] Saved {len(self.waypoints)} waypoints ke {WAYPOINTS_FILE}")
        except Exception as e:
            self.show_status("Gagal simpan waypoints!", "#e74c3c")
            print(f"[ERROR] {e}")

    def _refresh_waypoint_buttons(self):
        for widget in self.waypoint_frame.winfo_children():
            widget.destroy()

        for wp in self.waypoints:
            name = wp["name"]
            x, y = wp["x"], wp["y"]

            row = ctk.CTkFrame(self.waypoint_frame, fg_color="transparent")
            row.pack(fill="x", pady=2)

            btn = ctk.CTkButton(
                row, text=f"🚀 {name}  ({x:.2f}, {y:.2f})",
                anchor="w",
                command=lambda n=name: self._send_by_name(n))
            btn.pack(side="left", expand=True, fill="x", padx=(0, 4))

            del_btn = ctk.CTkButton(
                row, text="✕", width=28,
                fg_color="#c0392b", hover_color="#922b21",
                command=lambda n=name: self._delete_waypoint(n))
            del_btn.pack(side="right")

    def save_current_position(self):
        """Simpan posisi robot SEKARANG dari odometry sebagai waypoint baru."""
        x = self.ros_node.current_x
        y = self.ros_node.current_y

        dialog = WaypointDialog(self, round(x, 3), round(y, 3))
        self.wait_window(dialog)

        if dialog.result_name:
            existing = next((w for w in self.waypoints if w["name"] == dialog.result_name), None)
            if existing:
                existing["x"] = round(x, 3)
                existing["y"] = round(y, 3)
                self.show_status(f"Waypoint '{dialog.result_name}' diupdate ✓", "#f1c40f")
            else:
                self.waypoints.append({
                    "name": dialog.result_name,
                    "x": round(x, 3),
                    "y": round(y, 3)
                })
                self.show_status(f"Waypoint '{dialog.result_name}' ditambahkan ✓", "#2ecc71")

            self._refresh_waypoint_buttons()
            self._save_waypoints()
            self.draw_map_grid()

    def _send_by_name(self, name):
        wp = next((w for w in self.waypoints if w["name"] == name), None)
        if wp:
            self.send_robot(wp["x"], wp["y"], name)
        else:
            self.show_status(f"Waypoint '{name}' tidak ditemukan!", "#e74c3c")

    def _delete_waypoint(self, name):
        self.waypoints = [w for w in self.waypoints if w["name"] != name]
        self._refresh_waypoint_buttons()
        self._save_waypoints()
        self.draw_map_grid()
        self.show_status(f"Waypoint '{name}' dihapus", "#e67e22")

    def _on_canvas_right_click(self, event):
        """Klik kanan di canvas → pakai posisi robot sekarang (bukan posisi klik)."""
        self.save_current_position()

    # ─────────────────────────────────────────
    #  SLAM / MAPPING
    # ─────────────────────────────────────────
    def toggle_slam(self):
        if not self.mapping_active:
            self._start_slam()
        else:
            self._stop_slam()

    def _start_slam(self):
        def run():
            try:
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(SSH_HOST, username=SSH_USER, password=SSH_PASS, timeout=5)

                command = (
                    "bash -c 'source /opt/ros/humble/setup.bash && "
                    "source ~/turtlebot3_ws/install/setup.bash && "
                    "export TURTLEBOT3_MODEL=burger && "
                    "ros2 launch slam_toolbox online_async_launch.py "
                    "use_sim_time:=False 2>&1'"
                )
                self.after(0, lambda: self.slam_btn.configure(
                    text="⏹  STOP MAPPING", fg_color="#c0392b", hover_color="#922b21"))
                self.mapping_active = True
                self.after(0, lambda: self.show_status(
                    "SLAM Mapping aktif... gerakin robot lalu klik 📍 SAVE CURRENT POSITION", "#f1c40f"))
                self.after(0, lambda: self.hint_label.configure(text_color="#f1c40f"))

                self._slam_ssh = ssh
                stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
                for line in iter(stdout.readline, ""):
                    print(f"[SLAM]: {line}", end="")
                    if not self.mapping_active:
                        break
            except Exception as e:
                err = str(e)  # FIX: simpan sebelum 'e' hilang
                self.after(0, lambda: self.show_status(f"SLAM Error: {err}", "#e74c3c"))
                self.mapping_active = False

        threading.Thread(target=run, daemon=True).start()

    def _stop_slam(self):
        self.mapping_active = False
        self.slam_btn.configure(
            text="▶  START MAPPING", fg_color="#1a6b3c", hover_color="#145730")
        self.hint_label.configure(text_color="#888")
        self.show_status("SLAM Mapping dihentikan", "#e67e22")
        try:
            if self._slam_ssh:
                self._slam_ssh.exec_command("pkill -f slam_toolbox")
                self._slam_ssh.close()
                self._slam_ssh = None
        except Exception:
            pass

    def save_map(self):
        def run():
            try:
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(SSH_HOST, username=SSH_USER, password=SSH_PASS, timeout=5)
                map_name = f"my_map"
                ssh.exec_command("mkdir -p ~/maps")
                command = (
                    f"bash -c 'source /opt/ros/humble/setup.bash && "
                    f"ros2 run nav2_map_server map_saver_cli -f ~/maps/{map_name}'"
                )
                stdin, stdout, stderr = ssh.exec_command(command)
                stdout.channel.recv_exit_status()
                ssh.close()
                self.after(0, lambda: self.show_status(
                    f"Map disimpan: ~/maps/{map_name} ✓", "#2ecc71"))
            except Exception as e:
                err = str(e)  # FIX: simpan sebelum 'e' hilang
                self.after(0, lambda: self.show_status(f"Gagal simpan map: {err}", "#e74c3c"))

        threading.Thread(target=run, daemon=True).start()
        self.show_status("Menyimpan map...", "#3498db")

    # ─────────────────────────────────────────
    #  HARDWARE
    # ─────────────────────────────────────────
    def start_robot_hardware(self):
        def run():
            try:
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(SSH_HOST, username=SSH_USER, password=SSH_PASS, timeout=5)

                stdin, stdout, stderr = ssh.exec_command("pgrep -f hardware.launch.py")
                if stdout.read().decode().strip():
                    self.after(0, lambda: self.show_status("Hardware already running!", "#f1c40f"))
                    ssh.close()
                    return

                command = (
                    "bash -c 'source /opt/ros/humble/setup.bash && "
                    "source ~/turtlebot3_ws/install/setup.bash && "
                    "export LDS_MODEL=LDS-01 && "
                    "ros2 launch turtlebot3_gix_bringup hardware.launch.py'"
                )
                self.after(0, lambda: self.show_status("Hardware starting...", "#2ecc71"))
                threading.Thread(target=self.stream_logs, args=(ssh, command), daemon=True).start()
            except Exception as e:
                err = str(e)  # FIX: simpan sebelum 'e' hilang
                self.after(0, lambda: self.show_status(f"SSH Error: {err}", "#e74c3c"))
                print(f"Error: {err}")

        threading.Thread(target=run, daemon=True).start()

    def stream_logs(self, ssh, command):
        try:
            stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
            for line in iter(stdout.readline, ""):
                print(f"[PI LOG]: {line}", end="")
            ssh.close()
        except Exception as e:
            print(f"SSH Log Stream Error: {e}")

    # ─────────────────────────────────────────
    #  SERVO
    # ─────────────────────────────────────────
    def toggle_servo(self):
        if self.ros_node.servo_pos == 1.0:
            target_pos = 2.0
            label_text = "⬇  LOWER SERVO"
        else:
            target_pos = 1.0
            label_text = "⬆  LIFT SERVO"
        self.ros_node.servo_pos = target_pos
        self.ros_node.set_servo_position(target_pos)
        self.servo_btn.configure(text=label_text)
        self.show_status(f"Servo: {target_pos}", "#3498db")

    # ─────────────────────────────────────────
    #  TELEOP
    # ─────────────────────────────────────────
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

    # ─────────────────────────────────────────
    #  MAP CANVAS
    # ─────────────────────────────────────────
    def draw_map_grid(self, event=None):
        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        cx, cy = w / 2, h / 2

        for i in range(-20, 21):
            offset = i * self.pixels_per_meter
            color = "#2a2a3e" if i % 5 != 0 else "#3a3a55"
            self.canvas.create_line(cx + offset, 0, cx + offset, h, fill=color)
            self.canvas.create_line(0, cy + offset, w, cy + offset, fill=color)

        # Origin
        self.canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="white", outline="")
        self.canvas.create_text(cx + 6, cy - 12, text="(0,0)", fill="#888", font=("Arial", 8))

        # Waypoint dots
        self.waypoint_dots = {}
        for wp in self.waypoints:
            sx = cx + wp["x"] * self.pixels_per_meter
            sy = cy - wp["y"] * self.pixels_per_meter
            dot = self.canvas.create_oval(sx - 7, sy - 7, sx + 7, sy + 7,
                                          fill="#f39c12", outline="#fff", width=1)
            self.canvas.create_text(sx, sy - 14, text=wp["name"],
                                    fill="#f39c12", font=("Arial", 9, "bold"))
            self.waypoint_dots[wp["name"]] = dot

        # Robot
        self.update_robot_marker(self.robot_pos[0], self.robot_pos[1])

    def update_robot_marker(self, x, y):
        self.robot_pos = (x, y)
        # Update label posisi
        self.pos_label.configure(text=f"Pos: ({x:.2f}, {y:.2f})")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        cx, cy = w / 2, h / 2
        sx = cx + x * self.pixels_per_meter
        sy = cy - y * self.pixels_per_meter
        if self.robot_dot:
            self.canvas.delete(self.robot_dot)
        self.robot_dot = self.canvas.create_oval(
            sx - 9, sy - 9, sx + 9, sy + 9,
            fill="#3498db", outline="white", width=2)

    # ─────────────────────────────────────────
    #  NAVIGATION
    # ─────────────────────────────────────────
    def send_robot(self, x, y, location_name):
        self.ros_node.cancel_movement()
        self.show_status(f"Auto: Moving to {location_name}...", "#f1c40f")
        self.start_time = time.time()
        self.timer_running = True
        self.update_timer()
        self.ros_node.send_waypoint(x, y)

    def cancel_movement(self):
        self.ros_node.cancel_movement()
        self.timer_running = False
        self.show_status("Auto: Cancelled", "#e74c3c")

    def update_distance(self, dist):
        self.dist_label.configure(text=f"Dist: {dist:.2f} m")

    def update_person_status(self, detected: bool):
        if detected:
            self.person_warning.configure(
                text="⚠️  PERSON DETECTED — Robot re-routing...")
        else:
            self.person_warning.configure(text="")

    def show_status(self, text, color):
        self.status_label.configure(text=text, text_color=color)
        if any(word in text for word in ["Reached", "Stopped", "Cancelled"]):
            self.timer_running = False

    def update_timer(self):
        if self.timer_running:
            elapsed = time.time() - self.start_time
            self.time_label.configure(text=f"Time: {elapsed:.1f} s")
            self.after(100, self.update_timer)

    # ─────────────────────────────────────────
    #  CLEANUP
    # ─────────────────────────────────────────
    def on_closing(self):
        self.mapping_active = False
        try:
            if self._slam_ssh:
                self._slam_ssh.close()
        except Exception:
            pass
        if rclpy.ok():
            self.ros_node.destroy_node()
            rclpy.shutdown()
        self.destroy()


# ─────────────────────────────────────────────
if __name__ == "__main__":
    app = RobotDashboard()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
