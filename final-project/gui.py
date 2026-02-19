import customtkinter as ctk
from tkinter import messagebox
import subprocess
import threading
import re
import tkinter as tk

import time

# Set the appearance and theme
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class RobotDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Robot Operator Hub")
        self.geometry("900x600")

        self.current_process = None
        self.monitoring = False

        # Configure grid for layout
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Sidebar Frame
        self.sidebar = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")  


        self.logo_label = ctk.CTkLabel(self.sidebar, text="NAV2 CONTROL", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Sidebar Buttons
        # Dispatch Buttons moved to Sidebar for better layout
        self.create_nav_button("Home / Kitchen", 0.0, 0.0, "gray")
        self.create_nav_button("Dispatch to Table 1", 1.0, 1.0, "blue")
        self.create_nav_button("Dispatch to Table 2", 2.5, -1.0, "blue")
        self.create_nav_button("Dispatch to Table 3", -1.5, 2.0, "blue")

        # Emergency Cancel
        self.cancel_btn = ctk.CTkButton(self.sidebar, text="CANCEL ALL GOALS", 
                                        fg_color="#e74c3c", hover_color="#c0392b",
                                        command=self.cancel_navigation)
        self.cancel_btn.grid(row=10, column=0, pady=30, padx=20, sticky="ew")

        # Main Control Frame
        self.main_frame = ctk.CTkFrame(self, corner_radius=15)
        self.main_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        self.main_frame.grid_columnconfigure(0, weight=1)

        # Status Section
        self.status_label = ctk.CTkLabel(self.main_frame, text="System Ready", text_color="#2ecc71", font=("Arial", 24, "bold"))
        self.status_label.pack(pady=10)

        # Info Frame (Time, Distance)
        self.info_frame = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        self.info_frame.pack(pady=10, fill="x", padx=20)
        
        self.dist_label = ctk.CTkLabel(self.info_frame, text="Dist: --", font=("Arial", 16))
        self.dist_label.pack(side="left", fill="x", expand=True)
        
        self.time_label = ctk.CTkLabel(self.info_frame, text="Time: --", font=("Arial", 16))
        self.time_label.pack(side="right", fill="x", expand=True)

        # Map Canvas
        self.map_frame = ctk.CTkFrame(self.main_frame)
        self.map_frame.pack(pady=20, expand=True, fill="both")
        
        # Using standard tk.Canvas for drawing primitives
        self.canvas = tk.Canvas(self.map_frame, bg="#2b2b2b", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=5, pady=5)
        self.canvas.bind("<Configure>", self.draw_map_grid)
        
        self.robot_dot = None
        self.goal_dot = None

    def draw_map_grid(self, event=None):
        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        cx, cy = w/2, h/2
        
        # Draw grid lines (assuming 1 meter = 40 pixels)
        self.pixels_per_meter = 40
        
        # Grid
        for i in range(-10, 11):
            offset = i * self.pixels_per_meter
            # Vertical
            self.canvas.create_line(cx + offset, 0, cx + offset, h, fill="#404040")
            # Horizontal (Y is inverted in screen coords, but loop handles both)
            self.canvas.create_line(0, cy + offset, w, cy + offset, fill="#404040")
            
        # Origin
        self.canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="white")
        self.canvas.create_text(cx+10, cy+10, text="(0,0)", fill="gray")

        # Redraw robot if position known
        if hasattr(self, 'robot_pos'):
            self.update_robot_marker(self.robot_pos[0], self.robot_pos[1])

    def update_robot_marker(self, x, y):
        self.robot_pos = (x, y)
        w = self.canvas.winfo_width() 
        h = self.canvas.winfo_height()
        cx, cy = w/2, h/2
        
        screen_x = cx + (x * self.pixels_per_meter)
        screen_y = cy - (y * self.pixels_per_meter) # Map Y is up, Screen Y is down
        
        if self.robot_dot:
            self.canvas.delete(self.robot_dot)
        
        # Draw robot
        self.robot_dot = self.canvas.create_oval(screen_x-8, screen_y-8, screen_x+8, screen_y+8, fill="#3498db", outline="white")

    def create_nav_button(self, name, x, y, theme):
        # Add to sidebar instead
        btn = ctk.CTkButton(self.sidebar, text=name, 
                             command=lambda: self.send_robot(x, y, name))
        btn.grid(sticky="ew", padx=20, pady=10)

    def send_robot(self, x, y, location_name):
        if self.current_process:
            self.cancel_navigation()

        # Update goal on map
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        cx, cy = w/2, h/2
        gx = cx + (x * 40)
        gy = cy - (y * 40)
        
        if self.goal_dot:
            self.canvas.delete(self.goal_dot)
        self.goal_dot = self.canvas.create_oval(gx-5, gy-5, gx+5, gy+5, fill="#f1c40f", outline="black")
        
        # [cite_start]ROS 2 CLI command from your report [cite: 34]
        # Keep feedback enabled to parse output
        command = [
            'ros2', 'action', 'send_goal', '/navigate_to_pose', 
            'nav2_msgs/action/NavigateToPose',
            f'{{ "pose": {{ "header": {{ "frame_id": "map" }}, "pose": {{ "position": {{ "x": {x}, "y": {y}, "z": 0.0 }}, "orientation": {{ "w": 1.0 }} }} }} }}',
            '--feedback'
        ]

        try:
            self.current_process = subprocess.Popen(
                command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            self.monitoring = True
            
            thread = threading.Thread(target=self.monitor_process, args=(self.current_process, location_name))
            thread.daemon = True
            
            self.start_time = time.time()
            thread.start()
            
            self.status_label.configure(text=f"Moving to: {location_name}", text_color="#f1c40f")
        except Exception as e:
            messagebox.showerror("System Error", f"Command failed: {e}")

    def monitor_process(self, process, location_name):
        # Output parsers
        dist_pattern = re.compile(r'distance_remaining:\s*([\d\.]+)')
        time_pattern = re.compile(r'navigation_time:.*?sec:\s*(\d+)', re.DOTALL) # Simplified
        
        # We need to capture block output for position, which is hard line-by-line. 
        # But usually 'feedback' prints 'current_pose' logic.
        # Let's try to simple grep for x/y if they appear in lines.
        # Typical CLI output:
        # feedback:
        #   current_pose: ...
        #     position:
        #       x: 0.0
        #       y: 0.0
        
        # We will parse line by line and keep state
        
        current_x = 0.0
        current_y = 0.0
        
        while self.monitoring:
            # Update time
            elapsed = time.time() - self.start_time
            self.after(0, lambda t=elapsed: self.time_label.configure(text=f"Time: {t:.1f} s"))

            line = process.stdout.readline()
            if not line:
                break
                
            # Check for Distance
            dist_match = dist_pattern.search(line)
            if dist_match:
                dist = float(dist_match.group(1))
                self.after(0, lambda d=dist: self.dist_label.configure(text=f"Dist: {d:.2f} m"))

            # Check for Time
            time_match = time_pattern.search(line) # Note: this regex is simple and might match wrong 'sec', but usually nav_time is distinct
            # Actually ROS feedback yaml is tricky. Let's just use start_time logic if regex fails
            # Or use a simpler approach: elapsed time since start
            
            pass # Skipping complex time parsing from regex for now to avoid errors, using a timer might be better.
            
            # Naive Position Parsing (matches "x: 1.2" lines)
            # This is risky if header has x/y, but header is usually deeper or standard
            if "x:" in line:
                try: 
                    val = float(line.split(":")[-1].strip())
                    # Heuristic: if indentation is deep, it's likely position
                    current_x = val
                except: pass
            if "y:" in line:
                try: 
                    val = float(line.split(":")[-1].strip())
                    current_y = val
                    # Assuming Y comes after X, we update map here
                    self.after(0, lambda x=current_x, y=current_y: self.update_robot_marker(x, y))
                except: pass

            # Check for success
            if "Goal reached" in line or "SUCCEEDED" in line:
                self.after(0, lambda: self.status_label.configure(text=f"Reached: {location_name}", text_color="#2ecc71"))
                self.monitoring = False

        # If process ends
        if process.poll() is not None and process.poll() != 0:
            # Maybe canceled or failed, check stderr?
            pass

    def cancel_navigation(self):
        # Cancel command for Nav2
        cancel_cmd = ['ros2', 'action', 'cancel_goal', '/navigate_to_pose', '-a']
        subprocess.Popen(cancel_cmd)
        
        self.monitoring = False
        if self.current_process:
            self.current_process.terminate()
            self.current_process = None
            
        self.status_label.configure(text="Navigation Cancelled", text_color="#e74c3c")
        self.dist_label.configure(text="Dist: --")
        self.time_label.configure(text="Time: Stopped")

if __name__ == "__main__":
    app = RobotDashboard()
    app.mainloop()