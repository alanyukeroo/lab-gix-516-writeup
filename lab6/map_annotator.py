import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
import pandas as pd
import threading
import sys
import os

class MapAnnotator(Node):
    def __init__(self):
        super().__init__('map_annotator')
        
        # Subscriber untuk tahu posisi robot saat ini (biasanya dari AMCL)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        # Publisher untuk mengirim perintah jalan (goto)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Publisher untuk mengirim daftar pose ke UI Marker [cite: 105]
        self.pose_array_pub = self.create_publisher(PoseArray, '/map_poses', 10)

        self.current_pose = None
        self.poses_dict = {} # Menyimpan data pose di memori
        
        # Setup file database
        # Kita pakai absolute path agar aman [cite: 152]
        self.file_path = os.path.expanduser('~/turtlebot4_ws/src/lab6/poses.csv')
        self.load_poses()

        # Timer untuk terus publish PoseArray agar marker muncul di Rviz
        self.create_timer(1.0, self.publish_pose_list)

        print("Map Annotator Started.")
        self.print_menu()

    def pose_callback(self, msg):
        # Simpan posisi robot terakhir yang diketahui
        self.current_pose = msg.pose.pose

    def load_poses(self):
        # Load data dari CSV jika ada [cite: 112]
        if os.path.exists(self.file_path):
            try:
                df = pd.read_csv(self.file_path)
                for index, row in df.iterrows():
                    p = Pose()
                    p.position.x = row['px']
                    p.position.y = row['py']
                    p.position.z = row['pz']
                    p.orientation.x = row['ox']
                    p.orientation.y = row['oy']
                    p.orientation.z = row['oz']
                    p.orientation.w = row['ow']
                    self.poses_dict[row['name']] = p
                print(f"Loaded {len(self.poses_dict)} poses from database.")
            except Exception as e:
                print(f"Error loading CSV: {e}")

    def save_poses_to_file(self):
        # Simpan data ke CSV saat exit [cite: 107]
        data = []
        for name, p in self.poses_dict.items():
            data.append({
                'name': name,
                'px': p.position.x, 'py': p.position.y, 'pz': p.position.z,
                'ox': p.orientation.x, 'oy': p.orientation.y, 
                'oz': p.orientation.z, 'ow': p.orientation.w
            })
        
        df = pd.DataFrame(data)
        # Format kolom sesuai permintaan [cite: 110]
        cols = ['name', 'px', 'py', 'pz', 'ox', 'oy', 'oz', 'ow']
        df = df[cols] if not df.empty else pd.DataFrame(columns=cols)
        
        df.to_csv(self.file_path, index=False)
        print(f"Poses saved to {self.file_path}")

    def publish_pose_list(self):
        # Publish PoseArray untuk node ui_markers
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for p in self.poses_dict.values():
            msg.poses.append(p)
        
        self.pose_array_pub.publish(msg)

    def print_menu(self):
        print("\ncommands:")
        print(" list")
        print(" save <name>")
        print(" delete <name>")
        print(" goto <name>")
        print(" help")
        print(" exit")

    def handle_input(self):
        # Loop ini jalan di thread terpisah agar tidak memblokir ROS
        while rclpy.ok():
            try:
                # Pakai input() biasa, command prompt tanda '>'
                cmd_str = input("> ").strip()
                parts = cmd_str.split()
                if not parts:
                    continue

                cmd = parts[0]
                arg = parts[1] if len(parts) > 1 else None

                if cmd == 'list':
                    print("Poses:")
                    if not self.poses_dict:
                        print(" No poses") # [cite: 131]
                    for name in self.poses_dict:
                        print(f" {name}")

                elif cmd == 'save':
                    if not arg:
                        print("Error: usage 'save <name>'")
                        continue
                    if self.current_pose is None:
                        print("Error: No pose received from robot yet.")
                        continue
                    
                    # Simpan pose saat ini [cite: 95]
                    self.poses_dict[arg] = self.current_pose
                    print(f"Saved pose '{arg}'")

                elif cmd == 'delete':
                    if not arg:
                        print("Error: usage 'delete <name>'")
                        continue
                    if arg in self.poses_dict:
                        del self.poses_dict[arg]
                        print(f"Deleted '{arg}'")
                    else:
                        print(f"'{arg}' does not exist") # [cite: 128]

                elif cmd == 'goto':
                    if not arg:
                        print("Error: usage 'goto <name>'")
                        continue
                    if arg in self.poses_dict:
                        target_pose = self.poses_dict[arg]
                        
                        # Buat pesan PoseStamped [cite: 101]
                        goal = PoseStamped()
                        goal.header.frame_id = 'map'
                        goal.header.stamp = self.get_clock().now().to_msg()
                        goal.pose = target_pose
                        
                        self.goal_pub.publish(goal)
                        print(f"Sent goal to '{arg}'")
                    else:
                        print(f"'{arg}' does not exist")

                elif cmd == 'help':
                    self.print_menu()

                elif cmd == 'exit':
                    self.save_poses_to_file()
                    print("Exiting...")
                    rclpy.shutdown()
                    break
                
                else:
                    print("Unknown command. Type 'help'.")

            except EOFError:
                break
            except Exception as e:
                print(f"Error processing input: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapAnnotator()
    
    # Jalankan input user di thread terpisah
    input_thread = threading.Thread(target=node.handle_input)
    input_thread.start()
    
    # Jalankan node ROS di main thread
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.save_poses_to_file()
            rclpy.shutdown()
        input_thread.join()

if __name__ == '__main__':
    main()
    