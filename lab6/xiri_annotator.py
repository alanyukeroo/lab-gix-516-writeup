#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient                       
from nav2_msgs.action import NavigateToPose                 
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf2_ros import TransformListener, Buffer
import pandas as pd
import os

class MapAnnotator(Node):
    def __init__(self):
        super().__init__('map_annotator')
        
        # Publisher for PoseArray (for ui_markers node)
        self.pose_array_pub = self.create_publisher(
            PoseArray, 
            '/saved_poses', 
            10
        )
        
        # ✅ FIXED: Use ActionClient instead of publisher
        # ❌ OLD: self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # TF listener to get robot's current pose in map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Dictionary to store poses {name: Pose}
        self.poses = {}
        
        # Absolute CSV file path
        self.csv_file = os.path.expanduser('~/turtlebot4_ws/poses2.csv')
        
        # Load existing poses from file
        self.load_poses()
        
        # Publish PoseArray every 1 second
        self.create_timer(1.0, self.publish_pose_array)
        
        self.print_help()
        self.run_command_loop()
    
    def print_help(self):
        print("\ncommands:")
        print("- list")
        print("- save <name>")
        print("- delete <name>")
        print("- goto <name>")
        print("- exit")
        print("- help")
    
    def load_poses(self):
        """Load poses from CSV file on startup"""
        if os.path.exists(self.csv_file):
            try:
                df = pd.read_csv(self.csv_file)
                for _, row in df.iterrows():
                    pose = Pose()
                    pose.position.x = float(row['px'])
                    pose.position.y = float(row['py'])
                    pose.position.z = float(row['pz'])
                    pose.orientation.x = float(row['ox'])
                    pose.orientation.y = float(row['oy'])
                    pose.orientation.z = float(row['oz'])
                    pose.orientation.w = float(row['ow'])
                    self.poses[row['name']] = pose
                print(f"Loaded {len(self.poses)} poses from file")
            except Exception as e:
                self.get_logger().error(f"Error loading poses: {e}")
        else:
            print("No existing poses file found")
    
    def save_poses(self):
        """Save all poses to CSV file"""
        try:
            data = []
            for name, pose in self.poses.items():
                data.append({
                    'name': name,
                    'px': pose.position.x,
                    'py': pose.position.y,
                    'pz': pose.position.z,
                    'ox': pose.orientation.x,
                    'oy': pose.orientation.y,
                    'oz': pose.orientation.z,
                    'ow': pose.orientation.w
                })
            df = pd.DataFrame(data)
            df.to_csv(self.csv_file, index=False)
            print(f"Saved {len(self.poses)} poses to file")
        except Exception as e:
            self.get_logger().error(f"Error saving poses: {e}")
    
    def get_current_pose(self):
        """Get robot's current pose in map frame via TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None
    
    def publish_pose_array(self):
        """Publish all saved poses as PoseArray for ui_markers"""
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = list(self.poses.values())
        self.pose_array_pub.publish(pose_array)
    
    def cmd_list(self):
        if not self.poses:
            print("No poses")
        else:
            print("Poses:")
            for name in self.poses.keys():
                print(f"- {name}")
    
    def cmd_save(self, name):
        pose = self.get_current_pose()
        if pose:
            self.poses[name] = pose
            print(f"Saved pose '{name}'")
        else:
            print("Failed to get current pose")
    
    def cmd_delete(self, name):
        if name in self.poses:
            del self.poses[name]
            print(f"Deleted '{name}'")
        else:
            print(f"'{name}' does not exist")
    
    def cmd_goto(self, name):
        """✅ FIXED: Send navigation goal via ActionClient"""
        if name not in self.poses:
            print(f"'{name}' does not exist")
            return
        
        # Wait for Nav2 action server to be ready
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            print("Nav2 action server not available! Is Nav2 running?")
            return
        
        # Build the goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # ✅ Fresh timestamp
        goal_msg.pose.pose = self.poses[name]
        
        # Send goal asynchronously
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda f: self._goal_response_callback(f, name)
        )
        print(f"Sent goal to '{name}'")
    
    def _goal_response_callback(self, future, name):
        """Called when Nav2 accepts or rejects the goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"Goal to '{name}' was REJECTED by Nav2!")
            return
        print(f"Goal to '{name}' accepted, robot is moving...")
        
        # Optional: wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: print(f"Reached '{name}' successfully!")
        )
    
    def run_command_loop(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
                cmd = input("> ").strip()
                
                if not cmd:
                    continue
                
                parts = cmd.split()
                command = parts[0].lower()
                
                if command == 'exit':
                    self.save_poses()
                    break
                elif command == 'help':
                    self.print_help()
                elif command == 'list':
                    self.cmd_list()
                elif command == 'save' and len(parts) == 2:
                    self.cmd_save(parts[1])
                elif command == 'delete' and len(parts) == 2:
                    self.cmd_delete(parts[1])
                elif command == 'goto' and len(parts) == 2:
                    self.cmd_goto(parts[1])
                else:
                    print("Invalid command. Type 'help' for commands.")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                self.save_poses()
                break
            except EOFError:
                print("\nExiting...")
                self.save_poses()
                break

def main():
    rclpy.init()
    node = MapAnnotator()
    rclpy.shutdown()

if __name__ == '__main__':
    main()