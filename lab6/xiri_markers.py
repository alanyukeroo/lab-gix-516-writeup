#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

class UIMarkers(Node):
    def __init__(self):
        super().__init__('ui_markers')
        
        self.create_subscription(
            PoseArray,
            '/saved_poses',
            self.pose_array_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pose_markers',
            10
        )
        
        # Store names for labeling (received via pose_names topic)
        self.pose_names = []
        self.get_logger().info("UI Markers node started")
    
    def pose_array_callback(self, msg):
        marker_array = MarkerArray()
        
        # Clear old markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(delete_marker)
        
        for i, pose in enumerate(msg.poses):
            
            # --- Arrow marker (shows position + orientation) ---
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose
            arrow.scale.x = 0.5   # arrow length
            arrow.scale.y = 0.08  # arrow width
            arrow.scale.z = 0.08  # arrow height
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            arrow.lifetime = rclpy.duration.Duration(seconds=3).to_msg()
            marker_array.markers.append(arrow)
            
            # --- Sphere marker (shows exact position) ---
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'spheres'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            sphere.lifetime = rclpy.duration.Duration(seconds=3).to_msg()
            marker_array.markers.append(sphere)
        
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = UIMarkers()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()