import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

class UiMarkers(Node):
    def __init__(self):
        super().__init__('ui_markers')
        
        # Subscribe ke topic yang dipublish map_annotator [cite: 133]
        self.sub = self.create_subscription(
            PoseArray,
            '/map_poses',
            self.callback,
            10)
        
        # Publish MarkerArray ke Rviz [cite: 135]
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        print("UI Markers Node Started.")

    def callback(self, msg):
        marker_array = MarkerArray()
        
        # Loop setiap pose dan buat markernya
        # Kita pakai 'Arrow' agar arah (orientasi) robot terlihat jelas
        for i, pose in enumerate(msg.poses):
            marker = Marker()
            marker.header = msg.header
            marker.ns = "saved_poses"
            marker.id = i
            marker.type = Marker.ARROW # [cite: 136]
            marker.action = Marker.ADD
            
            marker.pose = pose
            
            # Ukuran marker (panjang 0.5m)
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Warna marker (misal: Hijau)
            marker.color.a = 1.0 # Alpha harus 1.0 agar terlihat
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            
            # Opsional: Tambahkan TEXT_VIEW_FACING untuk nama (kalau data nama dikirim)
            # Karena PoseArray standar tidak bawa nama, kita pakai Arrow saja dulu.
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = UiMarkers()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()