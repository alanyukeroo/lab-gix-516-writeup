#!/usr/bin/env python3
"""
person_obstacle_node.py
=======================
Subscribe ke /yolo/detections → filter "person" → estimasi posisi
pakai LiDAR /scan → publish PointCloud2 ke Nav2 local costmap
sehingga robot auto re-route menghindari orang.

Run:
    ros2 run <your_pkg> person_obstacle_node
    atau langsung:
    python3 person_obstacle_node.py

Topics:
    Subscribe:
        /yolo/detections        (yolo_msgs/msg/DetectionArray)
        /scan                   (sensor_msgs/msg/LaserScan)
    Publish:
        /person_obstacles       (sensor_msgs/msg/PointCloud2)  → costmap layer
        /person_detected        (std_msgs/msg/Bool)            → dashboard GUI
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Header
import struct
import math
import numpy as np


# ── Konfigurasi ──────────────────────────────────────────────
IMAGE_WIDTH       = 640     # resolusi kamera USB (ganti kalau beda)
IMAGE_HEIGHT      = 480
HFOV_DEG          = 62.0    # horizontal FOV kamera USB (umumnya ~60-70°)
                             # cek spek kamera kamu, atau ukur manual
MIN_CONFIDENCE    = 0.45    # threshold confidence YOLO
OBSTACLE_RADIUS   = 0.4     # radius obstacle yang dipublish ke costmap (meter)
OBSTACLE_HEIGHT   = 0.05    # tinggi pointcloud obstacle (meter)
LIDAR_MATCH_TOL   = 0.15    # toleransi sudut LiDAR vs kamera (radian)
CLEAR_TIMEOUT     = 1.5     # detik sebelum obstacle dianggap hilang
# ─────────────────────────────────────────────────────────────


class PersonObstacleNode(Node):
    def __init__(self):
        super().__init__('person_obstacle_node')

        # QoS untuk costmap (butuh reliable)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Subscribers ──────────────────────────────────────
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.yolo_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ── Publishers ───────────────────────────────────────
        # PointCloud2 → dibaca oleh Nav2 obstacle_layer di costmap
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/person_obstacles',
            reliable_qos
        )

        # Bool → dibaca dashboard GUI untuk tampilkan warning
        self.detected_pub = self.create_publisher(
            Bool,
            '/person_detected',
            10
        )

        # ── State ────────────────────────────────────────────
        self.latest_scan = None
        self.last_detection_time = 0.0
        self.person_positions = []   # list of (x, y) dalam frame robot

        # Timer publish obstacle ke costmap @ 10Hz
        self.timer = self.create_timer(0.1, self.publish_obstacles)

        self.get_logger().info('PersonObstacleNode started!')
        self.get_logger().info(f'  IMAGE_WIDTH={IMAGE_WIDTH}, HFOV={HFOV_DEG}°')
        self.get_logger().info(f'  Subscribing: /yolo/detections, /scan')
        self.get_logger().info(f'  Publishing:  /person_obstacles, /person_detected')

    # ─────────────────────────────────────────────────────────
    #  CALLBACK: LiDAR Scan
    # ─────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # ─────────────────────────────────────────────────────────
    #  CALLBACK: YOLO Detections
    # ─────────────────────────────────────────────────────────
    def yolo_callback(self, msg: DetectionArray):
        persons = [
            d for d in msg.detections
            if d.class_name == 'person' and d.score >= MIN_CONFIDENCE
        ]

        if not persons:
            # Tidak ada orang terdeteksi
            bool_msg = Bool()
            bool_msg.data = False
            self.detected_pub.publish(bool_msg)
            return

        # Ada orang → estimasi posisi
        self.person_positions = []
        now = self.get_clock().now().nanoseconds / 1e9

        for person in persons:
            pos = self._estimate_position(person)
            if pos is not None:
                self.person_positions.append(pos)
                self.get_logger().info(
                    f'[PERSON] score={person.score:.2f} '
                    f'bbox_w={person.bbox.size.x:.0f}px '
                    f'→ estimated pos=({pos[0]:.2f}, {pos[1]:.2f})m'
                )

        if self.person_positions:
            self.last_detection_time = now
            bool_msg = Bool()
            bool_msg.data = True
            self.detected_pub.publish(bool_msg)

    # ─────────────────────────────────────────────────────────
    #  ESTIMASI POSISI ORANG
    #  Metode: sudut dari bbox center → cocokkan dengan LiDAR scan
    # ─────────────────────────────────────────────────────────
    def _estimate_position(self, detection):
        """
        Konversi bounding box pixel → (x, y) dalam frame robot.
        
        Langkah:
        1. Hitung sudut horizontal orang dari center kamera
        2. Cari jarak dari LiDAR di sudut yang sama
        3. Konversi (sudut, jarak) → (x, y) Cartesian
        
        Kalau LiDAR tidak tersedia, estimasi jarak dari lebar bbox.
        """
        bbox = detection.bbox

        # Sudut horizontal dari center kamera
        # pixel 0 = kiri, IMAGE_WIDTH = kanan
        # sudut 0 = depan robot, positif = kiri (sesuai ROS convention)
        hfov_rad = math.radians(HFOV_DEG)
        center_x_norm = (bbox.center.position.x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)
        angle_from_center = center_x_norm * (hfov_rad / 2)
        # Flip: kamera positif = kanan layar, ROS positif = kiri
        camera_angle = -angle_from_center

        # ── Coba dapat jarak dari LiDAR ──────────────────────
        distance = None
        if self.latest_scan is not None:
            distance = self._get_lidar_distance(camera_angle)

        # ── Fallback: estimasi jarak dari lebar bbox ──────────
        # Asumsi lebar badan orang ~0.5m, focal length estimasi
        if distance is None:
            PERSON_WIDTH_REAL = 0.5   # meter
            FOCAL_LENGTH_PX   = IMAGE_WIDTH / (2 * math.tan(hfov_rad / 2))
            bbox_width = bbox.size.x
            if bbox_width > 0:
                distance = (PERSON_WIDTH_REAL * FOCAL_LENGTH_PX) / bbox_width
                self.get_logger().debug(
                    f'LiDAR tidak cocok, fallback estimasi jarak={distance:.2f}m')
            else:
                return None

        # Cap jarak maksimum (LiDAR LDS-01 max ~3.5m efektif)
        distance = min(distance, 3.5)

        # ── Konversi polar → Cartesian (frame robot) ──────────
        # x = depan robot, y = kiri robot
        x = distance * math.cos(camera_angle)
        y = distance * math.sin(camera_angle)

        return (x, y)

    def _get_lidar_distance(self, target_angle_rad):
        """
        Cari jarak LiDAR di sudut yang paling mendekati target_angle.
        Return None kalau tidak ada reading valid.
        """
        scan = self.latest_scan
        if scan is None:
            return None

        # Cari index yang paling dekat dengan target angle
        angle = scan.angle_min
        best_dist = None
        best_diff = float('inf')

        for i, r in enumerate(scan.ranges):
            diff = abs(self._angle_diff(angle, target_angle_rad))
            if diff < LIDAR_MATCH_TOL and diff < best_diff:
                if scan.range_min < r < scan.range_max and not math.isnan(r) and not math.isinf(r):
                    best_diff = diff
                    best_dist = r
            angle += scan.angle_increment

        return best_dist

    @staticmethod
    def _angle_diff(a, b):
        """Selisih sudut yang aware terhadap wrap-around."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    # ─────────────────────────────────────────────────────────
    #  PUBLISH OBSTACLE KE NAV2 COSTMAP
    # ─────────────────────────────────────────────────────────
    def publish_obstacles(self):
        """
        Publish PointCloud2 berisi titik-titik obstacle di sekitar orang.
        Nav2 obstacle_layer akan baca ini dan masukkan ke costmap.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # Clear kalau sudah lama tidak ada deteksi
        if now - self.last_detection_time > CLEAR_TIMEOUT:
            self.person_positions = []

        # Publish pointcloud (kosong atau berisi obstacle)
        points = []

        for (px, py) in self.person_positions:
            # Buat lingkaran titik-titik di sekitar posisi orang
            # Nav2 butuh beberapa titik untuk membentuk obstacle di costmap
            for r in np.arange(0.0, OBSTACLE_RADIUS, 0.1):
                for theta in np.linspace(0, 2 * math.pi, max(6, int(r * 20))):
                    ox = px + r * math.cos(theta)
                    oy = py + r * math.sin(theta)
                    points.append((ox, oy, OBSTACLE_HEIGHT))

        cloud_msg = self._make_pointcloud2(points)
        self.obstacle_pub.publish(cloud_msg)

    def _make_pointcloud2(self, points):
        """Buat PointCloud2 message dari list of (x, y, z)."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'  # frame robot

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12  # 3 x float32 = 12 bytes
        data = bytearray()
        for (x, y, z) in points:
            data += struct.pack('fff', float(x), float(y), float(z))

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(points)
        msg.data = bytes(data)
        msg.is_dense = True

        return msg


# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PersonObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
