#!/usr/bin/env python3
"""
TurtleBot3 YOLO Person Detector
Detects moving people and publishes signals on /person_detected
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import json
import math
from collections import defaultdict

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError("Please install ultralytics: pip install ultralytics")


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('motion_threshold', 15.0)   # px/frame
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('history_frames', 5)
        self.declare_parameter('stop_on_person', True)
        self.declare_parameter('show_window', False)

        model_path       = self.get_parameter('model_path').value
        image_topic      = self.get_parameter('image_topic').value
        self.motion_thr  = self.get_parameter('motion_threshold').value
        self.conf_thr    = self.get_parameter('conf_threshold').value
        self.hist_frames = self.get_parameter('history_frames').value
        self.stop_robot  = self.get_parameter('stop_on_person').value
        self.show_window = self.get_parameter('show_window').value

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model loaded.')

        self.bridge = CvBridge()
        self.track_history = defaultdict(list)  # track_id -> [(cx, cy), ...]

        # Subscribers
        self.sub_img = self.create_subscription(
            Image, image_topic, self.image_cb, 10)

        # Publishers
        self.pub_detected = self.create_publisher(Bool,   '/person_detected', 10)
        self.pub_info     = self.create_publisher(String, '/person_info',     10)
        self.pub_cmd      = self.create_publisher(Twist,  '/cmd_vel',         10)

        self.get_logger().info(
            f'Person detector ready | topic={image_topic} '
            f'motion_thr={self.motion_thr}px conf={self.conf_thr}')

    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO tracking (classes=0 → person only)
        results = self.model.track(
            frame,
            persist=True,
            conf=self.conf_thr,
            classes=[0],
            verbose=False
        )

        moving_people = []

        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                if box.id is None:
                    continue
                track_id = int(box.id.item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                conf = float(box.conf.item())

                # Update position history
                history = self.track_history[track_id]
                history.append((cx, cy))
                if len(history) > self.hist_frames:
                    history.pop(0)

                # Compute speed (px/frame) over available history
                speed = 0.0
                if len(history) >= 2:
                    dx = history[-1][0] - history[0][0]
                    dy = history[-1][1] - history[0][1]
                    speed = math.sqrt(dx**2 + dy**2) / max(len(history) - 1, 1)

                if speed >= self.motion_thr:
                    moving_people.append({
                        'id':    track_id,
                        'cx':    round(cx, 1),
                        'cy':    round(cy, 1),
                        'conf':  round(conf, 3),
                        'speed': round(speed, 2)
                    })

        detected = len(moving_people) > 0

        # Publish Bool
        bool_msg = Bool()
        bool_msg.data = detected
        self.pub_detected.publish(bool_msg)

        # Publish JSON info
        info_msg = String()
        info_msg.data = json.dumps({
            'detected': detected,
            'count':    len(moving_people),
            'people':   moving_people
        })
        self.pub_info.publish(info_msg)

        # Optionally stop the robot
        if detected and self.stop_robot:
            self.pub_cmd.publish(Twist())   # all-zero = stop

        if detected:
            self.get_logger().info(
                f'Moving person detected! count={len(moving_people)} '
                f'details={moving_people}')

        # Optional live window
        if self.show_window:
            import cv2
            annotated = results[0].plot()
            cv2.imshow('Person Detector', annotated)
            cv2.waitKey(1)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
