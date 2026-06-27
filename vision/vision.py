#!/usr/bin/env python3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

COLOR_TOPIC = '/camera/camera/color/image_raw'
MODEL_PATH = 'yolov8n.pt'
CONF_THRESHOLD = 0.8
CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.last_labels = None
        self.display_available = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        self.sub = self.create_subscription(Image, COLOR_TOPIC, self.image_callback, 10)
        self.get_logger().info(f'Subscribed to {COLOR_TOPIC}, running YOLO object detection...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)[0]

        detections = []
        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < CONF_THRESHOLD:
                continue
            label = self.model.names[int(box.cls[0])]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append((label, conf, x1, y1, x2, y2))
            self.get_logger().info(f'detected: {label} ({conf:.2f})')

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        current_labels = tuple(sorted(label for label, *_ in detections))
        if detections and current_labels != self.last_labels:
            self.save_capture(frame, detections)
        self.last_labels = current_labels

        if self.display_available:
            cv2.imshow('YOLO Detection', frame)
            cv2.waitKey(1)

    def save_capture(self, frame, detections):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        image_path = os.path.join(CAPTURE_DIR, f'{stamp}.jpg')
        label_path = os.path.join(CAPTURE_DIR, f'{stamp}.txt')

        cv2.imwrite(image_path, frame)
        with open(label_path, 'w') as f:
            for label, conf, x1, y1, x2, y2 in detections:
                f.write(f'{label} {conf:.2f} {x1} {y1} {x2} {y2}\n')


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if node.display_available:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
