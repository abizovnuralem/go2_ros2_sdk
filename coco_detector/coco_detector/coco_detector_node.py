"""
Detects COCO objects in image and publishes in ROS2.

Subscribes to /image and publishes Detection2DArray message on topic /detected_objects.
Also publishes (by default) annotated image with bounding boxes on /annotated_image.

Uses PyTorch and FasterRCNN_MobileNet model from torchvision.
Bounding Boxes use image convention, i.e., center.y = 0 means top of image.
"""

import collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import torch
from torchvision.models import detection as detection_model
from torchvision.utils import draw_bounding_boxes
import cv2
from torch.cuda.amp import autocast


Detection = collections.namedtuple("Detection", "label, bbox, score")


class CocoDetectorNode(Node):
    """Detects COCO objects in image and publishes on ROS2.

    Subscribes to /image and publishes Detection2DArray on /detected_objects.
    Also publishes augmented image with bounding boxes on /annotated_image.
    """

    def __init__(self):
        super().__init__("coco_detector_node")

        # Declare parameters
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('detection_threshold', 0.9)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 360)

        # Read device parameter
        self.device = self.get_parameter('device').get_parameter_value().string_value
        if self.device != 'cpu' and not torch.cuda.is_available():
            self.get_logger().warn(
                f"Requested device '{self.device}' but CUDA is not available. Falling back to CPU."
            )
            self.device = 'cpu'

        self.detection_threshold = (
            self.get_parameter('detection_threshold').get_parameter_value().double_value
        )

        # QoS optimized for camera streams
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Image subscriber
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.listener_callback,
            image_qos)

        # Publishers
        self.detected_objects_publisher = self.create_publisher(
            Detection2DArray, "detected_objects", 10)

        if self.get_parameter('publish_annotated_image').get_parameter_value().bool_value:
            self.annotated_image_publisher = self.create_publisher(Image, "annotated_image", 10)
        else:
            self.annotated_image_publisher = None

        self.bridge = CvBridge()
        self.last_person_count = -1

        # Load model
        self.model = detection_model.fasterrcnn_mobilenet_v3_large_320_fpn(
            weights="FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.COCO_V1",
            progress=True,
            weights_backbone="MobileNet_V3_Large_Weights.IMAGENET1K_V1"
        )

        try:
            self.model = self.model.to(self.device)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to move model to device '{self.device}': {exc}. Using CPU."
            )
            self.device = 'cpu'
            self.model = self.model.to(self.device)

        self.class_labels = (
            detection_model.FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT.meta["categories"]
        )

        # Person label index
        self.person_label_index = self.class_labels.index("person")
        self.detected_persons_publisher = self.create_publisher(
            Detection2DArray, "detected_persons", 10)

        self.model.eval()
        self.get_logger().info("Node has started.")

    def mobilenet_to_ros2(self, detection, header):
        """Converts internal detection format to ROS2 Detection2D message."""
        detection2d = Detection2D()
        detection2d.header = header

        # Class hypothesis
        hypothesis = ObjectHypothesis()
        hypothesis.class_id = self.class_labels[detection.label]
        hypothesis.score = detection.score.detach().item()

        hypothesis_with_pose = ObjectHypothesisWithPose()
        hypothesis_with_pose.hypothesis = hypothesis
        detection2d.results.append(hypothesis_with_pose)

        # Bounding box
        bbox = BoundingBox2D()
        bbox.center.position.x = float((detection.bbox[0] + detection.bbox[2]) / 2)
        bbox.center.position.y = float((detection.bbox[1] + detection.bbox[3]) / 2)
        bbox.center.theta = 0.0
        bbox.size_x = float(2 * (bbox.center.position.x - detection.bbox[0]))
        bbox.size_y = float(2 * (bbox.center.position.y - detection.bbox[1]))

        detection2d.bbox = bbox
        return detection2d

    def publish_annotated_image(self, detections, header, image_tensor):
        """Draws bounding boxes and publishes annotated image."""
        if len(detections) > 0:
            boxes = torch.stack([d.bbox for d in detections])
            labels = [self.class_labels[d.label] for d in detections]
            annotated = draw_bounding_boxes(torch.tensor(image_tensor), boxes, labels, colors="yellow")
        else:
            annotated = torch.tensor(image_tensor)

        # To ROS Image
        img_msg = self.bridge.cv2_to_imgmsg(
            annotated.numpy().transpose(1, 2, 0),
            encoding="rgb8"
        )
        img_msg.header = header
        self.annotated_image_publisher.publish(img_msg)

    def listener_callback(self, msg):
        """Processes incoming frames and performs detection."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Resize
        resize_w = self.get_parameter('resize_width').get_parameter_value().integer_value
        resize_h = self.get_parameter('resize_height').get_parameter_value().integer_value
        if resize_w > 0 and resize_h > 0:
            cv_image = cv2.resize(cv_image, (resize_w, resize_h))

        # Convert to tensor
        image = cv_image.transpose((2, 0, 1))  # HWC → CHW
        tensor = torch.tensor(image / 255.0, dtype=torch.float, device=self.device).unsqueeze(0)

        # Inference
        with torch.no_grad():
            if self.device == 'cuda':
                with autocast():
                    outputs = self.model(tensor)[0]
            else:
                outputs = self.model(tensor)[0]

        # Filter by threshold
        detections = [
            Detection(label, box, score)
            for label, box, score
            in zip(outputs["labels"], outputs["boxes"], outputs["scores"])
            if score >= self.detection_threshold
        ]

        # Person-only filtering
        person_detections = [d for d in detections if d.label == self.person_label_index]
        count = len(person_detections)

        # Log only if count changes (prevents spam)
        if count != self.last_person_count:
            self.get_logger().info(
                f"Frame {msg.header.stamp.sec}.{msg.header.stamp.nanosec}: persons={count}"
            )
            self.last_person_count = count

        # Publish detected persons
        if count > 0:
            person_array = Detection2DArray()
            person_array.header = msg.header
            person_array.detections = [
                self.mobilenet_to_ros2(d, msg.header) for d in person_detections
            ]
            self.detected_persons_publisher.publish(person_array)

        # Annotated image
        if self.annotated_image_publisher is not None:
            self.publish_annotated_image(person_detections, msg.header, image)


# ROS2 node runner
def main():
    rclpy.init()
    node = CocoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
