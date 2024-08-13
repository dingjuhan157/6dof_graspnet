#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Image, CameraInfo
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Pose, PoseStamped

class Yolo_Detect:
    def __init__(self):
        # Load ROS parameters
        self.load_parameters()

        # Initialize YOLOv5 model
        self.initialize_model()

        # Initialize image processing variables
        self.color_image = None
        self.depth_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.getImageStatus = False

        # Initialize class colors dictionary
        self.classes_colors = {}

        # Set up ROS subscribers and publishers
        self.setup_ros_communication()

        # Define 3D object points for pose estimation
        self.object_points = np.array([
            [-0.1, -0.1, 0],
            [0.1, -0.1, 0],
            [0.1, 0.1, 0],
            [-0.1, 0.1, 0]
        ], dtype=np.float32)

        # Wait for the first image to arrive
        self.wait_for_first_image()

    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        self.yolov5_path = rospy.get_param('/yolov5_path', '')
        self.weight_path = rospy.get_param('~weight_path', '')
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw')
        self.pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        self.camera_color = rospy.get_param('~camera_color', '')
        self.conf = rospy.get_param('~conf', '0.5')
        self.use_cpu = rospy.get_param('/use_cpu', 'true')

    def initialize_model(self):
        """Initialize YOLOv5 model"""
        self.model = torch.hub.load(self.yolov5_path, 'custom', path=self.weight_path, source='local')
        self.model.cpu() if self.use_cpu else self.model.cuda()
        self.model.conf = float(self.conf)

    def setup_ros_communication(self):
        """Set up ROS subscribers and publishers"""
        # Subscribers
        self.color_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback,
                                          queue_size=1, buff_size=52428800)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # Publishers
        self.position_pub = rospy.Publisher(self.pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov5/detection_image', Image, queue_size=1)
        self.distance_pub = rospy.Publisher('/yolov5/distance', Float32, queue_size=1)
        self.pose_pub = rospy.Publisher('/yolov5/object_pose', PoseStamped, queue_size=1)

    def wait_for_first_image(self):
        """Wait for the first image to arrive"""
        while not self.getImageStatus:
            rospy.loginfo("Waiting for image...")
            rospy.sleep(2)

    def camera_info_callback(self, msg):
        """Callback for camera info messages"""
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, image):
        """Callback for color image messages"""
        self.getImageStatus = True
        self.color_image = self.cv_bridge_convert(image)
        
        # Perform object detection
        results = self.model(self.color_image)
        boxes = results.pandas().xyxy[0].values

        # Process detection results
        self.process_detections(boxes, image.header)

        # Display results
        self.display_results(self.color_image, boxes)

    def depth_callback(self, depth_image):
        """Callback for depth image messages"""
        self.depth_image = np.frombuffer(depth_image.data, dtype=np.uint16).reshape(
            depth_image.height, depth_image.width, -1)

    def cv_bridge_convert(self, image):
        """Convert ROS image message to OpenCV image"""
        cv_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    def process_detections(self, boxes, header):
        """Process detection results and publish bounding boxes"""
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = header
        self.boundingBoxes.image_header = header

        for box in boxes:
            boundingBox = self.create_bounding_box(box, len(boxes))
            self.boundingBoxes.bounding_boxes.append(boundingBox)

        self.position_pub.publish(self.boundingBoxes)

    def create_bounding_box(self, box, total_boxes):
        """Create a BoundingBox message from detection results"""
        boundingBox = BoundingBox()
        boundingBox.probability = np.float64(box[4])
        boundingBox.xmin = np.int64(box[0])
        boundingBox.ymin = np.int64(box[1])
        boundingBox.xmax = np.int64(box[2])
        boundingBox.ymax = np.int64(box[3])
        boundingBox.num = np.int16(total_boxes)
        boundingBox.Class = box[-1]
        return boundingBox

    def display_results(self, img, boxes):
        """Display detection results on the image"""
        for box in boxes:
            color = self.get_class_color(box[-1])
            self.draw_bounding_box(img, box, color)
            self.estimate_pose(img, box)

        cv2.imshow('YOLOv5', img)
        cv2.waitKey(3)

    def get_class_color(self, class_name):
        """Get or generate a color for a class"""
        if class_name not in self.classes_colors:
            self.classes_colors[class_name] = np.random.randint(0, 183, 3)
        return self.classes_colors[class_name]

    def draw_bounding_box(self, img, box, color):
        """Draw a bounding box on the image"""
        cv2.rectangle(img, (int(box[0]), int(box[1])),
                      (int(box[2]), int(box[3])), color.tolist(), 2)

    def estimate_pose(self, img, box):
        """Estimate the pose of the detected object"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        image_points = np.array([
            [box[0], box[1]],
            [box[2], box[1]],
            [box[2], box[3]],
            [box[0], box[3]]
        ], dtype=np.float32)

        success, rotation_vector, translation_vector = cv2.solvePnP(
            self.object_points, image_points, self.camera_matrix, self.dist_coeffs)

        if success:
            self.publish_pose(rotation_vector, translation_vector)
            self.draw_pose_info(img, box, translation_vector)

    def publish_pose(self, rotation_vector, translation_vector):
        """Publish the estimated pose"""
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        pose_msg = PoseStamped()
        pose_msg.header = self.boundingBoxes.header
        pose_msg.pose.position.x = translation_vector[0][0]
        pose_msg.pose.position.y = translation_vector[1][0]
        pose_msg.pose.position.z = translation_vector[2][0] * 0.1

        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)

    def draw_pose_info(self, img, box, translation_vector):
        """Draw pose information on the image"""
        pose_text = "X:{:.2f} Y:{:.2f} Z:{:.2f}".format(
            translation_vector[0][0], translation_vector[1][0], translation_vector[2][0])
        cv2.putText(img, pose_text, (int(box[0]), int(box[3])+40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert a 3x3 rotation matrix to a quaternion"""
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return [qx, qy, qz, qw]

def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    Yolo_Detect()
    rospy.spin()

if __name__ == "__main__":
    main()