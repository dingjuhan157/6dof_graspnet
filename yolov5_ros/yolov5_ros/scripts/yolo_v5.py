#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from cv2 import aruco

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Image, CameraInfo
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Pose, PoseStamped

class Yolo_Detect:
    def __init__(self):
        # 加载ROS参数
        self.load_parameters()

        # 初始化YOLOv5模型
        self.initialize_model()

        # 初始化ArUco检测器
        self.initialize_aruco()

        # 初始化图像处理变量
        self.color_image = None
        self.depth_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.getImageStatus = False

        # 初始化类别颜色字典
        self.classes_colors = {}

        # 设置ROS订阅器和发布器
        self.setup_ros_communication()

        # # 等待第一帧图像到达
        # self.wait_for_first_image()

    def initialize_aruco(self):
        """初始化ArUco检测器"""
        # 使用4x4_50的ArUco字典
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # ArUco标记的实际大小(米)
        self.marker_size = 0.05  # 5cm

    def load_parameters(self):
        """从ROS参数服务器加载参数"""
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
        """初始化YOLOv5模型"""
        self.model = torch.hub.load(self.yolov5_path, 'custom', path=self.weight_path, source='local')
        self.model.cpu() if self.use_cpu else self.model.cuda()
        self.model.conf = float(self.conf)

    def setup_ros_communication(self):
        """设置ROS订阅器和发布器"""
        # 订阅器
        self.color_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback,
                                          queue_size=1, buff_size=52428800)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # 发布器
        self.position_pub = rospy.Publisher(self.pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov5/detection_image', Image, queue_size=1)
        self.distance_pub = rospy.Publisher('/yolov5/distance', Float32, queue_size=1)
        self.pose_pub = rospy.Publisher('/yolov5/object_pose', PoseStamped, queue_size=1)

    def camera_info_callback(self, msg):
        """相机信息回调函数"""
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, image):
        """RGB图像回调函数"""
        self.getImageStatus = True
        self.color_image = self.cv_bridge_convert(image)
        
        # 执行YOLO目标检测
        results = self.model(self.color_image)
        boxes = results.pandas().xyxy[0].values

        # 处理检测结果
        self.process_detections(boxes, image.header)

        # 在每个检测到的目标区域中寻找ArUco标记
        for box in boxes:
            self.process_aruco_markers(self.color_image, box, image.header)

        # 显示结果
        self.display_results(self.color_image, boxes)

    def process_aruco_markers(self, image, box, header):
        """处理目标区域中的ArUco标记"""
        # 提取目标区域
        roi = image[int(box[1]):int(box[3]), int(box[0]):int(box[2])]
        
        # 检测ArUco标记
        corners, ids, rejected = aruco.detectMarkers(roi, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # 调整角点坐标到原始图像坐标系
            for corner in corners:
                corner[0][:, 0] += int(box[0])
                corner[0][:, 1] += int(box[1])
            
            # 估计ArUco标记的位姿
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, 
                                                             self.camera_matrix, self.dist_coeffs)
            
            # 发布位姿信息
            for i in range(len(ids)):
                self.publish_aruco_pose(rvecs[i], tvecs[i], header)
                # 在图像上绘制坐标轴
                aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, 
                             rvecs[i], tvecs[i], self.marker_size)

    def publish_aruco_pose(self, rvec, tvec, header):
        """发布ArUco标记的位姿信息"""
        # 转换旋转向量为旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # 创建位姿消息
        pose_msg = PoseStamped()
        pose_msg.header = header
        
        # 设置位置
        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[0][1]
        pose_msg.pose.position.z = tvec[0][2]
        
        # 计算并设置四元数
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        # 发布位姿消息
        self.pose_pub.publish(pose_msg)

    def depth_callback(self, depth_image):
        """深度图像回调函数"""
        self.depth_image = np.frombuffer(depth_image.data, dtype=np.uint16).reshape(
            depth_image.height, depth_image.width, -1)

    def cv_bridge_convert(self, image):
        """将ROS图像消息转换为OpenCV图像"""
        cv_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, depth_image.width, -1)
        return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    def process_detections(self, boxes, header):
        """处理检测结果并发布边界框"""
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = header
        self.boundingBoxes.image_header = header

        for box in boxes:
            boundingBox = self.create_bounding_box(box, len(boxes))
            self.boundingBoxes.bounding_boxes.append(boundingBox)

        self.position_pub.publish(self.boundingBoxes)

    def create_bounding_box(self, box, total_boxes):
        """创建边界框消息"""
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
        """显示检测结果"""
        for box in boxes:
            color = self.get_class_color(box[-1])
            self.draw_bounding_box(img, box, color)

        cv2.imshow('YOLOv5 with ArUco', img)
        cv2.waitKey(3)

    def get_class_color(self, class_name):
        """获取或生成类别颜色"""
        if class_name not in self.classes_colors:
            self.classes_colors[class_name] = np.random.randint(0, 183, 3)
        return self.classes_colors[class_name]

    def draw_bounding_box(self, img, box, color):
        """在图像上绘制边界框"""
        cv2.rectangle(img, (int(box[0]), int(box[1])),
                      (int(box[2]), int(box[3])), color.tolist(), 2)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """将3x3旋转矩阵转换为四元数"""
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
    """主函数"""
    rospy.init_node('yolov5_aruco_ros', anonymous=True)
    Yolo_Detect()
    rospy.spin()

if __name__ == "__main__":
    main()