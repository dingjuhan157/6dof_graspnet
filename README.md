## 1.安装Ubuntu 18.04

### 1.1镜像文件下载

- 🍊英伟达官方下载地址：[developer.nvidia.com/zh-cn/embed…](https://link.juejin.cn/?target=https%3A%2F%2Fdeveloper.nvidia.com%2Fzh-cn%2Fembedded%2Fdownloads)

### 1.2镜像烧录

## 2.安装Ros

### 2.1 一键安装

- wget http://fishros. com/install-O fishros && . fishros

### 2.2常规安装

- 以 x86_64 架构 Ubuntu 18.04 为例，在 Ubuntu 18.04 上安装 ROS的步骤如下：

- 1.设置 Ubuntu 软件仓库：
  打开终端，运行以下命令以设置添加 ROS 软件仓库到系统中：

  ```python
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
  ```

- 2.添加 ROS 密钥：
  继续在终端运行以下命令以添加 ROS 密钥：

  ```python
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  ```

- 3.更新软件包索引：
  执行以下命令来更新软件包索引：

  ```python
  sudo apt update
  ```

- 4.安装 ROS：
  现在可以安装 ROS 了。可以选择不同的安装配置，取决于用户需求。以下是完整安装 ROS
  桌面完整版（包括 ROS、rqt、rviz 等）的步骤：

  ```python
  sudo apt install ros-melodic-desktop-full
  ```

  请注意，这里使用的是 ROS Melodic 版本，适用于 Ubuntu 18.04。如果你希望安装其他版本
  的 ROS，只需将命令中的"melodic"替换为其他版本的代号（如"noetic"、"kinetic"等）。

- 5.初始化 ROS 环境：
  安装完成后，需要初始化 ROS 环境。在终端运行以下命令：

  ```python
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

- 6.安装 rosdep：

  rosdep 可以帮助安装 ROS 软件包的依赖项。在终端运行以下命令：

  ```
  sudo apt install python-rosdep
  sudo rosdep init
  ```

  **初始化失败连接**:(https://zhuanlan.zhihu.com/p/77483614)

  ```
  rosdep update
  ```

- 7.创建工作空间：
  需要创建一个 ROS 工作空间来组织和构建自己的 ROS 软件包。在终端运行以下命令来创建
  一个工作空间（假设工作空间名为"catkin_ws"）：

  ```python
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make
  ```

- 8.开始使用 ROS：
  现在已经成功安装了 ROS 并创建了一个工作空间。可以开始编写、构建和运行 ROS 软件包了。


## 3.Realsens D435 驱动安装和Ros功能包

- **ubuntu安装参考连接**:https://zhuanlan.zhihu.com/p/371410573
- **ubuntu安装参考连接**:https://blog.csdn.net/ZNC1998/article/details/132174375
- **jetnano安装参考连接**:http://admin.guyuehome.com/43699

## 4.机械臂手眼标定

- 1.手眼标定参考链接：https://blog.csdn.net/weixin_42157689/article/details/134719924?spm=1001.2014.3001.5502
- 2.世界坐标系和相机坐标系,图像坐标系的关系:https://blog.csdn.net/waeceo/article/details/50580607?tdsourcetag=s_pcqq_aiomsg
- 3.手眼标定:https://zhuanlan.zhihu.com/p/62292112。

## 5.YOLOv5训练自己的数据集

- 1.利用yolov5训练自己数据集:https://blog.csdn.net/qq_36756866/article/details/109111065

## 6. JAKA机械臂Ros功能包

### 6.1 ros功能包 https://github.com/JakaCobot/jaka_robot

### 6.2 参考功能包的jaka_ros使用说明

## 7.agilex松灵Ros功能包

### 7.1 Navis 的websocket的API Demo

- https://github.com/agilexrobotics/Navis 
- 参考readme文档配置相关环境

### 7.2 封装ros功能包

- 参考源码的robot control

## 8.graspnet

- 通用物体抓取:https://github.com/graspnet/graspnetAPI

## 9.ros_command
- ### #手眼标定

  roslaunch realsense2_camera rs_camera.launch
  
  roslaunch aruco_ros single_realsense.launch
  
  rosrun image_view image_view image:=/aruco_single/result
  
  roslaunch jaka_planner moveit_server.launch ip:=10.5.5.100 model:=zu7 
  
  roslaunch jaka_zu7_moveit_config demo.launch
  
  roslaunch easy_handeye jaka_eye_to_hand_calibration.launch

- ### #yolov5 node

  roslaunch realsense2_camera rs_camera.launch
  
  roslaunch yolov5_ros yolo_v5.launch
  
  rosrun tf static_transform_publisher 0.5 0 0.3 0 0 0 world camera_link 100
  
  roslaunch jaka_planner moveit_server.launch ip:=10.5.5.100 model:=zu7 
  
  roslaunch jaka_zu7_moveit_config demo.launch
  
  rosrun jaka_planner moveit_test

- ### # 查看

  rostopic echo /yolov5/object_pose
  
  rosrun tf view_frames 


