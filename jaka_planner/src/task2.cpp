#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/Empty.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>

#include "motor_controller.h"

using namespace std;
//rx = -87.503
//ry = 0.299
//rz = 179.661


//position.x = -0.018853
//position.y = -0.445533
//position.z = 0.443923
//orinazation.x = -0.204516
//orinazation.y = 0.174253
//orinazation.z = 0.930921
//orinazation.w = -0.247376
bool ar_marker_received = false;
geometry_msgs::Pose ar_marker_pose;

void arPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    if (!ar_marker_received && !msg->markers.empty())
    {
        ar_marker_pose = msg->markers[0].pose.pose;
        ar_marker_received = true;
        ROS_INFO("Received AR marker pose");
    }
}

// 定义一个简单的四元数结构体
struct Quaternion {
    double w, x, y, z;

    // 构造函数
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
};

// 欧拉角到四元数的转换函数
Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    // 计算半角的正弦和余弦
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    // 计算四元数的各个分量
    Quaternion q(
        cr * cp * cy + sr * sp * sy,  // w
        sr * cp * cy - cr * sp * sy,  // x
        cr * sp * cy + sr * cp * sy,  // y
        cr * cp * sy - sr * sp * cy   // z
    );

    return q;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaka_planner");
    ros::NodeHandle nh;

    // ros::Subscriber ar_sub = nh.subscribe("/ar_pose_marker", 1, arPoseCallback);
    
    // ROS_INFO("Waiting for AR marker data...");
    // while (ros::ok() && !ar_marker_received)
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // }
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 夹爪电机初始化
    ROS_INFO("Initializing motor controller: 115200-8-N-1");
    MotorController controller("/dev/ttyUSB0", 115200);

    if (!controller.init()) {
        ROS_ERROR("Failed to initialize motor controller");
        return -1;
    }

    // Add motor control sequence here
    ROS_INFO("Starting motor control sequence...");

    controller.setIOStop(false);          // Start IO
    ros::Duration(3.0).sleep();          // 3s delay

    controller.setIOStop(true);           // Stop IO
    ros::Duration(3.0).sleep();          // 3s delay

    controller.setTorque(10.0);           // Set torque
    ros::Duration(3.0).sleep();          // 3s delay

    controller.setAcceleration(30.0);     // Set acceleration
    ros::Duration(3.0).sleep();          // 3s delay

    controller.setVelocity(20.0);         // Set velocity
    ros::Duration(3.0).sleep();          // 3s delay

    //rx = -87.503
    //ry = 0.299
    //rz = 179.661
    

    // 设置规划组名称
    // MoveIt使用JointModelGroup来存储机器人手臂的关节信息
    // 这里的"jaka_zu7"是在SRDF配置文件中定义的规划组名称
    // static const std::string PLANNING_GROUP = "jaka_zu7";

    // // 创建move_group控制接口
    // // 这个接口用于控制机器人的运动规划和执行
    // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // // 创建场景接口
    // // 用于在虚拟环境中添加或删除障碍物
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // // 获取机器人当前状态下的关节模型组
    // const robot_state::JointModelGroup* joint_model_group = 
    //     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // // 创建5秒的延时对象（虽然在代码中未使用）
    // ros::Duration du_1(5);

    // // 以下是被注释掉的随机位姿设置示例
    // // move_group.setRandomTarget();

    // // 以下是被注释掉的笛卡尔空间目标点规划示例

    // //position.x = -0.018853
    // //position.y = -0.445533
    // //position.z = 0.443923
    // //orinazation.x = -0.204516
    // //orinazation.y = 0.174253
    // //orinazation.z = 0.930921
    // //orinazation.w = -0.247376
    // geometry_msgs::Pose target_pose1;
    // target_pose1.position.x = -0.552707;
    // target_pose1.position.y = 0.131240;
    // target_pose1.position.z = 0.541877;
    // // target_pose1.orientation.w = -0.384654;
    // // target_pose1.orientation.x = 0.211284;
    // // target_pose1.orientation.y = 0.898531;
    // // target_pose1.orientation.z = 0.006464;
    // // target_pose1.orientation.w = 0;
    // // target_pose1.orientation.x = 4.333;
    // // target_pose1.orientation.y = -49.911;
    // // target_pose1.orientation.z = -30.716;
    // // move_group.setPoseTarget(target_pose1);
    

    // // 创建运动规划对象
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // // 进行运动规划，并获取规划结果
    // bool success = (move_group.plan(my_plan) == 
    //                moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // // 以下是不同执行方式的示例（已注释）
    // move_group.move();   // 阻塞式移动，直接执行运动
    // move_group.execute(my_plan); // 阻塞式执行指定的运动规划
    // move_group.asyncExecute(my_plan);  // 非阻塞式执行
    // ros::Duration(2).sleep();
    // // move_group.stop();  // 停止运动

    // //rx = -87.503
    // //ry = 0.299
    // //rz = 179.661
    // // Quaternion q = eulerToQuaternion(-87.503, 0.299, 179.661);

    // // q.w = -0.384654 ,  q.x  = 0.211284 ,  q.y = 0.898531 ,  q.z = 0.006464 
    // Quaternion q = eulerToQuaternion(-90.895, -0.767, 96.842);
    // // 输出四元数
    // std::cout << "Quaternion: (" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")" << std::endl;
    // ROS_INFO_NAMED("tutorial", "Quaternion: ( q.w = %f ,  q.x  = %f ,  q.y = %f ,  q.z = %f )",q.w,q.x,q.y,q.z );
    // return 0;
    
    static const std::string PLANNING_GROUP = "jaka_zu7";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group.setStartStateToCurrentState();
    geometry_msgs::Pose target_pose = ar_marker_pose;
    move_group.setPoseTarget(target_pose);
    
    success = (move_group.plan(my_plan) == 
               moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", 
                   success ? "" : "FAILED");
    if (success)
    {
        move_group.move();
        ros::Duration(0.3).sleep();
    }

    // 第2个位置：设置6个关节的目标角度（单位：弧度）
    move_group.setStartStateToCurrentState();
    joint_group_positions[0] = 0.017444*5.025;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*177.611;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*86.453;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*85.457;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*90.988;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.967;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.3).sleep();  // 等待0.5秒

    // 第3个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*94.635;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*183.396;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*86.442;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*82.073;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*92.461;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.952;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(3).sleep();  // 等待0.5秒


    // 第4个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*94.632;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*181.261;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*77.670;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*75.936;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*92.477;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.954;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.1).sleep();  // 等待0.5秒

    // 夹爪运动到指定位置
    controller.setPosition(26.0);         // Set target position to 20mm
    ros::Duration(5.0).sleep();          // 3s delay

    // 第5个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*93.667;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*181.994;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*86.613;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*83.732;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*89.600;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.952;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.1).sleep();  // 等待0.5秒

 // 第6个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*88.897;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*181.992;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*94.298;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*93.234;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*79.624;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.952;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.1).sleep();  // 等待0.5秒

    // 第7个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*75.820;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*182.407;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*101.244;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*98.619;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*58.911;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.948;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(1).sleep();  // 等待0.5秒

    // 第8个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*66.652;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*182.987;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*103.801;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*99.994;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*44.558;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.945;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.1).sleep();  // 等待0.5秒

    // 第9个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*53.233;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*182.987;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*105.634;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*103.484;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*30.823;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.945;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.1).sleep();  // 等待0.5秒

  
    // 第10个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*53.362;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*182.887;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*101.256;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*98.386;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*25.822;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.942;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(0.1).sleep();  // 等待0.5秒

    // 第11个位置：设置6个关节的目标角度（单位：弧度）
    joint_group_positions[0] = 0.017444*47.835;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*183.389;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*101.691;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*99.957;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*18.534;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.941;      // 关节6: 0度
    move_group.setJointValueTarget(joint_group_positions);
    // 进行运动规划
    success = (move_group.plan(my_plan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", 
                    success ? "" : "FAILED");
    ROS_INFO("OK");
    
    // 执行运动
    move_group.move();
    ros::Duration(3).sleep();  // 等待0.5秒

      // 关闭ROS节点
    ros::shutdown(); 
    return 0;
}