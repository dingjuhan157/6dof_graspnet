#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/Empty.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 添加这个头文件
#include <Eigen/Dense>  // 用于五次多项式求解

#include "motor_controller.h"

using namespace std;


// 设置机械臂运动参数
void configureMovementParams(moveit::planning_interface::MoveGroupInterface& move_group)
{
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setGoalJointTolerance(0.0001);
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);
}

// 执行运动规划和控制
bool planAndMove(moveit::planning_interface::MoveGroupInterface& move_group, 
                moveit::planning_interface::MoveGroupInterface::Plan& my_plan,
                const std::string& position_name)
{
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning to %s position %s", position_name.c_str(), success ? "SUCCESS" : "FAILED");
    
    if(success) {
        move_group.move();
        ros::Duration(1.5).sleep();
    }
    return success;
}

// 执行笛卡尔路径
bool executeCartesianPath(moveit::planning_interface::MoveGroupInterface& move_group, 
                         std::vector<geometry_msgs::Pose>& waypoints,
                         const std::string& motion_name,
                         double scale = 1.0)
{
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;  // 1cm
    
    // 设置运动速度
    move_group.setMaxVelocityScalingFactor(scale);
    
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    ROS_INFO("%s path (%.2f%% achieved)", motion_name.c_str(), fraction * 100.0);
    
    if(fraction > 0.95) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
        ros::Duration(0.5).sleep();
        return true;
    }
    return false;
}

// 五次多项式轨迹规划
struct QuinticPolynomial {
    double a0, a1, a2, a3, a4, a5;
    
    QuinticPolynomial(double start_pos, double start_vel, double start_acc,
                     double end_pos, double end_vel, double end_acc,
                     double time) {
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 1> b;
        
        // 设置约束条件
        A << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 2, 0, 0, 0,
             1, time, pow(time,2), pow(time,3), pow(time,4), pow(time,5),
             0, 1, 2*time, 3*pow(time,2), 4*pow(time,3), 5*pow(time,4),
             0, 0, 2, 6*time, 12*pow(time,2), 20*pow(time,3);
             
        b << start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        
        // 求解系数
        Eigen::Matrix<double, 6, 1> x = A.inverse() * b;
        a0 = x(0); a1 = x(1); a2 = x(2); a3 = x(3); a4 = x(4); a5 = x(5);
    }
    
    // 计算位置
    double calcPoint(double t) {
        return a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    }
    
    // 计算速度
    double calcFirstDerivative(double t) {
        return a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    }
    
    // 计算加速度
    double calcSecondDerivative(double t) {
        return 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);
    }
};

bool doorOpeningMotionPlanning(moveit::planning_interface::MoveGroupInterface& move_group,
                             const geometry_msgs::Point& hinge_point,
                             const geometry_msgs::Point& handle_point)
{
    ROS_INFO("Executing optimized door opening motion planning...");
    
    // 1. Improved Initial Setup
    move_group.setMaxVelocityScalingFactor(0.05);     // Reduced from 0.1
    move_group.setMaxAccelerationScalingFactor(0.05); // Reduced from 0.1
    move_group.setGoalPositionTolerance(0.02);        // Increased from 0.01
    move_group.setGoalOrientationTolerance(0.04);     // Increased from 0.02
    
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    
    // 2. Improved Door Parameters Calculation
    double door_radius = std::hypot(
        handle_point.x - hinge_point.x,
        handle_point.y - hinge_point.y
    );
    
    // Reduced gripper compensation from 10cm to 5cm
    double adjusted_hinge_x = hinge_point.x - 0.05;
    
    ROS_INFO("Door parameters:");
    ROS_INFO("- Door radius: %.3f meters", door_radius);
    ROS_INFO("- Hinge point (B): x=%.3f(adjusted:%.3f), y=%.3f, z=%.3f", 
             hinge_point.x, adjusted_hinge_x, hinge_point.y, hinge_point.z);
    ROS_INFO("- Handle point (C): x=%.3f, y=%.3f, z=%.3f", 
             handle_point.x, handle_point.y, handle_point.z);
    
    // 3. Gradual Opening Angle
    const double opening_angle = M_PI/4;  // Reduced to 45 degrees from 90
    const double planning_time = 30.0;    // Reduced from 60s to 30s
    const int num_points = 45;            // Increased from 30 to 50 for smoother motion
    
    // Calculate initial angle more precisely
    double initial_angle = std::atan2(current_pose.position.y - hinge_point.y,
                                    current_pose.position.x - adjusted_hinge_x);
    
    // 4. Improved Trajectory Generation
    QuinticPolynomial angle_trajectory(
        0.0, 0.0, 0.0,           // Start: pos, vel, acc
        opening_angle, 0.0, 0.0,  // End: pos, vel, acc
        planning_time
    );
    
    ROS_INFO("Motion parameters:");
    ROS_INFO("- Initial angle: %.2f degrees", initial_angle * 180/M_PI);
    ROS_INFO("- Opening angle: %.1f degrees", opening_angle * 180/M_PI);
    ROS_INFO("- Planning time: %.1f seconds", planning_time);
    ROS_INFO("- Number of waypoints: %d", num_points);
    
    // 5. Generate Waypoints with Smaller Steps
    for (int i = 0; i <= num_points; i++) {
        double t = static_cast<double>(i) / num_points * planning_time;
        double angle_offset = angle_trajectory.calcPoint(t);
        double current_angle = initial_angle + angle_offset;
        
        geometry_msgs::Pose waypoint;
        
        // Calculate position with adjusted radius to account for arm reach
        waypoint.position.x = adjusted_hinge_x + door_radius * std::cos(current_angle);
        waypoint.position.y = hinge_point.y + door_radius * std::sin(current_angle);
        waypoint.position.z = handle_point.z;
        
        // Maintain end-effector orientation relative to door surface
        tf2::Quaternion q_current, q_rotation, q_result;
        tf2::convert(current_pose.orientation, q_current);
        q_rotation.setRPY(0, 0, angle_offset);
        q_result = q_current * q_rotation;
        q_result.normalize();
        
        tf2::convert(q_result, waypoint.orientation);
        waypoints.push_back(waypoint);
        
        // Log key waypoints
        if (i == 0 || i == num_points/2 || i == num_points) {
            ROS_INFO("Waypoint %d (t=%.2fs): x=%.3f, y=%.3f, z=%.3f, angle=%.2f°",
                     i, t, waypoint.position.x, waypoint.position.y, waypoint.position.z,
                     (current_angle - initial_angle) * 180/M_PI);
        }
    }
    
    // 6. Improved Cartesian Path Planning
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.02;  // Increased from 0.01 to 0.02
    
    double fraction = move_group.computeCartesianPath(waypoints, 
                                                    eef_step, 
                                                    jump_threshold, 
                                                    trajectory);
    
    ROS_INFO("Trajectory generation: %.2f%% of path achieved", fraction * 100.0);
    
    // 7. More Lenient Success Criteria
    if (fraction > 0.90) {  // Reduced from 0.95 to 0.90
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        
        ROS_INFO("Executing door opening motion...");
        bool success = (move_group.execute(plan) == 
                       moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ros::Duration(1.0).sleep();
        
        if (success) {
            geometry_msgs::Pose final_pose = move_group.getCurrentPose().pose;
            ROS_INFO("Motion completed successfully");
            ROS_INFO("Final position: x=%.3f, y=%.3f, z=%.3f",
                     final_pose.position.x, final_pose.position.y, final_pose.position.z);
        }
        
        return success;
    }
    
    ROS_ERROR("Failed to compute door opening trajectory!");
    return false;
}

// X 直线运动
bool straightLineMotion(moveit::planning_interface::MoveGroupInterface& move_group)
{
    ROS_INFO("Executing straight line motion along negative X axis...");
    
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
    
    // 打印起始位置
    ROS_INFO("Start pose: x=%.3f, y=%.3f, z=%.3f", 
             start_pose.position.x, start_pose.position.y, start_pose.position.z);
             
    waypoints.push_back(start_pose);
    
    // 沿X轴负方向直线运动10cm
    const int linear_points = 10;  // 直线运动的路径点数
    for(int i = 1; i <= linear_points; i++) {
        geometry_msgs::Pose waypoint = start_pose;
        double t = static_cast<double>(i) / linear_points;
        
        // 在X轴负方向移动10cm
        waypoint.position.x = start_pose.position.x - 0.10 * t;  // 负号表示负方向，0.10表示10cm
        
        waypoints.push_back(waypoint);
        
        ROS_INFO("Linear motion waypoint %d: x=%.3f, y=%.3f, z=%.3f", 
                 i, waypoint.position.x, waypoint.position.y, waypoint.position.z);
    }
    
    // 设置运动参数
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;  // 1cm步长
    
    // 设置速度和加速度限制为2%
    move_group.setMaxVelocityScalingFactor(0.01);  // 1%速度
    move_group.setMaxAccelerationScalingFactor(0.01);  // 1%加速度
    move_group.setGoalPositionTolerance(0.01);  // 1cm位置容差
    move_group.setGoalOrientationTolerance(0.02);  // 适当的姿态容差
    
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    ROS_INFO("Straight line motion path (%.2f%% achieved)", fraction * 100.0);
    
    if(fraction > 0.95) {  // 保持95%的规划成功率要求
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        
        bool success = move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        ros::Duration(2).sleep();
        return success;
    }
    return false;
}

// 打印当前位姿
void printCurrentPose(moveit::planning_interface::MoveGroupInterface& move_group, const string& position_name)
{
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    geometry_msgs::Pose pose = current_pose.pose;
    
    ROS_INFO("Current pose at %s position:", position_name.c_str());
    ROS_INFO("Position: x=%.3f, y=%.3f, z=%.3f", 
             pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
             
    vector<double> joint_values = move_group.getCurrentJointValues();
    ROS_INFO("Joint values at %s position:", position_name.c_str());
    for(size_t i = 0; i < joint_values.size(); i++) {
        ROS_INFO("Joint %zu: %.3f", i+1, joint_values[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaka_straight_motion");
    ros::NodeHandle nh;
    
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

    // 机械臂moveit
    static const std::string PLANNING_GROUP = "jaka_zu7";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // 打印起始位姿
    ROS_INFO("=== Initial Status ===");
    printCurrentPose(move_group, "start");

    configureMovementParams(move_group);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // 1. 移动到初始位置
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    move_group.setStartStateToCurrentState();
    move_group.setMaxVelocityScalingFactor(0.1);

    // 修改初始关节角度到更合适的位置
    joint_group_positions[0] = 0.017444*94.314;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*177.217;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*56.636;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*58.394;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*91.293;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.480;      // 关节6: 0度
    
    move_group.setJointValueTarget(joint_group_positions);
    if(planAndMove(move_group, my_plan, "first")) {
        ROS_INFO("=== After Moving to First Position ===");
        printCurrentPose(move_group, "first");

        // 夹爪运动到指定位置
        // controller.setPosition(26.0);         // Set target position to 20mm
        // ros::Duration(3.0).sleep();          // 3s delay
        
        ROS_INFO("Motor control sequence completed");
    }

    // 设置门转轴位置（B点）
    geometry_msgs::Point hinge_point;
    hinge_point.x = -0.536200;
    hinge_point.y = -0.553224; 
    hinge_point.z = 0.516610;    

    // 设置门把手位置（C点）
    geometry_msgs::Point handle_point;
    handle_point.x = -0.071413;    
    handle_point.y = -0.625297;    
    handle_point.z = 0.516610;  

    //开门函数
    if(doorOpeningMotionPlanning(move_group, hinge_point, handle_point)) {
        ROS_INFO("=== After Door Opening Motion ===");
        printCurrentPose(move_group, "after_door_opening");
    } else {
        ROS_ERROR("Door opening motion failed!");
        return -1;
    }

    // 5. 返回初始位置
    move_group.setStartStateToCurrentState();
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    
    joint_group_positions[0] = 0.017444*94.314;      // 关节1: 0度
    joint_group_positions[1] = 0.017444*177.217;   // 关节2: 90度
    joint_group_positions[2] = -0.017444*56.636;  // 关节3: -90度
    joint_group_positions[3] = 0.017444*58.394;   // 关节4: 90度
    joint_group_positions[4] = -0.017444*91.293;   // 关节5: 90度
    joint_group_positions[5] = -0.017444*0.480;      // 关节6: 0度
    
    move_group.setJointValueTarget(joint_group_positions);
    if(planAndMove(move_group, my_plan, "return_to_initial")) {
        ROS_INFO("=== After Returning to Initial Position ===");
        printCurrentPose(move_group, "final");
    }

    ros::shutdown();
    return 0;
}