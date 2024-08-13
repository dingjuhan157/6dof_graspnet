#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_state/conversions.h>

class JakaPlanner {
public:
    explicit JakaPlanner(ros::NodeHandle& nh) 
        : nh_(nh), move_group_(PLANNING_GROUP), has_executed_object_pose_(false), 
          tfBuffer_(), tfListener_(tfBuffer_) {
        joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        ROS_INFO("JakaPlanner initialized.");
        
        ROS_INFO_STREAM("Planning frame: " << move_group_.getPlanningFrame());
        ROS_INFO_STREAM("End effector link: " << move_group_.getEndEffectorLink());

        // Print current robot state
        printRobotState();

        // Set planning time
        move_group_.setPlanningTime(5.0); // Set maximum planning time to 5 seconds

        // Perform a test motion
        testMotion();
    }

    void run() {
        object_pose_sub_ = nh_.subscribe("/yolov5/object_pose", 1, &JakaPlanner::objectPoseCallback, this);
        ROS_INFO("Subscribed to /yolov5/object_pose topic, waiting for object pose...");
    }

private:
    void printRobotState() {
        moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
        std::vector<double> joint_values;
        current_state->copyJointGroupPositions(joint_model_group_, joint_values);
        for (std::size_t i = 0; i < joint_model_group_->getVariableNames().size(); ++i) {
            ROS_INFO("Joint %s: %f", joint_model_group_->getVariableNames()[i].c_str(), joint_values[i]);
        }
    }

    void testMotion() {
        ROS_INFO("Performing test motion...");
        move_group_.setNamedTarget("home");
        
        moveit::planning_interface::MoveGroupInterface::Plan test_plan;
        bool success = (move_group_.plan(test_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            ROS_INFO("Test motion planning succeeded.");
            move_group_.execute(test_plan);
            ROS_INFO("Test motion execution completed.");
        } else {
            ROS_ERROR("Test motion planning failed.");
        }
    }

    void objectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!has_executed_object_pose_) {
            ROS_INFO("Received object pose, starting processing");
            ROS_INFO_STREAM("Received pose in frame: " << msg->header.frame_id);
            
            geometry_msgs::PoseStamped target_pose_world;
            if (transformPose(*msg, target_pose_world, "world")) {
                ROS_INFO("Successfully transformed pose to world frame");
                geometry_msgs::PoseStamped target_pose_base;
                if (transformPose(target_pose_world, target_pose_base, move_group_.getPlanningFrame())) {
                    ROS_INFO_STREAM("Target pose: " << target_pose_base.pose.position.x << ", " 
                                    << target_pose_base.pose.position.y << ", " 
                                    << target_pose_base.pose.position.z);
                    
                    // Check if the target is within the workspace
                    if (isTargetWithinWorkspace(target_pose_base.pose)) {
                        if (planAndExecute(target_pose_base.pose)) {
                            ROS_INFO("Planning and execution completed.");
                            has_executed_object_pose_ = true;
                            object_pose_sub_.shutdown();
                            ROS_INFO("Unsubscribed from /yolov5/object_pose topic.");
                        } else {
                            ROS_WARN("Planning or execution failed.");
                        }
                    } else {
                        ROS_ERROR("Target pose is outside the robot's workspace.");
                    }
                } else {
                    ROS_ERROR("Failed to transform pose to planning frame");
                }
            } else {
                ROS_ERROR("Failed to transform pose to world frame");
            }
        }
    }

    bool isTargetWithinWorkspace(const geometry_msgs::Pose& target_pose) {
        // This is a very simple check. You should implement a more sophisticated check based on your robot's actual workspace.
        double distance = std::sqrt(std::pow(target_pose.position.x, 2) + 
                                    std::pow(target_pose.position.y, 2) + 
                                    std::pow(target_pose.position.z, 2));
        // Assuming the robot's maximum reach is 1 meter. Adjust this value based on your robot's specifications.
        return distance <= 1.0;
    }

    bool transformPose(const geometry_msgs::PoseStamped& input_pose, 
                       geometry_msgs::PoseStamped& output_pose, 
                       const std::string& target_frame) {
        for (int attempt = 0; attempt < 5; ++attempt) {
            try {
                geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(
                    target_frame, input_pose.header.frame_id, ros::Time(0));
                
                tf2::doTransform(input_pose, output_pose, transform);
                output_pose.header.frame_id = target_frame;
                
                ROS_INFO_STREAM("Transform attempt " << attempt + 1 << " succeeded");
                return true;
            } catch (tf2::TransformException &ex) {
                ROS_WARN_STREAM("Transform attempt " << attempt + 1 << " failed: " << ex.what());
                if (attempt == 4) {
                    ROS_ERROR("All transform attempts failed.");
                    return false;
                }
                ros::Duration(0.1).sleep();
            }
        }
        return false;
    }

    bool planAndExecute(const geometry_msgs::Pose& target_pose) {
        ROS_INFO("Setting pose target");
        move_group_.setPoseTarget(target_pose);
        
        ROS_INFO("Starting to plan");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        ROS_INFO("Calling plan() with timeout");
        ros::Time start_time = ros::Time::now();
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(my_plan);
        ros::Time end_time = ros::Time::now();
        ros::Duration planning_duration = end_time - start_time;
        
        ROS_INFO_STREAM("Planning took " << planning_duration.toSec() << " seconds");
        
        if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Planning succeeded, starting execution");
            moveit::planning_interface::MoveItErrorCode execute_result = move_group_.execute(my_plan);
            
            if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_INFO("Execution succeeded");
                return true;
            } else {
                ROS_ERROR_STREAM("Execution failed with error code: " << execute_result.val);
                return false;
            }
        } else {
            ROS_ERROR_STREAM("Planning failed with error code: " << plan_result.val);
            return false;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber object_pose_sub_;
    static const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group_;
    const robot_state::JointModelGroup* joint_model_group_;
    bool has_executed_object_pose_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

const std::string JakaPlanner::PLANNING_GROUP = "jaka_zu7";

int main(int argc, char **argv) {
    ros::init(argc, argv, "jaka_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    JakaPlanner planner(nh);
    planner.run();

    ros::waitForShutdown();
    return 0;
}