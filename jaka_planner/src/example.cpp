#include "motor_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "motor_controller_example");
    ros::NodeHandle nh;
    
    // Create motor controller instance
    MotorController controller("/dev/ttyUSB0", 115200);
    ROS_INFO("Initializing serial port: 115200-8-N-1");
    
    if (!controller.init()) {
        ROS_ERROR("Failed to initialize motor controller");
        return -1;
    }
    
    // Example: Execute a complete position control sequence with delays
    ROS_INFO("Starting position control sequence...");

    controller.setIOStop(false);          // Start IO
    ros::Duration(0.3).sleep();          // 100ms delay
    
    controller.setIOStop(true);           // Stop IO
    ros::Duration(0.3).sleep();          // 100ms delay
    
    controller.setTorque(10.0);           // Set torque
    ros::Duration(0.3).sleep();          // 100ms delay
    
    controller.setAcceleration(30.0);     // Set acceleration
    ros::Duration(0.3).sleep();          // 100ms delay
    
    controller.setVelocity(20.0);         // Set velocity
    ros::Duration(0.3).sleep();          // 100ms delay
    
    controller.setPosition(0.0);         // Set target position
    ros::Duration(5).sleep();          // 100ms delay

    controller.setPosition(20.0);         // Set target position
    ros::Duration(5).sleep();          // 100ms delay

    controller.setPosition(0.0);         // Set target position
    ros::Duration(5).sleep();          // 100ms delay

    controller.setPosition(15.0);         // Set target position
    ros::Duration(5).sleep();          // 100ms delay
    
    ros::spin();
    
    return 0;
}

