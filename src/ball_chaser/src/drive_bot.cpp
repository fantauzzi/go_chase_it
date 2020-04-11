#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//DONE: Include the ball_chaser "DriveToTarget" header file

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities


class DriveToTarget {

public:
    DriveToTarget(const ros::NodeHandle &node_handle) : ROS_node(node_handle) {
        motor_command_publisher = ROS_node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        service = ROS_node.advertiseService("/ball_chaser/command_robot", &DriveToTarget::ciccio, this);
    }

    bool ciccio(ball_chaser::DriveToTarget::Request &req,
                ball_chaser::DriveToTarget::Response &res) {
        geometry_msgs::Twist motor_command;
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;
        // Publish angles to drive the robot
        motor_command_publisher.publish(motor_command);
        res.msg_feedback = "motor command set with linear_x=" + std::to_string(req.linear_x) + " angular_x=" +
                           std::to_string(req.angular_z);
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }

private:
    ros::NodeHandle ROS_node;
    ros::Publisher motor_command_publisher;
    ros::ServiceServer service;
};

int main(int argc, char **argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    auto drive_to_target = DriveToTarget(n);

    // DONE: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ROS_INFO("Ready to receive drive commands");

    // DONE: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    ros::spin();
    // DONE: Handle ROS communication events

    return 0;
}

/* TODO:
 * give ciccio() a better name, perhaps use an operator()
 * check the warning on DriveToTarget
 * non-systematic issue with plug-in
*/