#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveToTarget {

public:
    /**
     * Instantiates the implementation for the ROS node that drives toward the white ball.
     * @param node_handle Handle to the ROS node.
     */
    explicit DriveToTarget(const ros::NodeHandle &node_handle) : ros_node(node_handle) {
        motor_command_publisher = ros_node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        command_robot_service = ros_node.advertiseService("/ball_chaser/command_robot", &DriveToTarget::operator(), this);
    }

    /**
     * Call-back that receives and executes the /ball_chaser/command_robot service requests.
     * @param req The service request.
     * @param res The service response.
     * @return True.
     */
    bool operator()(ball_chaser::DriveToTarget::Request &req,
                ball_chaser::DriveToTarget::Response &res) {
        geometry_msgs::Twist motor_command;
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;
        motor_command_publisher.publish(motor_command);
        res.msg_feedback = "Motor command set with linear_x=" + std::to_string(req.linear_x) + " angular_x=" +
                           std::to_string(req.angular_z);
        ROS_DEBUG("Motor command set with linear_x=%f angular_x=%f",motor_command.linear.x, motor_command.angular.z);

        return true;
    }

private:
    ros::NodeHandle ros_node;
    ros::Publisher motor_command_publisher;
    ros::ServiceServer command_robot_service;
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bot");

    // Create the node implementation
    ros::NodeHandle node;
    DriveToTarget drive_to_target(node);

    ROS_INFO("Ready to receive drive commands.");

    // Get to work!
    ros::spin();

    return 0;
}
