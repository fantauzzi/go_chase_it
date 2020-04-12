#include <tgmath.h>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>


class ProcessImage {
    void drive(double linear_speed, double angular_speed) {
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = linear_speed;
        srv.request.angular_z = angular_speed;
        auto ok = client.call(srv);
        if (!ok)
            ROS_ERROR("Failed to call service DriveToTarget");
    }

public:
    ProcessImage(const ros::NodeHandle &node_handle) : ROS_node(node_handle) {
        client = ROS_node.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        subscriber = ROS_node.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
    }

    bool is_pixel_of_color(const std::vector<unsigned char> & data, int width, int row, int col, int color[3]) {
        for (int i = 0; i < 3; ++i)
            if (data[width * row * 3 + col * 3 + i] != color[i])
                return false;
        return true;
    }

    void process_image_callback(const sensor_msgs::Image img) {

        int target_color[] = {255, 255, 255};
        double max_linear_speed = .25;
        double max_angular_speed = .5;

        /* Find the proportion of pixels with the target color in each third of the image, between the central third,
         * the right third and the left third. */

        // Partition the image in three vertical bands of (roughly) the same width
        int w = img.width;
        int h = img.height;
        int third_1 = static_cast<int>(round(w / 3.));
        int third_2 = static_cast<int>(round(w * 2 / 3.));

        // Count the number of pixels with target color in each vertical band (third of the image)
        int count_left = 0;
        int count_middle = 0;
        int count_right = 0;
        for (int row = 0; row < h; ++row) {
            for (int col = 0; col < third_1; ++col)
                count_left += is_pixel_of_color(img.data, w, row, col, target_color);
            for (int col = third_1; col < third_2; ++col)
                count_middle += is_pixel_of_color(img.data, w, row, col, target_color);
            for (int col = third_2; col < w; ++col)
                count_right += is_pixel_of_color(img.data, w, row, col, target_color);
        }
        std::cout << "White pixels count: " << count_left << " " << count_middle << " " << count_right << std::endl;

        /* Set the linear and angular speed of the robot based on the amount of pixels with target color in each
         * vertical band. If they are all in the middle band, then drive forward, angular speed is 0; if they are
         * all in the right or left band, drive in that direction; if they are nowhere to be found, stop the robot */

        double total = count_left + count_middle + count_right;
        if (total == 0 || (count_middle>=.98*(third_2-third_1)*h))
            drive(0, 0);
        else {
            double angular_speed = - max_angular_speed * count_right / total + max_angular_speed * count_left / total;
            double linear_speed = max_linear_speed * count_middle / total;
            drive(linear_speed, angular_speed);
        }
    }

private:
    ros::NodeHandle ROS_node;
    ros::ServiceClient client;
    ros::Subscriber subscriber;
};


int main(int argc, char **argv) {
    // Boot the ROS node
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // The object with the node implementation
    auto processImage = ProcessImage(n);
    ROS_INFO("Ready to receive images and drive the robot");

    // Get to work!
    ros::spin();

    return 0;
}