#include <ctgmath>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>


class ProcessImage {
    /**
     * Directs the DriveToTarget service to set the robot speed. Logs an error message if the setting fails.
     * @param linear_speed The linear speed (m/s)
     * @param angular_speed The angular speed (rad/s); positive is counter-clockwise as seen from above,
     * negative is clockwise.
     */
    void drive(double linear_speed, double angular_speed) {
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = linear_speed;
        srv.request.angular_z = angular_speed;
        auto ok = client.call(srv);
        if (!ok)
            ROS_ERROR("Failed to call service DriveToTarget.");
    }

    /**
     * Checks if the pixel of an image at given coordinates has a given color.
     * @param image The image, encoded as RGB8.
     * @param width The image horizontal resolution, in pixels.
     * @param row The pixel row coordinate.
     * @param col The pixel column coordinate.
     * @param color The given color.
     * @return True iff the pixel at the given coordinates has the given color.
     */
    static bool is_pixel_of_color(const std::vector<unsigned char> &image, int width, int row, int col,
                                  const std::vector<unsigned char> &color) {
        for (int i = 0; i < 3; ++i)
            if (image[width * row * 3 + col * 3 + i] != color[i])
                return false;
        return true;
    }

public:
    /**
     * Instantiates the implementation for the ROS node. Receives images from topic /camera/rgb/image_raw, process them
     * and invokes the /ball_chaser/command_robot service as needed.
     * @param node_handle The handle to the ROS node.
     */
    explicit ProcessImage(const ros::NodeHandle &node_handle) : ros_node(node_handle) {
        client = ros_node.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        subscriber = ros_node.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
    }

    void process_image_callback(const sensor_msgs::Image &img) {

        std::vector<unsigned char> target_color = {255, 255, 255};
        double max_linear_speed = .25;
        double max_angular_speed = .5;

        /* Find the proportion of pixels with the target color in each of three vertical bands the image is partitioned
         * into. The central band takes 10% of the image pixels, the left and right band 45% each. */

        int w = img.width;
        int h = img.height;
        // Set the beginning and end of the central vertical band, relative to image width
        int third_1 = static_cast<int>(round(w * .45));
        int third_2 = static_cast<int>(round(w * .55));

        // Count the number of pixels with target color in each vertical band
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
        ROS_DEBUG("Target color pixels count: %i %i %i.", count_left, count_middle, count_right);

        /* Set the linear and angular speed of the robot based on the amount of pixels with target color in each
         * vertical band. Steer to either side proportionately to the amount of pixels with target color in either
         * sides. Stop if pixels with target color is found anywhere, or if they fill most of the middle band
         * (indicating the robot is about to hit the ball). */

        double total = count_left + count_middle + count_right;
        if (total == 0)
            drive(0, 0);
        else {
            double angular_speed = -max_angular_speed * count_right / total + max_angular_speed * count_left / total;
            double middle_white_ratio = static_cast<double>(count_middle) / ((third_2 - third_1) * h);
            double linear_speed = (middle_white_ratio < .95) ? max_linear_speed * count_middle / total : .0;
            drive(linear_speed, angular_speed);
        }
    }

private:
    ros::NodeHandle ros_node;
    ros::ServiceClient client;
    //Subscriber is held here even if never accessed, because if it goes out of scope, the ROS subscriber is removed
    ros::Subscriber subscriber;
};


int main(int argc, char **argv) {
    // Boot the ROS node
    ros::init(argc, argv, "process_image");
    ros::NodeHandle node;

    // The object with the node implementation
    ProcessImage processImage(node);
    ROS_INFO("Ready to receive images and drive the robot.");

    // Get to work!
    ros::spin();

    return 0;
}

