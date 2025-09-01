/**
 * @file scan_filter.cpp
 * @author ruofei (brf@zju.edu.cn)
 * @brief This node has two functions:  
 *      1. If a laser point hits on any robot (including itself), reset its range value as 2 * range_max
 *      2. After that, find closest laser point to this robot, as the closest obstacle point. 
 *      The positions of this obstacle point is saved in the intensity field of the filtered_scan.
 * @version 0.1
 * @date 2024-08-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <deque>
#include <cmath>
#include <limits>

std::string name_space;
ros::Publisher filtered_scan_pub;
std::vector<std::string> robot_names;

message_filters::Subscriber<sensor_msgs::LaserScan> *scan_filter_sub_1;
tf::MessageFilter<sensor_msgs::LaserScan> *scan_filter_1;

tf::TransformListener* listener;

int robot_number;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    // Initialize the filtered scan message
    sensor_msgs::LaserScan filtered_scan = *scan_in;

    // Get the robot's own transform
    tf::StampedTransform transform;
    std::vector<std::pair<float, float>> otherPositions;
    tf::Vector3 relativeTranslation;
    for (auto& otherName: robot_names) {
        if (otherName.substr(0, 7) == name_space) {
            continue;
        }
        try {
            listener->lookupTransform(name_space+"/sensor_laser", otherName + "sensor_laser", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }
        relativeTranslation = transform.getOrigin();
        otherPositions.push_back({relativeTranslation.getX(), relativeTranslation.getY()});
        // std::string targetFrame = otherName + "sensor_laser";
        // ROS_INFO("%s position: %f, %f", targetFrame.c_str(), relativeTranslation.getX(), relativeTranslation.getY());
    }

    // Loop through each scan point
    double range_window_sum = 0;
    std::deque<int> window_indices;
    
    // Send the closest obstacle point to the robot, and this will serve as pushing force.
    float closest_scan_range = std::numeric_limits<float>::infinity();
    float closest_point_x = 0, closest_point_y = 0;
    for (size_t i = 0; i < scan_in->ranges.size(); ++i) { 
        if (std::isinf(scan_in->ranges[i])) {
            continue;
        }
        float range = scan_in->ranges[i];

        // Calculate the x and y position of the scan point in the robot's frame
        float angle = scan_in->angle_min + i * scan_in->angle_increment;
        float x = range * cos(angle);
        float y = range * sin(angle);

        // Remove scan that is very close to the robot itself
        float threshold_self_hit = 0.2;    // previous 0.2 for exploration; 0.1 for iot demo
        float threshold_other_hit = 0.6;    // previous 0.6 for exploration; 0.2 for iot demo
        if (std::abs(x) < threshold_self_hit &&  std::abs(y) < threshold_self_hit)  {
            filtered_scan.ranges[i] = 2 * scan_in->range_max;
        } else {
            // Check if the point is within the robot's body
            // Adjust these thresholds based on your robot's size
            for (const auto& otherPos: otherPositions) {
                if (std::abs(otherPos.first - x) < threshold_other_hit && std::abs(otherPos.second - y) < threshold_other_hit) {
                    filtered_scan.ranges[i] = 2 * scan_in->range_max;
                    // filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
                    break;
                }
            }
        }

        if (filtered_scan.ranges[i] < closest_scan_range) {
            closest_scan_range = filtered_scan.ranges[i];
            closest_point_x = x;
            closest_point_y = y;
        }
    }
    // This is to avoid there is no obstacles in the environment
    // If so, set a far obstacle point to remove its effect
    if (std::isinf(closest_scan_range)) {  
        filtered_scan.intensities[0] = scan_in->range_max;
        filtered_scan.intensities[1] = scan_in->range_max;
    } else {
        filtered_scan.intensities[0] = closest_point_x;
        filtered_scan.intensities[1] = closest_point_y;
    }

    // Publish the filtered scan
    filtered_scan_pub.publish(filtered_scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create a transform listener
    listener = new tf::TransformListener;

    name_space = nh_private.param("namespace", std::string("robot_1"));

    robot_number = nh.param("/robot_number", 2);  // Get global parameter
    for (int i = 0; i < robot_number; i++) {
        robot_names.push_back("robot_" + std::to_string(i+1) + "/");
    }

    scan_filter_sub_1 = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "laser_scan", 5);
    scan_filter_1 = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_1, *listener, name_space+"/sensor_laser", 5);
    scan_filter_1->registerCallback(boost::bind(scanCallback, _1));

    // ros::Subscriber scan_sub = nh.subscribe("base_scan", 1000, scanCallback);
    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("filter_scan", 1000);


    ros::spin();

    // 清理
    delete scan_filter_sub_1;
    delete scan_filter_1;

    return 0;
}
