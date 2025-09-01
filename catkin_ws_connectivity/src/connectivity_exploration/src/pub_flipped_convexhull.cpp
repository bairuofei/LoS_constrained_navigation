/**
 * @file pubSCP.cpp
 * @author ruofei (brf@zju.edu.cn)
 * @brief Subscribe laserScan, derive visibility constraints, take derivative, and then move following robot.
 * This file also visualize robot_2 before and after flipping w.r.t. robot_1, for debugging.
 * @version 0.1
 * @date 2024-07-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h> // 包含Float32消息类型
#include <gazebo_msgs/ModelStates.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Eigen>

std::string name_space;

float visiableRange; // Used to add augmented points into raw pointCloud
float flipRadius;    // Radius for spherical flipping
float testPointX, testPointY;
float alpha; // Coefficient in log-exp-sum relaxation

float flipD;     // Safe distance (to convexHull) to ignore visible constraints
float flipD_min; // Mimimum allowed distance to convexHull to maintain visible
float kS;        // Constant of the weight function

std::vector<float> robot_color; // The color to show visible region

geometry_msgs::PointStamped testPointInOdom;
tf2_ros::Buffer tfBuffer;

std::mutex testPointMutex;
cv::Point2f testPoint;

ros::Publisher pubScpBoundary;
ros::Publisher pubScpVertices;
ros::Publisher pubReflectPoints;
ros::Publisher pubAugmentPoints;
ros::Publisher pubMaxDistance;
ros::Publisher pubTestMarker;
ros::Publisher pubConvexhullVertices;

/**
 * @brief Perform spherical flipping operation.
 *
 * @param flippedPoint
 * @return cv::Point2f
 */
cv::Point2f sphericalFlipping(const cv::Point2f &flippedPoint, const double flipRadius)
{
    float norm = cv::norm(flippedPoint);
    // 映射函数 p' = 2R - p
    float newX = 2 * flipRadius * flippedPoint.x / norm - flippedPoint.x;
    float newY = 2 * flipRadius * flippedPoint.y / norm - flippedPoint.y;
    return cv::Point2f{newX, newY};
}

/**
 * @brief Use log-exp-sum relaxation to find the maximum distance in the vector.
 *
 * @param distances
 * @return double
 */
double findMaximum(const std::vector<float> &distances)
{
    double sumAll = 0;
    for (float dist : distances)
    {
        sumAll += std::exp(alpha * dist);
    }
    return std::log(sumAll) / alpha;
}

// Interpolate two points according to the given step
std::vector<cv::Point2f> interpolatePoints(const cv::Point2f &pt1, const cv::Point2f &pt2, float step)
{
    std::vector<cv::Point2f> points;
    points.push_back(pt1);

    float distance = cv::norm(pt2 - pt1);
    if (distance <= step)
    {
        points.push_back(pt2);
        return points;
    }
    cv::Point2f direction = (pt2 - pt1) / distance;
    float numSteps = std::floor(distance / step);

    for (int i = 1; i <= numSteps; ++i)
    {
        cv::Point2f newPoint = pt1 + direction * step * i;
        points.push_back(newPoint);
    }
    points.push_back(pt2);
    return points;
}

void visualizeAugmentedPointCloud(const std::vector<Eigen::Vector2f> &data)
{
    visualization_msgs::Marker pointMarker1;
    pointMarker1.header.frame_id = name_space + "/sensor_laser";
    pointMarker1.header.stamp = ros::Time::now();
    pointMarker1.ns = "points";
    pointMarker1.id = 3;
    pointMarker1.type = visualization_msgs::Marker::POINTS;
    pointMarker1.action = visualization_msgs::Marker::ADD;
    pointMarker1.pose.orientation.w = 1.0;
    pointMarker1.scale.x = 0.2; // 点的大小
    pointMarker1.scale.y = 0.2; // 点的大小
    pointMarker1.color.r = 1.0;
    pointMarker1.color.g = 1.0;
    pointMarker1.color.b = 0.2;
    pointMarker1.color.a = 1.0;
    for (const auto &point : data)
    {
        geometry_msgs::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0.0;
        pointMarker1.points.push_back(p);
    }
    pubAugmentPoints.publish(pointMarker1);
}

void visualizeVisibleRegionVertices(const std::vector<cv::Point2f> &vertexData)
{
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = name_space + "/sensor_laser";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "points";
    pointMarker.id = 0;
    pointMarker.type = visualization_msgs::Marker::POINTS;
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.scale.x = 0.2; // 点的大小
    pointMarker.scale.y = 0.2; // 点的大小
    pointMarker.color.r = 0.5;
    pointMarker.color.g = 0.5;
    pointMarker.color.b = 0.0;
    pointMarker.color.a = 0.7;
    for (const auto &point : vertexData)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        pointMarker.points.push_back(p);
    }
    pubScpVertices.publish(pointMarker);
}

void visualizeVisibleRegionBoundary(const std::vector<cv::Point2f> &flipVertexData)
{
    visualization_msgs::Marker lineMarker;
    lineMarker.header.frame_id = name_space + "/sensor_laser";
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.ns = "lines";
    lineMarker.id = 1;
    lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
    lineMarker.action = visualization_msgs::Marker::ADD;
    lineMarker.pose.orientation.w = 1.0;
    lineMarker.scale.x = 0.15; // default 0.1, linewidth  0.05; 0.1用于exploration更清晰； 0.02 for iot demo
    lineMarker.color.r = robot_color[0];
    lineMarker.color.g = robot_color[1];
    lineMarker.color.b = robot_color[2];
    lineMarker.color.a = 1; // 0.6
    float insert_step = 0.5;
    for (int i = 0; i < flipVertexData.size(); i++)
    {
        int i_next = (i + 1) % flipVertexData.size();
        std::vector<cv::Point2f> interpolateEdge = interpolatePoints(flipVertexData[i], flipVertexData[i_next], insert_step);
        // Pop last element
        interpolateEdge.pop_back();
        for (const cv::Point2f &point : interpolateEdge)
        {
            cv::Point2f innerPoint = sphericalFlipping(point, flipRadius);
            geometry_msgs::Point p;
            p.x = innerPoint.x;
            p.y = innerPoint.y;
            p.z = 0.0;
            lineMarker.points.push_back(p);
        }
    }
    pubScpBoundary.publish(lineMarker);
}

void visualizeTestPoints(const cv::Point2f &testPoint, const cv::Point2f &flipTestPoint)
{
    // 可视化testPoint和flipTestPoint到base_laser_link坐标系下
    visualization_msgs::Marker testMarker;
    testMarker.header.frame_id = name_space + "/sensor_laser";
    testMarker.header.stamp = ros::Time::now();
    testMarker.ns = "points";
    testMarker.id = 10;
    testMarker.type = visualization_msgs::Marker::POINTS;
    testMarker.action = visualization_msgs::Marker::ADD;
    testMarker.pose.orientation.w = 1.0;
    testMarker.scale.x = 0.8; // 点的大小
    testMarker.scale.y = 0.8; // 点的大小
    testMarker.color.r = 1;
    testMarker.color.g = 0.0;
    testMarker.color.b = 0.0;
    testMarker.color.a = 1.0;

    geometry_msgs::Point p1;
    p1.x = testPoint.x;
    p1.y = testPoint.y;
    p1.z = 0.0;
    testMarker.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = flipTestPoint.x;
    p2.y = flipTestPoint.y;
    p2.z = 0.0;
    testMarker.points.push_back(p2);
    pubTestMarker.publish(testMarker);
}

/**
 * @brief Take rawPoints, output flipped points and vertices of the flipped convex hull
 *
 * @param rawPoints the list of raw laser points
 * @param vertexData the list of raw laser points that corresponds to vertices of the flipped convexhull
 * @param flippedConvexVertex the list of vertices of the flipped convexhull
 */
void fromDataToSCP(std::vector<Eigen::Vector2f> &rawPoints, std::vector<cv::Point2f> &vertexData, std::vector<cv::Point2f> &flippedConvexVertex)
{
    // Flip all data in the original pointcloud,翻转后，最外部的点，对应翻转前距离原点最近的点
    std::vector<cv::Point2f> flipData(rawPoints.size(), cv::Point2f(0, 0)); // flipData里面包含了(0, 0)
    for (size_t i = 0; i < rawPoints.size(); i++)
    {
        cv::Point2f data_i{rawPoints[i](0), rawPoints[i](1)};
        flipData[i] = sphericalFlipping(data_i, flipRadius);
    }

    std::vector<int> vertexIndice; // 下标集合，指向构成convexHull顶点的点的下标
    // Return hull vertex indices in counter-clockwise order
    cv::convexHull(flipData, vertexIndice, false, false);

    // vertexData 保存convexHull顶点对应的原数据点，即starConvexRegion的顶点
    for (size_t i = 0; i < vertexIndice.size(); i++)
    {
        int v = vertexIndice[i]; // convexHull对应的下标
        vertexData.push_back(cv::Point2f(rawPoints[v](0), rawPoints[v](1)));
        flippedConvexVertex.push_back(cv::Point2f(flipData[v].x, flipData[v].y));
    }

    // Visualize flipped points
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = name_space + "/sensor_laser";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "points";
    pointMarker.id = 2;
    pointMarker.type = visualization_msgs::Marker::POINTS;
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.scale.x = 0.5; // 点的大小
    pointMarker.scale.y = 0.5; // 点的大小
    pointMarker.color.r = 0.2;
    pointMarker.color.g = 0.5;
    pointMarker.color.b = 0.0;
    pointMarker.color.a = 0.6;
    for (const auto &point : flipData)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        pointMarker.points.push_back(p);
    }

    // Visualize flipped convexHull
    visualization_msgs::Marker lineMarker;
    lineMarker.header.frame_id = name_space + "/sensor_laser";
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.ns = "lines";
    lineMarker.id = 33;
    lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
    lineMarker.action = visualization_msgs::Marker::ADD;
    lineMarker.pose.orientation.w = 1.0;
    lineMarker.scale.x = 0.5; // 线的宽度
    lineMarker.color.r = 0.0;
    lineMarker.color.g = 0.5;
    lineMarker.color.b = 1.0;
    lineMarker.color.a = 0.3;
    for (int i = 0; i <= vertexIndice.size(); i++)
    {
        int real_i = i % vertexIndice.size();
        geometry_msgs::Point p;
        p.x = flipData[vertexIndice[real_i]].x;
        p.y = flipData[vertexIndice[real_i]].y;
        p.z = 0.0;
        lineMarker.points.push_back(p);
    }

    pubReflectPoints.publish(pointMarker);
    pubReflectPoints.publish(lineMarker);
    return;
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    // Find the index of robot_2 in the ModelStates message
    auto it = std::find(msg->name.begin(), msg->name.end(), "robot_2");
    if (it != msg->name.end())
    {
        int index = std::distance(msg->name.begin(), it); // 计算两个迭代器之间的距离
        geometry_msgs::Pose robot2_pose = msg->pose[index];
        testPointMutex.lock();
        testPoint.x = robot2_pose.position.x;
        testPoint.y = robot2_pose.position.y;
        testPointMutex.unlock();
    }
    else
    {
        ROS_WARN("robot_2 not found in the model states.");
    }
}

void ScanHandler(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    ROS_DEBUG("Range nums: %d", int(scan->ranges.size())); // 901

    float angleMin = scan->angle_min;
    float angleMax = scan->angle_max;
    float angleIncre = scan->angle_increment;
    float rangeMax = scan->range_max;

    float currAngle = angleMin;
    std::vector<Eigen::Vector2f> data;
    for (float dist : scan->ranges)
    { // transform laserPoint into robot's local coordinate frame
        if (dist == 2 * rangeMax)
        { // Omit laser points that hit on the robot's body
            currAngle += angleIncre;
            continue;
        }
        else if (dist > 2 * rangeMax)
        { // these are the rays do not hit anything
            dist = std::max(visiableRange, rangeMax);
        }
        float px = dist * std::cos(currAngle);
        float py = dist * std::sin(currAngle);
        data.push_back(Eigen::Vector2f(px, py));
        currAngle += angleIncre;
    }

    std::vector<cv::Point2f> vertexData;
    std::vector<cv::Point2f> flipVertexData;
    fromDataToSCP(data, vertexData, flipVertexData);

    // Visualization of the flipped points
    visualizeAugmentedPointCloud(data);
    visualizeVisibleRegionVertices(vertexData);
    visualizeVisibleRegionBoundary(flipVertexData);

    // testPoint is in odom frame; SCP constraints are in master robot's local frame
    if (name_space == "robot_1")
    {
        geometry_msgs::PointStamped testPointInOdom;
        testPointInOdom.header.frame_id = "odom";
        testPointInOdom.header.stamp = ros::Time::now();
        testPointMutex.lock();
        testPointInOdom.point.x = testPoint.x;
        testPointInOdom.point.y = testPoint.y;
        testPointInOdom.point.z = 0;
        testPointMutex.unlock();

        geometry_msgs::PointStamped testPointInBaselink;
        try
        {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(name_space + "/sensor_laser", "odom", ros::Time(0), ros::Duration(3.0));
            tf2::doTransform(testPointInOdom, testPointInBaselink, transformStamped);
            ROS_DEBUG("odom: (%.2f, %.2f, %.2f) -----> base_laser_link: (%.2f, %.2f, %.2f) at time %.2f",
                      testPointInOdom.point.x, testPointInOdom.point.y, testPointInOdom.point.z,
                      testPointInBaselink.point.x, testPointInBaselink.point.y, testPointInBaselink.point.z,
                      testPointInBaselink.header.stamp.toSec());
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_laser_link\": %s", ex.what());
        }
        cv::Point2f testPointLocal{testPointInBaselink.point.x, testPointInBaselink.point.y};
        cv::Point2f flipTestPoint = sphericalFlipping(testPointLocal, flipRadius);
        visualizeTestPoints(testPointLocal, flipTestPoint);
    }

    // Broadcast visible convexhull to other ROS nodes
    std::vector<float> flipVertexDataArray;
    for (const cv::Point2f &point : flipVertexData)
    {
        flipVertexDataArray.push_back(point.x);
        flipVertexDataArray.push_back(point.y);
    }
    // Add closest laser point to the robot convexhull data structure, for reuse of the callback function
    flipVertexDataArray.push_back(scan->intensities[0]);
    flipVertexDataArray.push_back(scan->intensities[1]);
    // 创建 Float32MultiArray 消息
    std_msgs::Float32MultiArray msg;
    msg.data = flipVertexDataArray;
    pubConvexhullVertices.publish(msg);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_flipped_convexhull");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::param::get("/flip_radius", flipRadius);
    ros::param::get("/visiable_range", visiableRange);
    ROS_INFO("SCP radius: %f, visiable range: %f", flipRadius, visiableRange);

    ros::param::get("/d_los_max", flipD);
    ros::param::get("/d_los_min", flipD_min);
    ros::param::get("/k_omega", kS);

    float color_r = nh_private.param("color_r", 1.0);
    float color_g = nh_private.param("color_g", 1.0);
    float color_b = nh_private.param("color_b", 1.0);
    robot_color.assign({color_r, color_g, color_b});
    ROS_INFO("Set robot color: %f(r), %f(g), %f(b)", robot_color[0], robot_color[1], robot_color[2]);

    name_space = nh_private.param("namespace", std::string("robot_1"));
    ROS_INFO("Add name_space: %s", name_space.c_str());

    ros::Subscriber subScan = nh.subscribe<sensor_msgs::LaserScan>("filter_scan", 10, ScanHandler);
    ros::Subscriber subModelState = nh.subscribe("/gazebo/model_states", 100, modelStatesCallback);

    pubScpBoundary = nh.advertise<visualization_msgs::Marker>("scp_boundary", 2);
    pubScpVertices = nh.advertise<visualization_msgs::Marker>("scp_vertices", 2);
    pubReflectPoints = nh.advertise<visualization_msgs::Marker>("scp_reflectPoints", 2);
    pubAugmentPoints = nh.advertise<visualization_msgs::Marker>("scp_augment", 2);
    pubTestMarker = nh.advertise<visualization_msgs::Marker>("test_points", 2);
    pubMaxDistance = nh.advertise<std_msgs::Float32>("max_distance", 2);

    // publish ordered vertices of flipped convexHull
    pubConvexhullVertices = nh.advertise<std_msgs::Float32MultiArray>("vertices_convexhull", 2);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Duration(5.0).sleep(); // 确保变换缓冲区已经填充

    ros::spin();
}
