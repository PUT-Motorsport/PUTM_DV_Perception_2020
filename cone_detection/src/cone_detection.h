/*
    Header file with definitions for cone detection algorithm
*/
#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>

// Variables tweaking the algorithm
// Maximum amount of clouds filtered at once
#define QUEUE_SIZE 2
// Topic where clouds are published
#define CLOUD_TOPIC "/cloud"
// Topic where filtered clouds will be published
#define OUTPUT_TOPIC "/cloud/filtered"
// Name of the ros node
#define NODE_NAME "listener"

// Sizes of filtering box for restricting FOV
#define FOV_X_MIN 0
#define FOV_X_MAX 1000
#define FOV_Y_MIN 0
#define FOV_Y_MAX 1000
#define FOV_Z_MIN 0
#define FOV_Z_MAX 1000

class ConeDetector {
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;

        sensor_msgs::PointCloud2 outputMsg;

        // Callback function called when new cloud is being published
        void chatterCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
        // Function returning input cloud with restricted FOV
        void filterFOV(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr &output);

    public:
        ConeDetector(): nh("~") {
            pub = nh.advertise<sensor_msgs::PointCloud2>(OUTPUT_TOPIC, 1);
            sub = nh.subscribe<sensor_msgs::PointCloud2>(CLOUD_TOPIC, QUEUE_SIZE, &ConeDetector::chatterCallback, this);
        }
};

#endif
