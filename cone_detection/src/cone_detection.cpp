#include "cone_detection.h"

void ConeDetector::filterFOV(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr &output) {
    /* 
        Filter out points in cloud outside of a box with size specified by parameters
        set by the user.
    */
    pcl::CropBox<pcl::PointXYZI> cropBoxFilter(true);
    cropBoxFilter.setInputCloud(input);
    cropBoxFilter.setMin(Eigen::Vector4f (FOV_X_MIN, FOV_Y_MIN, FOV_Z_MIN, 1));
    cropBoxFilter.setMax(Eigen::Vector4f (FOV_X_MAX, FOV_Y_MAX, FOV_Z_MAX, 1));
    cropBoxFilter.filter(*output);
}

void ConeDetector::chatterCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::fromROSMsg(*cloudMsg, *input);

    filterFOV(input, output);

    pcl::toROSMsg(*output, outputMsg);
    outputMsg.header = cloudMsg->header;
    outputMsg.fields = cloudMsg->fields;
    pub.publish(outputMsg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ConeDetector detector;
    ROS_INFO("Ready to filter LIDAR");
    ros::spin();
    return 0;
}
