#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

ros::Subscriber depth_points_sub;
ros::Publisher surface_captured_pub;
ros::Publisher surface_accumulated_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

double max_cutoff_distance = 1.09; // Set your max cutoff distance here (Zivid Sensor has 1.1 m)

void depthPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert the ROS PointCloud2 message to a PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Filter out points beyond the max cutoff distance
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, max_cutoff_distance); // Keeping only points within the max cutoff distance
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*filtered_cloud);

    // Lookup the transform from camera_link_optical to base_link
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("base_link", "camera_link_optical", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    // Convert geometry_msgs::TransformStamped to Eigen::Affine3d
    Eigen::Affine3d transform = tf2::transformToEigen(transformStamped.transform);

    // Apply the transformation to the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, transform);

    // Add the new transformed cloud to the accumulated cloud
    *current_cloud = *transformed_cloud;
}

bool addPointCloud(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Assuming the latest point cloud has been received through the callback
    res.success = true;
    res.message = "Point cloud added to the accumulated surface.";

    // Publish the current Measurement
    sensor_msgs::PointCloud2 current;
    pcl::toROSMsg(*current_cloud, current);
    current.header.frame_id = "base_link"; // Set the appropriate frame
    surface_captured_pub.publish(current);

    // Publish the accumulated point cloud
    *accumulated_cloud += *current_cloud;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*accumulated_cloud, output);
    output.header.frame_id = "base_link"; // Set the appropriate frame
    surface_accumulated_pub.publish(output);

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_accumulator");
    ros::NodeHandle nh;

    // Initialize tfBuffer and tfListener
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Subscribe to the depth points topic
    depth_points_sub = nh.subscribe("/depth/points", 1, depthPointsCallback);

    // Advertise the accumulated point cloud topic
    surface_captured_pub = nh.advertise<sensor_msgs::PointCloud2>("current_measurement", 1);
    surface_accumulated_pub = nh.advertise<sensor_msgs::PointCloud2>("surface_accumulated", 1);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("capture_surface", addPointCloud);

    ros::spin();
    return 0;
}
