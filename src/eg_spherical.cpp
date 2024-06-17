#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>
#include <viewpoint_planning/LoadMesh.h> 
#include <std_srvs/Trigger.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "eg_spherical");
    ros::NodeHandle nh;

    std::string model_name = "structured_light_sensor_robot";
    std::string link_name = "sensor_link";
    // Create a client for the set_model_state service
    ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //initial pose of sensor
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 1;  // Height above base_link
    pose.orientation.w = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 1;
    pose.orientation.z = 0;

    // Create the set_model_state request
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = model_name;
    srv.request.model_state.pose = pose;
    srv.request.model_state.reference_frame = "world";

    // Call the service to set the model state
    if (set_model_state_client.call(srv))
    {
        ROS_INFO("Initial Pose published");
    }
    else
    {
        ROS_ERROR("Failed to call set_model_state service");
        return 1;
    }

    // Spawn bunny
    ros::ServiceClient load_mesh_client = nh.serviceClient<viewpoint_planning::LoadMesh>("/load_mesh");
    viewpoint_planning::LoadMesh lm_srv;
    lm_srv.request.mesh_file = "test_bunny";  // Set the mesh file name
    if (load_mesh_client.call(lm_srv))
    {
        ROS_INFO("Successfully called load_mesh service");
    }
    else
    {
        ROS_ERROR("Failed to call load_mesh service");
    }

    // Set the center of the circle
    double center_x = 0.0;
    double center_y = 0.0;
    double radius = 0.6;  // Adjust the radius as needed

    // Number of poses in the circle
    int num_poses = 6;

    // Calculate the angle between each pose
    double angle_increment = 2 * M_PI / num_poses;

    

    // Wait for user input to start publishing poses
    std::cout << "Press Enter to start publishing poses..." << std::endl;
    std::cin.get();

    for (int i = 0; i < num_poses; ++i)
    {
        // Calculate the pose
        double angle = i * angle_increment;
        geometry_msgs::Pose pose;
        pose.position.x = center_x + radius * std::cos(angle);
        pose.position.y = center_y + radius * std::sin(angle);
        pose.position.z = 0.7;  // Height above base_link
        // Calculate the orientation (pointing towards the origin)
        tf::Quaternion q;
        q.setRPY((M_PI/2) + std::atan2(pose.position.z, radius), 0, -(M_PI/2)+angle);
        geometry_msgs::Quaternion quat_msg;
        tf::quaternionTFToMsg(q, quat_msg);
        pose.orientation.w = quat_msg.w;
        pose.orientation.x = quat_msg.x;
        pose.orientation.y = quat_msg.y;
        pose.orientation.z = quat_msg.z;

        // Create the set_model_state request
        gazebo_msgs::SetModelState srv;
        srv.request.model_state.model_name = model_name;
        srv.request.model_state.pose = pose;
        srv.request.model_state.reference_frame = "world";

        // Call the service to set the model state
        if (set_model_state_client.call(srv))
        {
            ROS_INFO("Pose %d published", i + 1);
            // Wait for 1 second to ensure pose changes have been reflected
            ros::Duration(1.0).sleep();
            // Stage the Capture surface request
            ros::ServiceClient capture_surface_client = nh.serviceClient<std_srvs::Trigger>("/capture_surface");
            std_srvs::Trigger cs_srv;
            if (capture_surface_client.call(cs_srv))
            {
                ROS_INFO("Surface %d captured", i+1);
            }
            else
            {
                ROS_ERROR("Failed to call capture_surface service");
                return 1;
            }
        }
        else
        {
            ROS_ERROR("Failed to call set_model_state service");
            return 1;
        }

        // Wait for user input before sending the next pose
        if (i < num_poses - 1)
        {
            std::cout << "Press Enter to send the next pose..." << std::endl;
            std::cin.get();
        }
    }

    ROS_INFO("All poses published!");

    return 0;

}