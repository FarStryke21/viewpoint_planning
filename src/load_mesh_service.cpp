#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include "viewpoint_planning/LoadMesh.h"

bool loadMesh(viewpoint_planning::LoadMesh::Request &req, viewpoint_planning::LoadMesh::Response &res) 
{
    std::string mesh_file = req.mesh_file;
    
    std::string mesh_directory;

    // Get the mesh_directory parameter
    if (!ros::param::get("mesh_directory", mesh_directory)) 
    {
        // Parameter not set, use default value
        mesh_directory = "/path/to/meshes";
    }

    std::string model_name = req.mesh_file;
    std::string file_path = mesh_directory + "/" + mesh_file  + "/model.sdf";

    // Check if the file exists
    std::ifstream file(file_path);
    if (!file.good()) 
    {
        ROS_ERROR("Mesh file does not exist: %s", file_path.c_str());
        res.success = false;
        return true;
    }

    // Read the file contents
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string model_xml = buffer.str();

    // Define model pose
    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = 0.0;
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.0;

    // Delete the model if it already exists
    ros::NodeHandle nh;
    ros::ServiceClient delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel delete_srv;
    delete_srv.request.model_name = model_name;
    delete_client.call(delete_srv);

    // Spawn the model
    ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn_srv;
    spawn_srv.request.model_name = model_name;
    spawn_srv.request.model_xml = model_xml;
    spawn_srv.request.robot_namespace = "";
    spawn_srv.request.initial_pose = initial_pose;
    spawn_srv.request.reference_frame = "world";

    if (spawn_client.call(spawn_srv)) 
    {
        if (spawn_srv.response.success) 
        {
            ROS_INFO("Successfully spawned model: %s", model_name.c_str());
            res.success = true;
        } 
        else 
        {
            ROS_ERROR("Failed to spawn model: %s", spawn_srv.response.status_message.c_str());
            res.success = false;
        }
    } 
    else 
    {
        ROS_ERROR("Failed to call spawn service");
        res.success = false;
    }

    return true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "load_mesh_service_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("load_mesh", loadMesh);
    ROS_INFO("Ready to load mesh files.");
    ros::spin();

    return 0;
}
