/*
 *  Created By: Tan You Liang, Sept 2018
 *  --------- for 3D Slam Testing ------------
 *  change parameter in icpMatching() function
 *  This node is used in straightener.launch file, to straighten pointcloud
 *  Usage: pcd2octomap <input_file> <output_file> --rotate
 */

#include <iostream>
#include <assert.h>
 
// ROS
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <geometry_msgs/Twist.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
 
// octomap 
#include <octomap/octomap.h>

double octotree_co =  0.05; //octo tree coefficient
float roll  = 0;
float pitch = 0;
float yaw   = 0;
float angle_resolution = 0.002;


// convert pointcloudXYZ data to octotree format, and output it
void octotree_conversion(pcl::PointCloud<pcl::PointXYZ> cloud, std::string output_file){

    std::cout << "copy data into octomap..." << std::endl;
    // octo tree coefficient
    octomap::OcTree tree( octotree_co );
 
    for (int i = 0; i < cloud.points.size() ; i++)
    {
        tree.updateNode( octomap::point3d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z), true );
    }
 
    // update octomap
    tree.updateInnerOccupancy();
    // saving .bt file
    tree.writeBinary( output_file );
    std::cout << "Done All!!" << std::endl;
 
}

// leverage on turtlebot teleop arrow key control
void teleop_callback(const geometry_msgs::Twist::ConstPtr& KeyIn){

    roll = roll + angle_resolution*KeyIn->linear.x;
    pitch = pitch + angle_resolution*KeyIn->angular.z;
    yaw   = yaw + angle_resolution*KeyIn->linear.z;

    std::cout << "[teleop_cb] roll: " <<  roll << " pitch: " << pitch << "yaw: " << yaw << std::endl;
    std::cout << std::flush;
}

void pointcloud_rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){
    // Rotate the values obtained from teleop
    Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 
    transform.rotate (Eigen::AngleAxisf (roll,  Eigen::Vector3f::UnitX()));
    transform.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (yaw,  Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*input_cloud, *output_cloud, transform);

    // Rotate to set valid position
    Eigen::Affine3f trans_flip = Eigen::Affine3f::Identity();
    trans_flip.rotate(Eigen::AngleAxisf(M_PI,      Eigen::Vector3f::UnitY()));
    trans_flip.rotate(Eigen::AngleAxisf(M_PI/2.0f, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*output_cloud, *output_cloud, trans_flip);
}

void add_origin_crosses(pcl::PointCloud<pcl::PointXYZ>& cloud){
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (cloud, minPt, maxPt);

    const float cross_length = 0.3f;
    for (float cross_height = minPt.z ; cross_height < maxPt.z ; cross_height += 0.5){
        pcl::PointXYZ origin(0.0, 0.0, cross_height);
        cloud.push_back(origin);
        for (float i = 0 ; i < cross_length ; i=i + 0.01){
            pcl::PointXYZ point1(i, 0, cross_height);
            cloud.push_back(point1);
            pcl::PointXYZ point2(-i, 0, cross_height);
            cloud.push_back(point2);
            pcl::PointXYZ point3(0, i, cross_height);
            cloud.push_back(point3);
            pcl::PointXYZ point4(0, -i, cross_height);
            cloud.push_back(point4);
        }
    }
}


// // ======================================= Main ================================================

int main( int argc, char** argv )
{
    if (argc < 3 || pcl::console::find_switch (argc, argv, "-h") )
    {
        std::cout << " - Run this script to convert .pcd file to .bt file" << std::endl;
        std::cout << "Usage: pcd2octomap <input_file> <output_file>" << std::endl;
        std::cout << "Usage: pcd2octomap <input_file> <output_file> -s <size>" << std::endl;
        std::cout << " - Rotate Mode>" << std::endl;
        std::cout << "Usage: pcd2octomap <input_file> <output_file> --rotate " << std::endl;
        return -1;
    }

    //get arg point size
    if (pcl::console::find_switch (argc, argv, "-s")){
        int input_idx = pcl::console::find_argument (argc, argv, "-s") + 1;
        std::stringstream ss( argv[input_idx] );
        if ( !(ss >> octotree_co))
        std::cout << "Invalid double...\n";
    } 
    std::cout << "Input double arg for '-s' is " << octotree_co << std::endl;

    // io file
    std::string input_file = argv[1], output_file = argv[2];
    std::cout <<"Input File is: " << input_file << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPCDFile<pcl::PointXYZ> ( input_file, *cloud );
    
    std::cout << "OctoTree coefficient is = "<< octotree_co  << std::endl;
    std::cout << "point cloud loaded, point size = "<<cloud->points.size() << std::endl;
 
    // Rotate?
    if (pcl::console::find_switch (argc, argv, "--rotate")){

        std::cout << "\t Rotate Mode!!!\n";
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

        ros::init(argc, argv, "PCD_Rotate");
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/pcd", 100);
        ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, teleop_callback);
        ros::Rate loop_rate(5); //1hz

        while (ros::ok()){
            // rotate poincloud
            pointcloud_rotate( cloud, transformed_cloud);
            add_origin_crosses(*transformed_cloud);
            
            // Output msg
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*transformed_cloud, msg);
            msg.header.frame_id = "world";
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        octotree_conversion(*transformed_cloud, output_file);
    } 
    else{
        octotree_conversion(*cloud, output_file);
    }



    return 0;

}