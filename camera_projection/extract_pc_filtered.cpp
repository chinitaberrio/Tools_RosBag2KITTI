#include <ros/ros.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/flann.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <gmsl_frame_msg/FrameInfo.h>
#include <boost/foreach.hpp>
#include <sstream>
#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rosbag/bag.h>

using namespace Eigen;

cv::VideoCapture camera_video;

//velodyne_rawdata::RawData data;
std::string directory, name, save_images_directory, camera;


int main(int argc, char* argv[])
  {
     ros::init(argc, argv, "extract_pc_filtered");
     ros::Time::init();
     ros::NodeHandle nh;

     std::cout << "parsing \n";
     //Parsing arguments
     for (int i = 1; i < argc; i++) {
       if (i + 1 != argc){
           if (std::strcmp(argv[i], "-d") == 0) {
              directory = argv[i + 1];
           } else if (std::strcmp(argv[i], "-n") == 0) {
              name = argv[i + 1];
           } else if (std::strcmp(argv[i], "-s") == 0) {
              save_images_directory = argv[i + 1];
           }
        }
    }

    cv::String path_video(cv::format( "%s/%s-%s.mp4", directory.c_str(), name.c_str(),camera.c_str()));


     //Read bag
     rosbag::Bag bag;
     std::string bagname = directory + "/" + name + ".bag";
     std::cout << "bag directory " << bagname <<"\n";
     bag.open(bagname, rosbag::bagmode::Read);

     std::string velodyne_p =  "/ouster/points"; 

     // Topics to load
     std::vector<std::string> topics;

     topics.push_back(velodyne_p);

     rosbag::View view(bag, rosbag::TopicQuery(topics));


     std::cout << "reading bag  \n";

     BOOST_FOREACH(rosbag::MessageInstance const m, view)
     {

     if (m.getTopic() == velodyne_p || ("/" + m.getTopic() == velodyne_p))
       {
         sensor_msgs::PointCloud2::ConstPtr vel_p = m.instantiate<sensor_msgs::PointCloud2>();
         if (vel_p != NULL)
         {

            pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
            pcl_conversions::toPCL (*(vel_p), *(pcl_input));

            pcl::CropBox<pcl::PCLPointCloud2> cropBoxFilter (true);
            cropBoxFilter.setInputCloud (pcl_input);
            Eigen::Vector4f min_pt (4.0f,  -100.0f, -100.0f, 1.0f);
            Eigen::Vector4f max_pt (100.0f, 100.0f,  100.0f, 1.0f);

            // Cropbox slighlty bigger then bounding box of points
            cropBoxFilter.setMin (min_pt);
            cropBoxFilter.setMax (max_pt);

            // Indices
            std::vector<int> indices;
            cropBoxFilter.filter (indices);

            // Cloud
            pcl::PCLPointCloud2 cloud_out;
            cropBoxFilter.filter (cloud_out);


            std::string filename_pc = save_images_directory + "/"  + std::to_string(vel_p->header.stamp.sec) +"."+std::to_string(vel_p->header.stamp.nsec)+ ".pcd" ;
            std::cout << filename_pc << " \n";
            pcl::io::savePCDFile(filename_pc, cloud_out); 
         }
           
     }

    }
    bag.close();
    return 0;
  }
