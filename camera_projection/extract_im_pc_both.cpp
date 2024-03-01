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

template <class M>
   class BagSubscriber : public message_filters::SimpleFilter<M>
   {
   public:
     void newMessage(const boost::shared_ptr<M const> &msg)
     {
       this->signalMessage(msg);
     }
   };


void callback(const sensor_msgs::PointCloud2::ConstPtr point_cloud,
              const gmsl_frame_msg::FrameInfo::ConstPtr &c_info)
   {


    cv::Mat image;

    while (camera_video.get(CV_CAP_PROP_POS_FRAMES)< c_info->frame_counter)
      camera_video.read(image);
    

    cv::Size sz = image.size();
    int imageWidth = sz.width;

    if (imageWidth <1){
      return;
    }
  

    std::string s1 = std::to_string(c_info->header.stamp.nsec);
    unsigned int number_of_zeros1 = 9 - s1.length(); 
    s1.insert(0, number_of_zeros1, '0');
    std::string filename_0 = save_images_directory + "/" + camera + std::to_string(c_info->header.stamp.sec) +"."+ s1 + ".png" ;
    cv::imwrite(filename_0, image);

    std::string s = std::to_string(point_cloud->header.stamp.nsec);
    unsigned int number_of_zeros = 9 - s.length(); 
    s.insert(0, number_of_zeros, '0');
    std::string filename_pc = save_images_directory + "/"  + std::to_string(point_cloud->header.stamp.sec) +"."+s+ ".pcd" ;

    pcl::io::savePCDFile(filename_pc, *point_cloud); 
    
  }


int main(int argc, char* argv[])
  {
     ros::init(argc, argv, "extract_im_pc");
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
           } else if (std::strcmp(argv[i], "-c") == 0) {
              camera = argv[i + 1];
           }
        }
    }

    cv::String path_video(cv::format( "%s/%s-%s.mp4", directory.c_str(), name.c_str(),camera.c_str()));

    std::cout << "Video file: " << path_video << " \n";
    
     camera_video.open(path_video);
     if( !camera_video.isOpened() ){
             throw "Error when reading steam_h264";
     }

     std::cout << "Video loaded \n";

     //Read bag
     rosbag::Bag bag;
     std::string bagname = directory + "/" + name + ".bag";
     std::cout << "bag directory " << bagname <<"\n";
     bag.open(bagname, rosbag::bagmode::Read);

     std::string c_cam_info =  "/sekonix_camera/"+camera+"/frame_info";
     std::string velodyne_p =  "/ouster/points"; 

     // Topics to load
     std::vector<std::string> topics;

     topics.push_back(c_cam_info);
     topics.push_back("/sekonix_camera/port_a_cam_0/frame_info");
     topics.push_back("/sekonix_camera/port_a_cam_0/camera_info");
     topics.push_back("/sekonix_camera/port_a_cam_1/frame_info");
     topics.push_back("/sekonix_camera/port_a_cam_1/camera_info");
     topics.push_back("/sekonix_camera/port_b_cam_0/frame_info");
     topics.push_back("/sekonix_camera/port_b_cam_0/camera_info");
     topics.push_back("/sekonix_camera/port_b_cam_1/frame_info");
     topics.push_back("/sekonix_camera/port_b_cam_1/camera_info");
     topics.push_back("/sekonix_camera/port_c_cam_0/frame_info");
     topics.push_back("/sekonix_camera/port_c_cam_0/camera_info");
     topics.push_back("/sekonix_camera/port_c_cam_1/frame_info");
     topics.push_back("/sekonix_camera/port_c_cam_1/camera_info");
     topics.push_back("/sekonix_camera/port_d_cam_0/frame_info");
     topics.push_back("/sekonix_camera/port_d_cam_0/camera_info");
     topics.push_back("/sekonix_camera/port_d_cam_1/frame_info");
     topics.push_back("/sekonix_camera/port_d_cam_1/camera_info");

     topics.push_back(velodyne_p);

     rosbag::View view(bag, rosbag::TopicQuery(topics));

     // Set up fake subscribers to capture frameinfo, velodyne pc and odometry
     BagSubscriber<gmsl_frame_msg::FrameInfo> c_info_sub;
     BagSubscriber<sensor_msgs::PointCloud2> velodyne_p_sub;

     // Use ApproximateTime synchronizer to make sure we get properly synchronized images / pointcloud
     typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, gmsl_frame_msg::FrameInfo> MySyncPolicy;
     message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), velodyne_p_sub, c_info_sub);
     sync.registerCallback(boost::bind(&callback, _1, _2));

     std::cout << "reading bag  \n";

     BOOST_FOREACH(rosbag::MessageInstance const m, view)
     {

     if (m.getTopic() == velodyne_p || ("/" + m.getTopic() == velodyne_p))
       {
         sensor_msgs::PointCloud2::ConstPtr vel_p = m.instantiate<sensor_msgs::PointCloud2>();
         if (vel_p != NULL)
           velodyne_p_sub.newMessage(vel_p);
     }

      if (m.getTopic() == c_cam_info || ("/" + m.getTopic() == c_cam_info))
      {
        gmsl_frame_msg::FrameInfo::ConstPtr c_info = m.instantiate<gmsl_frame_msg::FrameInfo>();
        if (c_info != NULL)
          c_info_sub.newMessage(c_info);
      }

    }
    bag.close();
    return 0;
  }
