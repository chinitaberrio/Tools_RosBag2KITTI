# Lidar-camera-lidar Projection

Off-line projection Velodyne Point Cloud to Camera Image A0, A1 and A2, and A0 Camera Image RGB values to the Velodyne Point Cloud.

### Required packages

  - Velodyne  http://wiki.ros.org/velodyne_driver
  - Eigen
  - gmsl_frame_msg

### Installation

```sh
$ cd catkin_ws/src
$ git clone https://gitlab.acfr.usyd.edu.au/jber4282/extract_im_pc
$ cd ..
$ catkin_make
```

**Node**         : camrea_projection

**Publishes**    :

> -  /camera/image_c (sensor_msgs::Image)
> -  /camera/image_l (sensor_msgs::Image)
> -  /camera/image_r (sensor_msgs::Image)
> -  /velodyne_points (sensor_msgs::PointCloud2)
> -  /velodyne_color (sensor_msgs::PointCloud2)
> -  /vn100/odometry (nav_msgs::Odometry)
> -  /tf_static (tf2_msgs::TFMessage)
> -  /tf (tf2_msgs::TFMessage)

**Parameters**
-d Directory to the bag and video files
-n Name of the rosbag
-i Select "true" if you want to project the corresponding RGB values of A0 into the point cloud (default: false)
-p Select "true" if you want to save the images (default: false)
-s Directory where to save the images (default:  bag directory)

**Running**     : rosrun extract_im_pc extract_im_pc -d /home/acfr/Documents/2018-11-02_Projection_Test_3_cams -n 2018-11-02-13-58-35_Projection_Test_3_cams -p true -s /home/acfr/Documents/2018-11-02_Projection_Test_3_cams/images
