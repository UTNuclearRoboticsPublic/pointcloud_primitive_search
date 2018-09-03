
#ifndef PRIMITIVE_PROCESS_PUBLISHER_H
#define PRIMITIVE_PROCESS_PUBLISHER_H

#include <ros/ros.h>
// Rosbag Stuff
#include <rosbag/bag.h>
#include <rosbag/view.h>
// Primitive Search Stuff
#include "pointcloud_primitive_search/primitive_process.h"
// Point Cloud Library
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sstream>

class PrimitiveProcessPublisher
{
public:
	// Default constructor - if this is called, initialization is required prior to use
	PrimitiveProcessPublisher();
	// Prefered regular constructor (includes initialization of publishers)
	PrimitiveProcessPublisher(ros::NodeHandle nh, pointcloud_primitive_search::primitive_process process);
	// Initialize publishers to topics for each cloud
	void updatePublishers(pointcloud_primitive_search::primitive_process process);
	// Actually publish outputs on each topic, one for each cloud
	void publish(pointcloud_primitive_search::primitive_process);
	// Save bag file versions of the clouds
	void saveClouds(pointcloud_primitive_search::primitive_process process);
	// Checks whether the system is currently initialized (has active publishers) 
	bool isInitialized();

private:
	std::vector<ros::Publisher> clipping_pub_;
	std::vector<ros::Publisher> clipping_pub_less_;
	std::vector<ros::Publisher> primitive_pub_;
	std::vector<ros::Publisher> primitive_pub_less_; 
	std::vector<ros::Publisher> marker_pub_;

	ros::NodeHandle nh_;
	bool is_initialized_;
};
#endif //PRIMITIVE_PROCESS_PUBLISHER_H